"""指令文本渲染与 lint。这是与训练团队的契约层，不许自由发挥。

依据 `Instruction Spec — 采集端 prompt 格式 & 现有 pattern.html` 的 Part 1。
公共骨架::

    {Verb} the {obj} [{prep} the {target}] with the {arm} arm

只实现桌面 pick&place 需要的三个动词根（Pick up / Place / Pass），其余六个动词
（Pour/Put/Scoop/Open/Hold/Push）属于泡茶与 supermarket 业务，这里不生成，但
`LEGAL_VERBS` 仍收全，因为 lint 第 9 条要按完整集合判。

Spec 标题写「12 个合法 verb」，但表格末尾自己澄清「数一下：上面是 9 个动词根」——
那 12 是把 Place 的介词分支分开数的。动词根集合以 9 个为准。

**前后方位的 canonical 是 `in front of` / `behind`**。对面那份 RobotControl 实现文档
§9.3 写的是 `to the front of` / `to the back of`，那恰好命中 lint 第 13 条 —— 他们的
实现违反了自己的规格，别跟着抄。

lint 的定位（Spec §1.5）：**发布前**批量随机模拟时命中数必须为 0；**运行时**只记录
警告，绝不阻塞落盘 —— 数据丢了比数据脏难修复得多。
"""

from __future__ import annotations

import re
from dataclasses import dataclass

LEGAL_VERBS = ('Pick up', 'Place', 'Pass', 'Pour', 'Put', 'Scoop',
               'Open', 'Hold', 'Push')

#: Place 的介词分支 -> (英文介词, 中文模板)。中文用「把 X ... Y」句式。
PLACE_PREPS = {
    'into': ('into the', '放入{target}中'),
    'on': ('on the', '放在{target}上'),
    'left_of': ('to the left of the', '放在{target}左边'),
    'right_of': ('to the right of the', '放在{target}右边'),
    'in_front_of': ('in front of the', '放在{target}前面'),
    'behind': ('behind the', '放在{target}后面'),
}

ARM_ZH = {'left': '左手', 'right': '右手'}

#: 九宫格白名单，lint 第 14 条用。pick&place 目前不生成它，留着是为了校验外来文本。
NAMED_POSITIONS = ('front left', 'front center', 'front right',
                   'center left', 'center', 'center right',
                   'back left', 'back center', 'back right')

#: 多个原子步骤合并成一条时的分隔符，中英一致（Spec 要求步数对齐）。
STEP_SEP = '；'


@dataclass(frozen=True)
class Instruction:
    """一个**原子动作**。一条 episode 对应一个它。

    上游 pp_g1 的 skeleton 计数逐项相加恰好等于 episode 数（1737 与 1986 两组都对得上），
    所以交付粒度是「一条 episode = 一个 verb」，不是一整条多步指令。
    """

    verb: str                       # 'Pick up' | 'Place' | 'Pass'
    arm: str                        # 执行手 'left' | 'right'
    obj_en: str
    obj_zh: str
    obj_id: str = ''
    prep: str = ''                  # PLACE_PREPS 的键，仅 Place 用
    target_en: str = ''
    target_zh: str = ''
    target_id: str = ''
    arm_to: str = ''                # 仅 Pass 用
    step_index: int = 0
    step_total: int = 1

    def render_en(self) -> str:
        if self.verb == 'Pick up':
            return f'Pick up the {self.obj_en} with the {self.arm} arm'
        if self.verb == 'Pass':
            return (f'Pass the {self.obj_en} to the {self.arm_to} arm '
                    f'with the {self.arm} arm')
        if self.verb == 'Place':
            if not self.prep:
                return f'Place the {self.obj_en} with the {self.arm} arm'
            prep_en = PLACE_PREPS[self.prep][0]
            return (f'Place the {self.obj_en} {prep_en} {self.target_en} '
                    f'with the {self.arm} arm')
        raise ValueError(f'pick&place 不生成动词 {self.verb}')

    def render_zh(self) -> str:
        hand = ARM_ZH[self.arm]
        if self.verb == 'Pick up':
            return f'{hand}拿起{self.obj_zh}'
        if self.verb == 'Pass':
            return f'{hand}把{self.obj_zh}交给{ARM_ZH[self.arm_to]}'
        if self.verb == 'Place':
            if not self.prep:
                return f'{hand}放下{self.obj_zh}'
            tail = PLACE_PREPS[self.prep][1].format(target=self.target_zh)
            return f'{hand}把{self.obj_zh}{tail}'
        raise ValueError(f'pick&place 不生成动词 {self.verb}')

    def as_dict(self) -> dict:
        """落盘用。结构化字段和契约字段都留 —— 字符串是给模型看的，字段是给自己看的。"""
        d = {
            'verb': self.verb, 'arm': self.arm,
            'obj': {'id': self.obj_id, 'en': self.obj_en, 'zh': self.obj_zh},
            'step_index': self.step_index, 'step_total': self.step_total,
            'instruction_en': self.render_en(),
            'instruction_zh': self.render_zh(),
        }
        if self.prep:
            d['prep'] = self.prep
            d['target'] = {'id': self.target_id, 'en': self.target_en,
                           'zh': self.target_zh}
        if self.arm_to:
            d['arm_to'] = self.arm_to
        d['lint_warnings'] = lint(self.render_en())
        return d


def join_steps(instructions) -> str:
    """把多个原子步骤合并成一条多步文本。落盘按 episode 拆开，这个只给操作者看。"""
    return STEP_SEP.join(i.render_en() for i in instructions)


_VERB_RE = re.compile(r'^(Pick up|Place|Pass|Pour|Put|Scoop|Open|Hold|Push)\b')
_PASS_RE = re.compile(
    r'^Pass the (?P<obj>.+) to the (?P<to>left|right) arm '
    r'with the (?P<from>left|right) arm$')
_AT_POS_RE = re.compile(r'\bat the (?P<pos>.+?) of the ')


def lint(text: str, vocabulary: set[str] | None = None) -> list[str]:
    """Spec §1.5 的 14 条检查。返回命中的规则编号+说明，空列表 = 合规。

    运行时只用来记录警告，**不要拿它拦落盘**。
    """
    hits: list[str] = []
    if text.rstrip().endswith('.'):
        hits.append('1 句尾有句号')
    if re.search(r'using the (right|left) arm', text):
        hits.append('2 用了 using the ... arm')
    if re.search(r'with the (right|left) hand', text):
        hits.append('3 用了 hand 而不是 arm')
    if 'the held ' in text:
        hits.append('4 冗余修饰 the held')
    if text.startswith('Put down '):
        hits.append('5 Put down 开头，应改 Place ... on the table')
    if text.startswith('Hand over '):
        hits.append('6 Hand over 开头，应改 Pass')
    if 'draining tray' in text:
        hits.append('7 draining tray 应合并为 tea tray')
    if re.search(r'\bon the tray\b', text):
        hits.append('8 裸 on the tray，应写全 on the tea tray')
    m = _VERB_RE.match(text)
    if not m:
        hits.append('9 动词不在 9 个合法动词根内')
    if text.startswith('Pass '):
        pm = _PASS_RE.match(text.strip())
        if not pm:
            hits.append('11 Pass 句式不匹配标准模板')
        elif pm.group('to') == pm.group('from'):
            hits.append('10 Pass 的 arm_from 与 arm_to 相同')
    if vocabulary is not None:
        for phrase in _extract_entities(text):
            if phrase not in vocabulary:
                hits.append(f'12 词表外的物品/目标: {phrase}')
    if 'to the front of ' in text or 'to the back of ' in text:
        hits.append('13 应使用 canonical 的 in front of / behind')
    pm = _AT_POS_RE.search(text)
    if pm and pm.group('pos') not in NAMED_POSITIONS:
        hits.append(f'14 命名区域不在九宫格白名单: {pm.group("pos")}')
    if re.search(r'\bin front of the table\b', text):
        hits.append('14 桌面区域不能写成 in front of the table')
    return hits


_ENTITY_RES = (
    re.compile(r'^Pick up the (?P<a>.+?) with the (?:left|right) arm$'),
    re.compile(r'^Pass the (?P<a>.+?) to the (?:left|right) arm '
               r'with the (?:left|right) arm$'),
    re.compile(r'^Place the (?P<a>.+?) (?:into|on|in front of|behind) the '
               r'(?P<b>.+?) with the (?:left|right) arm$'),
    re.compile(r'^Place the (?P<a>.+?) to the (?:left|right) of the '
               r'(?P<b>.+?) with the (?:left|right) arm$'),
    re.compile(r'^Place the (?P<a>.+?) with the (?:left|right) arm$'),
)


def _extract_entities(text: str) -> list[str]:
    """从成句的指令里抠出 obj / target。只用于 lint 第 12 条。

    生成端本来就知道 slot 值，正常路径不需要反解 —— 这个函数是给校验外来文本用的。
    """
    for pattern in _ENTITY_RES:
        m = pattern.match(text.strip())
        if m:
            return [v for v in m.groupdict().values() if v]
    return []

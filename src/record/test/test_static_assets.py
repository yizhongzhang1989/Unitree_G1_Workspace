"""前端引用的 class 有没有对应的 CSS 规则。

写这个是因为改样式最常见的翻车方式不是报错，而是**静默地没生效** ——
JS 里 `classList.add('mute')`、CSS 里没这条，页面照样渲染，只是丑；
而且往往是 hover / 展开之后才出现的状态，浏览器里扫一眼未必撞得到。

只查一个方向（用了但没定义）。反方向查不动：类名经常是当变量传的
（`banner(text, 'bad')`），静态扫描分不清哪些是样式钩子。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

STATIC = Path(__file__).resolve().parents[1] / 'record' / 'static'
PAGES = {
    'data': ('data.html', 'data.js', 'common.js'),
    'recorder': ('index.html', 'app.js', 'common.js'),
}

_BLOCK = re.compile(r'/\*.*?\*/', re.S)
_LINE = re.compile(r'(?<![:\w])//[^\n]*')
CSS = _BLOCK.sub('', (STATIC / 'app.css').read_text(encoding='utf-8'))
DEFINED = set(re.findall(r'\.([a-zA-Z][\w-]*)', CSS))

#: 这些不是样式钩子：JS 拿它们定位元素或存状态，没有对应规则是正常的
NOT_STYLING = {'ep', 'episodes', 'split', 'wide', 'panel', 'note', 'acts'}


def used_classes(names) -> set:
    out = set()
    for name in names:
        text = _LINE.sub('', _BLOCK.sub('', (STATIC / name).read_text(encoding='utf-8')))
        for attr in re.findall(r'class(?:Name)?\s*=\s*[\'"`]([^\'"`]*)', text):
            out |= {c for c in attr.split() if c and '$' not in c}
        out |= set(re.findall(r'classList\.(?:add|toggle|remove)\(\s*[\'"]([\w-]+)', text))
        # `cls = 'root' + (x ? ' active' : '')` 这种拼接，只在提到 class 的行上找
        for line in text.splitlines():
            if re.search(r'class(Name|List)?\s*[+=]', line):
                out |= set(re.findall(r"['\"]\s+([a-zA-Z][\w-]*)\s*['\"]", line))
    return {c for c in out if not c.isupper()}


@pytest.mark.parametrize('page', sorted(PAGES))
def test_every_class_has_a_rule(page):
    missing = sorted(used_classes(PAGES[page]) - DEFINED - NOT_STYLING)
    assert not missing, f'{page} 用了但 app.css 里没有：{missing}'


def test_the_check_would_catch_a_typo():
    """守着上面那条断言本身 —— 抽不出类名的话它永远是绿的。"""
    found = used_classes(('data.js',))
    for expect in ('card', 'twist', 'dhead', 'preview', 'mute', 'spacer'):
        assert expect in found, f'抽不到 {expect}，提取规则失效了'


@pytest.mark.parametrize('page', sorted(PAGES))
def test_ids_referenced_by_js_exist_in_html(page):
    """`$('x')` 拿不到元素时是运行时 TypeError，整个脚本停在那里。"""
    names = PAGES[page]
    have = set(re.findall(r'id="([\w-]+)"', (STATIC / names[0]).read_text(encoding='utf-8')))
    for js in names[1:]:
        text = _LINE.sub('', (STATIC / js).read_text(encoding='utf-8'))
        want = set(re.findall(r"\$\(\s*'([\w-]+)'\s*\)", text))
        assert want <= have, f'{page} 缺元素：{sorted(want - have)}'

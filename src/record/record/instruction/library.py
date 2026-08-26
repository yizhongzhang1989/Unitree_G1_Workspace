"""离线物品库读取与物品组挑选。

对面把库放在 Azure Blob 上，靠 ETag + 60 s TTL + 本地 cache 三级结构访问。采集机连不上
公司网，所以这里只保留最后一级：库随包分发。

导出包原本有 773 个文件 / 57 MB，入库的只有 `db/item_library.db`（代码只读它）。
`preview/` 那 662 张历史 backfill 图占了 49 MB，`crops/` 的参考图代码也不读，都与采集无关，
已删。不再启动时核对 SHA256 —— db 直接从 git 里出来，git 本身就是内容寻址的。

usage 统计**用我们自己的**，不用对方的历史 snapshot —— cold-tail 排序要的是「我们还没
采过什么」，不是「他们还没采过什么」。
"""

from __future__ import annotations

import json
import os
import random
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path

#: Scene 生成必需的属性。任何一个缺失，该 item 直接排除出可用集合 ——
#: 对面的做法是让整库加载失败，那会让一个未补全的新物品拖垮整个采集。
REQUIRED_ATTRS = ('footprint_mm', 'portable', 'main_color', 'is_container',
                  'is_surface', 'supported_poses', 'pose_svgs', 'pose_footprints_mm')

DEFAULT_CACHE = Path(os.environ.get('RECORD_HOME', Path.home() / '.ros' / 'record'))


@dataclass(frozen=True)
class Item:
    item_id: str
    name_en: str
    name_zh: str
    color: str
    width: float                  # 默认姿态占地，米，沿 y（横向）
    depth: float                  # 米，沿 x（纵深）
    portable: bool
    is_container: bool
    is_surface: bool
    poses: tuple[str, ...]
    pose_svgs: dict = field(repr=False, default_factory=dict)
    pose_footprints: dict = field(repr=False, default_factory=dict)

    @property
    def role(self) -> str:
        if self.is_container:
            return 'container'
        if self.is_surface:
            return 'surface'
        return 'ordinary' if self.portable else 'fixed'

    def footprint(self, pose: str | None = None) -> tuple[float, float]:
        """某个姿态下的占地 (宽, 深)，单位米。姿态缺尺寸就退回默认占地。"""
        if pose and pose in self.pose_footprints:
            fp = self.pose_footprints[pose]
            return fp['w'] / 1000.0, fp['d'] / 1000.0
        return self.width, self.depth

    def svg(self, pose: str | None = None) -> str:
        if pose and pose in self.pose_svgs:
            return self.pose_svgs[pose]
        return next(iter(self.pose_svgs.values()), '')


class LibraryError(RuntimeError):
    pass


LIBRARY_SUBDIR = 'item-library'


class ItemLibrary:
    """只读的物品库。构造时一次性把 active item 读进内存，之后不再碰磁盘。"""

    def __init__(self, library_dir: str | os.PathLike,
                 usage_path: str | os.PathLike | None = None) -> None:
        self.dir = Path(library_dir)
        db = self.dir / 'db' / 'item_library.db'
        if not db.is_file():
            raise LibraryError(f'{db} 不存在')
        self.usage_path = Path(usage_path) if usage_path else DEFAULT_CACHE / 'item_usage.json'
        self.items: dict[str, Item] = {}
        self.skipped: dict[str, list[str]] = {}
        self._load(db)
        self._usage = self._load_usage()

    def _load(self, db: Path) -> None:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        con.row_factory = sqlite3.Row
        try:
            rows = con.execute("SELECT * FROM items WHERE status='active'").fetchall()
        finally:
            con.close()
        for r in rows:
            try:
                attrs = json.loads(r['attributes_json'] or '{}')
            except json.JSONDecodeError:
                self.skipped[r['item_id']] = ['attributes_json 不是合法 JSON']
                continue
            missing = [k for k in REQUIRED_ATTRS if k not in attrs]
            if not r['canonical_en']:
                missing.append('canonical_en')
            if missing:
                self.skipped[r['item_id']] = missing
                continue
            fp = attrs['footprint_mm']
            self.items[r['item_id']] = Item(
                item_id=r['item_id'],
                name_en=r['canonical_en'],
                name_zh=r['canonical_zh'] or r['canonical_en'],
                color=attrs.get('main_color') or '#888888',
                width=float(fp['w']) / 1000.0,
                depth=float(fp['d']) / 1000.0,
                portable=bool(attrs['portable']),
                is_container=bool(attrs['is_container']),
                is_surface=bool(attrs['is_surface']),
                poses=tuple(attrs['supported_poses']),
                pose_svgs=dict(attrs['pose_svgs']),
                pose_footprints=dict(attrs['pose_footprints_mm']),
            )

    def _load_usage(self) -> dict:
        try:
            return json.loads(self.usage_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return {}

    def usage(self, item_id: str, role: str) -> int:
        return int(self._usage.get(item_id, {}).get(role, 0))

    def bump_usage(self, item_id: str, role: str, n: int = 1) -> None:
        """采完一条 episode 就累加。cold-tail 排序靠它。"""
        entry = self._usage.setdefault(item_id, {})
        entry[role] = entry.get(role, 0) + n
        entry['total'] = entry.get('total', 0) + n

    def save_usage(self) -> None:
        self.usage_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.usage_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self._usage, ensure_ascii=False, indent=1), encoding='utf-8')
        tmp.replace(self.usage_path)

    def usable(self, geometry, clearance: float | None = None) -> list[Item]:
        """能放进可达域的物品。**对掩码判**，不是对外接矩形判。"""
        kwargs = {} if clearance is None else {'clearance': clearance}
        out = []
        for item in self.items.values():
            for pose in item.poses or ('single',):
                w, d = item.footprint(pose)
                # 横竖各试一次：可达域是条「香蕉形」的带，转 90° 常常就塞得下
                if geometry.can_fit(w, d, **kwargs) or geometry.can_fit(d, w, **kwargs):
                    out.append(item)
                    break
        return out

    def choose_group(self, geometry, size: int = 4, rng: random.Random | None = None,
                     cold_pool: int = 12) -> list[Item]:
        """挑一组物品：1 个容器 + 其余普通物品，按 cold-tail 优先。

        对面的构成是「1 容器 + 1 承载面 + N 普通」，但我们的可达域只有他们的 1/4，
        220x220 的承载面只剩 5 个可放位置，强行要求会让布局频繁失败 —— 所以承载面
        改成可选，由调用方按需要加。
        """
        rng = rng or random.Random()
        if size < 2:
            raise ValueError('一组至少 2 件')
        pool = self.usable(geometry)
        containers = [i for i in pool if i.is_container]
        ordinary = [i for i in pool if i.role == 'ordinary']
        if not containers:
            raise LibraryError('可达域内没有容器，place_in 无法生成')
        if len(ordinary) < size - 1:
            raise LibraryError(f'可用普通物品只有 {len(ordinary)} 件，凑不够 {size - 1}')

        containers.sort(key=lambda i: (self.usage(i.item_id, 'as_y'), i.item_id))
        ordinary.sort(key=lambda i: (self.usage(i.item_id, 'as_x'), i.item_id))
        chosen = [rng.choice(containers[:min(cold_pool, len(containers))])]
        chosen += rng.sample(ordinary[:max(cold_pool, size * 3)], size - 1)
        return chosen

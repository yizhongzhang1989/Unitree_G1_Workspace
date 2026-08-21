"""摆放样例 SVG：操作者照着这张图把真实桌面摆出来。

不现画物品形状 —— 每件物品在库里已有审核过的俯视 SVG，这里只做「按真实毫米占地
等比缩放 → 旋转 → 合成到桌面画布」。画布用机器人坐标：屏幕向上 = 远离机器人。
"""

from __future__ import annotations

import math
import re

_SVG_OPEN = re.compile(r'<svg[^>]*viewBox\s*=\s*["\']([^"\']+)["\'][^>]*>',
                       re.IGNORECASE)
_SVG_CLOSE = re.compile(r'</svg\s*>\s*$', re.IGNORECASE)

MM = 1000.0        # 米 -> 毫米，画布单位就用毫米


def _inner(svg: str) -> tuple[str, float, float]:
    """抠出 <svg> 的内容和 viewBox 尺寸。取不到就退回 100x100 的空白。"""
    m = _SVG_OPEN.search(svg or '')
    if not m:
        return '', 100.0, 100.0
    parts = m.group(1).replace(',', ' ').split()
    try:
        vw, vh = float(parts[2]), float(parts[3])
    except (IndexError, ValueError):
        vw = vh = 100.0
    body = _SVG_CLOSE.sub('', svg[m.end():])
    return body, (vw or 100.0), (vh or 100.0)


def render_scene(placements, geometry, *, title: str = '') -> str:
    """把一组 Placement 渲染成摆放样例。返回完整的 SVG 文本。"""
    x0, x1, y0, y1 = geometry.bounds
    pad = 0.04
    # 画布 u 轴 = -y（机器人左手在屏幕左侧），v 轴 = -x（远离机器人在屏幕上方）
    u0, u1 = -(y1 + pad), -(y0 - pad)
    v0, v1 = -(x1 + pad), -(x0 - pad)
    w, h = (u1 - u0) * MM, (v1 - v0) * MM

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{u0 * MM:.1f} '
           f'{v0 * MM:.1f} {w:.1f} {h:.1f}" width="{w:.0f}" height="{h + 46:.0f}">',
           '<rect x="-9999" y="-9999" width="19998" height="19998" fill="#0f172a"/>']

    # 可达格：铺一层底，操作者能看出哪些位置根本放不了东西
    cell = geometry.cell * MM
    for i, x in enumerate(geometry.xs):
        for j, y in enumerate(geometry.ys):
            left, right = bool(geometry.left[i, j]), bool(geometry.right[i, j])
            if not (left or right):
                continue
            fill = '#1e3a5f' if left and right else ('#14342a' if left else '#3a1e2e')
            out.append(f'<rect x="{-y * MM - cell / 2:.1f}" y="{-x * MM - cell / 2:.1f}" '
                       f'width="{cell:.1f}" height="{cell:.1f}" fill="{fill}"/>')

    out.append(f'<line x1="0" y1="{v0 * MM:.1f}" x2="0" y2="{v1 * MM:.1f}" '
               'stroke="#64748b" stroke-dasharray="12 8" stroke-width="2"/>')
    out.append(f'<text x="{u0 * MM + 12:.1f}" y="{v1 * MM - 10:.1f}" fill="#94a3b8" '
               'font-size="26" font-family="sans-serif">机器人在这一侧</text>')

    for p in placements:
        body, vw, vh = _inner(p.item.svg(p.pose))
        w_m, d_m = p.item.footprint(p.pose)
        used_w, used_d = w_m * MM, d_m * MM
        s = min(used_w / vw, used_d / vh)          # 等比，不拉伸
        cu, cv = -p.cy * MM, -p.cx * MM
        rot = -math.degrees(p.rotation)
        out.append(
            f'<g transform="translate({cu:.1f},{cv:.1f}) rotate({rot:.1f}) '
            f'translate({-used_w / 2:.1f},{-used_d / 2:.1f}) '
            f'translate({(used_w - vw * s) / 2:.1f},{(used_d - vh * s) / 2:.1f}) '
            f'scale({s:.4f})">{body}</g>')
        out.append(f'<text x="{cu:.1f}" y="{cv + used_d / 2 + 22:.1f}" fill="#e2e8f0" '
                   f'font-size="22" text-anchor="middle" '
                   f'font-family="sans-serif">{_escape(p.item.name_zh)}</text>')

    if title:
        out.append(f'<text x="{u0 * MM + 12:.1f}" y="{v0 * MM + 30:.1f}" fill="#f8fafc" '
                   f'font-size="28" font-family="sans-serif">{_escape(title)}</text>')
    out.append('</svg>')
    return ''.join(out)


def _escape(text: str) -> str:
    return (text.replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;').replace('"', '&quot;'))

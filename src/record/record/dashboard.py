"""采集面板（默认 :8220）。只是观察者 + 命令入口，HTTP 骨架见 ``webui``。"""

from __future__ import annotations

from record.webui import Panel, make_handler


def panel(rec, port: int = 8220, host: str = '0.0.0.0') -> Panel:
    actions = {
        '/api/session/start': lambda b: rec.start_session(
            b.get('streams') or {}, b.get('note', '')),
        '/api/session/stop': lambda b: rec.stop_session(),
        '/api/round/preview': lambda b: rec.preview_round(
            b.get('seed'), keep_items=bool(b.get('keep_items'))),
        '/api/table': lambda b: rec.set_table(
            float(b['depth_mm']) / 1000.0, float(b['width_mm']) / 1000.0,
            float(b['near_mm']) / 1000.0),
        '/api/round/start': lambda b: rec.start_round(b.get('seed')),
        '/api/round/end': lambda b: rec.end_round(),
        '/api/episode/start': lambda b: rec.start_episode(int(b['index'])),
        '/api/episode/end': lambda b: rec.end_episode(
            b.get('outcome', 'success'), b.get('note', '')),
        '/api/episode/relabel': lambda b: rec.relabel_take(
            int(b['slot']), int(b['take']), b['outcome']),
    }

    def scene_svg(h, arg):
        svg = rec.pending_svg()          # 预览优先：重 roll 后要立刻看到新的
        if not svg and rec.session is not None and rec.session.round_index >= 0:
            f = rec.session.paths.rounds / f'round_{rec.session.round_index:03d}.svg'
            svg = f.read_text(encoding='utf-8') if f.is_file() else ''
        return h.send_bytes(200, svg.encode(), 'image/svg+xml; charset=utf-8')

    gets = {
        '/api/state': lambda h, arg: h.send_json(
            {'status': rec.status(), 'streams': rec.stream_overview()}),
        '/api/round/svg': scene_svg,
        '/api/snapshot': lambda h, arg: h.send_bytes(
            200, rec.snapshot(arg('key')), 'image/jpeg'),
        '/api/preview': lambda h, arg: h.send_bytes(
            200, rec.preview(arg('key')), 'image/jpeg'),
    }
    return Panel(rec, make_handler(rec, actions, gets), port, '采集面板', host)

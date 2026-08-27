"""采集面板（默认 :8220）。只是观察者 + 命令入口，HTTP 骨架见 ``webui``。"""

from __future__ import annotations

from urllib.parse import parse_qs

from record.webui import Panel, make_handler


def panel(rec, port: int = 8220, host: str = '0.0.0.0') -> Panel:
    actions = {
        '/api/session/start': lambda b: rec.start_session(
            b.get('streams') or {}, b.get('note', '')),
        '/api/session/stop': lambda b: rec.stop_session(),
        '/api/round/preview': lambda b: rec.preview_round(
            b.get('seed'), keep_items=bool(b.get('keep_items'))),
        '/api/round/start': lambda b: rec.start_round(b.get('seed')),
        '/api/round/end': lambda b: rec.end_round(),
        '/api/episode/start': lambda b: rec.start_episode(int(b['index'])),
        '/api/episode/end': lambda b: rec.end_episode(
            b.get('outcome', 'success'), b.get('note', '')),
    }

    def route(h, u):
        if u.path in ('/', '/index.html'):
            return h.send_static('index.html')
        if u.path in ('/app.js', '/app.css', '/common.js'):
            return h.send_static(u.path.lstrip('/'))
        if u.path == '/api/state':
            return h.send_json({'status': rec.status(),
                                'streams': rec.stream_overview()})
        if u.path == '/api/round/svg':
            svg = rec.pending_svg()      # 预览优先：重 roll 后要立刻看到新的
            if not svg and rec.session is not None and rec.session.round_index >= 0:
                f = (rec.session.paths.rounds
                     / f'round_{rec.session.round_index:03d}.svg')
                svg = f.read_text(encoding='utf-8') if f.is_file() else ''
            return h.send_bytes(200, svg.encode(), 'image/svg+xml; charset=utf-8')
        if u.path == '/api/snapshot':
            return h.send_bytes(
                200, rec.snapshot((parse_qs(u.query).get('key') or [''])[0]), 'image/jpeg')
        if u.path == '/api/preview':
            return h.send_bytes(
                200, rec.preview((parse_qs(u.query).get('key') or [''])[0]), 'image/jpeg')
        if u.path in actions:
            # 前端漏传 body 就会变成 GET，一律 404 的话错误条完全看不出来是方法错了
            return h.send_json({'error': f'{u.path} 只接受 POST'}, 405)
        return h.send_json({'error': 'not found'}, 404)

    return Panel(rec, make_handler(rec, actions, route), port, '采集面板', host)

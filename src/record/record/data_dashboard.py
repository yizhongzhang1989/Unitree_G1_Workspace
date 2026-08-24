"""数据管理面板（默认 :8221）。HTTP 骨架见 ``webui``。

单独一个端口而不是并进采集面板：回放会让机器人真动、删除不可逆，和「正在采集」是
性质不同的操作，放在一起容易误点。
"""

from __future__ import annotations

from urllib.parse import parse_qs

from record.webui import Panel, make_handler


def panel(rec, port: int = 8221, host: str = '0.0.0.0') -> Panel:
    actions = {
        '/api/replay/start': lambda b: rec.start(
            b['session'], float(b['t0']), float(b['t1']),
            float(b.get('speed', 1.0)), b.get('label', '')),
        '/api/replay/stop': lambda b: rec.stop(),
        '/api/session/delete': lambda b: rec.delete(b['session'], b.get('confirm', '')),
        '/api/control/engage': lambda b: rec.trigger('engage'),
        '/api/control/estop': lambda b: rec.trigger('estop'),
    }

    def route(h, u):
        q = parse_qs(u.query)
        if u.path in ('/', '/index.html'):
            return h.send_static('data.html')
        if u.path in ('/app.css', '/data.js', '/common.js'):
            return h.send_static(u.path.lstrip('/'))
        if u.path == '/api/state':
            return h.send_json({'status': rec.status(), 'sessions': rec.sessions()})
        if u.path == '/api/session':
            return h.send_json(rec.detail((q.get('id') or [''])[0]))
        if u.path == '/api/frame':
            return h.send_bytes(200, rec.frame((q.get('id') or [''])[0],
                                               (q.get('stream') or [''])[0],
                                               float((q.get('t') or ['0'])[0])),
                                'image/jpeg')
        if u.path in actions:
            return h.send_json({'error': f'{u.path} 只接受 POST'}, 405)
        return h.send_json({'error': 'not found'}, 404)

    return Panel(rec, make_handler(rec, actions, route), port, '数据管理面板', host)

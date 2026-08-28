"""数据管理面板（默认 :8221）。HTTP 骨架见 ``webui``。

单独一个端口而不是并进采集面板：回放会让机器人真动、删除不可逆，和「正在采集」是
性质不同的操作，放在一起容易误点。

下载分两路，形状不同是有原因的：

* **原始数据逐个文件下**。一小时的采集 5.4 GB，流式 zip 断了要从头来，
  而单文件带 ``Range``，浏览器自己会续传。真要整份搬走请用 rsync。
* **转换后的现转现下**。产物极少用到，不值得长期占盘，所以转到临时目录、下完就删。
  但转换是 0.13x 实时（一小时素材 8 分钟），一个请求从头等到尾必然超时，
  所以拆成「触发 → 轮询进度 → 取件」三步。
"""

from __future__ import annotations

from record.webui import Panel, make_handler, tree_entries


def panel(rec, port: int = 8221, host: str = '0.0.0.0') -> Panel:
    actions = {
        '/api/replay/start': lambda b: rec.start(
            b['session'], float(b['t0']), float(b['t1']),
            float(b.get('speed', 1.0)), b.get('label', '')),
        '/api/replay/stop': lambda b: rec.stop(),
        '/api/session/delete': lambda b: rec.delete(b['session'], b.get('confirm', '')),
        '/api/episode/delete': lambda b: rec.delete_episode(
            b['session'], b.get('label', '')),
        '/api/convert/start': lambda b: rec.start_convert(b['session'], b['format']),
        '/api/render/start': lambda b: rec.start_render(
            b['session'], float(b['t0']), float(b['t1']), b.get('label', '')),
        '/api/render/stop': lambda b: rec.stop_render(),
        '/api/render/drop': lambda b: rec.drop_render(b['session'], b.get('label', '')),
        '/api/control/engage': lambda b: rec.trigger('engage'),
        '/api/control/estop': lambda b: rec.trigger('estop'),
    }

    def raw_file(h, arg):
        path = rec.raw_path(arg('id'), arg('file'))
        return h.send_file(path, path.name)

    def raw_zip(h, arg):
        root = rec.raw_dir(arg('id'), arg('dir'))
        name = arg('dir').replace('/', '_') or arg('id')
        return h.send_zip(f'{name}.zip', tree_entries(root))

    def bundle_zip(h, arg):
        token = arg('token')
        root = rec.bundle(token)
        # 取件即销毁：产物只为这一次下载而生，留着就是白占盘
        try:
            return h.send_zip(f'{token}.zip', tree_entries(root))
        finally:
            rec.drop_bundle(token)

    gets = {
        '/api/state': lambda h, arg: h.send_json(
            {'status': rec.status(), 'sessions': rec.sessions(),
             'convert': rec.convert_state(), 'render': rec.render_state(),
             'formats': rec.formats()}),
        '/api/session': lambda h, arg: h.send_json(rec.detail(arg('id'))),
        '/api/raw': lambda h, arg: h.send_json(rec.raw_files(arg('id'))),
        '/api/preview': lambda h, arg: h.send_json(rec.preview(arg('id'), arg('file'))),
        '/api/frame': lambda h, arg: h.send_bytes(
            200, rec.frame(arg('id'), arg('stream'), float(arg('t') or 0.0)),
            'image/jpeg'),
        '/raw': raw_file,
        '/raw.zip': raw_zip,
        # B 一次拿全：tools/ + final.urdf + calibration.yaml
        '/tools.zip': lambda h, arg: h.send_zip('record-tools.zip', rec.tools_bundle()),
        # 内联放而不是下载：这东西是拿来在页上来回拖着看的
        '/verify.mp4': lambda h, arg: h.send_file(
            rec.render_video(arg('id'), arg('label')), ctype='video/mp4'),
        '/bundle.zip': bundle_zip,
    }
    return Panel(rec, make_handler(rec, actions, gets, 'data.html'),
                 port, '数据管理面板', host)

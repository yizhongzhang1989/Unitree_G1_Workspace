#!/usr/bin/env python3
"""读/改腕部 IP 相机（ONVIF_ICAMERA H800_AF）主码流参数，并自检两台是否一致。

    python3 scripts/set_wrist_camera_fps.py                     # 看两台现状
    python3 scripts/set_wrist_camera_fps.py --diff              # 逐项列出不一致
    python3 scripts/set_wrist_camera_fps.py 192.168.123.98 fps 30 --apply
    python3 scripts/set_wrist_camera_fps.py 192.168.123.97 gop 90 --apply

**两台必须保持一致**：参数不同会让两路数据没法用同一套参数处理。出厂就不一样：
.97 是 30 fps / GOP 120，.98 是 25 fps / GOP 75。改完**延迟标定要重做** —— 两路原本
差的 81 ms 约等于 25 fps 的两个帧周期，说明延迟跟编码参数直接相关。

GOP 决定开录后多久等到首个 IDR（`-c copy` 不解码，必须从 IDR 起），直接决定每次
开录开头浪费多少。相机会自己按帧率换算维持固定秒数：.98 从 75@25 自动变成 90@30。

踩过的坑：

* 容器里设了 HTTP_PROXY，走代理到不了内网相机，必须用空 ProxyHandler。
* 相机返回的配置子元素带 ``tt:`` 前缀，写回时请求里必须声明这个命名空间，
  否则 XML 不合法，相机回一个没有 Reason 文本的空故障，极难定位。
* 只能整段取回、只改要改的那个字段再写回。逐字段重建会漏掉 H264Profile /
  Multicast / SessionTimeout，漏掉的相机按默认值覆盖。
* ``UseCount`` 是只读的，原样送回去有的固件会拒绝整个请求。
* ``GovLength`` 在 MPEG4 段里也有一个（恒为 0）且排在前面，取/改都要限定在 H264 段内。
"""

import argparse
import base64
import hashlib
import os
import re
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone

MEDIA = 'http://www.onvif.org/ver10/media/wsdl'
SCHEMA = 'http://www.onvif.org/ver10/schema'
WSSE = ('http://docs.oasis-open.org/wss/2004/01/'
        'oasis-200401-wss-wssecurity-secext-1.0.xsd')
WSU = ('http://docs.oasis-open.org/wss/2004/01/'
       'oasis-200401-wss-wssecurity-utility-1.0.xsd')
DIGEST = ('http://docs.oasis-open.org/wss/2004/01/'
          'oasis-200401-wss-username-token-profile-1.0#PasswordDigest')
B64 = ('http://docs.oasis-open.org/wss/2004/01/'
       'oasis-200401-wss-soap-message-security-1.0#Base64Binary')

WRISTS = ('192.168.123.97', '192.168.123.98')


def _security(user: str, password: str) -> str:
    nonce = os.urandom(16)
    created = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    digest = base64.b64encode(
        hashlib.sha1(nonce + created.encode() + password.encode()).digest()).decode()
    return (f'<s:Header><Security s:mustUnderstand="1" xmlns="{WSSE}">'
            f'<UsernameToken><Username>{user}</Username>'
            f'<Password Type="{DIGEST}">{digest}</Password>'
            f'<Nonce EncodingType="{B64}">{base64.b64encode(nonce).decode()}</Nonce>'
            f'<Created xmlns="{WSU}">{created}</Created>'
            f'</UsernameToken></Security></s:Header>')


def soap(host: str, body: str, user: str, password: str, timeout: float = 15.0) -> str:
    env = ('<?xml version="1.0" encoding="UTF-8"?>'
           '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
           + _security(user, password) + '<s:Body>' + body + '</s:Body></s:Envelope>')
    req = urllib.request.Request(
        f'http://{host}/onvif/media_service', data=env.encode(),
        headers={'Content-Type': 'application/soap+xml; charset=utf-8'})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        return opener.open(req, timeout=timeout).read().decode('utf-8', 'replace')
    except urllib.error.HTTPError as exc:
        return exc.read().decode('utf-8', 'replace')


def main_config(host: str, user: str, password: str) -> tuple[str, str]:
    xml = soap(host, f'<GetVideoEncoderConfigurations xmlns="{MEDIA}"/>', user, password)
    for token, body in re.findall(
            r'<[\w:]*Configurations[^>]*token="([^"]+)"[^>]*>(.*?)</[\w:]*Configurations>',
            xml, re.S):
        if token == 'VideoEncodeMain':
            return token, body
    raise SystemExit(f'{host}: 没读到 VideoEncodeMain\n{xml[:300]}')


def _field(body: str, name: str) -> str:
    m = re.search(rf'<[\w:]*{name}>([^<]*)</[\w:]*{name}>', body)
    return m.group(1) if m else '?'


def _h264_gov(body: str) -> str:
    """GovLength 在 MPEG4 段里也有一个（恒为 0）且排在前面，必须限定在 H264 段内取。"""
    seg = re.search(r'<[\w:]*H264>(.*?)</[\w:]*H264>', body, re.S)
    return _field(seg.group(1), 'GovLength') if seg else '?'


def show(host: str, user: str, password: str) -> None:
    _, body = main_config(host, user, password)
    print(f'  {host}  {_field(body, "Width")}x{_field(body, "Height")}'
          f'  {_field(body, "FrameRateLimit")} fps'
          f'  {_field(body, "BitrateLimit")} kbps'
          f'  GOP {_h264_gov(body)}')


def set_field(host: str, what: str, target: int, user: str, password: str,
              apply_it: bool) -> int:
    token, body = main_config(host, user, password)
    cur = int(_h264_gov(body) if what == 'gop' else _field(body, 'FrameRateLimit'))
    print(f'{host} {token} {what}: {cur} -> {target}')
    if cur == target:
        print('  已是目标值')
        return 0
    if not apply_it:
        print('  预演，未写入。加 --apply 才真改')
        return 0

    if what == 'gop':
        # 只替换 H264 段里的那个，MPEG4 段的同名字段不能碰
        new = re.sub(r'(<[\w:]*H264>.*?<[\w:]*GovLength>)\d+(</[\w:]*GovLength>)',
                     rf'\g<1>{target}\g<2>', body, flags=re.S)
    else:
        new = re.sub(r'(<[\w:]*FrameRateLimit>)\d+(</[\w:]*FrameRateLimit>)',
                     rf'\g<1>{target}\g<2>', body)
    new = re.sub(r'<[\w:]*UseCount>\d+</[\w:]*UseCount>', '', new)
    resp = soap(host, f'<SetVideoEncoderConfiguration xmlns="{MEDIA}" xmlns:tt="{SCHEMA}">'
                      f'<Configuration token="{token}">{new}</Configuration>'
                      f'<ForcePersistence>true</ForcePersistence>'
                      f'</SetVideoEncoderConfiguration>', user, password)
    if 'Fault' in resp:
        text = re.findall(r'<[\w:]*Text[^>]*>([^<]+)<', resp)
        print('  失败:', text or re.sub(r'\s+', ' ', resp)[-300:])
        return 1

    _, after = main_config(host, user, password)
    got = int(_h264_gov(after) if what == 'gop' else _field(after, 'FrameRateLimit'))
    print(f'  读回 {got}')
    if what == 'fps':
        print('  注意：帧率变了，这一路的延迟标定作废，要重标')
    return 0 if got == target else 1


def diff(user: str, password: str) -> int:
    """两台的主码流参数逐项对比。不一致就返回非零，方便当检查用。"""
    a, b = (main_config(h, user, password)[1] for h in WRISTS)
    fields = [('分辨率', lambda x: f'{_field(x, "Width")}x{_field(x, "Height")}'),
              ('帧率', lambda x: _field(x, 'FrameRateLimit')),
              ('码率', lambda x: _field(x, 'BitrateLimit')),
              ('画质', lambda x: _field(x, 'Quality')),
              ('编码', lambda x: _field(x, 'Encoding')),
              ('H264 profile', lambda x: _field(x, 'H264Profile')),
              ('GOP', _h264_gov),
              ('编码间隔', lambda x: _field(x, 'EncodingInterval'))]
    bad = 0
    for name, get in fields:
        va, vb = get(a), get(b)
        flag = '' if va == vb else '   <- 不一致'
        bad += va != vb
        print(f'  {name:14} {WRISTS[0]}={va:<10} {WRISTS[1]}={vb}{flag}')
    print(f'\n{"两台一致" if not bad else f"有 {bad} 项不一致"}')
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('host', nargs='?', help='相机 IP，省略则只列出两台的现状')
    ap.add_argument('what', nargs='?', choices=('fps', 'gop'), help='要改哪个参数')
    ap.add_argument('value', nargs='?', type=int, help='目标值')
    ap.add_argument('--diff', action='store_true', help='逐项对比两台，不一致则退出码非零')
    ap.add_argument('--apply', action='store_true', help='真写入，默认只预演')
    ap.add_argument('--user', default='admin')
    ap.add_argument('--password', default='123456')
    args = ap.parse_args()

    if args.diff:
        print('腕部相机主码流逐项对比:')
        return diff(args.user, args.password)
    if args.host is None:
        print('腕部相机主码流现状:')
        for h in WRISTS:
            show(h, args.user, args.password)
        return 0
    if args.what is None or args.value is None:
        ap.error('给了 IP 就要给参数名和目标值，如: 192.168.123.98 fps 30')
    limit = (1, 30) if args.what == 'fps' else (1, 300)
    if not limit[0] <= args.value <= limit[1]:
        ap.error(f'{args.what} 超出范围 {limit[0]}~{limit[1]}')
    return set_field(args.host, args.what, args.value, args.user, args.password,
                     args.apply)


if __name__ == '__main__':
    sys.exit(main())

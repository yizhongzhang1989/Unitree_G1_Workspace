#!/usr/bin/env python3
"""读/改腕部 IP 相机（ONVIF_ICAMERA H800_AF）主码流参数，并自检两台是否一致。

    python3 scripts/set_wrist_camera_fps.py                     # 看两台现状
    python3 scripts/set_wrist_camera_fps.py --diff              # 逐项列出不一致
    python3 scripts/set_wrist_camera_fps.py 192.168.123.98 fps 30 --apply
    python3 scripts/set_wrist_camera_fps.py 192.168.123.97 gop 90 --apply
    python3 scripts/set_wrist_camera_fps.py --osd               # 看画面上的叠加层
    python3 scripts/set_wrist_camera_fps.py --osd-blank osd_title --apply
    python3 scripts/set_wrist_camera_fps.py --time              # 相机时钟差多少
    python3 scripts/set_wrist_camera_fps.py --time-sync --apply # 改走 NTP

**两台必须保持一致**：参数不同会让两路数据没法用同一套参数处理。出厂就不一样：
.97 是 30 fps / GOP 120，.98 是 25 fps / GOP 75。改完**延迟标定要重做** —— 两路原本
差的 81 ms 约等于 25 fps 的两个帧周期，说明延迟跟编码参数直接相关。

GOP 决定开录后多久等到首个 IDR（`-c copy` 不解码，必须从 IDR 起），直接决定每次
开录开头浪费多少。相机会自己按帧率换算维持固定秒数：.98 从 75@25 自动变成 90@30。

## OSD 与对时

**OSD 是烧进像素的**，录下来就抹不掉。出厂带两条：左上的 ``osd_title``（常量文字
"Camera"）和右下的 ``osd_time``（日期时间）。

* ``osd_title`` **能关**：``--osd-blank osd_title`` 把 PlainText 置空，实测画面上就没了。
* ``osd_time`` **关不掉**，四条路都试过：``DeleteOSD`` 回成功但 OSD 还在（空壳）；
  ``SetOSD`` 改位置能生效，但不允许把它从 DateAndTime 改成 Plain
  （``GetOSDOptions`` 里 ``PlainText="1"``，名额被 ``osd_title`` 占着，先腾也没用）；
  ``DateFormat``/``TimeFormat`` 置空不收；相机的 ``/IPC`` 是 gSOAP 只吃 SOAP，
  Web UI 的 JS 里根本没有 OSD 页。

所以**留着它，改成让它显示正确的时间**：对好时之后那行字就是一个免费的带内时间参考，
可以直接和 ``pts.bin`` 的到达时刻比，而不是污染。注意画面上是**本地时间**
（TZ ``CST-8`` = UTC+8，无夏令时），换成 epoch 要减 8 小时。

对时走 NTP，指向 ``NTP_SERVER``。出厂配的是 ``time.windows.com`` 而且模式是 Manual，
内网到不了那个域名，等于根本没在对时（实测两台分别停在 2018-01-01，彼此还差三小时）。
**手动设整秒不够用**：ONVIF 只收整秒，而相机应用它的时刻自己带几十毫秒的量化，
提前发做补偿不收敛（实测提前 47 ms，偏差从 −47 只动到 −41）。

踩过的坑：

* 容器里设了 HTTP_PROXY，走代理到不了内网相机，必须用空 ProxyHandler。
* 相机返回的配置子元素带 ``tt:`` 前缀，写回时请求里必须声明这个命名空间，
  否则 XML 不合法，相机回一个没有 Reason 文本的空故障，极难定位。
* 只能整段取回、只改要改的那个字段再写回。逐字段重建会漏掉 H264Profile /
  Multicast / SessionTimeout，漏掉的相机按默认值覆盖。
* ``UseCount`` 是只读的，原样送回去有的固件会拒绝整个请求。
* ``GovLength`` 在 MPEG4 段里也有一个（恒为 0）且排在前面，取/改都要限定在 H264 段内。
* 量时钟偏差时，本机参考时刻要取**请求中点**而不是收到回应的时刻：一次 SOAP 往返
  实测 21~26 ms（编安全头要算 SHA1、gSOAP 解 XML 都不快，远比 0.6 ms 的网络 RTT 大），
  拿回应到达时刻去减会凭空多出几十毫秒的「相机慢了」。
"""

import argparse
import base64
import hashlib
import os
import re
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone

MEDIA = 'http://www.onvif.org/ver10/media/wsdl'
DEVICE = 'http://www.onvif.org/ver10/device/wsdl'
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

#: 相机对时指向这里。**G1 的机载计算单元自己就跑着 NTP**（stratum 10），实测它
#: 与开发机的时钟差 0.011 ms，所以指向它就等于和本机同步，不用另外装服务端。
#: 判据：`python3 -c "..."` 往 UDP/123 发一个包看有没有应答（扫过整段，只有它有）。
NTP_SERVER = '192.168.123.161'


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


def soap(host: str, body: str, user: str, password: str, timeout: float = 15.0,
         service: str = 'media') -> str:
    env = ('<?xml version="1.0" encoding="UTF-8"?>'
           '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
           + _security(user, password) + '<s:Body>' + body + '</s:Body></s:Envelope>')
    req = urllib.request.Request(
        f'http://{host}/onvif/{service}_service', data=env.encode(),
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


def osd_list(host: str, user: str, password: str) -> dict:
    """返回 {token: {'kind','position','text'}}。"""
    xml = soap(host, f'<GetOSDs xmlns="{MEDIA}"/>', user, password)
    out = {}
    for token, body in re.findall(
            r'<[\w:]*OSDs[^>]*token="([^"]+)"[^>]*>(.*?)</[\w:]*OSDs>', xml, re.S):
        kind = _field(body, 'Type')
        seg = re.search(r'<[\w:]*TextString>(.*?)</[\w:]*TextString>', body, re.S)
        inner = seg.group(1) if seg else ''
        text = _field(inner, 'PlainText')
        out[token] = {
            'kind': _field(inner, 'Type'),
            'position': _position(body),
            'text': ('' if text == '?' and 'PlainText' in inner else
                     text if text != '?' else
                     f'{_field(inner, "DateFormat")} {_field(inner, "TimeFormat")}'),
            'osd_type': kind,
        }
    return out


def _position(body: str) -> str:
    seg = re.search(r'<[\w:]*Position>(.*?)</[\w:]*Position>', body, re.S)
    return _field(seg.group(1), 'Type') if seg else '?'


def _describe(entry: dict) -> str:
    return f'{entry["kind"]:11} {entry["position"]:<11} {entry["text"]!r}'


def osd_blank(host: str, token: str, user: str, password: str, apply_it: bool) -> int:
    """把一条 Plain 叠加层的文字置空，画面上就不渲染了。

    只对 Plain 类型有效。``osd_time`` 是 DateAndTime，固件不让改类型也不接受
    空格式，关不掉 —— 那一条只能导出时盖（见模块 docstring）。
    """
    entries = osd_list(host, user, password)
    if token not in entries:
        print(f'{host}: 没有 {token}（现有 {", ".join(entries) or "无"}）')
        return 1
    print(f'{host} 清空 {token}: {_describe(entries[token])}')
    if entries[token]['kind'] != 'Plain':
        print(f'  {token} 是 {entries[token]["kind"]}，置空对它无效，改不动')
        return 1
    if not apply_it:
        print('  预演，未写入。加 --apply 才真改')
        return 0
    position = entries[token]['position']
    resp = soap(host, (f'<SetOSD xmlns="{MEDIA}" xmlns:tt="{SCHEMA}">'
                       f'<OSD token="{token}">'
                       f'<tt:VideoSourceConfigurationToken>VideoSourceMain'
                       f'</tt:VideoSourceConfigurationToken><tt:Type>Text</tt:Type>'
                       f'<tt:Position><tt:Type>{position}</tt:Type></tt:Position>'
                       f'<tt:TextString><tt:Type>Plain</tt:Type>'
                       f'<tt:PlainText></tt:PlainText></tt:TextString>'
                       f'</OSD></SetOSD>'), user, password)
    if 'Fault' in resp:
        print('  失败:', re.findall(r'<[\w:]*Text[^>]*>([^<]+)<', resp)
              or re.sub(r'\s+', ' ', resp)[-300:])
        return 1
    after = osd_list(host, user, password)[token]
    print(f'  读回 {_describe(after)}')
    return 0 if after['text'] == '' else 1


# ---------------------------------------------------------------------- 对时


def _camera_utc(host: str, user: str, password: str) -> tuple[float, float, float, str]:
    """(相机 UTC 的 epoch 秒, 本机参考时刻, 往返耗时, 时区串)。相机只给整秒。

    本机参考时刻取**请求中点**，不是收到回应的时刻 —— 相机读自己时钟是在处理
    这条请求的中间某处，拿回应到达时刻去减会把整个往返都算成「相机慢了」。
    这个偏差实测有 40~60 ms：编安全头要算 SHA1、gSOAP 解 XML 都不快，远比
    0.6 ms 的网络 RTT 大。而且它在对时前后同样存在，不修的话怎么补偿都不收敛。
    """
    sent = time.time()
    xml = soap(host, f'<GetSystemDateAndTime xmlns="{DEVICE}"/>',
               user, password, service='device')
    received = time.time()
    seg = re.search(r'<[\w:]*UTCDateTime>(.*?)</[\w:]*UTCDateTime>', xml, re.S)
    if seg is None:
        raise SystemExit(f'{host}: 读不到时间\n{xml[-300:]}')
    field = {name: int(_field(seg.group(1), name))
             for name in ('Year', 'Month', 'Day', 'Hour', 'Minute', 'Second')}
    stamp = datetime(field['Year'], field['Month'], field['Day'], field['Hour'],
                     field['Minute'], field['Second'], tzinfo=timezone.utc).timestamp()
    zone = re.search(r'<[\w:]*TZ>([^<]*)</[\w:]*TZ>', xml)
    return (stamp, (sent + received) / 2.0, received - sent,
            zone.group(1) if zone else '')


def clock_offset(host: str, user: str, password: str,
                 window: float = 2.5) -> tuple[float, float]:
    """(偏差秒, 不确定度)。相机时钟快为正。

    ONVIF 只给整秒，直接相减是 ±0.5 s。这里盯**秒进位的那一瞬**：轮询到相机的秒值
    变了，那次进位一定发生在前后两次轮询之间，取中点当相机的整秒时刻。
    不确定度 = 半个轮询间隔，而轮询间隔就是一次 SOAP 往返。
    """
    previous = _camera_utc(host, user, password)
    deadline = time.time() + window
    while time.time() < deadline:
        current = _camera_utc(host, user, password)
        if current[0] != previous[0]:
            local = (previous[1] + current[1]) / 2.0
            return current[0] - local, (current[1] - previous[1]) / 2.0
        previous = current
    return previous[0] - previous[1], 0.5


def show_clock(host: str, user: str, password: str) -> None:
    offset, uncertainty = clock_offset(host, user, password)
    stamp, _, trip, zone = _camera_utc(host, user, password)
    shown = datetime.fromtimestamp(stamp, timezone.utc)
    print(f'  {host}  相机 UTC {shown:%Y-%m-%d %H:%M:%S}  TZ={zone or "?"}  '
          f'偏差 {offset:+.3f} s ±{uncertainty:.3f}  （SOAP 往返 {trip * 1e3:.0f} ms）')


def sync_clock(host: str, user: str, password: str, apply_it: bool,
               server: str = NTP_SERVER) -> int:
    """让相机走 NTP 对时。

    出厂配的是 ``time.windows.com``，而且模式是 Manual —— 内网到不了那个域名，
    等于根本没在对时（实测两台分别停在 2018-01-01，彼此还差三小时）。

    改成手动设整秒也不行：ONVIF 只收整秒，而相机应用它的时刻自己带几十毫秒的
    量化，补偿不收敛（实测提前 47 ms 发，偏差从 −47 只动到 −41）。NTP 才是对的
    工具 —— 它自己做往返补偿，还持续管住晶振漂移。
    """
    offset, uncertainty = clock_offset(host, user, password)
    print(f'{host} 对时前偏差 {offset:+.3f} s ±{uncertainty:.3f}')
    if not apply_it:
        print(f'  预演，未写入。加 --apply 才真改（会指向 {server}）')
        return 0

    zone = _camera_utc(host, user, password)[3] or 'UTC0'
    _device(host, user, password, (
        f'<SetNTP xmlns="{DEVICE}" xmlns:tt="{SCHEMA}">'
        f'<FromDHCP>false</FromDHCP>'
        f'<NTPManual><tt:Type>IPv4</tt:Type>'
        f'<tt:IPv4Address>{server}</tt:IPv4Address></NTPManual>'
        f'</SetNTP>'), 'SetNTP')
    _device(host, user, password, (
        f'<SetSystemDateAndTime xmlns="{DEVICE}" xmlns:tt="{SCHEMA}">'
        f'<DateTimeType>NTP</DateTimeType>'
        f'<DaylightSavings>false</DaylightSavings>'
        f'<TimeZone><tt:TZ>{zone}</tt:TZ></TimeZone>'
        f'</SetSystemDateAndTime>'), 'SetSystemDateAndTime')

    # 切过去之后相机不是立刻就去问服务器，给它一点时间收敛。
    deadline = time.time() + 45.0
    while True:
        offset, uncertainty = clock_offset(host, user, password)
        if abs(offset) < 0.05 or time.time() > deadline:
            break
    print(f'  指向 {server}，对时后偏差 {offset:+.3f} s ±{uncertainty:.3f}')
    return 0 if abs(offset) < 0.05 else 1


def _device(host: str, user: str, password: str, body: str, what: str) -> None:
    resp = soap(host, body, user, password, service='device')
    if 'Fault' in resp:
        reason = re.findall(r'<[\w:]*Text[^>]*>([^<]+)<', resp)
        raise SystemExit(f'{host} {what} 失败: {reason or resp[-200:]}')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('host', nargs='?', help='相机 IP，省略则对两台都做')
    ap.add_argument('what', nargs='?', choices=('fps', 'gop'), help='要改哪个参数')
    ap.add_argument('value', nargs='?', type=int, help='目标值')
    ap.add_argument('--diff', action='store_true', help='逐项对比两台，不一致则退出码非零')
    ap.add_argument('--osd', action='store_true', help='列出烧进画面的叠加层')
    ap.add_argument('--osd-blank', metavar='TOKEN',
                    help='把一条 Plain 叠加层的文字置空，如 osd_title')
    ap.add_argument('--time', action='store_true', help='看相机时钟与本机差多少')
    ap.add_argument('--time-sync', action='store_true', help='让相机走 NTP 对时')
    ap.add_argument('--ntp-server', default=NTP_SERVER,
                    help=f'对时服务器，默认 {NTP_SERVER}')
    ap.add_argument('--apply', action='store_true', help='真写入，默认只预演')
    ap.add_argument('--user', default='admin')
    ap.add_argument('--password', default='123456')
    args = ap.parse_args()

    hosts = [args.host] if args.host else list(WRISTS)
    if args.time:
        print(f'本机 UTC {datetime.now(timezone.utc):%Y-%m-%d %H:%M:%S}')
        for host in hosts:
            show_clock(host, args.user, args.password)
        return 0
    if args.time_sync:
        return max(sync_clock(h, args.user, args.password, args.apply, args.ntp_server)
                   for h in hosts)
    if args.osd:
        for host in hosts:
            print(f'{host}:')
            for token, entry in osd_list(host, args.user, args.password).items():
                print(f'  {token:<12} {_describe(entry)}')
        return 0
    if args.osd_blank:
        return max(osd_blank(h, args.osd_blank, args.user, args.password, args.apply)
                   for h in hosts)
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

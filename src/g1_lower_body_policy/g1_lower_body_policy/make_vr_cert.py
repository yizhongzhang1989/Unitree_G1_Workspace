#!/usr/bin/env python3
"""给 vr_teleop 的 WebXR 服务签一张局域网自签名证书。

WebXR 只在**安全上下文**里可用，所以头显要么走 ``http://localhost``（靠
``adb reverse`` 转发），要么走 ``https://<本机IP>``。adb 那条路的转发规则会因为
头显休眠、USB 重枚举、无线超时而静默失效，HTTPS 这条不依赖 adb，更适合长时间用。

用法::

    ros2 run g1_lower_body_policy make_vr_cert              # 自动探测本机所有 IPv4
    ros2 run g1_lower_body_policy make_vr_cert 192.168.137.149
    ros2 run g1_lower_body_policy make_vr_cert -o /tmp/mycert

默认写到 ``~/.ros/g1_vr/``，正是 ``vr_teleop.launch.py`` 默认去找的位置——签完
直接起节点就是 HTTPS，不用传参数。放在 ``~/.ros`` 而不是源码树里有两个原因：
私钥不该待在 git 仓库旁边，而且它不会被 ``colcon build`` 或删 ``build/`` 清掉。

证书里必须带**头显实际访问的那个 IP** 作为 SAN。签完在头显浏览器里打开
``https://<IP>:8443``，会跳“连接不是私密连接”，点 **高级 → 继续前往**。
"""

from __future__ import annotations

import argparse
import datetime
import fcntl
import ipaddress
import socket
import struct
from pathlib import Path

# 证书位置与 TLS 端口的**唯一定义**，vr_teleop.py 和 vr_teleop.launch.py 都从这里
# import。各写一遍的话，改了一处另一处就静默地去找不存在的文件、退回明文。
DEFAULT_DIR = Path.home() / '.ros' / 'g1_vr'
DEFAULT_TLS_PORT = 8443
_SIOCGIFADDR = 0x8915       # <linux/sockios.h>，取某个网卡的 IPv4 地址


def local_ips() -> list[str]:
    """本机所有网卡的 IPv4 地址。

    **不要**用 ``getaddrinfo(gethostname())``：Ubuntu 的 /etc/hosts 把主机名映射到
    127.0.1.1，实测那样只能拿到这一个回环地址，签出来的证书在局域网上根本用不了
    （头显访问 https://192.168.x.x 会直接报 CERT_COMMON_NAME_INVALID）。
    """
    found = {'127.0.0.1'}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        for _, name in socket.if_nameindex():
            try:
                packed = fcntl.ioctl(probe.fileno(), _SIOCGIFADDR,
                                     struct.pack('256s', name[:15].encode()))
            except OSError:
                continue        # 该接口没配 IPv4，跳过。
            found.add(socket.inet_ntoa(packed[20:24]))
    return sorted(found)


def write_cert(directory: Path, ips: list[str]) -> tuple[Path, Path]:
    # cryptography 只有真要签证书时才需要。放在函数里，是为了让 launch 文件能
    # 无条件 import 本模块拿 DEFAULT_DIR —— 没装 cryptography 也不该起不来。
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

    san = [x509.DNSName('localhost')]
    san += [x509.IPAddress(ipaddress.ip_address(ip)) for ip in ips]

    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'g1-vr-bridge')])
    now = datetime.datetime.now(datetime.timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        # 往前挪一天：机器人和头显的时钟经常差几分钟，不留余量会直接判成"尚未生效"。
        .not_valid_before(now - datetime.timedelta(days=1))
        .not_valid_after(now + datetime.timedelta(days=825))
        .add_extension(x509.SubjectAlternativeName(san), critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None),
                       critical=True)
        # 有些 TLS 栈会因为缺 serverAuth 直接拒握手，补上不花钱。
        .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]),
                       critical=False)
        .sign(key, hashes.SHA256())
    )

    directory.mkdir(parents=True, exist_ok=True)
    cert_path, key_path = directory / 'cert.pem', directory / 'key.pem'
    cert_path.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
    # 两行都不能省：touch 的 mode 只在**创建**时生效，保证新签的私钥不会有一瞬是 0644；chmod 管的是**重签**，此时旧文件已存在、touch 不会改它的权限
    key_path.touch(mode=0o600, exist_ok=True)
    key_path.chmod(0o600)
    key_path.write_bytes(key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption(),
    ))
    return cert_path, key_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description='给 vr_teleop 的 WebXR 服务签一张局域网自签名证书。')
    parser.add_argument('ips', nargs='*',
                        help='写进证书 SAN 的 IP；不给就自动探测本机所有网卡')
    parser.add_argument('-o', '--out-dir', type=Path, default=DEFAULT_DIR,
                        help=f'输出目录（默认 {DEFAULT_DIR}）')
    args = parser.parse_args()

    ips = args.ips or local_ips()
    cert_path, key_path = write_cert(args.out_dir, ips)
    routable = [ip for ip in ips if not ip.startswith('127.')]
    print(f'证书已签好，有效期 825 天：\n  {cert_path}\n  {key_path} (0600)')
    print(f'覆盖的地址：{", ".join(ips)}')
    if not routable:
        print('⚠ 只签出了回环地址，头显连不上。用 `ip -4 -o addr show` 查本机在'
              '头显网段的地址，再 `make_vr_cert <那个IP>` 重签一次。')
        return
    print('\n起节点后在头显浏览器里打开（点「高级 → 继续前往」跳过证书警告）：')
    for ip in routable:
        print(f'  https://{ip}:{DEFAULT_TLS_PORT}')


if __name__ == '__main__':
    main()

# 导出机（B）用的工具

**怎么拿到这一包**：打开机器人的数据管理面板（`http://<A 的 IP>:8221`），
点右上角的 **⤓ 导出工具**，得到 `record-tools.zip`。解开就是：

```
tools/              你正在看的这一套
calibration.yaml    转换要用（见 §4）
final.urdf          同上
```

后两个文件不在 session 里也不在 `tools/` 里，以前要另外拷 —— 现在随包一起给了。

**不要只拷其中几个文件**，各脚本之间按相对路径互相引用，
`format/YB/export.py` 会去上一级找 `session_reader.py`。

这里的代码**刻意不依赖 ROS**，B 上不需要装 ROS 2，也装不了（Windows 上没有）。
有一条测试用正则守着这点：`tools/` 下任何文件 import `rclpy` 都会让 CI 红。

```
tools/
  README.md            ← 你在读的这份：B 侧环境配置 + 怎么用
  session_reader.py    读 session 的唯一入口
  inspect_session.py   看一眼 / 校验 sha256 / 估视频偏移
  urdf_fk.py           纯 numpy 正运动学（B 上没有 pinocchio）
  align_video.py       标定视频管线延迟
  resample_video.py    视频重采样到统一栅格
  converters.py        格式注册表 —— A 的面板和 B 的命令行读的是同一份
  convert.py           转换的统一入口
  format/YB/export.py  YB 格式的转换脚本
  format/YB/README.md  YB 格式规范（交数据给别人时给这一份）
```

---

## 0. 日常怎么跑

环境只配一次（§1）。之后每次增量同步 + 转换就是一条命令：

```powershell
.\sync_and_convert.ps1
```

它按顺序做完 §2 的两趟 rsync、§2 的 `--verify`、§4 的转换，最后补一层产物自检
（h5 行数 == mp4 帧数、有效率里没有 `0%`、腕相机外参不变量）。
**任何一步不过就停下来报错**，不会把半成品往下传。可以反复跑，已经转过的会自动跳过。

| 开关 | |
| --- | --- |
| `-SyncOnly` | 只拉数据和校验，先不转 |
| `-NoSync` | 不联网，只处理本地已有的 session |
| `-Force` | 已经转过的也重转一遍 |
| `-Session 20260826_050938` | 只处理指定的那一条 |
| `-RobotHost <IP>` | 换一台机器人 |

密码不在脚本里，也不写进任何文件 —— ssh 自己弹提示，由你手输（两趟 rsync 各问一次）。
配了公钥免密就一次都不问。

> **脚本不在这个导出包里**，放在 `tools/` 的上一级、和 `final.urdf` 一起。
> `tools/` 每次从面板重新导出都会被**整个覆盖**，所以别把它挪进 `tools/`。

下面几节讲的是这条命令背后到底做了什么 —— 出问题时照着排查。

---

## 1. B 上要装什么

| 用途 | 需要 |
| --- | --- |
| 读 session、校验 | Python 3.10+、numpy |
| 转成 YB | 再加 h5py、PyYAML、**ffmpeg 和 ffprobe 在 PATH 里** |
| 从 A 拉数据 | rsync（ssh 客户端 Windows 11 自带） |

Windows 11 **自带 OpenSSH 客户端**（`ssh.exe`），但**不带 rsync**。
Git for Windows 也**不带** rsync —— 别在那儿找。

**唯一让原生 Windows 变麻烦的就是 rsync 这一件事。** Python 那半边是可移植的：
已逐条核过 `tools/` 没有 POSIX 专有模块、没有硬编码 `/` 路径、所有文本读写都显式给了
`encoding='utf-8'`（中文 Windows 默认 cp936，漏一个就崩）、读解码流没误开 text 模式。
**但没有在真 Windows 上跑过**，只是静态核过。

怎么选看一件事：**转出来的数据最后在哪儿用。**

| | 选 WSL2 | 选原生 Windows |
| --- | --- | --- |
| 适合 | 训练/上云也在 Linux 侧 | 训练在 Windows（比如本机 GPU + Windows 版 PyTorch） |
| 装依赖 | 一行 apt 装齐四样 | 四样分别装，rsync 要靠 MSYS2 |
| 数据落在 | WSL 的 ext4 VHDX 里 | 直接在 D 盘 |
| 代价 | **VHDX 只涨不缩** —— 在里面删了文件，磁盘空间不会还给 Windows，得手动 `Optimize-VHD`。要在 Windows 侧用数据还得跨一层 | MSYS2 的 rsync 配 Windows 的 ssh 有路径转换（`D:/x` vs `/d/x`）和远程 shell 找不到的老毛病，第一次配要花点时间 |

**折中办法**：只拿 WSL 当 rsync 的载体，直接往 `/mnt/d/...` 写，转换和后续都在原生
Windows 做。拉数据时网络（WiFi）是瓶颈，9p 那点开销无所谓；这样两边的长处都占上了。

### 路线 A：WSL2

```powershell
wsl --install -d Ubuntu          # 装完要重启一次
```

进 Ubuntu 之后：

```bash
sudo apt update
sudo apt install -y rsync python3-numpy python3-h5py python3-yaml ffmpeg
python3 -c "import numpy, h5py, yaml; print(numpy.__version__, h5py.__version__)"
ffmpeg -version | head -1
```

> 数据放 WSL 自己的文件系统里（如 `~/data`）比放 `/mnt/d/` 快 —— 后者要走 9p。
> 但转换是 CPU 密集（3.8 核、0.13x 实时），磁盘不是瓶颈，放 `/mnt/d/` 也能接受，
> 换来的是数据直接落在 Windows 盘上、不占 VHDX。
> 要在资源管理器里看 WSL 里的文件走 `\\wsl$\Ubuntu\home\<用户>\data`。

### 路线 B：原生 Windows

要自己凑齐四样：

```powershell
winget install Python.Python.3.12
pip install numpy h5py pyyaml
winget install Gyan.FFmpeg          # 装完确认 ffmpeg/ffprobe 进了 PATH
winget install MSYS2.MSYS2          # 然后在 MSYS2 里 pacman -S rsync openssh
```

装完自检：

```powershell
python -c "import numpy, h5py, yaml; print('py ok')"
ffprobe -version
rsync --version
```

> **MSYS2 那两个老毛病，具体是这么来的，照着做就绕开了。**
>
> 1. **`pacman -S` 时把 `openssh` 一起装上。** rsync 需要一个远程 shell，MSYS2 默认不带，
>    于是它顺着 PATH 找到 Windows 自带的 `C:\Windows\System32\OpenSSH\ssh.exe` ——
>    两边路径语义不一样，「远程 shell 找不到」和路径转换的怪毛病都是这么来的。
>    装上 MSYS2 自己的 ssh，再用 `RSYNC_RSH=/usr/bin/ssh` 把它钉死。
> 2. **`C:\msys64\usr\bin` 追加到 PATH 末尾，不要前置。** 那个目录里有 `find.exe`、
>    `sort.exe`、`tar.exe`、`ssh.exe` 一堆和 Windows 同名的程序，前置会把系统命令盖掉。
>
> 另外路径别写盘符：`C:\data` 里的冒号会被 rsync 当成 `host:path`，要写成 `/c/data`。
> 最省事的是**用相对路径**，直接绕开这件事（`sync_and_convert.ps1` 就是这么做的）。

> Windows 有 260 字符路径上限，但实测够用：最长的那个是
> `video_rightcam/00000001-20260821_102522__g1__round0_episode0.mp4`，64 字符，
> 放在 `C:\Users\...\Documents\robot\datasets\20260821\` 下也才 116。除非你把输出
> 目录嵌得非常深，不用管这条。

---

## 2. 从 A 把数据拉过来

**B 主动拉，A 什么都不用起** —— A 的 sshd 已经在跑，session 在 `~/.ros/record/sessions/`。

```bash
rsync -av --partial \
      --exclude=DONE --exclude='*.ts.txt' \
      unitree@<A 的 IP>:.ros/record/sessions/  ~/data/sessions/
```

三条规矩，都不是可选的：

- **`--exclude=DONE`** —— `DONE` 是「这次采集完整收尾了」的标记。边录边拉时它还没有
  意义，先同步过去会让你误以为可以开导了。**最后一趟再把它拉过来。**
- **`--exclude='*.ts.txt'`** —— 封口时会被转成 `pts.bin` 然后删掉的中间文件。
- **绝不加 `--delete`** —— A 上封口时会删中间文件，带 `--delete` 会把 B 上已经拉全的
  那份也一起删了。

> **目的地的父目录得先建出来。** rsync 只创建路径的**最后一级**：目标写
> `datasets/sessions/` 而 `datasets` 不存在时，它直接报
> `mkdir "..." failed: No such file or directory (2)`，退出码 11。先 `mkdir -p`。
> 重新解开导出包之后特别容易碰到 —— 那一下可能把 `datasets/` 一起换掉了。

**采集还在进行时也能拉。** 录制期间对 session 目录的写只有两种：追加
（`signals/*.bin`、`events.jsonl`、mkv）和一次性创建（`manifest.json`、`meta.json`、
`rounds/*.json`），**没有任何文件会被改写**，所以拉到的前缀一定是完好的。
但 `pts.bin` / `schema.json` / `DONE` 要到封口才出现，**没封口的 session 拉过去也读不了**。

> **别加 `--append`。** 直觉上它能省掉「重传已拉过的那一半」，实测**并不能**：走 ssh 时
> rsync 默认就用增量算法，一个 80 MB 的文件已拉一半时，加不加 `--append` 都只发 40 MB，
> 区别只是本地少算一遍校验（0.12 s vs 0.75 s）。网络才是瓶颈，这点 CPU 省得毫无意义。
> 代价却是实打实的：`--append` **认定目标文件是源文件的前缀**，一旦哪天有文件被改写
> （尤其改短），它会直接跳过、在 B 上留一份永远修不好的旧文件。

### 收尾那一趟

```bash
rsync -av --checksum unitree@<A 的 IP>:.ros/record/sessions/  ~/data/sessions/
python3 inspect_session.py ~/data/sessions/<session>  --verify
```

`--checksum` 逐文件比内容而不是比「大小+时间」，把增量过程中任何错漏补上；
`--verify` 拿 `DONE` 里记的 sha256 逐文件核对，**输出「全部一致」才算搬完**。

> **A 的 sshd 跑在容器里**，容器没起就连不上。
> 认证走 `unitree` 的密码；要免密就把 B 的公钥追加到 A 的 `~/.ssh/authorized_keys`
> —— 注意那个目录**不是挂载出来的，容器重建会丢**。

---

## 3. 先看一眼拿到了什么

```bash
python3 inspect_session.py <session 目录>            # 概览：时长、信号、episode
python3 inspect_session.py <session 目录> --verify   # 逐文件核对 sha256
python3 inspect_session.py <session 目录> --align    # 估视频时间偏移（要 ffprobe）
```

要自己读原始数据：

```python
from session_reader import Session
s = Session(r'D:\data\sessions\20260821_102522')

t, q = s.table('joint_states')                # 时间列 + 数据列
for ep in s.episodes():                       # 默认不含 discard 掉的
    print(ep['instruction_en'], ep['outcome'])
    t, q = s.slice_table('joint_states', ep['t0'], ep['t1'])
    frames = s.slice_frames('head', ep['t0'], ep['t1'])
```

---

## 4. 转成目标格式

先看有哪些格式、依赖齐没齐：

```bash
python3 convert.py --list
```

缺什么它会直接说（比如「缺 python 模块 h5py」「缺 命令 ffmpeg」），别等跑到一半才发现。

```bash
python3 convert.py <session 目录> --to yb -o <输出目录> \
        --urdf final.urdf --calibration calibration.yaml
```

可选参数：`--video-height`（默认 360，给 0 保持源分辨率）、`--hz`（默认 30）、
`--keep-idle`（默认 1.0）、`--no-trim`。

**默认会把每条 episode 首尾的无动作空转裁掉**，首尾各留 `--keep-idle` 秒。
操作者要走去点「开始/成功」，实测这种空转能占到总时长的四成。判据是遥操作**目标**
在不在变（末端目标位姿的速率 + 夹爪开度的幅度），所以「全程夹着东西递给另一只手」
不会被误判成一直有动作，VR 扬机抖一下也不会。
裁了多少在导出日志里逐条打出来，原始采集不动，`--no-trim` 可关。

**只导标注为 success 的 episode**，`fail` / `discard` 一律跳过，丢掉几条会在
日志第一行报出来。失败轨迹对模仿学习是负样本，混进去等于教模型把东西碰倒。

**`--urdf` 是必填的。** 这两个文件不在 session 里，但已经随「导出工具」包一起给你了
（就在 `tools/` 旁边）。它们在 A 上的出处：

| 文件 | A 上的位置 |
| --- | --- |
| `final.urdf` | `src/unitree_g1_description/model/final.urdf` |
| `calibration.yaml` | `src/camera_calibration/config/calibration.yaml` |

为什么要它们：`end_space` 的末端位姿、腕相机的外参在原始数据里都没有，得靠正运动学
从关节角现算，而机器人当时用的 URDF 是「`final.urdf` 叠上 `calibration.yaml` 里的
`urdf_overrides`」。**`--calibration` 对本机是必填** —— 腕相机的 `camera_left` /
`camera_right` 两个 link 是标定文件用 `create` 现建的，裸 URDF 里根本没有，不给就直接
报错退出（头部则是静默退回 URDF 名义值，和实际装配差几毫米）。两个文件的 sha256 会写进
`meta/end_space/fk_provenance`，两次导出结果对不上时先比这个。

**转换会打一行有效率，别跳过它。** 某一路整段没数据时，输出的形状完全正常、内容全是
NaN，不看这一行发现不了 —— `fpc_commands` 就这么整整丢过两个 session。

产出长什么样、每个字段什么含义，看 [`format/YB/README.md`](format/YB/README.md)。
**把数据交给别人时，连那份规范一起给。**

### 想加一种新格式

一个格式一个 `format/<名>/` 目录，里面至少要有 `export.py`（命令行接口是
`<session> -o <out> [选项]`）和 `README.md`（格式规范），然后往 `converters.py` 的
`CONVERTERS` 里加一条。A 的面板下拉框和 `convert.py --list` 会自动出现这一项。

通用的东西（读 session、FK、视频重采样）留在 `tools/` 根下共用，**别拷进格式目录** ——
两份 FK 一漂，两种格式导出的末端位姿就对不上了。

---

## 5. 出问题时

| 症状 | 多半是 |
| --- | --- |
| `！session 没有 DONE，未正常收尾` | session 还没封口，或者你 rsync 时漏了收尾那一趟 |
| `！找不到 ffmpeg/ffprobe，跳过视频` | 没进 PATH。`ffprobe -version` 自己试一下 |
| `跑不了 yb：缺 python 模块 h5py` | `pip install h5py`，或 WSL 里 `apt install python3-h5py` |
| `跑不了 yb：缺 python 模块 yaml` | `pip install pyyaml`，或 `apt install python3-yaml`。**别改成不带 `--calibration` 绕过去** —— 腕相机的 link 就在那份文件里，不给直接报错 |
| `跑不了 yb：缺 命令 ffmpeg` | 同上，ffmpeg 没进 PATH |
| 有效率那行出现 `0%` | 那一路整段没录上，别拿这份数据训练 |
| `--verify` 报不一致 | 传输没搞完，重跑一趟 `--checksum` |
| Windows 上路径报错 | 260 字符上限，输出目录挪浅一点 |
| `rsync: ... mkdir "..." failed: No such file or directory (2)`，退出码 11 | 目的地的**父目录**不存在。rsync 只建最后一级，先 `mkdir -p`（§2） |
| `Connection closed by <IP> port 22`，退出码 12 | **不是密码错**（密码错是 `Permission denied, please try again.`）。A 的 sshd 容器没起或刚好在重启。先探一下 `ssh -o BatchMode=yes unitree@<A 的 IP> exit`：回 `Permission denied (publickey,password)` 就说明 sshd 活着，重跑即可 |
| `'rsync' 不是内部或外部命令` | MSYS2 没进 PATH。把 `C:\msys64\usr\bin` 追加到**末尾**（§1 路线 B） |
| ffmpeg 明明装了，却还是说找不到 | 终端继承的是**它启动时**的环境快照，早于 winget 那次安装。重开终端，或当场执行 `$env:Path = [Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [Environment]::GetEnvironmentVariable('Path','User')` |
| `.ps1` 里的中文变乱码、报一堆语法错误 | Windows PowerShell 5.1 对**没有 BOM** 的 `.ps1` 按系统 ANSI 代码页解码（中文 Windows 是 cp936）。脚本要存成 **UTF-8 with BOM** |

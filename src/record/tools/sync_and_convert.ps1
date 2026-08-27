<#
.SYNOPSIS
    从机器人 A 增量同步 session，校验，并转换成 YB 数据集。

.DESCRIPTION
    严格按 tools/README.md 执行：

      1. rsync -av --partial --exclude=DONE --exclude='*.ts.txt'    增量拉取（§2）
      2. rsync -av --checksum                                       收尾那一趟，补齐 DONE（§2）
      3. inspect_session.py --verify                                必须输出「全部一致」（§2）
      4. convert.py --to yb --urdf ... --calibration ...            转换（§4）
      5. 产物自检：h5 行数 == mp4 帧数；有效率无 0%；腕相机外参不变量（YB 规范 §5.2）

    不加 --append、不加 --delete，原因见 README §2。

    密码不写在这里，也不写进任何文件 —— ssh 自己弹提示，由你手动输入。
    两趟 rsync 会各问一次；配了公钥免密就一次都不问
    （注意 A 的 ~/.ssh 不是挂载出来的，容器重建会丢，见 README §2）。

    可以反复跑：已经转换过的 session 会自动跳过，要重转加 -Force。

.PARAMETER SyncOnly
    只同步和校验，不转换。

.PARAMETER NoSync
    跳过同步，直接对本地已有的 session 做校验和转换。

.PARAMETER Force
    已经转换过的 session 也重新转一遍。

.EXAMPLE
    .\sync_and_convert.ps1
    日常增量同步 + 转换。

.EXAMPLE
    .\sync_and_convert.ps1 -SyncOnly
    只把数据拉回来，先不转。

.EXAMPLE
    .\sync_and_convert.ps1 -NoSync -Session 20260826_050938 -Force
    不联网，重转指定的一条。
#>
#Requires -Version 5.1
[CmdletBinding()]
param(
    [string]   $RobotHost   = '192.168.137.149',
    [string]   $User        = 'unitree',
    [string]   $RemoteRoot  = '.ros/record/sessions/',
    [string]   $DataDir     = 'datasets/sessions',
    [string]   $OutRoot     = 'datasets/yb',
    [string[]] $Session,
    [int]      $VideoHeight = 360,
    [double]   $Hz          = 30,
    [switch]   $SyncOnly,
    [switch]   $NoSync,
    [switch]   $Force
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$env:PYTHONIOENCODING = 'utf-8'
# 不加这个的话 convert.py 自己的 print 会被块缓冲攒到退出才吐，
# 顺序排到它调起的 export.py 后面去
$env:PYTHONUNBUFFERED = '1'
Set-Location $PSScriptRoot

$script:PathReloaded = $false

function Write-Step([string]$Title) {
    Write-Host ''
    Write-Host ('=' * 66) -ForegroundColor DarkCyan
    Write-Host "  $Title" -ForegroundColor Cyan
    Write-Host ('=' * 66) -ForegroundColor DarkCyan
}

function Stop-With([string]$Message, [string]$Hint) {
    Write-Host ''
    Write-Host "[x] $Message" -ForegroundColor Red
    if ($Hint) { Write-Host "    $Hint" -ForegroundColor Yellow }
    exit 1
}

# VS Code 的终端继承的是它启动时的环境快照，之后 winget 装的东西看不见，
# 表现成 README §5 的「缺 命令 ffmpeg」。所以找不到时先从注册表重载一次 PATH。
function Resolve-Native([string]$Name, [string[]]$Probe) {
    $c = Get-Command $Name -CommandType Application -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }

    if (-not $script:PathReloaded) {
        $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                    [Environment]::GetEnvironmentVariable('Path', 'User')
        $script:PathReloaded = $true
        $c = Get-Command $Name -CommandType Application -ErrorAction SilentlyContinue
        if ($c) { return $c.Source }
    }

    foreach ($dir in $Probe) {
        $exe = Join-Path $dir "$Name.exe"
        if (Test-Path $exe) {
            # 追加到末尾而不是开头：MSYS2 的 usr\bin 里有 find/sort/tar 等和 Windows 同名的
            # 工具，前置会把系统命令盖掉。
            if (($env:Path -split ';') -notcontains $dir) {
                $env:Path = $env:Path.TrimEnd(';') + ';' + $dir
            }
            return $exe
        }
    }
    return $null
}

function Invoke-Native([string]$Exe, [string[]]$Arguments) {
    $lines = & $Exe @Arguments 2>&1 | ForEach-Object {
        $text = "$_"
        Write-Host $text
        $text
    }
    return [pscustomobject]@{ ExitCode = $LASTEXITCODE; Lines = $lines }
}

# ---------------------------------------------------------------- 0. 环境自检

Write-Step '0. 环境自检'

$toolsDir = Join-Path $PSScriptRoot 'tools'
foreach ($f in 'convert.py', 'inspect_session.py', 'converters.py', 'session_reader.py') {
    if (-not (Test-Path (Join-Path $toolsDir $f))) {
        Stop-With "缺 tools/$f" ('整包重新从 A 的面板导出（⤓ 导出工具）。' +
                                 '不要只拷其中几个文件，脚本之间按相对路径互相引用。')
    }
}

$urdf  = Join-Path $PSScriptRoot 'final.urdf'
$calib = Join-Path $PSScriptRoot 'calibration.yaml'
foreach ($f in $urdf, $calib) {
    if (-not (Test-Path $f)) {
        Stop-With "缺 $(Split-Path $f -Leaf)" '它随导出工具包一起给，就在 tools/ 旁边（README §4）'
    }
}

$py = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) {
    $found = Get-Command python -CommandType Application -ErrorAction SilentlyContinue
    if (-not $found) { Stop-With '找不到 python' 'README §1 路线 B：winget install Python.Python.3.12' }
    $py = $found.Source
}
& $py -c "import numpy, h5py, yaml" 2>$null
if ($LASTEXITCODE -ne 0) {
    Stop-With 'python 里缺 numpy / h5py / pyyaml' `
              "$py -m pip install numpy h5py pyyaml    （README §5）"
}

$wingetLinks = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links'
$ffprobe = Resolve-Native 'ffprobe' @($wingetLinks)
$ffmpeg  = Resolve-Native 'ffmpeg'  @($wingetLinks)
if (-not $ffprobe -or -not $ffmpeg) {
    Stop-With '找不到 ffmpeg / ffprobe' `
              'winget install Gyan.FFmpeg，然后重启 VS Code 让终端拿到新 PATH（README §5）'
}

$rsync = $null
if (-not $NoSync) {
    $rsync = Resolve-Native 'rsync' @('C:\msys64\usr\bin')
    if (-not $rsync) {
        Stop-With '找不到 rsync' `
                  ('Windows 11 和 Git for Windows 都不带 rsync。' +
                   'winget install MSYS2.MSYS2，然后在 MSYS2 里 pacman -S rsync openssh（README §1 路线 B）')
    }
    # MSYS2 的 rsync 必须用 MSYS2 自己的 ssh。让它顺着 PATH 找到
    # C:\Windows\System32\OpenSSH\ssh.exe 就会踩 README §1 说的路径转换老毛病。
    if (Test-Path 'C:\msys64\usr\bin\ssh.exe') {
        $env:RSYNC_RSH = '/usr/bin/ssh'
    } else {
        Stop-With 'MSYS2 里没装 ssh' 'C:\msys64\usr\bin\bash.exe -lc "pacman -S --noconfirm openssh"'
    }
}

Write-Host ("  python   {0}" -f $py)
Write-Host ("  ffprobe  {0}" -f $ffprobe)
if ($rsync) { Write-Host ("  rsync    {0}  (RSYNC_RSH={1})" -f $rsync, $env:RSYNC_RSH) }
& $py (Join-Path $toolsDir 'convert.py') --list

# ------------------------------------------------------------------ 1/2. 同步

if (-not $NoSync) {
    $remote = "$User@${RobotHost}:$RemoteRoot"
    $dest   = "./$DataDir/"

    # rsync 只创建目的地的最后一级；父目录不存在会直接报 code 11。
    $null = New-Item -ItemType Directory -Force -Path $DataDir

    $syncHint = @'
排查顺序：
      code 11  目的地父目录不存在（脚本已建，正常不会碰到）
      code 12  连接被断开。A 的 sshd 跑在容器里，容器没起或瞬时重启都会这样。
               先探一下：ssh -o BatchMode=yes unitree@<A 的 IP> exit
               回 "Permission denied (publickey,password)" 说明 sshd 活着，重跑即可
      code 23  部分文件没传成，重跑一趟
'@

    Write-Step '1. 增量拉取（README §2）'
    Write-Host "  源    $remote"
    Write-Host "  目的  $dest"
    Write-Host '  密码由 ssh 自己问，脚本不经手' -ForegroundColor DarkGray
    & $rsync -av --partial --exclude=DONE --exclude='*.ts.txt' $remote $dest
    if ($LASTEXITCODE -ne 0) { Stop-With "rsync 增量拉取失败（code $LASTEXITCODE）" $syncHint }

    Write-Step '2. 收尾那一趟 --checksum（README §2）'
    Write-Host '  逐文件比内容而不是比「大小+时间」，并把 DONE 拉过来' -ForegroundColor DarkGray
    & $rsync -av --checksum $remote $dest
    if ($LASTEXITCODE -ne 0) { Stop-With "rsync 收尾失败（code $LASTEXITCODE）" $syncHint }
}

# -------------------------------------------------------------- 3. 校验 session

Write-Step '3. sha256 核对（README §2）'

if (-not (Test-Path $DataDir)) { Stop-With "$DataDir 不存在" '先跑一次同步（去掉 -NoSync）' }
$candidates = @(Get-ChildItem -Directory $DataDir -ErrorAction SilentlyContinue)
if ($Session) { $candidates = @($candidates | Where-Object { $Session -contains $_.Name }) }
if ($candidates.Count -eq 0) { Stop-With "$DataDir 下没有匹配的 session" '检查 -Session 参数，或先同步' }

$sealed = @()
foreach ($s in $candidates) {
    if (Test-Path (Join-Path $s.FullName 'DONE')) {
        $sealed += $s
    } else {
        # 没封口的 session 缺 pts.bin / schema.json / DONE，读不了（README §2）
        Write-Host "  跳过 $($s.Name)：没有 DONE，未正常收尾" -ForegroundColor Yellow
    }
}
if ($sealed.Count -eq 0) {
    Stop-With '没有已封口的 session' '等 A 侧收尾后再跑一趟本脚本（DONE 是在 --checksum 那趟才拉过来的）'
}

foreach ($s in $sealed) {
    Write-Host ''
    Write-Host "--- $($s.Name) ---" -ForegroundColor White
    $r = Invoke-Native $py @((Join-Path $toolsDir 'inspect_session.py'), $s.FullName, '--verify')
    if ($r.ExitCode -ne 0) { Stop-With "inspect_session.py 失败：$($s.Name)（code $($r.ExitCode)）" }
    if (-not ($r.Lines -match '全部一致')) {
        Stop-With "$($s.Name)：sha256 核对没有输出「全部一致」" `
                  '传输没搞完。重跑一趟本脚本（--checksum 会把错漏补上），README §5'
    }
}

if ($SyncOnly) {
    Write-Step '完成（-SyncOnly，未转换）'
    $sealed | ForEach-Object { Write-Host "  $($_.FullName)" }
    exit 0
}

# ------------------------------------------------------------------ 4. 转换

Write-Step '4. 转成 YB（README §4）'

$produced = @()
foreach ($s in $sealed) {
    $outDir = Join-Path $OutRoot $s.Name
    if ((Test-Path (Join-Path $outDir 'episodes_all.json')) -and -not $Force) {
        Write-Host "  跳过 $($s.Name)：已经转换过（要重转加 -Force）" -ForegroundColor Yellow
        $produced += $outDir
        continue
    }

    Write-Host ''
    Write-Host "--- $($s.Name) -> $outDir ---" -ForegroundColor White
    $r = Invoke-Native $py @(
        (Join-Path $toolsDir 'convert.py'), $s.FullName,
        '--to', 'yb', '-o', $outDir,
        '--urdf', $urdf, '--calibration', $calib,
        '--video-height', "$VideoHeight", '--hz', "$Hz"
    )
    if ($r.ExitCode -ne 0) {
        Stop-With "转换失败：$($s.Name)（code $($r.ExitCode)）" `
                  '缺依赖看 README §5 那张表。别为了绕过报错去掉 --calibration —— 腕相机的 link 就在那份文件里'
    }

    # 有效率那行：某一路整段没录上时形状完全正常、内容全是 NaN，只有这行看得出来。
    # 用 (?<!\d)0% 才能把 100% 里的 "0%" 排除掉。
    foreach ($line in ($r.Lines | Where-Object { $_ -match '有效率' })) {
        if ($line -match '(?<!\d)0%') {
            Stop-With "有效率出现 0%：$($line.Trim())" `
                      '那一路整段没录上，别拿这份数据训练（README §5）。回 A 侧查话题发布者'
        }
    }
    $produced += $outDir
}

# -------------------------------------------------------------- 5. 产物自检

Write-Step '5. 产物自检'

$checker = Join-Path $env:TEMP 'yb_postcheck.py'
$checkerSource = @'
import json
import subprocess
import sys
from pathlib import Path

import h5py
import numpy as np

out = Path(sys.argv[1])
ffprobe = sys.argv[2]
problems = []

eps = json.loads((out / 'episodes_all.json').read_text(encoding='utf-8'))['episodes']
print('  episode %d 条' % len(eps))

for ep in eps:
    name = ep['episode_name']
    print('  %s' % name)
    with h5py.File(out / 'data' / (name + '.h5'), 'r') as f:
        n = int(f['timestamp'].shape[0])
        cams = [c.decode() if isinstance(c, bytes) else c
                for c in f['meta/camera_space/names'][:]]
        ends = [c.decode() if isinstance(c, bytes) else c
                for c in f['meta/end_space/names'][:]]
        ext = f['state/camera_space/extrinsic'][:]
        pose = f['state/end_space/pose'][:]

    declared = ep['end_frame'] - ep['start_frame'] + 1
    if declared != n:
        problems.append('%s: 索引声明 %d 帧，h5 是 %d 行' % (name, declared, n))

    # h5 第 k 行 == mp4 第 k 帧，是 YB 唯一必须记住的一条，所以逐帧解码硬核对
    for cam in cams:
        path = out / ('video_' + cam) / (name + '.mp4')
        if not path.is_file():
            problems.append('%s: 缺 %s 的 mp4' % (name, cam))
            continue
        r = subprocess.run([ffprobe, '-v', 'error', '-select_streams', 'v:0',
                            '-count_frames', '-show_entries', 'stream=nb_read_frames',
                            '-of', 'csv=p=0', str(path)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            k = int(r.stdout.decode().strip())
        except ValueError:
            problems.append('%s: ffprobe 数不出 %s 的帧数' % (name, cam))
            continue
        print('    %-12s mp4 %5d 帧   h5 %5d 行   %s'
              % (cam, k, n, 'ok' if k == n else '不一致'))
        if k != n:
            problems.append('%s: %s 是 %d 帧，h5 是 %d 行' % (name, cam, k, n))

    # YB 规范 §5.2 的自查办法一：腕相机拧在夹爪上，到同侧末端的距离必须全程是常数。
    # 这条一旦不成立，多半是外参方向反了（认公式不认名字）。
    # 相机名要和 meta/camera_space/names 逐字一致，写错了下面一条也不查、还报"通过"。
    for cam, arm in (('leftcam', 0), ('rightcam', 1)):
        if cam not in cams:
            problems.append('%s: h5 里没有 %s 这一路，自检的外参不变量没跑' % (name, cam))
            continue
        if arm >= pose.shape[1]:
            continue
        ci = cams.index(cam)
        d = np.linalg.norm(ext[:, ci, :, 3] - pose[:, arm, :3], axis=1)
        d = d[np.isfinite(d)]
        if d.size == 0:
            problems.append('%s: %s 的外参整段是 NaN' % (name, cam))
            continue
        spread = float(np.ptp(d))
        print('    %-12s -> %-16s %.4f cm   极差 %.6f mm'
              % (cam, ends[arm], d.mean() * 100, spread * 1000))
        if spread > 1e-6:
            problems.append('%s: %s 到同侧末端的距离不是常数（极差 %.4f mm），'
                            '外参方向可能反了（YB 规范 §5.2）'
                            % (name, cam, spread * 1000))

if problems:
    print('')
    print('产物自检未通过:')
    for p in problems:
        print('  - ' + p)
    sys.exit(1)

print('')
print('产物自检通过')
'@
Set-Content -Path $checker -Value $checkerSource -Encoding UTF8

foreach ($outDir in $produced) {
    Write-Host ''
    Write-Host "--- $outDir ---" -ForegroundColor White
    $r = Invoke-Native $py @($checker, (Resolve-Path $outDir).Path, $ffprobe)
    if ($r.ExitCode -ne 0) { Stop-With "产物自检未通过：$outDir" '上面列出了具体哪一项对不上' }
}

# ---------------------------------------------------------------------- 收尾

Write-Step '全部完成'
foreach ($outDir in $produced) {
    Write-Host "  $((Resolve-Path $outDir).Path)" -ForegroundColor Green
}
Write-Host ''
Write-Host '把数据交给别人时，连 tools/format/YB/README.md 那份规范一起给。' -ForegroundColor DarkGray

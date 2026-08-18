"""把部署侧的观测装配与训练侧逐位对拍。

观测错一位，策略在实机上就是输出垃圾，而且**看不出来**——网络照样给你 29 个有限的数。
所以这里不测「能跑通」，只测「和训练时算出来的数字一模一样」。

**必须在 GRU 版本的训练代码上跑**。训练仓库主线已经把 `G1-Gloria-MotionTracking`
换成了 RGMT 架构（多组 token 观测），跟本包的 866 维单组不是一回事，直接跑只会得到
一个毫无意义的维度错误。先拉一个只读的 worktree（`53c97e6` 是 GRU 版）::

    cd ~/g1_lower_rl && git worktree add /tmp/gmt_gru 53c97e6
    cd /tmp/gmt_gru && PYTHONPATH=.:<本包路径> MUJOCO_GL=egl \
      python <本包路径>/test/cross_check_obs.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch


def main() -> int:
    import g1_lower_rl.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import load_env_cfg

    from g1_gmt_tracking.gmt_runtime import GmtPolicy, load_policy
    from g1_gmt_tracking.motion_library import MotionClip, resolve_indices

    pkg = Path(__file__).resolve().parent.parent
    onnx = pkg / 'config' / 'policy.onnx'
    clip_path = pkg / 'config' / 'motions' / 'proc_stand.npz'

    # 训练环境只放这一条动作，好让两侧的帧号能对上。
    solo = Path('/tmp/gmt_cross_check_motions')
    solo.mkdir(exist_ok=True)
    for old in solo.glob('*.npz'):
        old.unlink()
    (solo / clip_path.name).write_bytes(clip_path.read_bytes())

    cfg = load_env_cfg('G1-Gloria-MotionTracking', play=True)
    cfg.commands['motion'].motion_dir = str(solo)
    cfg.scene.num_envs = 1
    # 关掉复位扰动，两侧才是同一个初始状态。
    cfg.commands['motion'].pose_range = {}
    cfg.commands['motion'].velocity_range = {}
    cfg.commands['motion'].joint_position_range = (0.0, 0.0)
    # encoder_bias 会给**观测**里的关节角注入 ±0.01 的仿真偏置，真机上没有这一项。
    # 不关掉的话 joint_pos 会稳定差在 1e-2 量级，掩盖真正的布局错误。
    cfg.events.pop('encoder_bias', None)
    env = ManagerBasedRlEnv(cfg=cfg, device='cpu')

    session, spec = load_policy(str(onnx))
    idx = resolve_indices(spec.all_body_names, spec.anchor_body_name,
                          spec.root_body_name, spec.obs_joint_names,
                          spec.action_joint_names, spec.control_dt)
    clip = MotionClip(clip_path, **idx)
    policy = GmtPolicy(session, spec,
                       target_lower=np.full(spec.action_dim, -10.0),
                       target_upper=np.full(spec.action_dim, +10.0))

    robot = env.scene['robot']
    torso = list(robot.body_names).index(spec.anchor_body_name)

    obs_dict, _ = env.reset()
    policy.reset()
    clip.align_yaw(robot.data.body_link_quat_w[0, torso].cpu().numpy().astype(np.float64))

    worst = 0.0
    for step in range(30):
        cmd = env.command_manager.get_term('motion')
        policy.frame = int(cmd.phase[0].item())

        mine = policy.observe(
            clip=clip,
            joint_pos=robot.data.joint_pos[0].cpu().numpy().astype(np.float64),
            joint_vel=robot.data.joint_vel[0].cpu().numpy().astype(np.float64),
            ang_vel=np.asarray(
                env.scene.sensors['robot/imu_ang_vel'].data[0].cpu().numpy(),
                dtype=np.float64),
            anchor_quat=robot.data.body_link_quat_w[0, torso].cpu().numpy().astype(np.float64),
        )
        theirs = obs_dict['actor'][0].cpu().numpy().astype(np.float64)

        # 历史项前几拍两侧的填充方式不同（训练侧复位时用首帧填满），只比无历史的部分
        # 和窗口填满之后的整条向量。
        limit = spec.obs_dim if step >= spec.history_length else 396
        diff = float(np.max(np.abs(mine[:limit] - theirs[:limit])))
        worst = max(worst, diff)
        if step == 0 or diff > 1e-4:
            _report(mine, theirs, spec, step)

        action = torch.zeros(1, spec.action_dim)
        obs_dict, _, _, _, _ = env.step(action)
        policy._push(policy._hist_actions, np.zeros(spec.action_dim))

    print(f'\n30 拍内最大逐元素误差 = {worst:.3e}')
    ok = worst < 1e-4
    print('对拍通过 ✅' if ok else '对拍失败 ❌ —— 观测布局与训练侧不一致，禁止上机')
    return 0 if ok else 1


def _report(mine: np.ndarray, theirs: np.ndarray, spec, step: int) -> None:
    k = len(spec.lookahead_steps) * 39
    n_j, h = len(spec.obs_joint_names), spec.history_length
    bounds = [('command', 0, k), ('anchor_ori_b', k, k + 6),
              ('base_ang_vel', k + 6, k + 6 + 3 * h),
              ('joint_pos', k + 6 + 3 * h, k + 6 + 3 * h + n_j * h),
              ('joint_vel', k + 6 + 3 * h + n_j * h, k + 6 + 3 * h + 2 * n_j * h),
              ('actions', k + 6 + 3 * h + 2 * n_j * h, spec.obs_dim)]
    print(f'--- step {step} 分项最大误差')
    for name, lo, hi in bounds:
        print(f'    {name:<16} {np.max(np.abs(mine[lo:hi] - theirs[lo:hi])):.3e}')


if __name__ == '__main__':
    sys.exit(main())

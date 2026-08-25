"""外参标定：求腕相机相对手臂 link 的固定变换 T_link<-cam。

两条互相独立的路子，都算一遍再对比：

1. **参考相机法**（用户流程）。头部相机外参认为准，用它把板定位到 torso 下：
   ``T_base_board = T_base_ref @ T_ref_board``，再和腕相机看到的板一起消掉板：
   ``T_link_cam = inv(T_base_link) @ T_base_board @ inv(T_cam_board)``
   一张就能解，但结果里带着头部外参的全部误差。

2. **手眼标定 AX=XB**（``cv2.calibrateHandEye``）。只用腕相机自己看板 + 手臂 FK，
   **完全不碰头部外参**。需要多个姿态且旋转轴不能都平行。

两者差得多，说明头部外参（URDF 里的 d435_joint）不准 —— 这正是做交叉验证的意义。
"""

from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares

from camera_calibration import transforms

MIN_HANDEYE_POSES = 3
# 板固定时，各组反推出的板位姿差异就是头部 PnP 的噪声，实测在几毫米 / 不到 1°。
# 超过这个量级就只能是板被挪了。
BOARD_FIXED_TRANS_MM = 20.0
BOARD_FIXED_ANGLE_DEG = 3.0
# 条件数爆掉说明这批姿态撑不起这么多未知量，解出来的量别当真
WELL_POSED_CONDITION = 1e4
_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def board_in_base(sample: dict) -> np.ndarray:
    return np.asarray(sample['T_base_ref'], float) @ np.asarray(sample['T_ref_board'], float)


def board_stability(samples: list[dict]) -> dict:
    """头部每组测到的板位姿，换算到 base 下看重不重合。

    这是判断能不能跑 AX=XB 的关键：它的推导第一步就用了「板在 base 下不动」，
    板被挪过的话算出来的 X 是错的，而且错得很安静。
    """
    spread = transforms.spread([board_in_base(s) for s in samples])
    spread['fixed'] = bool(spread['trans_max_mm'] <= BOARD_FIXED_TRANS_MM
                           and spread['angle_max_deg'] <= BOARD_FIXED_ANGLE_DEG)
    return spread


def solve_from_reference(samples: list[dict]) -> dict:
    """samples 每项要有 T_base_ref / T_ref_board / T_base_link / T_cam_board"""
    if not samples:
        return {'ok': False, 'reason': '一组样本都没有'}
    estimates, boards = [], []
    for sample in samples:
        base_board = board_in_base(sample)
        boards.append(base_board)
        estimates.append(
            transforms.invert(sample['T_base_link']) @ base_board
            @ transforms.invert(sample['T_cam_board']))
    mean = transforms.average_transforms(estimates)
    return {
        'ok': True,
        'method': 'reference',
        'transform': transforms.matrix_to_dict(mean),
        'matrix': np.asarray(mean).tolist(),
        'spread': transforms.spread(estimates, mean),
        # 板是不动的，所以头部算出来的板位姿在各样本间应该完全一致。
        # 这一项散了就是头部 PnP 本身在抖，跟手臂无关。
        'board_spread': transforms.spread(boards),
        'consistency': board_consistency(samples, mean),
    }


def solve_handeye(samples: list[dict], methods=None) -> dict:
    """只用 T_base_link 和 T_cam_board，不依赖任何参考相机。"""
    if len(samples) < MIN_HANDEYE_POSES:
        return {'ok': False,
                'reason': f'AX=XB 至少要 {MIN_HANDEYE_POSES} 个姿态，现在只有 {len(samples)} 个'}
    gripper = [np.asarray(s['T_base_link'], float) for s in samples]
    target = [np.asarray(s['T_cam_board'], float) for s in samples]
    if _rotation_axes_parallel(gripper):
        return {'ok': False,
                'reason': 'AX=XB 无解：各姿态之间的旋转轴几乎平行，把手腕换几个方向再采'}

    results = {}
    for name in (methods or _METHODS):
        rotation, translation = cv2.calibrateHandEye(
            [m[:3, :3] for m in gripper], [m[:3, 3] for m in gripper],
            [m[:3, :3] for m in target], [m[:3, 3] for m in target],
            method=_METHODS[name])
        matrix = transforms.rt_to_matrix(rotation, np.asarray(translation).reshape(3))
        results[name] = {
            'transform': transforms.matrix_to_dict(matrix),
            'matrix': matrix.tolist(),
            'consistency': board_consistency(samples, matrix),
        }
    best = min(results, key=lambda k: results[k]['consistency']['trans_rms_mm'])
    return {
        'ok': True, 'method': 'handeye', 'best': best,
        'transform': results[best]['transform'],
        'matrix': results[best]['matrix'],
        'consistency': results[best]['consistency'],
        'per_method': results,
        'method_spread': transforms.spread(
            [np.asarray(v['matrix'], float) for v in results.values()]),
    }


def board_consistency(samples: list[dict], transform, correction=None) -> dict:
    """腕相机反推的板位姿 vs 头部实测的板位姿，**逐组配对**比。

    不能用「各组反推值之间的离散度」：板挪过的话它们本来就不同，那个数量的是
    板移动了多少而不是外参准不准。实机上踩过：最小二乘残差 3.2 mm，那个错指标却报 73 mm。

    correction 给了就拿修正后的头部外参比，否则用 TF 原值。
    """
    correction = np.eye(4) if correction is None else np.asarray(correction, float)
    transform = np.asarray(transform, float)
    deltas = []
    for sample in samples:
        head = (np.asarray(sample['T_base_ref'], float) @ correction
                @ np.asarray(sample['T_ref_board'], float))
        wrist = (np.asarray(sample['T_base_link'], float) @ transform
                 @ np.asarray(sample['T_cam_board'], float))
        deltas.append(transforms.transform_delta(head, wrist))
    return transforms.deviation(deltas)


def compare(a, b) -> dict:
    angle, distance = transforms.transform_delta(a, b)
    return {'angle_deg': round(angle, 4), 'trans_mm': round(distance * 1e3, 3)}


def _correction(matrix) -> dict:
    """头部修正量：既给人看的角度/距离，也给能再拼回矩阵的四元数"""
    return {**compare(np.eye(4), matrix), **transforms.matrix_to_dict(matrix)}


def _fit(residuals, size) -> tuple[np.ndarray, dict]:
    """跑一次 LM，返回最优参数和一组可信度指标"""
    result = least_squares(residuals, np.zeros(size), method='lm', xtol=1e-12)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float('inf')
    return result.x, {
        'residual_mm_before': _rms_mm(residuals(np.zeros(size))),
        'residual_mm': _rms_mm(result.fun),
        'condition': round(condition, 1),
        'well_posed': bool(condition < WELL_POSED_CONDITION),
    }


def _rms_mm(values) -> float:
    return round(float(np.sqrt(np.mean(np.asarray(values) ** 2))) * 1e3, 3)


def _residual_rows(transform, correction, samples, lever):
    rows = []
    for sample in samples:
        head = (np.asarray(sample['T_base_ref'], float) @ correction
                @ np.asarray(sample['T_ref_board'], float))
        wrist = (np.asarray(sample['T_base_link'], float) @ transform
                 @ np.asarray(sample['T_cam_board'], float))
        delta = transforms.log_delta(transforms.invert(head) @ wrist)
        rows.append(np.concatenate([delta[:3] * lever, delta[3:]]))
    return rows


def solve_all(groups: dict, lever: float = 0.5) -> dict:
    """三个相机一起解：头部外参偏差 ΔH 是**共享**未知量，每只手臂各有自己的 X。

    比逐个相机单独解好在三点：
    1. ΔH 只有一个答案。分开解会得到两个不一样的值（实机上是 2.44° 和 2.29°），
       没有哪个更对，只是各自被自己那点噪声带偏。
    2. 两只手臂的运动一起激励 ΔH，条件数好得多 —— 单臂转轴张不开时往往还能救回来。
    3. 只有一只手看得见板的那些组也能给 ΔH 出力，间接帮到另一只手。

    未知量 6(ΔH) + 6×相机数；每组每相机给 6 个方程。
    """
    groups = {name: rows for name, rows in groups.items() if rows}
    if not groups:
        return {'ok': False, 'reason': '一组样本都没有'}
    names = sorted(groups)

    starts = {}
    for name in names:
        first = solve_from_reference(groups[name])
        if not first.get('ok'):
            return {'ok': False, 'reason': f'{name}: {first.get("reason")}'}
        starts[name] = np.asarray(first['matrix'], float)

    def unpack(params):
        correction = transforms.perturb(np.eye(4), params[:6])
        return correction, {name: transforms.perturb(starts[name],
                                                     params[6 + 6 * i:12 + 6 * i])
                            for i, name in enumerate(names)}

    def residuals(params):
        correction, matrices = unpack(params)
        rows = []
        for name in names:
            rows.extend(_residual_rows(matrices[name], correction,
                                       groups[name], lever))
        return np.concatenate(rows)

    params, fit = _fit(residuals, 6 + 6 * len(names))
    correction, matrices = unpack(params)
    # T_base_ref 是固定安装，各组应当一样，取第一组即可
    nominal = np.asarray(groups[names[0]][0]['T_base_ref'], float)

    return {
        'ok': True, 'method': 'all',
        'cameras': {name: {
            'transform': transforms.matrix_to_dict(matrices[name]),
            'matrix': matrices[name].tolist(),
            'samples': len(groups[name]),
            'consistency': board_consistency(groups[name], matrices[name], correction),
        } for name in names},
        'reference_correction': _correction(correction),
        'reference_absolute': transforms.matrix_to_dict(nominal @ correction),
        'reference_nominal': transforms.matrix_to_dict(nominal),
        **fit,
    }


def solve_joint(samples: list[dict], lever: float = 0.5,
                refine_reference: bool = True) -> dict:
    """把所有组一起做最小二乘，同时估腕相机外参 X 和头部外参的修正量 ΔH。

    **板每组都挪也没关系**：头部每组各测一次板位姿，板的位置不进未知量。
    这是 AX=XB 做不到的：那条路不看头部，板一动每多一组就多 6 个未知量、
    也只多 6 个方程，永远欠定。

    **refine_reference=True 时 X 不再依赖头部外参准不准。** 令
    ``C_i = T_base_ref⁻¹ T_base_link_i``、``D_i = Mʷ_i (Mʰ_i)⁻¹``，约束就是
    ``ΔH = C_i X D_i`` 对每组都成立；两组相除后 ``T_base_ref`` 被约掉，剩下
    ``(T_base_link_j⁻¹ T_base_link_i) X = X (…)`` —— 又一个不含头部外参的 AX=XB。
    实测：头部偏 10° 时参考相机法偏 9.0°/54mm，而这里仍然精确。
    代价是继承了 AX=XB 的那个前提：**各姿态之间的转轴不能都平行**，否则 12 个
    未知量撑不起来（看返回里的 ``well_posed``）。

    **只估 X 的话，它和「逐组解再取平均」几乎等价**（实测差不到 2%）：
    残差写在位姿空间、每组等权，数学上就是在求均值。别拿它当“精度更高”卖。
    （要在精度上真的趁过平均，得把残差写成像素重投影，那要把角点也存进 meta。）

    lever 是把旋转残差折算成米的力臂（板到相机的典型距离），否则 rad 和 m 混在
    一起，量纲大的那个会主导。
    """
    if len(samples) < 2 and refine_reference:
        return {'ok': False, 'reason': '同时估头部修正量至少要 2 组'}
    if not samples:
        return {'ok': False, 'reason': '一组样本都没有'}

    start = solve_from_reference(samples)
    if not start.get('ok'):
        return start
    initial = np.asarray(start['matrix'], float)
    size = 12 if refine_reference else 6

    def unpack(params):
        return (transforms.perturb(initial, params[:6]),
                transforms.perturb(np.eye(4), params[6:12]) if refine_reference
                else np.eye(4))

    def residuals(params):
        transform, correction = unpack(params)
        return np.concatenate(_residual_rows(transform, correction, samples, lever))

    params, fit = _fit(residuals, size)
    matrix, correction = unpack(params)

    out = {
        'ok': True, 'method': 'joint',
        'transform': transforms.matrix_to_dict(matrix),
        'matrix': matrix.tolist(),
        'samples': len(samples),
        'consistency': board_consistency(samples, matrix, correction),
        **fit,
    }
    if refine_reference:
        out['reference_correction'] = _correction(correction)
    return out


def reference_bias(samples: list[dict], transform) -> dict:
    """已知腕相机外参时，反推头部外参该是多少。

    ``T_base_ref = T_base_link @ X @ T_cam_board @ inv(T_ref_board)``。
    把它和 TF 给的头部外参一比，就知道 URDF 里的 d435_joint 偏了多少、往哪偏。
    """
    transform = np.asarray(transform, float)
    implied = [np.asarray(s['T_base_link'], float) @ transform
               @ np.asarray(s['T_cam_board'], float)
               @ transforms.invert(s['T_ref_board']) for s in samples]
    mean = transforms.average_transforms(implied)
    current = transforms.average_transforms(
        [np.asarray(s['T_base_ref'], float) for s in samples])
    return {
        'implied': transforms.matrix_to_dict(mean),
        'current': transforms.matrix_to_dict(current),
        'delta': compare(current, mean),
        'spread': transforms.spread(implied, mean),
    }


def _rotation_axes_parallel(poses, threshold_deg: float = 5.0) -> bool:
    """相邻姿态之间的相对旋转轴张不开时，AX=XB 的旋转部分是欠定的"""
    axes = []
    for index in range(1, len(poses)):
        delta = transforms.invert(poses[index - 1]) @ poses[index]
        rvec = cv2.Rodrigues(delta[:3, :3])[0].reshape(3)
        angle = float(np.linalg.norm(rvec))
        if angle > np.deg2rad(threshold_deg):
            axes.append(rvec / angle)
    if len(axes) < 2:
        return True
    axes = np.stack(axes)
    return bool(np.linalg.matrix_rank(axes, tol=0.1) < 2)

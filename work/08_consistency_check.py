"""投影与反投影的一致性检查

把「三维点 -> 像素 -> 三维点」和「像素 + 深度 -> 三维点 -> 像素」
两条链走通，并搞清楚这个检查能证明什么、不能证明什么。

本脚本不引入新的几何模型，用的都是前几个任务里已经实现并验证过的函数：
    work/02_project_points.py       project_points
    work/03_backproject.py          backproject_pixels
    work/05_depth_to_pointcloud.py  load_frame / depth_to_points / INTRINSIC
"""

import importlib.util
from pathlib import Path

import numpy as np


def load_module(module_name, filename):
    """按文件路径加载同目录下的模块

    本目录的文件名以数字开头，不能用 import 语句直接导入，
    因此用 importlib 按路径加载。样板代码。
    """
    path = Path(__file__).resolve().parent / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def distance_to_axial_depth(distance, uv, K):
    """把「到光心的距离」换算成「沿光轴的 Z」

    Parameters
    ----------
    distance : ndarray, shape (N,)
        像素到相机光心的直线距离
    uv : ndarray, shape (N, 2)
        像素坐标 (u, v)
    K : ndarray, shape (3, 3)
        内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

    Returns
    -------
    z : ndarray, shape (N,)
        沿光轴方向的深度，可直接交给 backproject_pixels 使用
    """
    # TODO 1
    u, v = uv[:, 0], uv[:, 1]
    fx, cx = K[0, 0], K[0, 2]
    fy, cy = K[1, 1], K[1, 2]
    z = distance / np.sqrt((u - cx) ** 2 / fx**2 + (v - cy) ** 2 / fy**2 + 1)
    return z
    # raise NotImplementedError("distance_to_axial_depth 还没实现")


def main():
    """跑一致性检查，并给出「这个检查证明了什么」的结论"""
    m02 = load_module("m02", "02_project_points.py")
    m03 = load_module("m03", "03_backproject.py")
    m05 = load_module("m05", "05_depth_to_pointcloud.py")

    K = np.array(
        [
            [525.0, 0.0, 319.5],
            [0.0, 525.0, 239.5],
            [0.0, 0.0, 1.0],
        ]
    )

    # 下面 TODO 2 到 8 会用到 m02、m03、m05、K，
    # 在补完之前 ruff 会报 F841 未使用变量，属预期。

    # TODO 2：合成点走一遍 3D -> 像素 -> 3D
    test_points = np.array(
        [
            [0, 0, 2],
            [0.01, 0.01, 2],
            [0.6, 0.45, 2],
            [2.0, 0, 1],
            [0.05, 0.05, 0.3],
            [1.0, 0.5, 8.0],
        ]
    )
    uv, valid = m02.project_points(test_points, K)
    assert valid.all(), "valid 有无效点"
    back, valid_back = m03.backproject_pixels(uv, test_points[:, 2], K)
    assert valid_back.all(), "valid_back 有无效点"
    errors = np.abs(back - test_points).max(axis=1)
    assert errors.max() < 1e-12
    print(errors)

    # TODO 3：像素加深度走一遍 像素 -> 3D -> 像素
    uv_test = np.array(
        [
            [0.0, 0.0],
            [639.0, 479.0],
            [320.0, 240.0],
            [700.0, 240.0],
            [-50.0, 100.0],
            [100.0, 300.0],
        ]
    )
    depths_test = np.array([5.0, 0.8, 2.0, 1.5, 3.0, 1.234])
    pts_test, valid_test = m03.backproject_pixels(uv_test, depths_test, K)
    uv_rt, _ = m02.project_points(pts_test, K)
    assert np.array_equal(pts_test[:, 2], depths_test), "反投影没有原样保留深度"
    pixel_errors = np.abs(uv_rt - uv_test).max(axis=1)
    assert pixel_errors.max() < 1e-9, f"像素未恢复: {pixel_errors}"
    print(pixel_errors)

    # TODO 4：真实点云上重复 TODO 2
    color, depth = m05.load_frame(m05.DATA_DIR, 0)
    points, valid_px = m05.depth_to_points(depth, m05.INTRINSIC, m05.DEPTH_SCALE)
    # assert valid_px.all(), "valid_px 有无效点"
    uv, valid_proj = m02.project_points(points, K)
    assert valid_proj.all(), "valid_proj 有无效点"
    back, valid_back = m03.backproject_pixels(uv, points[:, 2], K)
    assert valid_back.all(), "valid_back 有无效点"
    assert np.array_equal(back[:, 2], points[:, 2]), "深度未原样保留"
    errors_true = np.abs(back - points).max(axis=1)
    assert errors_true.max() < 1e-9, (
        f"往返误差过大，最差点下标 {np.argmax(errors_true)}，"
        f"误差 {errors_true.max():.3e}"
    )
    print(errors_true)

    # ---- 检查 5：量化误差 ----
    # 真实点云的像素本来就是整数（它是从整数像素反投影出来的），
    # 对它取整是空操作，测不出任何东西。这里另生成一批随机三维点，
    # 它们的投影像素是任意小数，取整的效果才会显现。
    rng = np.random.default_rng(0)
    quant_points = np.column_stack(
        [
            rng.uniform(-1.0, 1.0, 20000),
            rng.uniform(-0.6, 0.6, 20000),
            rng.uniform(0.9, 2.8, 20000),
        ]
    )
    uv_q, valid_q = m02.project_points(quant_points, K)
    assert valid_q.all(), "量化测试的随机点里出现了无效点"

    uv_q_round = np.round(uv_q)
    pixel_shift = np.abs(uv_q - uv_q_round)
    print(
        f"像素取整偏移: 最大 {pixel_shift.max():.3f} 像素, "
        f"中位数 {np.median(pixel_shift):.3f} 像素"
    )

    back_q, valid_back_q = m03.backproject_pixels(uv_q_round, quant_points[:, 2], K)
    assert valid_back_q.all(), "量化测试的反投影出现无效点"

    err_q = np.linalg.norm(back_q - quant_points, axis=1)
    print(
        f"三维误差: 中位数 {np.median(err_q) * 1000:.4f} mm, "
        f"均值 {err_q.mean() * 1000:.4f} mm, "
        f"最大 {err_q.max() * 1000:.4f} mm"
    )

    # 量化误差应与深度成正比。一个像素的横向跨度约等于 Z / fx，
    # 两轴像素误差在 [-0.5, 0.5] 上均匀分布，合成后的系数是 sqrt(1/6)。
    assert np.median(err_q) > 1e-4, "量化误差过小，取整可能没有生效"

    print()
    # 表头第一列的「深度区间」是 4 个汉字，等宽环境里占 8 列，
    # 与数据行的 "0.9~1.2m" 等宽，后面统一用 6 个空格分隔即可对齐
    print(f"{'深度区间'}{'':<6}{'点数':>8}{'实测 (mm)':>14}{'理论 (mm)':>13}")
    z_q = quant_points[:, 2]
    for lo, hi in [(0.9, 1.2), (1.2, 1.5), (1.5, 2.0), (2.0, 2.4), (2.4, 2.8)]:
        sel = (z_q >= lo) & (z_q < hi)
        if not sel.any():
            continue
        measured = np.median(err_q[sel]) * 1000
        theory = np.sqrt(1.0 / 6.0) * np.median(z_q[sel]) / K[0, 0] * 1000
        print(
            f"{lo:.1f}~{hi:.1f}m{'':<6}{int(sel.sum()):>8}"
            f"{measured:>12.4f}{theory:>13.4f}"
        )

    # ---- 检查 6：自洽性的边界，错误的内参照样闭合 ----
    # 投影与反投影共用同一组参数时，参数错误会在往返中互相抵消。
    K_wrong = K.copy()
    K_wrong[0, 2] += 100.0
    K_wrong[1, 2] += 100.0

    uv_wrong, valid_wrong = m02.project_points(points, K_wrong)
    assert valid_wrong.all(), "错误内参下出现无效投影"
    back_wrong, valid_back_wrong = m03.backproject_pixels(
        uv_wrong, points[:, 2], K_wrong
    )
    assert valid_back_wrong.all(), "错误内参下反投影出现无效点"

    err_same = np.abs(back_wrong - points).max()
    assert err_same < 1e-9, f"同一组参数往返竟不闭合: {err_same:.3e}"
    print()
    print("检查6  主点各偏 100 像素，两端用同一组错内参")
    print(f"       往返误差 {err_same:.3e}  -> 依然闭合")

    # 但只要让两端用不同的参数，错误立刻暴露
    back_mixed, _ = m03.backproject_pixels(uv_wrong, points[:, 2], K)
    err_mixed = np.abs(back_mixed - points).max()
    z_max = points[:, 2].max()
    print("       投影用错内参、反投影用对内参")
    print(f"       误差 {err_mixed:.3e} m  -> 暴露")
    print(f"       与理论值 100 * Zmax / fx = {100.0 * z_max / K[0, 0]:.3e} m 吻合")
    print("       结论：往返闭合只说明两个函数互逆，不说明内参正确")

    # ---- 检查 7：把「到光心的距离」当成深度 ----
    # 这是真实数据里常见的约定混淆：某些传感器存的是到光心的距离，
    # 而反投影公式要的是沿光轴的 Z。两者在画面中心接近，边缘差很多。
    radius = np.linalg.norm(points, axis=1)
    back_dist, valid_dist = m03.backproject_pixels(uv, radius, K)
    assert valid_dist.all(), "距离当深度时反投影出现无效点"

    err_dist = np.linalg.norm(back_dist - points, axis=1)
    print()
    print(
        f"检查7  把距离当深度: 最大误差 {err_dist.max():.3f} m, "
        f"中位数 {np.median(err_dist) * 1000:.1f} mm"
    )

    # 视场半径取归一化形式 (u-cx)/fx 与 (v-cy)/fy 的合成，
    # 它等于 tan(视场角)，与像素尺寸和焦距无关
    field_r = np.sqrt(
        ((uv[:, 0] - K[0, 2]) / K[0, 0]) ** 2 + ((uv[:, 1] - K[1, 2]) / K[1, 1]) ** 2
    )
    for lo, hi in [(0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 1.0)]:
        sel = (field_r >= lo) & (field_r < hi)
        if not sel.any():
            continue
        print(
            f"       视场半径 {lo:.1f}~{hi:.1f}: {int(sel.sum()):>7} 点, "
            f"误差中位数 {np.median(err_dist[sel]) * 1000:>7.1f} mm, "
            f"最大 {err_dist[sel].max() * 1000:>7.1f} mm"
        )
    print("       误差随视场半径增长，画面中心几乎没错、边缘错得最多")

    # ---- 检查 8：把距离纠正回 Z，往返应当恢复 ----
    z_fixed = distance_to_axial_depth(radius, uv, K)
    err_z = np.abs(z_fixed - points[:, 2]).max()
    back_fixed, valid_fixed = m03.backproject_pixels(uv, z_fixed, K)
    assert valid_fixed.all(), "纠正后反投影出现无效点"
    err_fixed = np.abs(back_fixed - points).max()
    assert err_fixed < 1e-9, f"纠正后往返仍未恢复: {err_fixed:.3e}"

    print()
    print("检查8  用 distance_to_axial_depth 把距离换回 Z")
    print(f"       换算出的 Z 与真实 Z 最大差 {err_z:.3e}")
    print(f"       往返误差恢复到 {err_fixed:.3e}  -> 说明机制搞清楚了")
    print()
    print("=" * 68)
    print("总结：前四项检查通过，说明投影与反投影互为逆运算、实现没有公式错误；")
    print("     但它们【不能】证明内参标定正确，也不能证明深度的物理含义正确。")
    print("     要验证这些，需要往返之外的参照：已知答案、独立实现、或已知尺寸。")
    print("=" * 68)


if __name__ == "__main__":
    main()

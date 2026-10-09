"""像素坐标加深度到相机坐标系三维点的反投影，project_points 的逆运算。"""

import numpy as np


def backproject_pixels(uv, depths, K):
    """把像素坐标和深度反投影成相机坐标系中的点。

    Parameters
    ----------
    uv : ndarray, shape (N, 2)
        像素坐标 (u, v)，允许小数
    depths : ndarray, shape (N,)
        深度，单位米。约定为沿光轴的 Z（不是到光心的距离），
        <= 0 表示该像素无有效测量
    K : ndarray, shape (3, 3)
        针孔相机内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

    Returns
    -------
    points_cam : ndarray, shape (N, 3)
        相机坐标系下的点，单位米。三列依次为 X、Y、Z，
        深度无效的行填 nan
    valid : ndarray, shape (N,), dtype bool
        depths > 0 的掩码

    Notes
    -----
    Z = depth，X = (u - cx) * Z / fx，Y = (v - cy) * Z / fy

    """
    uv = np.asarray(uv, dtype=np.float64)
    depths = np.asarray(depths, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = uv[:, 0], uv[:, 1]
    z = depths

    # 反投影不含除法，valid 只用于标记哪些深度不可信。
    valid = z > 0

    points_cam = np.full((uv.shape[0], 3), np.nan)
    points_cam[valid] = np.column_stack(
        [(u[valid] - cx) * z[valid] / fx, (v[valid] - cy) * z[valid] / fy, z[valid]]
    )

    return points_cam, valid


if __name__ == "__main__":
    K = np.array(
        [
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )

    uv = np.array(
        [
            [370.0, 215.0],
            [320.0, 240.0],
            [370.0, 215.0],
        ]
    )
    depths = np.array([2.0, 2.0, 0.0])

    points_cam, valid = backproject_pixels(uv, depths, K)

    print("points_cam.shape:", points_cam.shape)
    print("points_cam:\n", points_cam)
    print("valid:", valid)

    # 文件名以数字开头，不能用 import 语句直接导入，按路径加载 02。
    import importlib.util
    from pathlib import Path

    _here = Path(__file__).resolve().parent
    _spec = importlib.util.spec_from_file_location(
        "project_points_module", _here / "02_project_points.py"
    )
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    project_points = _mod.project_points

    # 往返一致性检查：三维点 -> 投影 -> 反投影，应当回到原位。
    np.random.seed(42)
    N = 100
    points = np.random.uniform(-0.5, 0.5, size=(N, 2))
    points_z = np.random.uniform(1.0, 3.0, size=(N, 1))
    points = np.hstack([points, points_z])
    uv2, valid2 = project_points(points, K)
    points_back, valid_back = backproject_pixels(uv2, points[:, 2], K)

    # 不用 np.nanmax：它会跳过 nan，反投影失败的行会被静默忽略。
    # 先断言全部有效，再用普通 max()，一旦出现 nan 立即暴露。
    assert valid2.all(), "投影阶段出现无效点，先查这一批输入"
    assert valid_back.all(), "反投影阶段出现无效点，先查两个函数的搭配"
    max_abs_error = np.abs(points - points_back).max()

    print("max_abs_error:", max_abs_error)
    print("  (1e-15 量级 = 浮点精度；若是 0.1 这种量级，说明某一步公式有问题)")

    # 同一像素偏移在不同深度上各值多少米。
    uv1 = np.array([[370.0, 215.0]])
    depths1 = np.array([2.0])
    points1, _ = backproject_pixels(uv1, depths1, K)
    uv1_half_pixel = np.array([[370.5, 215.0]])
    points1_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths1, K)
    error1 = np.abs(points1_half_pixel - points1)
    print("Depth 2.0, half-pixel error in meters:", error1)
    depths2 = np.array([0.5])
    points2, _ = backproject_pixels(uv1, depths2, K)
    points2_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths2, K)
    error2 = np.abs(points2_half_pixel - points2)
    print("Depth 0.5, half-pixel error in meters:", error2)
    depths3 = np.array([5.0])
    points3, _ = backproject_pixels(uv1, depths3, K)
    points3_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths3, K)
    error3 = np.abs(points3_half_pixel - points3)
    print("Depth 5.0, half-pixel error in meters:", error3)

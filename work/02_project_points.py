"""相机坐标系三维点到像素坐标的针孔投影。"""

import numpy as np


def project_points(points_cam, K):
    """把相机坐标系中的点投影到像素平面。

    Parameters
    ----------
    points_cam : ndarray, shape (N, 3)
        相机坐标系下的点，单位米。三列依次为 X（向右）、Y（向下）、
        Z（沿光轴向前）
    K : ndarray, shape (3, 3)
        针孔相机内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

    Returns
    -------
    uv : ndarray, shape (N, 2)
        像素坐标，Z <= 0 的无效点整行填 nan
    valid : ndarray, shape (N,), dtype bool
        Z > 0 的掩码

    Notes
    -----
    u = fx * X / Z + cx，v = fy * Y / Z + cy，仅在 Z > 0 时成立

    """
    points_cam = np.asarray(points_cam, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    X, Y, Z = points_cam[:, 0], points_cam[:, 1], points_cam[:, 2]

    # Z <= 0 的点位于光心平面上或光心后方，不存在对应像点。
    valid = Z > 0

    # 先过滤再做除法，避免 Z 中的 0 触发 divide by zero 警告。
    uv = np.full((points_cam.shape[0], 2), np.nan)
    uv[valid] = np.column_stack(
        [fx * X[valid] / Z[valid] + cx, fy * Y[valid] / Z[valid] + cy]
    )

    return uv, valid


if __name__ == "__main__":
    K = np.array(
        [
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )

    points = np.array(
        [
            [0.2, -0.1, 2.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.1, 0.05, 0.0],
        ]
    )

    uv, valid = project_points(points, K)

    print("uv.shape:", uv.shape)
    print("uv:\n", uv)
    print("valid.shape:", valid.shape)
    print("valid:", valid)

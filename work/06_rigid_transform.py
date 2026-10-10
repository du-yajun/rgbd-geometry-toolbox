"""刚体变换：旋转加平移，把点从一个坐标系换到另一个坐标系"""

import math

import numpy as np
import open3d as o3d


def make_transform(rotation, translation):
    """由旋转矩阵和平移向量拼出 4x4 齐次变换矩阵

    Parameters
    ----------
    rotation : ndarray, shape (3, 3)
        旋转矩阵，作用在列向量左侧：p_rotated = rotation @ p
    translation : ndarray, shape (3,)
        平移向量，单位与点相同（本项目是米）

    Returns
    -------
    T : ndarray, shape (4, 4)
        齐次变换矩阵，形式为 [[R, t], [0, 0, 0, 1]]

    Notes
    -----
    本函数只负责组装，不检查 rotation 是否真是合法旋转矩阵。
    """
    T = np.zeros((4, 4))
    T[0:3, 0:3] = rotation
    T[0:3, 3] = translation
    T[3, 3] = 1
    return T


def transform_points(points, T):
    """把点集用齐次变换矩阵变换到另一个坐标系

    Parameters
    ----------
    points : ndarray, shape (N, 3)
        待变换的点，每行一个点
    T : ndarray, shape (4, 4)
        齐次变换矩阵。本实现假设最后一行是 [0, 0, 0, 1]（刚体变换），
        传入投影矩阵会得到错误结果

    Returns
    -------
    points_new : ndarray, shape (N, 3)
        变换后的点，是新数组，不修改输入

    Notes
    -----
    采用列向量约定：p_new = T @ p。
    点集是 (N, 3) 的行堆叠，因此写成 points @ R.T 而不是 R @ points。
    """
    R = T[:3, :3]
    t = T[:3, 3]
    return points @ R.T + t


def invert_transform(T):
    """求刚体变换的逆

    Parameters
    ----------
    T : ndarray, shape (4, 4)
        齐次变换矩阵

    Returns
    -------
    T_inv : ndarray, shape (4, 4)
        T 的逆矩阵

    Notes
    -----
    若 T = [[R, t], [0, 1]]，则 T^-1 = [[R^T, -R^T @ t], [0, 1]]。
    依据是旋转矩阵的正交性 R^T R = I，即 R^-1 = R^T，
    比调用 np.linalg.inv 做通用求逆更快、数值也更稳。
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.zeros((4, 4))
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    T_inv[3, 3] = 1
    return T_inv


def rotation_z(theta):
    """绕 Z 轴旋转 theta 弧度，返回 3x3 旋转矩阵

    Parameters
    ----------
    theta : float
        旋转角，单位弧度，逆时针为正（从 +Z 方向往下看）

    Returns
    -------
    R : ndarray, shape (3, 3)
    """
    R = np.array(
        [
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1],
        ]
    )
    return R


def main():
    """五项已知答案测试，逐项验证变换实现是否正确"""
    rng = np.random.default_rng(0)
    sample = rng.uniform(-1.0, 1.0, size=(20, 3))
    print(f"测试用点集: {sample.shape}")

    # ---- 测试 1：单位变换不改变点 ----
    T_identity = make_transform(np.eye(3), np.zeros(3))
    out = transform_points(sample, T_identity)
    assert np.allclose(out, sample), "单位变换应当不改变任何点"
    print(f"测试1 通过：单位变换最大偏移 {np.abs(out - sample).max():.3e}")

    # ---- 测试 2：已知平移产生预期结果 ----
    point = np.array([[0.5, 0.5, 0.5]])
    T = make_transform(np.eye(3), np.array([1.0, 2.0, 3.0]))
    moved = transform_points(point, T)
    assert np.allclose(moved, [[1.5, 2.5, 3.5]]), f"纯平移结果错误: {moved}"
    print(f"测试2 通过：平移最大偏移 {np.abs(moved - [[1.5, 2.5, 3.5]]).max():.3e}")

    # ---- 测试 3：绕 Z 轴 90 度，X 轴上的点应转到 Y 轴 ----
    # cos(pi/2) 在浮点下是 6.12e-17 而非 0，因此比较用 allclose
    T = make_transform(rotation_z(np.pi / 2), np.zeros(3))
    rotated = transform_points(np.array([[1.0, 0.0, 0.0]]), T)
    assert np.allclose(rotated, [[0.0, 1.0, 0.0]]), f"旋转结果错误: {rotated}"
    print(f"测试3 通过：旋转最大偏移 {np.abs(rotated - [[0.0, 1.0, 0.0]]).max():.3e}")

    # ---- 测试 4：旋转加平移，再做逆变换应回到原位 ----
    # t 的三个分量都不为 0，R^T 与 -R^T @ t 两部分才会都被检验到
    T = make_transform(rotation_z(np.pi / 5), np.array([1.0, 2.0, 3.0]))
    T_inv = invert_transform(T)

    # 4a 点级别，正方向
    back = transform_points(transform_points(sample, T), T_inv)
    error = np.abs(sample - back).max()
    assert error < 1e-12, f"往返误差过大: {error:.3e}"
    print(f"测试4a 通过：点往返最大误差 {error:.3e}")

    # 4b 点级别，反方向
    back2 = transform_points(transform_points(sample, T_inv), T)
    error2 = np.abs(sample - back2).max()
    assert error2 < 1e-12, f"反向往返误差过大: {error2:.3e}"
    print(f"测试4b 通过：反向往返最大误差 {error2:.3e}")

    # 4c 与 np.linalg.inv 比对，两条运算路径的差异应在浮点精度量级
    T_inv_ref = np.linalg.inv(T)
    diff = np.abs(T_inv - T_inv_ref).max()
    assert np.allclose(T_inv, T_inv_ref), f"与 np.linalg.inv 不一致: {diff:.3e}"
    print(f"测试4c 通过：与 np.linalg.inv 最大差异 {diff:.3e}")

    # 4d 不变量检查：T_inv @ T 必须是单位矩阵。
    # 这一条不依赖任何参考实现，能抓住「点级别看不出来」的错误，
    # 例如漏设 T_inv[3, 3] = 1 时，前三列坐标仍然正确，但矩阵乘法已经错了
    composed = T_inv @ T
    error4 = np.abs(composed - np.eye(4)).max()
    assert error4 < 1e-12, f"T_inv @ T 偏离单位矩阵: {error4:.3e}"
    print(f"测试4d 通过：T_inv @ T 与单位矩阵最大差异 {error4:.3e}")

    # ---- 测试 5：与 Open3D 交叉验证，确认列向量约定一致 ----
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(sample)
    pcd.transform(T)
    ref = np.asarray(pcd.points)

    mine = transform_points(sample, T)
    diff5 = np.abs(mine - ref).max()
    assert np.allclose(mine, ref), f"与 Open3D 不一致，最大差异 {diff5:.3e}"
    print(f"测试5 通过：与 Open3D 最大差异 {diff5:.3e}")


if __name__ == "__main__":
    main()

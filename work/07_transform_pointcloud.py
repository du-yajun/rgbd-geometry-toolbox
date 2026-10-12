"""把刚体变换用到真实点云上，验证变换保持几何性质"""

from pathlib import Path

import numpy as np
import open3d as o3d

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "redwood_rgbd"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs"
FRAME_INDEX = 0
DEPTH_SCALE = 1000.0

INTRINSIC = {
    "width": 640,
    "height": 480,
    "fx": 525.0,
    "fy": 525.0,
    "cx": 319.5,
    "cy": 239.5,
}


def load_frame(data_dir, index):
    """读出一帧彩色图与深度原始值，样板代码"""
    stem = f"{index:05d}"
    color = np.asarray(o3d.io.read_image(str(data_dir / "color" / f"{stem}.jpg")))
    depth = np.asarray(o3d.io.read_image(str(data_dir / "depth" / f"{stem}.png")))
    return color, depth


def pixel_grid(height, width):
    """生成像素坐标网格，与 work/05 中的实现相同"""
    row = np.arange(height).reshape(height, 1)
    col = np.arange(width).reshape(1, width)
    return np.meshgrid(col, row)


def depth_to_points(depth, intrinsic, depth_scale):
    """整张深度图反投影成点云，与 work/05 中的实现相同"""
    u, v = pixel_grid(depth.shape[0], depth.shape[1])
    u = u.flatten()
    v = v.flatten()
    z = depth.astype(np.float64).flatten() / depth_scale
    valid = depth > 0
    valid_flat = valid.flatten()
    X = (u[valid_flat] - intrinsic["cx"]) / intrinsic["fx"] * z[valid_flat]
    Y = (v[valid_flat] - intrinsic["cy"]) / intrinsic["fy"] * z[valid_flat]
    Z = z[valid_flat]
    return np.column_stack([X, Y, Z]), valid


def points_to_colors(color, valid):
    """取出与点一一对应的颜色，与 work/05 中的实现相同"""
    return color[valid] / 255


def make_transform(rotation, translation):
    """由旋转矩阵和平移向量拼出 4x4 齐次变换矩阵

    与 work/06_rigid_transform.py 中的实现相同，这里复制一份，
    是为了让本脚本可以独立运行，不必依赖按文件路径加载。
    """
    T = np.zeros((4, 4))
    T[0:3, 0:3] = rotation
    T[0:3, 3] = translation
    T[3, 3] = 1
    return T


def transform_points(points, T):
    """把点集变换到另一个坐标系，列向量约定 p' = T @ p"""
    R = T[:3, :3]
    t = T[:3, 3]
    return points @ R.T + t


def invert_transform(T):
    """求刚体变换的逆，利用 R^T = R^-1"""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.zeros((4, 4))
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    T_inv[3, 3] = 1
    return T_inv


def rotation_z(theta):
    """绕 Z 轴旋转 theta 弧度"""
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def pairwise_distances(points, pairs):
    """计算指定点对的欧氏距离

    Parameters
    ----------
    points : ndarray, shape (N, 3)
        点云
    pairs : ndarray, shape (M, 2)
        每行是 (i, j)，表示要计算第 i 个点与第 j 个点的距离

    Returns
    -------
    distances : ndarray, shape (M,)
        每对点的距离
    """
    first = points[pairs[:, 0]]
    second = points[pairs[:, 1]]
    # axis=1 表示对每一行求长度；不加它会把整个数组当成一个长向量，返回单个标量
    return np.linalg.norm(first - second, axis=1)


def main():
    """在真实点云上验证变换的刚体性、质心一致性、可逆性与实现一致性"""
    color, depth = load_frame(DATA_DIR, FRAME_INDEX)
    points, valid = depth_to_points(depth, INTRINSIC, DEPTH_SCALE)
    colors = points_to_colors(color, valid)
    print(f"点云点数: {len(points)}")

    # 造一个有旋转和平移的变换
    T = make_transform(rotation_z(np.deg2rad(30)), np.array([1.0, 2.0, 3.0]))
    moved = transform_points(points, T)

    # 随机抽 500 对点，用于检查 1 的距离比较
    rng = np.random.default_rng(0)
    idx = rng.choice(len(points), 500, replace=False)
    pairs = np.column_stack([idx, idx[::-1]])

    # ---- 检查 1：刚体性 ----
    # 距离只由相对位置决定，平移项在相减时抵消，所以点间距离必须完全不变
    distance_before = pairwise_distances(points, pairs)
    distance_after = pairwise_distances(moved, pairs)
    gap_rigid = np.abs(distance_before - distance_after).max()
    assert gap_rigid < 1e-9, f"点间距离改变了，变换不是刚体: {gap_rigid:.3e}"
    print(f"检查1 通过：500 对点距离最大变化 {gap_rigid:.3e}")

    # ---- 检查 2：质心一致性 ----
    # 求均值和施加仿射变换都是线性运算，可以交换顺序，
    # 因此「先求质心再变换」应等于「先变换再求质心」。
    # transform_points 的契约是 (N, 3)，单个质心要 reshape 成 (1, 3)
    centroid = points.mean(axis=0)
    centroid_moved = transform_points(centroid.reshape(1, 3), T)[0]
    gap_centroid = np.abs(centroid_moved - moved.mean(axis=0)).max()
    assert gap_centroid < 1e-9, f"质心变换不一致: {gap_centroid:.3e}"
    print(f"检查2 通过：质心偏差 {gap_centroid:.3e}")

    # ---- 检查 3：可逆性 ----
    # 施加 T 再施加 T 的逆，应回到原处
    restored = transform_points(moved, invert_transform(T))
    gap_inverse = np.abs(points - restored).max()
    assert gap_inverse < 1e-9, f"逆变换未能还原: {gap_inverse:.3e}"
    print(f"检查3 通过：逆变换最大误差 {gap_inverse:.3e}")

    # ---- 检查 4：交叉实现比对 ----
    # 两边吃同一个输入、用同一个 T，所以比较的是两份输出：
    # 自实现的 moved 与 Open3D 的 ref。
    # 这一条能抓住行/列向量约定不一致这类「两边各自自洽」的错误
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.transform(T)
    ref = np.asarray(pcd.points)
    gap_open3d = np.abs(ref - moved).max()
    assert np.allclose(moved, ref), f"与 Open3D 不一致: {gap_open3d:.3e}"
    print(f"检查4 通过：与 Open3D 最大差异 {gap_open3d:.3e}")

    # ---- 保存变换后的点云，颜色沿用原始颜色 ----
    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(moved)
    out_pcd.colors = o3d.utility.Vector3dVector(colors)
    out_path = OUTPUT_DIR / "07_pointcloud_transformed.ply"
    ok = o3d.io.write_point_cloud(str(out_path), out_pcd)
    print(f"保存 {out_path.name}: {ok}，点数 {len(moved)}")


if __name__ == "__main__":
    main()

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
    # TODO 1: 用 fancy indexing 取出两端的点，再算范数。
    #   提示：points[pairs[:, 0]] 得到 (M,3)，points[pairs[:, 1]] 同理。
    #   求每行向量的长度用 np.linalg.norm(..., axis=1)。
    raise NotImplementedError("pairwise_distances 还没实现")


def main():
    """在真实点云上验证变换的三条性质"""
    color, depth = load_frame(DATA_DIR, FRAME_INDEX)
    points, valid = depth_to_points(depth, INTRINSIC, DEPTH_SCALE)
    colors = points_to_colors(color, valid)
    print(f"点云点数: {len(points)}")

    # 造一个有旋转和平移的变换
    T = make_transform(rotation_z(np.deg2rad(30)), np.array([1.0, 2.0, 3.0]))
    moved = transform_points(points, T)

    # ---- 检查 1：变换是刚体的，点间距离必须不变 ----
    # 随机抽 500 对点，比较变换前后的距离
    rng = np.random.default_rng(0)
    idx = rng.choice(len(points), 500, replace=False)
    pairs = np.column_stack([idx, idx[::-1]])

    # 下面 TODO 2 到 6 会用到 colors、moved、pairs，
    # 在补完之前 ruff 会报 F841 未使用变量，属预期。

    # TODO 2: 用 pairwise_distances 算变换前后的距离，
    #   断言最大差异小于 1e-9，并打印实际值。

    # ---- 检查 2：质心也应按同一个 T 变换 ----
    # TODO 3: 计算 points 的质心 centroid（沿第 0 轴平均），
    #   验证 transform_points(centroid reshaped to (1,3), T) 等于 moved 的质心。
    #   想想为什么这个性质成立（提示：矩阵乘法对加法是线性的）。

    # ---- 检查 3：逆变换能把点云还原 ----
    # TODO 4: 用 invert_transform 得到 T_inv，
    #   对 moved 施加 T_inv，断言与原始 points 的最大差异小于 1e-9。

    # ---- 检查 4：与 Open3D 的结果对照 ----
    # TODO 5: 用 o3d 对同一份点云施加同一个 T，比较逐点差异。
    #   注意 o3d 的 Vector3dVector 接受 (N,3) 的 float64 数组。

    # ---- 输出：保存变换后的点云，便于在窗口里查看 ----
    # TODO 6: 把移动后的点云（带原颜色）写到
    #   OUTPUT_DIR / "07_pointcloud_transformed.ply"
    #   可以直接用 open3d 的写法：
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(moved)
    #     pcd.colors = o3d.utility.Vector3dVector(colors)
    #     o3d.io.write_point_cloud(str(path), pcd)


if __name__ == "__main__":
    main()

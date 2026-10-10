"""深度图反投影生成点云，并与 Open3D 的实现交叉验证"""

from pathlib import Path

import numpy as np
import open3d as o3d

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "redwood_rgbd"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs"

# 本次处理第几帧，以及深度原始值到米的换算系数（见 notes/dataset.md）
FRAME_INDEX = 0
DEPTH_SCALE = 1000.0

# 相机内参，来源见 notes/dataset.md
INTRINSIC = {
    "width": 640,
    "height": 480,
    "fx": 525.0,
    "fy": 525.0,
    "cx": 319.5,
    "cy": 239.5,
}


def load_frame(data_dir, index):
    """读出一帧彩色图与深度原始值

    与 work/04_inspect_rgbd.py 的读取方式一致，样板代码。
    """
    stem = f"{index:05d}"
    color = np.asarray(o3d.io.read_image(str(data_dir / "color" / f"{stem}.jpg")))
    depth = np.asarray(o3d.io.read_image(str(data_dir / "depth" / f"{stem}.png")))
    return color, depth


def pixel_grid(height, width):
    """生成与图像同形状的像素坐标网格

    Parameters
    ----------
    height, width : int
        图像的高与宽

    Returns
    -------
    u : ndarray, shape (height, width)
        每个位置的横坐标 u
    v : ndarray, shape (height, width)
        每个位置的纵坐标 v

    Notes
    -----
    要求 u[i, j] 等于第 i 行第 j 列像素的 u，v[i, j] 等于该像素的 v。
    """
    # TODO 1: 用 np.meshgrid 生成。
    #   注意 np.meshgrid 的每个参数对应哪一维，先用小的尺寸验证再继续。
    row = np.arange(height).reshape(height, 1)
    col = np.arange(width).reshape(1, width)
    u, v = np.meshgrid(col, row)
    return u, v
    # raise NotImplementedError("pixel_grid 还没实现")


def depth_to_points(depth, intrinsic, depth_scale):
    """把整张深度图反投影成点云

    Parameters
    ----------
    depth : ndarray, shape (H, W)
        深度原始值，整数，0 表示无效
    intrinsic : dict
        相机内参，含 fx, fy, cx, cy, width, height
    depth_scale : float
        深度原始值除以它得到米

    Returns
    -------
    points : ndarray, shape (N, 3)
        相机坐标系下的点，单位米，三列依次 X, Y, Z
    valid : ndarray, shape (H, W), dtype bool
        有效像素掩码，供取颜色时使用

    Notes
    -----
    N 等于有效像素个数，且点的顺序必须与像素的扫描顺序一致，
    否则上色时会错位。
    """
    # 像素网格与深度都要拉平成一维，两者位置才能一一对应
    u, v = pixel_grid(depth.shape[0], depth.shape[1])
    u = u.flatten()
    v = v.flatten()

    # 深度原始值除以 depth_scale 得到米，先转 float64 避免整数运算的边界问题
    z = depth.astype(np.float64) / depth_scale
    z = z.flatten()

    # 二维掩码留给调用方取颜色用，拉平版本用于筛点
    valid = depth > 0
    valid_flat = valid.flatten()

    # 无效像素的深度为 0，不筛掉会在相机原点堆出一团假点，因此只对有效像素反投影
    X = (u[valid_flat] - intrinsic["cx"]) / intrinsic["fx"] * z[valid_flat]
    Y = (v[valid_flat] - intrinsic["cy"]) / intrinsic["fy"] * z[valid_flat]
    Z = z[valid_flat]

    # 三个一维数组合成一列点，得到 (N, 3)
    points = np.column_stack([X, Y, Z])
    return points, valid


def points_to_colors(color, valid):
    """取出与点一一对应的颜色

    Parameters
    ----------
    color : ndarray, shape (H, W, 3)
        彩色图，uint8，RGB 顺序
    valid : ndarray, shape (H, W), dtype bool
        与 depth_to_points 返回的掩码相同

    Returns
    -------
    colors : ndarray, shape (N, 3), dtype float64
        取值范围 0 到 1。Open3D 的点云颜色使用这个范围，
        直接传入 0 到 255 的整数会被当作超范围值，颜色失真。
    """
    # TODO 3: 把 (H, W, 3) 拉平成 (H*W, 3)，再用拉平后的 valid 取行。
    #   拉平顺序必须与 depth_to_points 中一致，否则颜色与点错位。
    colors = color[valid].astype(np.float64) / 255
    return colors
    # raise NotImplementedError("points_to_colors 还没实现")


def save_pointcloud(points, colors, path):
    """写出 PLY 点云文件，样板代码"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    ok = o3d.io.write_point_cloud(str(path), pcd)
    print(f"写入 {path.name}: {ok}，点数 {len(points)}")


def open3d_reference(color, depth, intrinsic, depth_scale):
    """用 Open3D 自带接口生成同一帧的点云，作为对照，样板代码"""
    intr = o3d.camera.PinholeCameraIntrinsic(
        intrinsic["width"],
        intrinsic["height"],
        intrinsic["fx"],
        intrinsic["fy"],
        intrinsic["cx"],
        intrinsic["cy"],
    )
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color),
        o3d.geometry.Image(depth),
        depth_scale=depth_scale,
        depth_trunc=100.0,
        convert_rgb_to_intensity=False,
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr)
    return np.asarray(pcd.points)


def main():
    """先小尺寸验证像素网格，再把整帧反投影成点云并与 Open3D 对照"""
    # 先用 3x4 的小网格确认 u、v 没写反。
    # 同一组下标 [2, 1] 下 u 应为 1、v 应为 2，两者不同才说明两个方向都没搞错。
    u, v = pixel_grid(height=3, width=4)
    print("pixel_grid(3, 4):")
    print(u)
    print(v)
    print(f"形状 u={u.shape} v={v.shape}, u[2,1]={u[2, 1]}, v[2,1]={v[2, 1]}")

    color, depth = load_frame(DATA_DIR, FRAME_INDEX)
    print()
    print(
        f"帧 {FRAME_INDEX}: color {color.shape} {color.dtype}, "
        f"depth {depth.shape} {depth.dtype}"
    )

    points, valid = depth_to_points(depth, INTRINSIC, DEPTH_SCALE)
    colors = points_to_colors(color, valid)

    print(f"点数 {len(points)}，有效像素 {int(valid.sum())}/{valid.size}")
    for axis, name in enumerate("XYZ"):
        column = points[:, axis]
        print(f"  {name}: {column.min():.3f} ~ {column.max():.3f} m")
    print(
        f"颜色 {colors.shape} {colors.dtype}，"
        f"范围 [{colors.min():.3f}, {colors.max():.3f}]"
    )

    # 两份点云的遍历顺序一致（都按行优先、都跳过无效像素），
    # 因此第 k 个点指向同一物理点，可以直接逐点相减。
    # 若顺序可能不同，就必须先做最近邻匹配，否则差异会被高估。
    points_ref = open3d_reference(color, depth, INTRINSIC, DEPTH_SCALE)
    assert points_ref.shape == points.shape, (
        f"点数不一致: {points_ref.shape} vs {points.shape}"
    )
    max_diff = np.abs(points_ref - points).max()
    print(f"与 Open3D 对照: 点数 {len(points_ref)}，逐点最大差异 {max_diff:.3e} m")

    out_path = OUTPUT_DIR / "05_pointcloud_frame0.ply"
    save_pointcloud(points, colors, out_path)
    print()
    print("在窗口里查看点云：")
    print(
        f'  python -c "import open3d as o3d; '
        f"o3d.visualization.draw_geometries(["
        f"o3d.io.read_point_cloud('{out_path}')])\""
    )


if __name__ == "__main__":
    main()

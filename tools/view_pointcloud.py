"""在 Open3D 窗口中查看点云

用法
----
    conda activate rgbd
    python tools/view_pointcloud.py outputs/05_pointcloud_frame0.ply
    python tools/view_pointcloud.py outputs/*.ply
    python tools/view_pointcloud.py outputs/07_pointcloud_transformed.ply --axes
    python tools/view_pointcloud.py outputs/05_pointcloud_frame0.ply --point-size 1

参数
----
path          一个或多个点云文件（.ply / .pcd），可同时给多个以便对比
--axes        额外画一个坐标轴，便于判断朝向
--point-size  点的大小，默认 2.0；点云密集时调小更容易看清结构
--background  背景灰度，0 到 1，默认 0.05（接近纯黑）

窗口操作
--------
    左键拖动    旋转
    滚轮        缩放
    中键拖动    平移
    Q 或 Esc    退出

关于朝向
--------
Open3D 的观察窗口默认 Y 轴朝上，而本项目的相机坐标系 Y 轴朝下，
所以在窗口里看到的点云可能是上下颠倒的。这是显示约定，不是数据错误 ——
前面已经用数值验证过自实现的坐标与 Open3D 完全一致（差异 0.0）。
把屏幕上的 Y 轴在脑子里翻过来看即可。
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def load_pointclouds(paths):
    """读取一组点云文件，并逐个打印点数与包围盒

    Parameters
    ----------
    paths : list of str
        文件路径

    Returns
    -------
    clouds : list of open3d.geometry.PointCloud
        成功读入的点云，跳过不存在或不含点的文件
    """
    clouds = []
    for raw in paths:
        path = Path(raw)
        if not path.exists():
            print(f"[跳过] 文件不存在: {path}")
            continue
        cloud = o3d.io.read_point_cloud(str(path))
        count = len(cloud.points)
        if count == 0:
            print(f"[跳过] 没有读到点: {path}")
            continue
        points = np.asarray(cloud.points)
        print(f"{path.name}: {count} 点, 颜色 {'有' if cloud.has_colors() else '无'}")
        print(
            f"    包围盒 X {points[:, 0].min():.2f}~{points[:, 0].max():.2f}"
            f"  Y {points[:, 1].min():.2f}~{points[:, 1].max():.2f}"
            f"  Z {points[:, 2].min():.2f}~{points[:, 2].max():.2f}"
        )
        clouds.append(cloud)
    return clouds


def show(geometries, window_name, point_size, background):
    """在窗口中显示几何体，直到用户关闭窗口

    Parameters
    ----------
    geometries : list of open3d.geometry.Geometry
        要显示的几何体
    window_name : str
        窗口标题
    point_size : float
        点的大小
    background : float
        背景灰度，0 到 1

    Notes
    -----
    点大小与背景色只能通过 Visualizer 设置，
    draw_geometries 没有这两个参数。
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=1000, height=700)
    for geometry in geometries:
        vis.add_geometry(geometry)
    option = vis.get_render_option()
    option.point_size = point_size
    option.background_color = np.array([background] * 3)
    vis.run()
    vis.destroy_window()


def main():
    """解析参数，读点云并显示"""
    parser = argparse.ArgumentParser(description="在 Open3D 窗口中查看点云")
    parser.add_argument("paths", nargs="+", help="点云文件路径")
    parser.add_argument("--axes", action="store_true", help="显示坐标轴")
    parser.add_argument("--point-size", type=float, default=2.0, help="点的大小")
    parser.add_argument(
        "--background", type=float, default=0.05, help="背景灰度 0 到 1"
    )
    args = parser.parse_args()

    clouds = load_pointclouds(args.paths)
    if not clouds:
        print("没有可显示的点云", file=sys.stderr)
        return 1

    geometries = list(clouds)
    if args.axes:
        geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3))

    print("\n正在打开窗口，按 Q 或 Esc 退出")
    show(geometries, f"点云查看器 ({len(clouds)} 个)", args.point_size, args.background)
    return 0


if __name__ == "__main__":
    sys.exit(main())

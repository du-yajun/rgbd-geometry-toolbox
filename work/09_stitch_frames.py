"""按给定位姿把 5 帧点云拼到同一个世界坐标系

这是项目边界之外的拓展。数据集已经给出每帧的相机位姿，本脚本不做配准，
只做「按位姿变换 + 叠加」，用来直观看到多帧如何共处一个坐标系。

数据集限制：这 5 帧相机累计只移动 97.9 mm、转动 3.0 度，视野几乎没有变化
（见 notes/dataset.md），所以拼接带来的主要是点云变密，而不是覆盖范围扩大。
"""

import importlib.util
from pathlib import Path

import numpy as np
import open3d as o3d

FRAME_COUNT = 5
DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "redwood_rgbd"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs"


def load_module(module_name, filename):
    """按文件路径加载同目录下的模块，样板代码"""
    path = Path(__file__).resolve().parent / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_poses(path):
    """解析 trajectory.log，返回每帧的 4x4 位姿矩阵

    文件格式：一行 "帧号 帧号 帧号+1"，随后 4 行组成一个 4x4 矩阵。

    Returns
    -------
    poses : list of ndarray, shape (4, 4)
        每帧一个矩阵。经本项目实测，这些矩阵是 T_world_camera，
        即直接把相机坐标系的点左乘即可得到世界坐标。
    """
    with open(path) as handle:
        lines = [line.rstrip("\n") for line in handle]
    poses = []
    index = 0
    while index < len(lines):
        if len(lines[index].split()) == 3:
            block = [
                [float(value) for value in lines[index + 1 + row].split()]
                for row in range(4)
            ]
            poses.append(np.array(block))
            index += 5
        else:
            index += 1
    return poses


def transform_to_world(points, pose, assume_cam_to_world=True):
    """把相机坐标系的点变换到世界坐标系

    Parameters
    ----------
    points : ndarray, shape (N, 3)
        相机坐标系下的点
    pose : ndarray, shape (4, 4)
        位姿矩阵
    assume_cam_to_world : bool
        True 表示 pose 是 T_world_camera，直接作用；
        False 表示 pose 是 T_camera_world，需要先求逆。

    Returns
    -------
    points_world : ndarray, shape (N, 3)
    """
    if assume_cam_to_world:
        return points @ pose[:3, :3].T + pose[:3, 3]
    rotation = pose[:3, :3]
    rotation_inv = rotation.T
    translation_inv = -rotation_inv @ pose[:3, 3]
    return points @ rotation_inv.T + translation_inv


def alignment_error(points_a, points_b, rng, sample=4000):
    """两片点云互相重合的程度：最近邻距离的中位数

    把两帧变换到同一坐标系后，如果方向正确，它们应当基本重合；
    如果方向错误，它们会分开约两倍的实际位移。用这个量可以判定方向。
    """
    # 注意：不能把临时的 Vector3dVector 直接传给 KDTreeFlann，
    # 临时对象被回收后 KD-tree 会指向已释放的内存，搜索时报 RuntimeError。
    # 必须建一个 PointCloud 并让它在本函数内一直存活。
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_b)
    tree = o3d.geometry.KDTreeFlann(cloud)
    index = rng.choice(len(points_a), min(sample, len(points_a)), replace=False)
    distances = []
    for i in index:
        count, _, squared = tree.search_knn_vector_3d(points_a[i], 2)
        if count >= 2:
            distances.append(np.sqrt(squared[1]))
    return float(np.median(distances))


def distance_to_reference(points_new, points_ref, rng, sample=20000):
    """抽样统计 points_new 中每个点到 points_ref 的最近距离

    Returns
    -------
    distances : ndarray, shape (M,)
        单位米。用它可以判断拼接相对单帧新增了多少覆盖：
        小距离说明只是同一片表面被重复观测（点变密），
        大距离才说明看到了单帧没看到的表面。
    """
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_ref)
    tree = o3d.geometry.KDTreeFlann(cloud)
    index = rng.choice(len(points_new), min(sample, len(points_new)), replace=False)
    distances = []
    for i in index:
        count, _, squared = tree.search_knn_vector_3d(points_new[i], 1)
        if count == 1:
            distances.append(np.sqrt(squared[0]))
    return np.array(distances)


def main():
    """判定位姿方向，拼接 5 帧，统计覆盖变化并保存"""
    m05 = load_module("m05", "05_depth_to_pointcloud.py")
    rng = np.random.default_rng(0)

    poses = load_poses(DATA_DIR / "trajectory.log")
    print(f"读到位姿 {len(poses)} 个")
    assert len(poses) == FRAME_COUNT, f"位姿数量与预期不符: {len(poses)}"

    clouds = []
    for index in range(FRAME_COUNT):
        _, depth = m05.load_frame(m05.DATA_DIR, index)
        points, _ = m05.depth_to_points(depth, m05.INTRINSIC, m05.DEPTH_SCALE)
        clouds.append(points)
    print(f"每帧点数: {[len(c) for c in clouds]}")

    # ---- 判定位姿方向：用相隔最远的两帧做重合度比较 ----
    sample_a = clouds[0][rng.choice(len(clouds[0]), 20000, replace=False)]
    sample_b = clouds[4][rng.choice(len(clouds[4]), 20000, replace=False)]

    errors = {}
    for assume in (True, False):
        world_a = transform_to_world(sample_a, poses[0], assume)
        world_b = transform_to_world(sample_b, poses[4], assume)
        errors[assume] = alignment_error(world_a, world_b, rng)

    name = {True: "pose 即 T_world_cam，直接作用", False: "pose 是 T_cam_world，先求逆"}
    print()
    print("位姿方向判定（帧0 与帧4 变换后两片点云的重合度）:")
    for assume in (True, False):
        print(f"  {name[assume]}: 最近邻距离中位数 {errors[assume] * 1000:.2f} mm")

    best = min(errors, key=errors.get)
    worst = max(errors, key=errors.get)
    ratio = errors[worst] / errors[best]
    print(f"  -> 判定为「{name[best]}」，比另一方向好 {ratio:.1f} 倍")
    assert errors[worst] > 3.0 * errors[best], "两个方向差别不够明显，无法判定"

    # ---- 按判定结果拼接全部 5 帧 ----
    stitched = np.vstack(
        [transform_to_world(points, poses[i], best) for i, points in enumerate(clouds)]
    )
    print()
    print(f"拼接后点数: {len(stitched)}（单帧约 {len(clouds[0])}）")
    for axis, label in enumerate("XYZ"):
        column = stitched[:, axis]
        print(f"  {label}: {column.min():.3f} ~ {column.max():.3f} m")

    # ---- 拼接到底新增了多少覆盖 ----
    # 只看「离单帧点多远」还不够：同一个表面被二次观测时，
    # 因为视角与噪声的差异也会有毫米级偏移。必须换几个阈值一起看。
    single = transform_to_world(clouds[0], poses[0], best)
    offsets = distance_to_reference(stitched, single, rng) * 1000
    print()
    print("拼接后的点到「单帧点云」的最近距离:")
    for quantile in (50, 90, 99):
        print(f"  {quantile}% 分位: {np.percentile(offsets, quantile):.2f} mm")
    print("  超过阈值算作「单帧没看到的新表面」:")
    for threshold in (5, 20, 100):
        share = (offsets > threshold).mean() * 100
        print(f"    >{threshold:>4} mm: {share:>5.1f}%")
    print("  深度噪声在 2 m 处就有几毫米量级，所以小阈值测到的主要是噪声；")
    print("  把阈值放到 20 mm 以上才接近真实的新增覆盖，可见拼接主要是让点变密。")

    # ---- 保存 ----
    colors = []
    for index in range(FRAME_COUNT):
        color, _ = m05.load_frame(m05.DATA_DIR, index)
        depth = np.asarray(
            o3d.io.read_image(str(m05.DATA_DIR / "depth" / f"{index:05d}.png"))
        )
        colors.append(m05.points_to_colors(color, depth > 0))
    all_colors = np.vstack(colors)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(stitched)
    cloud.colors = o3d.utility.Vector3dVector(all_colors)
    out_path = OUTPUT_DIR / "09_stitched_5frames.ply"
    ok = o3d.io.write_point_cloud(str(out_path), cloud)
    print()
    print(f"保存 {out_path.name}: {ok}，点数 {len(stitched)}")


if __name__ == "__main__":
    main()

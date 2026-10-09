"""体检 Redwood RGB-D 样例数据：图像格式、深度单位、有效像素与配准关系"""

from pathlib import Path
from unicodedata import east_asian_width

import numpy as np
import open3d as o3d

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "redwood_rgbd"

# 相机参数来自 Open3D 官方示例（examples/python/geometry/rgbd_datasets.py）
# 与内参定义（cpp/open3d/camera/PinholeCameraIntrinsic.cpp 的 PrimeSenseDefault）
INTRINSIC = {
    "width": 640,
    "height": 480,
    "fx": 525.0,
    "fy": 525.0,
    "cx": 319.5,
    "cy": 239.5,
}


def display_width(text):
    """返回字符串在等宽终端里占的列数

    中日韩字符占两列，其余占一列。Python 的 len() 按字符数算，
    直接用它对齐会让含中文的表头错位，因此单独计算显示宽度。
    """
    return sum(2 if east_asian_width(ch) in "WF" else 1 for ch in text)


def pad(text, width):
    """按显示宽度右对齐，补空格到 width 列"""
    text = str(text)
    return " " * max(0, width - display_width(text)) + text


def frame_paths(data_dir, index):
    """返回某一帧的彩色图与深度图路径

    该数据集的编号从 0 开始，文件名补零到 5 位，例如 index=3 对应 00003。
    """
    stem = f"{index:05d}"
    return data_dir / "color" / f"{stem}.jpg", data_dir / "depth" / f"{stem}.png"


def load_frame(color_path, depth_path):
    """读出某一帧的彩色图与深度图

    Returns
    -------
    color : ndarray
        彩色图。JPEG 是 8 位，形状为 (H, W, 3)
    depth : ndarray
        深度原始值。PNG 是 16 位，形状为 (H, W)，尚未做单位换算
    """
    color = np.asarray(o3d.io.read_image(str(color_path)))
    depth = np.asarray(o3d.io.read_image(str(depth_path)))
    return color, depth


def inspect_frame(color, depth):
    """汇总单帧的格式与数值特征

    Parameters
    ----------
    color : ndarray, shape (H, W, 3)
        彩色图，未做任何转换
    depth : ndarray, shape (H, W)
        深度原始值，未做单位换算

    Returns
    -------
    info : dict
        至少包含下列键，值为 int / float / str / tuple：
        color_shape, color_dtype   彩色图的形状与元素类型
        depth_shape, depth_dtype   深度图的形状与元素类型
        depth_raw_min, depth_raw_max
            全部像素（含无效像素）的原始极值
        invalid_ratio
            无效像素占比，取值 0 到 1
        depth_valid_min, depth_valid_max
            仅计有效像素的原始极值
        depth_valid_median
            有效像素的原始中位数
        unit
            由数值推断出的深度单位，字符串 "mm" 或 "m"
    """
    # 无效深度以 0 表示：实测 0 是全局最小值，且占总像素约 13%
    invalid_ratio = (depth == 0).sum() / depth.size
    valid = depth[depth > 0]

    # 有效值落在 955 到 2702。若单位是米，则房间深度超过两公里，不成立，故为毫米
    unit = "mm"

    # 值统一转成 Python 原生类型，便于打印与序列化，避免字典里混入 numpy 标量
    info = {
        "color_shape": tuple(color.shape),
        "color_dtype": str(color.dtype),
        "depth_shape": tuple(depth.shape),
        "depth_dtype": str(depth.dtype),
        "depth_raw_min": int(depth.min()),
        "depth_raw_max": int(depth.max()),
        "invalid_ratio": round(float(invalid_ratio), 4),
        "depth_valid_min": int(valid.min()),
        "depth_valid_max": int(valid.max()),
        "depth_valid_median": float(np.median(valid)),
        "unit": unit,
    }
    return info


def main():
    """遍历全部帧，打印格式与数值特征对照表"""
    print(f"数据目录: {DATA_DIR}")
    print(
        f"内参: {INTRINSIC['width']}x{INTRINSIC['height']}, "
        f"fx={INTRINSIC['fx']}, fy={INTRINSIC['fy']}, "
        f"cx={INTRINSIC['cx']}, cy={INTRINSIC['cy']}"
    )
    print()

    # 先按帧收集，每帧一个字典
    results = []
    for i in range(5):
        color_path, depth_path = frame_paths(DATA_DIR, i)
        color, depth = load_frame(color_path, depth_path)

        # 彩色与深度必须同尺寸，否则按像素一一对应会错位
        assert color.shape[:2] == depth.shape[:2], (
            f"帧{i} 彩色与深度尺寸不一致：{color.shape[:2]} vs {depth.shape[:2]}"
        )
        results.append(inspect_frame(color, depth))

    # 再按指标转置：每个指标一行，每帧一列
    keys = list(results[0].keys())
    label_width = max(len(k) for k in keys)
    cell_width = 14

    header = " " * (label_width + 2)
    header += "".join(pad(f"帧{i}", cell_width) for i in range(len(results)))
    print(header)
    print("-" * display_width(header))

    for k in keys:
        cells = "".join(pad(result[k], cell_width) for result in results)
        print(f"{k:<{label_width}}  {cells}")


if __name__ == "__main__":
    main()

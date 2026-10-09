# 数据集说明

## Redwood RGB-D 样例（Open3D 官方提供）

### 来源与获取

- 上游数据集：Redwood 室内数据集（Robust Reconstruction of Indoor Scenes 论文）
  项目主页 http://redwood-data.org/indoor/
- 本项目使用 Open3D 官方提供的 5 帧节选，随 Open3D 文档与示例分发
- 下载地址（Open3D 官方数据仓库的 release 资产）
  https://github.com/isl-org/open3d_downloads/releases/download/20220201-data/SampleRedwoodRGBDImages.zip
- 压缩包大小 3772230 字节，约 3.8 MB，解压后约 6.7 MB
- 复现下载：

  ```bash
  mkdir -p data/redwood_rgbd
  curl -L -o /tmp/redwood.zip \
    https://github.com/isl-org/open3d_downloads/releases/download/20220201-data/SampleRedwoodRGBDImages.zip
  unzip -o /tmp/redwood.zip -d data/redwood_rgbd
  ```

- 数据本身不入库（.gitignore 已忽略 data/），只保留本文档描述获取方式

### 内容清单

| 路径 | 内容 |
|---|---|
| color/00000.jpg 至 00004.jpg | 5 帧彩色图 |
| depth/00000.png 至 00004.png | 5 帧深度图，编号与彩色图一一对应 |
| example_tsdf_pcd.ply | 上游用这 5 帧重建出的参考点云 |
| odometry.log | 相邻帧之间的里程计位姿估计 |
| rgbd.match | 帧间匹配关系 |
| trajectory.log | 相机轨迹 |

注意：压缩包里没有相机参数文件。内参来自 Open3D 示例代码里硬编码的
PrimeSenseDefault，见下一节。这是本数据集的一个已知不足。

### 相机内参

Open3D 示例 examples/python/geometry/rgbd_datasets.py 对这份数据使用
`PinholeCameraIntrinsicParameters.PrimeSenseDefault`，
其定义在 cpp/open3d/camera/PinholeCameraIntrinsic.cpp：

| 参数 | 值 |
|---|---|
| 分辨率 | 640 x 480 |
| fx | 525.0 |
| fy | 525.0 |
| cx | 319.5 |
| cy | 239.5 |

注意 cx=319.5、cy=239.5 都不是整数，比图像中心 (320, 240) 各偏 0.5 像素。
这是常见情形，因为像素中心落在半整数坐标上。

### 格式

下表由 work/04_inspect_rgbd.py 实测输出填写，5 帧逐列对应。

| 项目 | 实测值 |
|---|---|
| 彩色图 | 分辨率 640x480（宽x高），ndarray 形状 (480, 640, 3)，uint8，RGB 顺序 |
| 深度图 | 分辨率 640x480，ndarray 形状 (480, 640)，uint16，单通道 |
| 深度原始取值范围 | 0 到 2702 的整数，其中 0 不是距离，而是无效标记 |
| 无效像素占比 | 0.1304 0.1285 0.1270 0.1256 0.1242，对应有效占比 0.8696 到 0.8758 |
| 无效深度的表示 | 0。实测 0 是全局最小值且占比约 13%，与「测不到」的预期一致 |
| 深度单位 | mm，除以 1000 得到米 |
| 深度是沿光轴的 Z 还是到光心的距离 | 按 Open3D 约定视为沿光轴的 Z。依据是 create_from_color_and_depth 的文档，非本数据实测 |

关于深度的两点说明：

- 单位判断依据：有效值落在 955 到 2702。若单位是米，房间深度将超过两公里，
  与室内场景不符，因此只能是毫米。
- 最后一行的性质与其他行不同。其余各项都能从数据直接读出，而「沿光轴的 Z」
  只能依据接口约定。数据本身分不出它和「到光心的距离」，两者只在画面边缘
  相差百分之几。记录时分开实测与依据，避免后人误以为这一条有数据支撑。

### 已知限制

- 只有 5 帧，且是静态场景连续帧，无法用于评估大范围重建
- 内参是通用参数，不是针对这台设备标定的，可能存在系统偏差
- 彩色与深度来自同一传感器，但本数据集未提供配准矩阵，
  「同尺寸」不等于「已配准」，详见检查记录
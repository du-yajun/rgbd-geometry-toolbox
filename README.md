# RGB-D 几何工具箱

一个面向初学者的 RGB-D 几何练习项目：从相机模型出发，自己动手完成深度反投影、
点云生成、刚体变换和一致性检查，最终整理成可运行、可解释的小工具集。

项目的重点不是堆功能，而是把每一步的几何含义讲清楚，并且用测试验证它。
没有 GPU 也能跑，全部基于 CPU 和小规模公开数据。

## 环境

- Python 3.11（conda 环境名 `rgbd`）
- numpy / matplotlib / open3d / pytest

```bash
mamba create -n rgbd -c conda-forge python=3.11 numpy matplotlib -y
conda activate rgbd
pip install -r requirements.txt

# macOS 额外一步：Open3D 依赖系统库 libusb，缺少时 import open3d 会失败
brew install libusb
```

本项目在以下环境实测通过（2026-10-03）：
macOS / Apple M4 / Python 3.11.16 / numpy 2.4.6 / matplotlib 3.11.2 / Open3D 0.20.0。

## 目录结构

```
.
├── README.md
├── requirements.txt
├── notes/
│   └── learning-log.md        # 学习日志：每步做了什么、验证了什么、还剩什么疑问
├── outputs/                   # 效果图等产出
└── work/                      # 练习脚本
    └── 01_pixel_and_index.py  # 任务 1：像素坐标 (u,v) 与数组下标 [row,col]
```

## 运行

```bash
conda activate rgbd
python work/01_pixel_and_index.py
```

## 进度

- [x] 阶段 0：环境搭建与检查
- [ ] 任务 1：像素坐标 vs 数组下标（代码已完成，含四个问题的标准答案）
- [ ] 阶段 1：合成数据下的相机模型
- [ ] 阶段 2：读取并检查真实 RGB-D 数据
- [ ] 阶段 3：自己实现深度反投影生成点云
- [ ] 阶段 4：坐标变换与一致性测试
- [ ] 阶段 5：点云过滤与项目整理

## 目前的核心结论

- 图像数组的第 0 轴是竖直方向（v），第 1 轴是水平方向（u），
  所以「像素坐标 (u, v)」翻译成数组下标要写成 `[v, u]`。
- 相机坐标系（OpenCV/Open3D 约定）：X 右、Y 下、Z 沿光轴向前。
- 程序不报错不等于结果正确。行列互换、RGB/BGR、深度单位、内参分辨率不匹配
  都属于「能跑完但算错」，必须靠主动设计的检查发现。

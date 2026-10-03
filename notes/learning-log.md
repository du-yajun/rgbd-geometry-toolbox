# RGB-D 几何工具箱 · 学习日志

记录原则：只写有证据的结论。跑过的命令、看到的数字、画出的图才算"已验证"；
没验证过的写进"待确认"，不写成结论。

---

## 2026-10-03 · 阶段 0：环境与条件确认

### 已完成（导师代做）

- 新建 conda 环境 `rgbd`：`/opt/homebrew/Caskroom/miniforge/base/envs/rgbd`
  - Python 3.11.16、numpy 2.4.6、matplotlib 3.11.2、open3d 0.20.0、pillow 12.3.0、pytest 9.1.1
- 安装了 Homebrew 的 `libusb`（Open3D 的 macOS wheel 依赖它，缺了会 import 失败）
- 桌面入口：`~/Desktop/rgbd-geometry-toolbox` 是指向本项目的符号链接

### 已验证（有实测输出）

- numpy 数组运算正常
- matplotlib `savefig` 存图正常，`macosx` 交互窗口能创建、渲染、关闭
- Open3D 点云创建、PLY 读写、`PinholeCameraIntrinsic` 正常
- 16 位深度图 PNG 写入后读回，数值完全一致
- `create_from_rgbd_image` 能生成 N×3 点云和 N×3 颜色
- Open3D 图形窗口能创建并渲染

### 已知限制 / 未验证

- `gh` 命令行工具未安装（以后推 GitHub 再决定走网页 + HTTPS 还是装 gh）
- git 未设置全局默认分支名（本项目会用 `git init -b main`）
- 真实 RGB-D 数据集尚未下载（阶段 2 做）

---

## 2026-10-03 · 任务 1：像素坐标 vs 数组下标

### 我亲手写的

- `work/01_pixel_and_index.py`
- 用广播构造数组：`row = np.arange(H).reshape(H, 1)`、`col = np.arange(W).reshape(1, W)`、
  `depth = row * 10 + col`

### 已验证（实测输出）

- `depth.shape == (4, 5)`、`depth.dtype == int64`
- `depth[2, 3] == 23`，`depth[3, 2] == 32`
- `depth[:, 3] == [3, 13, 23, 33]`（竖着的一列，u 固定）
- `depth[2, :] == [20, 21, 22, 23, 24]`（横着的一行，v 固定）
- `outputs/01_pixel_grid.png`：4 行 5 列，左上最深（值 0）、右下最浅（值 24），
  x 轴刻度 0–4，y 轴刻度 0–3 且向下递增 —— 原点在左上角，与数学课的 y 轴朝上相反

### 已能解释

- `arr[row, col]` 的第一个轴是竖直方向、第二个是水平方向
- 像素坐标 `(u, v)` 与数组下标的关系是 `u = col`、`v = row`，所以读深度写 `depth[v, u]`
- 一张 (480, 640, 3) 的图，u 最大 639、v 最大 479（整数坐标代表像素中心）
- 行列写反时：只有在越界时才报 IndexError，没越界就静默算错 —— 所以要主动加形状检查

### 仍需确认

- "图像上方（v 小）对应真实世界的哪个方向"只是猜想，阶段 2 用真实深度图验证
- Open3D 的离屏渲染（不弹窗截图）尚未测试，阶段 3 如果要用再说

### 下一步最小任务

任务 2：手算一个三维点的投影，并自己实现 `project_points(points_cam, K)`


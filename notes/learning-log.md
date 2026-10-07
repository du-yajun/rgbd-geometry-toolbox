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


---

## 2026-10-03 · 任务 1 收尾与仓库初始化

### 脚本修改（导师代做，用户已同意）

`work/01_pixel_and_index.py` 在原代码基础上补充：

- 每个代码块的作用说明（广播、下标、切片、存图、伪彩色图）
- 四个问题的标准答案（写在文件末尾的注释块里）
- `fake_rgb` 从 `np.random.rand` 改为 `np.stack([depth, depth, depth], axis=-1)`，
  这样三个通道有确定含义，能说明「彩色图 = 三张同尺寸二维图叠起来」
- 存图路径改用 `pathlib` 从脚本位置反推，不再写死绝对路径
- 补中文字体设置（否则 matplotlib 默认字体渲染中文会出现方块）
  —— 第一次运行时确实是方块，加了 `PingFang SC` 回退后才正常，已用看图工具确认

### 标准答案要点

- Q1：`depth[2, 3] = 23` 对应 `(u, v) = (3, 2)`；索引顺序是 `[row, col]`
- Q2：图像上方（v 小）对应相机坐标系 -Y；约定为 X 右、Y 下、Z 向前
- Q3：`(480, 640, 3)` 的图 `u_max = 639`、`v_max = 479`（先写 H,W 再分配 u,v）
- Q4：`u >= 480` 抛 IndexError；`u < 480` 静默算错；`u == v` 的像素恰好正确，
  所以抽检必须挑 `u != v` 的像素

### 仓库

- 已 `git init -b main` 并完成首次提交 `9a60e56`
- 提交内容：`.gitignore`、`README.md`、`requirements.txt`、`notes/`、`outputs/`、`work/`
- 生成物 `outputs/01_pixel_grid.png` 一并入库，方便 GitHub 上直接看到效果图
- 尚未做：创建 GitHub 远程仓库、`git remote add`、`git push`

### 待确认

- Open3D 的离屏渲染（不弹窗截图）尚未测试，阶段 3 需要时再验
- 真实 RGB-D 数据尚未下载，阶段 2 处理

---

## 2026-10-07 · 任务 2：投影 project_points()

### 我亲手写的

- `work/02_project_points.py` 的 `project_points()` 函数体（TODO 1~5 全部完成）
- 手算 A、B 两点，并判断 C、D 的处理方式

### 已验证（实测输出）

- `uv.shape == (4, 2)`、`valid.shape == (4,)`、`valid.dtype == bool`
- 输出：`[[370. 215.] [320. 240.] [nan nan] [nan nan]]`，`valid = [True True False False]`
- 用 `-W error::RuntimeWarning` 运行，无除零警告 —— 说明「先算 valid、再做除法」的顺序是对的
- 手算 A 与代码一致：u = 500*0.2/2.0 + 320 = 370，v = 500*(-0.1)/2.0 + 240 = 215
- 比例检查：(u-cx)/(v-cy) = -2.0，与 X/Y = 0.2/-0.1 = -2.0 一致

### 已能解释

- 投影公式 u = fx*X/Z + cx、v = fy*Y/Z + cy 的由来（相似三角形），以及每个符号的单位
- 为什么必须先判断 `valid = Z > 0` 再做除法（避免除零警告，同时排除光心后方的点）
- 光轴上的点无论远近都投影到主点 —— 这是单目无法从单个像素判断距离的根源

### 反馈中纠正的两点

1. 无效条件不是 `Z == 0`，而是 `Z <= 0`。光心后方的点（Z < 0）同样无效，
   而且它们照公式算出来的像素值在数值上"看起来很正经"，不会报错 —— 更容易悄悄出错。
   另外 z=0 那个平面是过光心、垂直于光轴的平面（光心平面），
   光轴本身是一条线，不存在"光轴平面"这个说法。
2. 问「Z = 0.001 时 u、v 的量级」时答了 1e3，实际是 1e5 量级。
   实测量级（fx=500, X=0.2）：Z=2.0 -> 1e2，Z=0.01 -> 1e4，Z=0.001 -> 1e5。
   量级估算方法是先写 u - cx = fx * X / Z，再把数代进去。
   "非常敏感"的定性结论是对的：Z 变化 1% 时，u 在 Z=0.001 处会移动约 990 像素。

### 仍未自己验证

- Z = 0.001 的具体量级（导师代算：100320），建议自己再手算一遍确认
- 任务 3 的往返一致性检查（下一步做）

### 下一步最小任务

任务 3：实现 `backproject_pixels()`，并做「投影 -> 反投影」往返一致性检查

---

## 2026-10-07 · 任务 3：反投影 backproject_pixels() 与往返一致性

### 我亲手写的

- `work/03_backproject.py` 的 `backproject_pixels()`（TODO 1~4）
- 三个像素的手算：P1 (370,215,d=2.0) -> (0.2,-0.1,2.0)；P2 主点 -> (0,0,2.0)；P3 d=0 -> nan
- 自检 2 的 100 点往返检查、自检 3 的三个深度对比

### 遇到并解决的报错

`TypeError: cannot unpack non-iterable NoneType object`（第 106 行）

- 原因：函数算完了 `points_cam` 和 `valid`，但末尾没有 `return`，函数默认返回 None
- 骨架的 TODO 只写到「填数组」，没把 return 单列一步 —— 这是导师出题时的疏漏
- 修复：在函数末尾补 `return points_cam, valid`
- 排查经验：看到 `cannot unpack non-iterable NoneType object`，
  基本可以直接定位为「右边的函数忘了 return」

### 已验证（实测输出）

- `points_cam.shape == (3, 3)`，`valid == [True, True, False]`
- P2 主点还原成 `(0., 0., 2.)` —— X、Y 精确为 0
- 往返一致性误差 `1.665e-16`（浮点精度量级），说明投影与反投影互逆
- 加 `-W error::RuntimeWarning` 运行无任何除零警告
- 半个像素对应的真实距离随深度线性增长：

| 深度 | 半个像素 = 多少米 |
|---|---|
| 0.5 m | 0.0005 m |
| 2.0 m | 0.002 m |
| 5.0 m | 0.005 m |

规律：`dx = 0.5 * Z / fx = Z / 1000`，即误差与深度成正比。

### 学到的一个具体检查习惯

`np.nanmax` 会「忽略」nan，用它检查误差会把失败的点悄悄跳过。
实测对比（2 个点里 1 个无效）：
- 旧写法 `np.nanmax(...)` 打印误差 0.0，看起来完美，实际只算了 1 个点
- 新写法 `assert valid.all()` + 普通 `max()` 直接抛 AssertionError 拦住

结论：**先确认数据有效，再算指标**。这个习惯在阶段 3 处理整图时很关键
（整图里无效像素本来就多，检查逻辑不能也放任 nan）。

### 仍未自己验证

- 反投影公式为什么没有除法（只有乘法）—— 待自己复述一遍
- 「往返检查通过能否证明深度单位和内参正确」—— 阶段 4 专门讨论

### 下一步最小任务

阶段 2：确认 Open3D 官方 SampleRedwoodRGBDImages 是否适合本机，下载并检查
真实 RGB-D 数据的格式、单位、配准关系。

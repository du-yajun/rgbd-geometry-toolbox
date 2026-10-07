"""任务 3：反投影 —— 把「像素 + 深度」还原成相机坐标系中的三维点。

运行方式：
    conda activate rgbd
    python work/03_backproject.py

这是任务 2 的逆运算。完成后我们会用它和 project_points() 做往返一致性检查。
你需要补完 backproject_pixels() 的 TODO 1~4，以及 __main__ 里的自检。
"""

import numpy as np


def backproject_pixels(uv, depths, K):
    """把像素坐标 + 深度反投影为相机坐标系中的三维点。

    参数
    ----------
    uv : np.ndarray, shape (N, 2)
        像素坐标，一列是 u（水平，向右增大），一列是 v（竖直，向下增大）。
        可以是小数，不要求是整数。
    depths : np.ndarray, shape (N,)
        每个像素对应的深度值，单位米。
        约定：这个深度是沿光轴方向的 Z（不是到相机中心的距离），
        且 depths[i] <= 0 表示该像素没有有效测量。
    K : np.ndarray, shape (3, 3)
        内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]。

    返回
    ----------
    points_cam : np.ndarray, shape (N, 3)
       相机坐标系中的点，三列依次是 X, Y, Z，单位米。
       深度无效的点，该行填 np.nan。
    valid : np.ndarray, shape (N,), dtype bool
        该像素的深度是否有效（depths > 0）。

    公式（针孔模型的逆，只在 Z > 0 时成立）：
        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

    为什么长这样：投影时 X 被除以 Z「压平」成了像素偏移，
    反投影就要把偏移乘回去。可以理解成：
        u - cx = fx * X / Z   =>   X = (u - cx) * Z / fx
    """
    uv = np.asarray(uv, dtype=np.float64)
    depths = np.asarray(depths, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64)

    # TODO 1: 从 K 取出 fx, fy, cx, cy
    #         （和任务 2 一样的位置，这次自己回想一下，别翻回去看）
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    # TODO 2: 从 uv 取出 u 和 v 两列，从 depths 取出 z
    u, v = uv[:, 0], uv[:, 1]
    z = depths

    # TODO 3: 计算 valid = (z > 0)，形状 (N,)
    valid = z > 0

    # TODO 4: 先建一个形状 (N, 3) 全为 nan 的数组，再只给 valid 的行赋值。
    #         三列依次是 X, Y, Z。
    #         注意：用布尔索引赋值时，等号右边也要只取 valid 的部分。
    #         另外 Z 这一列就是深度本身，原样放进去即可。
    points_cam = np.full((uv.shape[0], 3), np.nan)
    points_cam[valid] = np.column_stack(
        [(u[valid] - cx) * z[valid] / fx, (v[valid] - cy) * z[valid] / fy, z[valid]]
    )
    return points_cam, valid
    # raise NotImplementedError("TODO 1~4 还没完成")


if __name__ == "__main__":
    K = np.array(
        [
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )

    # ---- 自检 1：手算三个像素 ----
    # TODO: 先手算，把结果写在下面注释里，再跑代码对答案。
    #   像素 P1 = (370, 215)，深度 2.0
    #       X = (370 - 320) * 2.0 / 500 = 0.2
    #       Y = (215 - 240) * 2.0 / 500 = -0.1
    #       Z = 2.0
    #   像素 P2 = (320, 240)，深度 2.0   <- 主点上的像素，会还原成什么？
    #       X = (320 - 320) * 2.0 / 500 = 0.0
    #       Y = (240 - 240) * 2.0 / 500 = 0.0
    #       Z = 2.0
    #   也就是原点
    #   像素 P3 = (370, 215)，深度 0.0   <- 无效深度，应该得到什么？
    #       X = nan
    #       Y = nan
    #       Z = nan
    uv = np.array(
        [
            [370.0, 215.0],
            [320.0, 240.0],
            [370.0, 215.0],
        ]
    )
    depths = np.array([2.0, 2.0, 0.0])

    points_cam, valid = backproject_pixels(uv, depths, K)
    # TODO: 打印 points_cam.shape、points_cam、valid，逐条核对上面三个手算结果
    print("points_cam.shape:", points_cam.shape)
    print("points_cam:\n", points_cam)
    print("valid:", valid)
    # ---- 自检 2：往返一致性（本任务的重点）----
    # 思路：拿一批三维点 -> 投影成像素 -> 再反投影回三维点 ->
    #       看能不能回到原来的位置。这是阶段 4 一致性检查的雏形。
    #
    # 下面这段是样板代码：文件名以数字开头，不能用 import 语句直接导入，
    # 所以用 importlib 按文件路径加载。这段可以直接抄，不用自己研究。
    import importlib.util
    from pathlib import Path

    _here = Path(__file__).resolve().parent  # .../work
    _spec = importlib.util.spec_from_file_location(
        "project_points_module", _here / "02_project_points.py"
    )
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    project_points = _mod.project_points
    # 注意：如果任务 2 的 project_points 抛了 NotImplementedError，
    # 这里就会直接报错 —— 那说明需要先把任务 2 的 TODO 补完。
    #
    # TODO: 构造一组随机的有效三维点（比如 100 个，X/Y 在 -0.5~0.5，Z 在 1~3），
    #       依次做：
    #         uv2, valid2 = project_points(points, K)
    #         points_back, valid_back = backproject_pixels(uv2, points[:, 2], K)
    #       然后打印 points 和 points_back 的最大绝对误差。
    #       预期：应该在 1e-12 量级（浮点误差），而不是 0.1 这种量级。
    #       如果不是，先别改代码，想一想哪一步可能出了问题。
    np.random.seed(42)
    N = 100
    points = np.random.uniform(-0.5, 0.5, size=(N, 2))
    points_z = np.random.uniform(1.0, 3.0, size=(N, 1))
    points = np.hstack([points, points_z])
    uv2, valid2 = project_points(points, K)
    points_back, valid_back = backproject_pixels(uv2, points[:, 2], K)
    # 这里刻意不用 np.nanmax：nanmax 会「忽略」nan，
    # 万一有反投影失败的行，误差看起来依然很小，问题会被悄悄吞掉。
    # 先断言这批点全部有效，再用普通 max()——nan 一旦出现就会立刻暴露。
    assert valid2.all(), "投影阶段出现无效点，先查这一批输入"
    assert valid_back.all(), "反投影阶段出现无效点，先查两个函数的搭配"
    max_abs_error = np.abs(points - points_back).max()
    print("max_abs_error:", max_abs_error)
    print("  (1e-15 量级 = 浮点精度；若是 0.1 这种量级，说明某一步公式有问题)")

    # ---- 自检 3：量化「一个像素的误差值多少米」----
    # TODO: 取像素 P1 = (370, 215)，深度 2.0，先算出 X0。
    #       再把 u 加 0.5（半个像素）得到 X1，打印 |X1 - X0|。
    #       然后对深度 0.5 和 5.0 重复一遍。
    #       问题：深度越大，同样的半个像素误差对应的真实距离误差是变大还是变小？为什么？
    #    预期：深度越大，误差越大。因为投影时 X 被除以 Z 压平了，反投影时要乘回去。
    uv1 = np.array([[370.0, 215.0]])
    depths1 = np.array([2.0])
    points1, _ = backproject_pixels(uv1, depths1, K)
    uv1_half_pixel = np.array([[370.5, 215.0]])
    points1_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths1, K)
    error1 = np.abs(points1_half_pixel - points1)
    print("Depth 2.0, half-pixel error in meters:", error1)
    depths2 = np.array([0.5])
    points2, _ = backproject_pixels(uv1, depths2, K)
    points2_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths2, K)
    error2 = np.abs(points2_half_pixel - points2)
    print("Depth 0.5, half-pixel error in meters:", error2)
    depths3 = np.array([5.0])
    points3, _ = backproject_pixels(uv1, depths3, K)
    points3_half_pixel, _ = backproject_pixels(uv1_half_pixel, depths3, K)
    error3 = np.abs(points3_half_pixel - points3)
    print("Depth 5.0, half-pixel error in meters:", error3)

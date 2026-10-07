"""任务 2：把相机坐标系中的三维点投影成像素坐标。

运行方式：
    conda activate rgbd
    python work/02_project_points.py

你需要补完 project_points() 的函数体（TODO 1~5），以及文件末尾 __main__ 里的自检。
"""

import numpy as np


def project_points(points_cam, K):
    """把相机坐标系中的三维点投影为像素坐标。

    参数
    ----------
    points_cam : np.ndarray, shape (N, 3), dtype float64
        相机坐标系中的点，三列依次是 X, Y, Z，单位米。
        坐标系约定：X 向右、Y 向下、Z 沿光轴向前。
    K : np.ndarray, shape (3, 3), dtype float64
        内参矩阵：
            [[fx,  0, cx],
             [ 0, fy, cy],
             [ 0,  0,  1]]

    返回
    ----------
    uv : np.ndarray, shape (N, 2), dtype float64
        每个点的投影像素坐标 (u, v)。
        对 Z <= 0 的无效点，该行填 np.nan。
    valid : np.ndarray, shape (N,), dtype bool
        每个点是否有效（即 Z > 0）。

    公式（针孔模型，只在 Z > 0 时成立）：
        u = fx * X / Z + cx
        v = fy * Y / Z + cy
    """
    points_cam = np.asarray(points_cam, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64)

    # TODO 1: 从 K 里取出 fx, fy, cx, cy
    #         提示：K[0, 0] 是 fx，K[1, 1] 是 fy，K[0, 2] 是 cx，K[1, 2] 是 cy
    #         如果你发现自己写成了 K[2, 0] 之类，回头看看内参矩阵长什么样
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    # TODO 2: 从 points_cam 里取出 X, Y, Z 三列
    #         这三列分别在 points_cam 的哪一列？（提示：第 0 列、第 1 列、第 2 列）
    X, Y, Z = points_cam[:, 0], points_cam[:, 1], points_cam[:, 2]

    # TODO 3: 计算 valid = (Z > 0)，形状 (N,)
    valid = Z > 0

    # TODO 4: 只对 valid 的点算 u 和 v。
    #         要求：先算 valid 再做除法，这样不会出现除零的 RuntimeWarning。
    #         思路：先建一个全为 nan 的 uv 数组（形状 (N, 2)），
    #              再用 valid 做布尔索引，只给有效行赋值。
    #         注意：用布尔索引时，等号右边也要只取 valid 的部分，形状才能对上。
    uv = np.full((points_cam.shape[0], 2), np.nan)
    uv[valid] = np.column_stack(
        [fx * X[valid] / Z[valid] + cx, fy * Y[valid] / Z[valid] + cy]
    )

    # TODO 5: 返回 (uv, valid)
    return uv, valid
    # raise NotImplementedError("TODO 1~5 还没完成")


if __name__ == "__main__":
    # 内参：fx = fy = 500，主点 (320, 240)，对应 640x480 的图像
    K = np.array(
        [
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )

    # TODO: 先自己手算下面四个点，把结果写在注释里，再跑代码对答案。
    #   A = ( 0.2, -0.1, 2.0)   <- 有数值结果，必须手算
    # u = 500 * 0.2 / 2.0 + 320 = 370
    # v = 500 * -0.1 / 2.0 + 240 = 215
    #   B = ( 0.0,  0.0, 1.0)   <- 应该落在主点上
    # u = 500 * 0.0 / 1.0 + 320 = 320
    # v = 500 * 0.0 / 1.0 + 240 = 240
    #   C = ( 0.0,  0.0, -1.0)  <- 想想为什么不能投影
    # C点在光心后面
    #   D = ( 0.1,  0.05, 0.0)  <- 想想这里会发生什么
    # D点在光心上，Z=0，除零
    points = np.array(
        [
            [0.2, -0.1, 2.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.1, 0.05, 0.0],
        ]
    )

    uv, valid = project_points(points, K)

    # TODO: 打印 uv.shape、uv、valid，然后逐条核对：
    #   1) uv.shape 必须是 (4, 2)，valid.shape 必须是 (4,)
    #   2) B 的投影必须是 (320, 240)
    #   3) A 的 u 应该 > 320（X 为正），v 应该 < 240（Y 为负）—— 先预测再验证
    #   4) C、D 的 valid 必须是 False，对应 uv 行必须是 nan
    #   5) 运行过程不应出现 RuntimeWarning: divide by zero
    print("uv.shape:", uv.shape)
    print("uv:\n", uv)
    print("valid.shape:", valid.shape)
    print("valid:", valid)

"""检查像素坐标 (u, v) 与数组下标 [row, col] 的对应关系。"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# 输出目录由脚本位置推导，避免硬编码绝对路径。
OUT_DIR = Path(__file__).resolve().parents[1] / "outputs"

H, W = 4, 5
row = np.arange(H).reshape(H, 1)
col = np.arange(W).reshape(1, W)
depth = row * 10 + col

print(depth.shape)
print(depth.dtype)

print(depth[2, 3])
print("这对应像素坐标 u=3, v=2")

print(depth[3, 2])
print("这对应像素坐标 u=2, v=3")

print(depth[:, 3])
print("这是竖着的一列(u=3)")

print(depth[2, :])
print("这是横着的一行(v=2)")

plt.imshow(depth, cmap="viridis")
plt.savefig(OUT_DIR / "01_pixel_grid.png", dpi=120)

fake_rgb = np.random.rand(H, W, 3)
print(fake_rgb.shape)

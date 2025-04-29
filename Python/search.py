import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib import cm

# Параметры диапазонов
azimuth_range = (0, 337)
elevation_range = (0, 183)

# Шаги сканирования
az_step_deg = 30  # шаг по азимуту (увеличен в 2 раза)
el_step_deg = 15  # шаг по наклону

# Центры
az_steps = np.arange(azimuth_range[0], azimuth_range[1] + 1, az_step_deg)
el_steps = np.arange(elevation_range[0], elevation_range[1] + 1, el_step_deg)
az_c_idx = len(az_steps) // 2
el_c_idx = len(el_steps) // 2

# Построение координат спирали
def generate_spiral(az_steps, el_steps, center_idx):
    cx, cy = center_idx
    visited = set()
    path = []
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # право, вверх, влево, вниз
    dir_idx = 0
    x, y = cx, cy
    step_size = 1
    while True:
        for _ in range(2):
            dx, dy = directions[dir_idx % 4]
            for _ in range(step_size):
                if 0 <= x < len(az_steps) and 0 <= y < len(el_steps):
                    if (x, y) not in visited:
                        path.append((az_steps[x], el_steps[y]))
                        visited.add((x, y))
                x += dx
                y += dy
            dir_idx += 1
        step_size += 1
        if len(visited) >= len(az_steps) * len(el_steps):
            break
    return path

# Генерация маршрута
path = generate_spiral(az_steps, el_steps, (az_c_idx, el_c_idx))
x_vals, y_vals = zip(*path)

# Визуализация
fig, ax = plt.subplots(figsize=(12, 6))
points = np.arange(len(x_vals))
colors = cm.plasma(points / max(points))

for i in range(len(x_vals) - 1):
    ax.add_patch(FancyArrowPatch((x_vals[i], y_vals[i]), (x_vals[i+1], y_vals[i+1]),
                                  arrowstyle='->', mutation_scale=10, color=colors[i]))

# Стартовая и конечная точки
ax.plot(x_vals[0], y_vals[0], 'o', color='green', markersize=10, label='Start')
ax.plot(x_vals[-1], y_vals[-1], 'o', color='red', markersize=10, label='Finish')

# Настройки графика
ax.set_title("Spiral Scanning Path with Non-uniform Steps", fontsize=14)
ax.set_xlabel("Azimuth (degrees)", fontsize=12)
ax.set_ylabel("Elevation (degrees)", fontsize=12)
ax.set_aspect('equal', adjustable='box')
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_xlim(azimuth_range[0] - 10, azimuth_range[1] + 10)
ax.set_ylim(elevation_range[0] - 10, elevation_range[1] + 10)
ax.legend()
plt.tight_layout()
plt.show()

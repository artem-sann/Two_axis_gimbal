import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

# Настройки сетки
azimuth_range = (0, 337)
elevation_range = (0, 183)
az_step = 20  # уменьшен шаг
el_step = 10
az_grid = np.arange(azimuth_range[0], azimuth_range[1]+1, az_step)
el_grid = np.arange(elevation_range[0], elevation_range[1]+1, el_step)

# Источник сигнала — движется по синусоиде
def moving_signal_source(t):
    az = 170 + 100 * np.sin(0.05 * t)
    el = 90 + 40 * np.cos(0.07 * t)
    return az, el

# RSSI функция
def rssi_at(az, el, src_az, src_el):
    dist = np.sqrt((az - src_az)**2 + (el - src_el)**2)
    return -dist

# Локальные соседи
def get_neighbors(az, el):
    neighbors = []
    for da in [-az_step, 0, az_step]:
        for de in [-el_step, 0, el_step]:
            if da == 0 and de == 0:
                continue
            na, ne = az + da, el + de
            if azimuth_range[0] <= na <= azimuth_range[1] and elevation_range[0] <= ne <= elevation_range[1]:
                neighbors.append((na, ne))
    return neighbors

# Трекер
class SignalTracker:
    def __init__(self):
        self.az = 168
        self.el = 90
        self.history = [(self.az, self.el)]
        self.source_path = []

    def step(self, t):
        src_az, src_el = moving_signal_source(t)
        self.source_path.append((src_az, src_el))

        current_rssi = rssi_at(self.az, self.el, src_az, src_el)
        best_az, best_el = self.az, self.el
        best_rssi = current_rssi

        for na, ne in get_neighbors(self.az, self.el):
            rssi = rssi_at(na, ne, src_az, src_el)
            if rssi > best_rssi:
                best_rssi = rssi
                best_az, best_el = na, ne

        self.az, self.el = best_az, best_el
        self.history.append((self.az, self.el))

    def get_paths(self):
        return np.array(self.history), np.array(self.source_path)

# Запуск
tracker = SignalTracker()
frames = 100
for t in range(frames):
    tracker.step(t)

track_path, source_path = tracker.get_paths()

# Визуализация
fig, ax = plt.subplots(figsize=(10, 6))

# Траектория источника
for i in range(len(source_path)-1):
    ax.add_patch(FancyArrowPatch(
        (source_path[i,0], source_path[i,1]), (source_path[i+1,0], source_path[i+1,1]),
        arrowstyle='->', color='red', mutation_scale=8))

# Траектория трекера
for i in range(len(track_path)-1):
    ax.add_patch(FancyArrowPatch(
        (track_path[i,0], track_path[i,1]), (track_path[i+1,0], track_path[i+1,1]),
        arrowstyle='->', color='blue', mutation_scale=8))

# Конечные точки
ax.plot(source_path[0,0], source_path[0,1], 'rx', label='Signal Start', markersize=10)
ax.plot(track_path[0,0], track_path[0,1], 'go', label='Tracker Start', markersize=10)
ax.plot(source_path[-1,0], source_path[-1,1], 'rX', label='Signal End', markersize=10)
ax.plot(track_path[-1,0], track_path[-1,1], 'gP', label='Tracker End', markersize=10)

# Оформление
ax.set_xlim(azimuth_range[0], azimuth_range[1])
ax.set_ylim(elevation_range[0], elevation_range[1])
ax.set_title("Слежение за движущимся источником сигнала", fontsize=14)
ax.set_xlabel("1-я ось (°)", fontsize=12)
ax.set_ylabel("2-я ось (°)", fontsize=12)
ax.grid(True, linestyle='--', alpha=0.5)
ax.legend(loc='upper right')
ax.set_aspect('equal')
plt.tight_layout()
plt.show()


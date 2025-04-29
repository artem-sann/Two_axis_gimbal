import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Настройки сетки
azimuth_range = (0, 337)
elevation_range = (0, 183)
az_step = 30
el_step = 15
az_grid = np.arange(azimuth_range[0], azimuth_range[1]+1, az_step)
el_grid = np.arange(elevation_range[0], elevation_range[1]+1, el_step)

# RSSI источник — движется по синусоиде
def moving_signal_source(t):
    az = 170 + 100 * np.sin(0.05 * t)
    el = 90 + 40 * np.cos(0.07 * t)
    return az, el

# RSSI функция (максимум в источнике, убывает по расстоянию)
def rssi_at(az, el, src_az, src_el):
    dist = np.sqrt((az - src_az)**2 + (el - src_el)**2)
    return -dist  # чем ближе, тем выше (менее отрицательное значение)

# Локальный поиск вокруг точки
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

# Алгоритм слежения
class SignalTracker:
    def __init__(self):
        self.az = 168  # старт
        self.el = 90
        self.history = [(self.az, self.el)]

    def step(self, t):
        src_az, src_el = moving_signal_source(t)
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

        return self.az, self.el, src_az, src_el

# Визуализация
tracker = SignalTracker()
fig, ax = plt.subplots(figsize=(10, 6))

def update(frame):
    ax.clear()
    az, el, src_az, src_el = tracker.step(frame)
    history = np.array(tracker.history)

    ax.plot(history[:,0], history[:,1], 'b.-', label='Tracking path')
    ax.plot(src_az, src_el, 'rx', markersize=10, label='Signal Source')
    ax.plot(az, el, 'go', label='Tracker Position')

    ax.set_xlim(azimuth_range[0], azimuth_range[1])
    ax.set_ylim(elevation_range[0], elevation_range[1])
    ax.set_title(f"Signal Tracking Frame {frame}")
    ax.set_xlabel("Azimuth (°)")
    ax.set_ylabel("Elevation (°)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='upper right')
    ax.set_aspect('equal')

ani = FuncAnimation(fig, update, frames=100, interval=200)
plt.show()
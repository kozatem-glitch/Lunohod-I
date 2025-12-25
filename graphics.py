import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
import math

# -------------------------------------------------
# 1. ЗАГРУЗКА ДАННЫХ ИЗ KSP
# -------------------------------------------------
ksp_time = []
ksp_altitude = []
ksp_speed = []
ksp_mass = []

with open("ksp.txt", encoding="UTF-8") as ksp_data:
    for i, line in enumerate(ksp_data):
        if i > 0:  
            data = line.rstrip().split(',')
            ksp_time.append(float(data[0]))
            ksp_altitude.append(float(data[1]))
            ksp_speed.append(float(data[4]))
            ksp_mass.append(float(data[7]))

# -------------------------------------------------
# 2. ПАРАМЕТРЫ
# -------------------------------------------------
mu = 3.5316e12  # гравитационный параметр Кербина (м³/с²)
R = 600000.0  # радиус Кербина (м)
G = 6.67430e-11 #гравитационная постоянная (Н*м^2/кг^2)
g0 = 9.80665  # стандартное ускорение свободного падения (м/с²)
Mk = 5.2915e22  # масса Кербина (кг)
Isp = 295.0  # удельный импульс (с)
T = 5.619e6  # тяга (Н)
d = 5.9  # диаметр ракеты (м)
C_d = 0.5  # коэффициент лобового сопротивления
S = 0.25 * math.pi * d ** 2  # площадь поперечного сечения (м²)
rho0 = 1.225  # плотность атмосферы на уровне моря (кг/м³)
H = 5000.0  # масштаб высоты атмосферы (м)

# -------------------------------------------------
# 3. НАЧАЛЬНЫЕ УСЛОВИЯ
# -------------------------------------------------
x0 = 0.0  # начальная горизонтальная координата (м)
y0 = R  # начальная вертикальная координата = радиус Кербина
v_x0 = 0.0  # начальная горизонтальная скорость (м/с)
v_y0 = 0.0  # начальная вертикальная скорость (м/с)
m0 = 370700.0  # начальная масса (кг) 


# -------------------------------------------------
# 4. УГОЛ ТАНГАЖА
# -------------------------------------------------
def pitch_profile(h):
    """Угол тангажа в градусах от горизонта (0° = горизонтально, 90° = вертикально)"""
    if h <= 10000:
        return 90.0
    elif h <= 20000:
        fraction = (h - 10000) / 10000
        return 90.0 - fraction * 60.0
    elif h <= 70000:
        fraction = (h - 20000) / 50000
        return 30.0 - fraction * 30.0
    else:
        return 0.0


# -------------------------------------------------
# 5. ПЛОТНОСТЬ АТМОСФЕРЫ
# -------------------------------------------------
def atmosphere_density(h):
    """Плотность атмосферы на высоте h (м)"""
    if h < 0:
        h = 0
    if h > 70000:  # атмосфера заканчивается на 70 км
        return 0.0
    return rho0 * np.exp(-h / H)


# -------------------------------------------------
# 6. МОДЕЛЬ ПОЛЁТА (ИСПРАВЛЕННАЯ)
# -------------------------------------------------
def ascent_model(t, state):
    x, y, vx, vy, m = state

    # Текущая высота над поверхностью
    r = np.sqrt(x ** 2 + y ** 2)
    h = r - R

    # Текущая скорость
    v = np.sqrt(vx ** 2 + vy ** 2)
    if v < 1e-6:
        v = 1e-6

    # Угол тангажа
    pitch_deg = pitch_profile(h)
    pitch_rad = np.radians(pitch_deg)

    # Определяем, работает ли двигатель
    if t <= 126.0:
        engine_on = True
        thrust = T
    elif 126.0 < t < 245.8:
        engine_on = False
        thrust = 0.0
    else:
        engine_on = True
        thrust = T

    # Компоненты тяги
    thrust_x = thrust * np.cos(pitch_rad) if engine_on else 0.0
    thrust_y = thrust * np.sin(pitch_rad) if engine_on else 0.0

    # Гравитация
    r3 = r ** 3
    g_scale = mu / r3
    grav_x = -g_scale * x
    grav_y = -g_scale * y

    # Аэродинамическое сопротивление
    rho = atmosphere_density(h)
    F_drag = 0.5 * rho * v ** 2 * C_d * S
    drag_x = -F_drag * vx / v
    drag_y = -F_drag * vy / v

    # Суммарные ускорения
    ax = (thrust_x + drag_x) / m + grav_x
    ay = (thrust_y + drag_y) / m + grav_y

    # Расход массы
    if engine_on and m > 96300:
        dmdt = -thrust / (Isp * g0)
    else:
        dmdt = 0.0

    # Производные: dx/dt = vx, dy/dt = vy, dvx/dt = ax, dvy/dt = ay, dm/dt = dmdt
    return [vx, vy, ax, ay, dmdt]


# -------------------------------------------------
# 7. ЧИСЛЕННОЕ ИНТЕГРИРОВАНИЕ
# -------------------------------------------------
t_end = 280.0
initial_state = [x0, y0, v_x0, v_y0, m0]

solution = solve_ivp(
    ascent_model,
    [0, t_end],
    initial_state,
    method='RK45',
    dense_output=True,
    max_step=0.1,
    rtol=1e-8,
    atol=1e-10
)

# Извлечение результатов
t_mod = solution.t
x_mod = solution.y[0]
y_mod = solution.y[1]
vx_mod = solution.y[2]
vy_mod = solution.y[3]
m_mod = solution.y[4]

# Вычисление производных величин
r_mod = np.sqrt(x_mod ** 2 + y_mod ** 2)
h_mod = r_mod - R  # высота над поверхностью
v_mod = np.sqrt(vx_mod ** 2 + vy_mod ** 2)  # модуль скорости

# -------------------------------------------------
# 8. ВЫВОД РЕЗУЛЬТАТОВ
# -------------------------------------------------
print("=== РЕЗУЛЬТАТЫ МОДЕЛИ ===")
print(f"Максимальная скорость: {np.max(v_mod):.1f} м/с")
print(f"Конечная скорость: {v_mod[-1]:.1f} м/с")
print(f"Максимальная высота: {np.max(h_mod):.1f} м")
print(f"Конечная высота: {h_mod[-1]:.1f} м")
print(f"Конечная масса: {m_mod[-1]:.1f} кг")

# -------------------------------------------------
# 9. ПОСТРОЕНИЕ ГРАФИКОВ
# -------------------------------------------------
plt.figure(figsize=(15, 4))

plt.subplot(1, 3, 1)
plt.plot(ksp_time, ksp_altitude, 'b-', label='KSP (эксперимент)', linewidth=1.5, alpha=0.7)
plt.plot(t_mod, h_mod, 'r--', label='Модель (теория)', linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Высота, м")
plt.title("Высота над Кербином")
plt.grid(True, alpha=0.3)
plt.legend()

plt.subplot(1, 3, 2)
plt.plot(ksp_time, ksp_speed, 'b-', label='KSP', linewidth=1.5, alpha=0.7)
plt.plot(t_mod, v_mod, 'r--', label='Модель', linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Скорость, м/с")
plt.title("Модуль скорости")
plt.grid(True, alpha=0.3)
plt.legend()

plt.subplot(1, 3, 3)
plt.plot(ksp_time, ksp_mass, 'b-', label='KSP', linewidth=1.5, alpha=0.7)
plt.plot(t_mod, m_mod, 'r--', label='Модель', linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Масса, кг")
plt.title("Масса ракеты")
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.show()

# sim_kalman_parachute.py
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# ---------- Parâmetros de simulação ----------
dt = 0.02                # s
t_final = 60.0           # s
t = np.arange(0, t_final, dt)
n = len(t)

g = 9.80665              # m/s^2
target_apogee = 1000.0   # m

# motor to reach approx apogee
v0 = np.sqrt(2 * g * target_apogee)
t_thrust = 2.0           # s
a_thrust = v0 / t_thrust

# ---------- Flight model (1D vertical) ----------
acc_true = np.zeros(n)
vel_true = np.zeros(n)
alt_true = np.zeros(n)
for i in range(1, n):
    time = t[i]
    if time <= t_thrust:
        acc_true[i] = a_thrust
    else:
        acc_true[i] = -g
    vel_true[i] = vel_true[i-1] + acc_true[i-1] * dt
    alt_true[i] = max(alt_true[i-1] + vel_true[i-1] * dt + 0.5 * acc_true[i-1] * dt**2, 0.0)

idx_apogee_true = np.argmax(alt_true)
time_apogee_true = t[idx_apogee_true]
alt_apogee_true = alt_true[idx_apogee_true]

# ---------- Sensor noise model (BMP280-like) ----------
rng = np.random.default_rng(2025)
NOISE_START_ALT = 200.0   # m (as suggested)
base_sigma_baro = 3.5
alpha_sigma = 1.2
base_spike_prob = 0.0005
max_spike_prob = 0.01
spike_mean = 50.0
spike_std = 25.0

baro_noise = np.zeros(n)
alt_baro = np.zeros(n)
for k in range(n):
    alt_k = alt_true[k]
    if alt_k < NOISE_START_ALT:
        sigma_k = 0.2  # pouco ruído no solo
        g_noise = rng.normal(0, sigma_k)
        spike = 0.0
    else:
        frac = (alt_k - NOISE_START_ALT) / max(1.0, (target_apogee - NOISE_START_ALT))
        frac = np.clip(frac, 0.0, 1.0)
        sigma_k = base_sigma_baro * (1.0 + alpha_sigma * frac)
        g_noise = rng.normal(0, sigma_k)
        p_spike = base_spike_prob + (max_spike_prob - base_spike_prob) * frac
        spike = 0.0
        if rng.random() < p_spike:
            spike = rng.normal(spike_mean, spike_std) * (0.5 + rng.random())
    baro_noise[k] = g_noise + spike
    alt_baro[k] = alt_k + baro_noise[k]

# ---------- Accelerometer noise (MPU6050-like) ----------
sigma_acc = 0.7
bias_walk = np.cumsum(rng.normal(0, 0.0008, n))
acc_bias = 0.05 * rng.normal()
acc_noise = rng.normal(0, sigma_acc, n) + acc_bias + bias_walk
acc_meas = acc_true + acc_noise  # in m/s^2

# ---------- Parameters aligned with parachute.h ----------
ALT_MOVING_AVG_N = 5
APOGEE_CONFIRM_M = 4
KQ00 = 0.05
KQ11 = 0.3
KR = 9.0                   # variance (sigma^2)
V_NEG_THRESHOLD = -0.2
HEIGHT_FOR_2_STAGE = 3.0
TIME_BETWEEN_ACTIVATIONS = 8000  # ms
SKIB_TIME = 1000                 # ms
SAFE_MARGIN_ALTITUDE_ERROR = 1.0

# ---------- Kalman 2x2 initialization ----------
k_x_alt = alt_baro[0]
k_x_vel = 0.0
k_P00, k_P01, k_P10, k_P11 = 10.0, 0.0, 0.0, 10.0

def kalman_step(z_alt, a_meas, dt):
    global k_x_alt, k_x_vel, k_P00, k_P01, k_P10, k_P11
    if dt <= 0: dt = 0.02
    # predict
    alt_pred = k_x_alt + k_x_vel * dt + 0.5 * a_meas * dt * dt
    vel_pred = k_x_vel + a_meas * dt
    P00p = k_P00 + (k_P01 + k_P10) * dt + k_P11 * dt * dt + KQ00
    P01p = k_P01 + k_P11 * dt
    P10p = k_P10 + k_P11 * dt
    P11p = k_P11 + KQ11
    # update
    S = P00p + KR
    K0 = P00p / S
    K1 = P10p / S
    y = z_alt - alt_pred
    k_x_alt = alt_pred + K0 * y
    k_x_vel = vel_pred + K1 * y
    # update P
    P00n = (1.0 - K0) * P00p
    P01n = (1.0 - K0) * P01p
    P10n = -K1 * P00p + P10p
    P11n = -K1 * P01p + P11p
    k_P00, k_P01, k_P10, k_P11 = P00n, P01n, P10n, P11n
    return k_x_alt, k_x_vel

# ---------- Moving average helper ----------
alt_mov_buf = np.zeros(ALT_MOVING_AVG_N)
alt_mov_pos = 0
alt_mov_count = 0
def moving_avg_update(x):
    global alt_mov_pos, alt_mov_count
    alt_mov_buf[alt_mov_pos] = x
    alt_mov_pos = (alt_mov_pos + 1) % ALT_MOVING_AVG_N
    if alt_mov_count < ALT_MOVING_AVG_N:
        alt_mov_count += 1
    return alt_mov_buf[:alt_mov_count].mean()

# ---------- Run simulation and parachute logic ----------
estimated_alt = []
estimated_vel = []
vel_neg_count = 0
apogee_detected = False
apogee_time = None
apogee_alt_kf = None

parachute1_armed = False
parachute2_armed = False
time_stage1 = None
time_stage2 = None
enough_height = True

t_plot = []
alt_true_plot = []
vel_true_plot = []
alt_baro_plot = []
alt_kalman_plot = []
vel_kalman_plot = []

for k in range(n):
    dt_k = dt if k == 0 else t[k] - t[k-1]
    baro = alt_baro[k]
    acc = acc_meas[k]
    
    # Moving average no barômetro
    baro_f = moving_avg_update(baro)
    
    # Kalman update
    alt_kf, vel_kf = kalman_step(baro_f, acc, dt_k)
    
    # Salvar dados até agora
    t_plot.append(t[k])
    alt_true_plot.append(alt_true[k])
    vel_true_plot.append(vel_true[k])
    alt_baro_plot.append(baro)
    alt_kalman_plot.append(alt_kf)
    vel_kalman_plot.append(vel_kf)
    
    # Lógica de apogeu
    if vel_kf < 0 and (k == 0 or estimated_vel[-1] >= 0):
        vel_neg_count = 1
    elif vel_kf < 0:
        vel_neg_count += 1
    else:
        vel_neg_count = 0
    if (not apogee_detected) and (vel_neg_count >= APOGEE_CONFIRM_M):
        apogee_detected = True
        apogee_time = t[k - (APOGEE_CONFIRM_M-1)]
        apogee_alt_kf = alt_kalman_plot[-APOGEE_CONFIRM_M]
        parachute1_armed = True
        time_stage1 = int(apogee_time*1000)
    
    # Stage activation
    if parachute1_armed and (not parachute2_armed):
        max_alt_seen = max(alt_true_plot)
        enough_height = (max_alt_seen - HEIGHT_FOR_2_STAGE > SAFE_MARGIN_ALTITUDE_ERROR)
        if enough_height and alt_kalman_plot[-1] < HEIGHT_FOR_2_STAGE:
            parachute2_armed = True
            time_stage2 = int(t[k]*1000)
        elif (not enough_height) and (time_stage1 is not None) and ((int(t[k]*1000) - time_stage1) > TIME_BETWEEN_ACTIVATIONS):
            parachute2_armed = True
            time_stage2 = int(t[k]*1000)
    
    # Parar simulação quando tocar o chão
    if alt_true[k] <= 0 and k > 5:
        break
    
    estimated_alt.append(alt_kf)
    estimated_vel.append(vel_kf)

# Plot altitude
# Marcar apogeu verdadeiro
idx_apogee_true = np.argmax(alt_true_plot)
time_apogee_true = t_plot[idx_apogee_true]
alt_apogee_true = alt_true_plot[idx_apogee_true]

# Plot altitude
plt.figure()
plt.plot(t_plot, alt_true_plot, label='Altitude verdadeira')
plt.plot(t_plot, alt_baro_plot, label='Altitude medida (barômetro ruidoso)', alpha=0.6)
plt.plot(t_plot, alt_kalman_plot, label='Altitude estimada (Kalman)')
plt.axvline(time_apogee_true, color='g', linestyle='--', label='Apogeu real')
if apogee_detected:
    plt.axvline(apogee_time, color='r', linestyle=':', label='Apogeu Kalman')
plt.xlabel('Tempo (s)')
plt.ylabel('Altitude (m)')
plt.title('Altitude: verdadeira vs medida vs Kalman')
plt.legend()
plt.grid(True)

# Plot velocidade
plt.figure()
plt.plot(t_plot, vel_true_plot, label='Velocidade verdadeira')
plt.plot(t_plot, vel_kalman_plot, label='Vel. estimada (Kalman)')
plt.axvline(time_apogee_true, color='g', linestyle='--', label='Apogeu real')
if apogee_detected:
    plt.axvline(apogee_time, color='r', linestyle=':', label='Detecção Kalman')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (m/s)')
plt.title('Velocidade: verdadeira vs Kalman')
plt.legend()
plt.grid(True)

# Plot zoom no apogeu
zoom_start = max(0, time_apogee_true - 2)
zoom_end = min(t_plot[-1], time_apogee_true + 2)
mask_zoom = (np.array(t_plot) >= zoom_start) & (np.array(t_plot) <= zoom_end)

plt.figure()
plt.plot(np.array(t_plot)[mask_zoom], np.array(alt_true_plot)[mask_zoom], label='Altitude verdadeira')
plt.plot(np.array(t_plot)[mask_zoom], np.array(alt_baro_plot)[mask_zoom], label='Altitude medida (barômetro ruidoso)', alpha=0.6)
plt.plot(np.array(t_plot)[mask_zoom], np.array(alt_kalman_plot)[mask_zoom], label='Altitude estimada (Kalman)')
plt.axvline(time_apogee_true, color='g', linestyle='--', label='Apogeu real')
if apogee_detected:
    plt.axvline(apogee_time, color='r', linestyle=':', label='Apogeu Kalman')
plt.xlabel('Tempo (s)')
plt.ylabel('Altitude (m)')
plt.title('Zoom no Apogeu')
plt.legend()
plt.grid(True)

plt.show()
import krpc
import time
import os
import csv


conn = krpc.connect(name='Lunohod mission - Fly Me To The Moon')
vessel = conn.space_center.active_vessel
auto_pilot = vessel.auto_pilot


turn_start_alt = 250
turn_end_alt = 45000
target_ap = 100000
roll = 90
pitch0, pitch1 = 90, 15

flight = vessel.flight()
orbit = vessel.orbit
altitude = conn.add_stream(getattr, flight, 'mean_altitude')
apoapsis = conn.add_stream(getattr, orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr, orbit, 'periapsis_altitude')
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
mass = conn.add_stream(getattr, vessel, 'mass')
dyn_press = conn.add_stream(getattr, flight, 'dynamic_pressure')


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "flight_data.csv")

with open(CSV_FILE, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["time", "altitude", "apoapsis", "periapsis", "speed", "pitch", "throttle", "mass", "dynamic_pressure"])
start_t = time.time()

def data():
    cur_t = time.time() - start_t
    with open(CSV_FILE, "a", newline="") as file_1:
        w = csv.writer(file_1)
        w.writerow([
            round(cur_t, 2),
            round(altitude(), 2),
            round(apoapsis(), 2),
            round(periapsis(), 2),
            round(speed(), 2),
            vessel.flight().pitch,
            vessel.control.throttle,
            round(mass(), 2),
            round(dyn_press(), 2)
        ])



vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1

auto_pilot.engage()
auto_pilot.target_pitch_and_heading(pitch0, roll)

time.sleep(5)
print("Запуск")
vessel.control.activate_next_stage()
time.sleep(1)

while True:
    h = altitude()
    data()

    if turn_start_alt < h < turn_end_alt:
        k = (h - turn_start_alt) / (turn_end_alt - turn_start_alt)
        new_pitch = pitch0 - k * (pitch0 - pitch1)
        auto_pilot.target_pitch_and_heading(new_pitch, roll)

    if apoapsis() >= target_ap:
        vessel.control.throttle = 0
        break
    time.sleep(0.1)

print("Ожидание апоцентра")
while abs(apoapsis() - altitude()) > 500:
    data()
    time.sleep(0.1)

print("Округление орбиты")
auto_pilot.target_pitch_and_heading(0, roll)
vessel.control.throttle = 1
while periapsis() < target_ap:
    data()
    time.sleep(0.1)

vessel.control.throttle = 0
print("Орбита построена")
time.sleep(2)

print("Отделение ступени")
vessel.control.activate_next_stage()
time.sleep(1)
vessel.control.throttle = 0.2
time.sleep(0.1)
vessel.control.throttle = 0


time.sleep(1)
print("Миссия завершена")

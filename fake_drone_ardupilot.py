from pymavlink import mavutil
import time
import math

# Визначення змінних для IP та порту
ip = '192.168.88.253'
port = '14550'

# Створення MAVLink-з'єднання
master = mavutil.mavlink_connection(f'udpout:{ip}:{port}')

lat = 51.19080 * 1e7  # Початкова широта (Бахмач)
lon = 32.82809 * 1e7  # Початкова довгота
alt = 1000 * 1000     # 1000 м у мм

t = 0
start_time = time.monotonic()

# Радіус Землі для переведення метрів у градуси
R_earth = 6378137  # м

# Параметри для змійки
speed = 36.11  # м/с
angle_rad = math.radians(160)  # кут для компонент швидкості
vx = int(-speed * math.cos(angle_rad) * 100)  # см/с
vy = int(-speed * math.sin(angle_rad) * 100)  # см/с
vz = 0  # вертикальна швидкість
lat_initial = 51.19080  # градуси
lon_initial = 32.82809  # градуси
# Амплітуда для зміщення: 3 км (1 км вліво + 2 км вправо) = 0.0428° на широті 51.19080°
amplitude = 0.0214  # половина 0.0428°, оскільки синус коливається в обидва боки
# Період: 3 км по траєкторії за 36.11 м/с ≈ 83 с
omega = 2 * math.pi / 83  # кутова частота (рад/с)

try:
    while True:
        # HEARTBEAT
        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

        # SYS_STATUS (12.5 В, ~75% батареї)
        master.mav.sys_status_send(
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            load=500,                   # CPU load (0.1%)
            voltage_battery=12500,      # мВ
            current_battery=2000,       # мА
            battery_remaining=75,       # %
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=0,
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )

        # GLOBAL_POSITION_INT (рух із змійкою)
        # Лінійна зміна широти
        lat_sim = int((lat_initial + (vy / 100.0 * t / R_earth) * (180 / math.pi)) * 1e7)
        # Синусоїдальне зміщення для довготи
        lon_offset = amplitude * math.sin(omega * t)  # зміщення в градусах
        lon_sim = int((lon_initial + (vx / 100.0 * t / R_earth / math.cos(math.radians(lat_initial))) * (180 / math.pi) + lon_offset) * 1e7)

        # Обчислення динамічного курсу
        # Базова швидкість: vx, vy (см/с)
        # Латеральна швидкість від змійки: похідна від lon_offset
        lon_offset_m = lon_offset * (R_earth * math.cos(math.radians(lat_initial)) * math.pi / 180)  # зміщення в метрах
        v_lon = amplitude * omega * math.cos(omega * t) * (R_earth * math.cos(math.radians(lat_initial)) * math.pi / 180)  # м/с
        v_lon_cm = v_lon * 100  # см/с
        vx_total = vx + v_lon_cm  # загальна швидкість по x (см/с)
        vy_total = vy  # швидкість по y без змін
        # Курс: arctan2(vx_total, vy_total) для відповідності MAVLink (0° = північ, позитивний за годинниковою)
        hdg_rad = math.atan2(vx_total, vy_total) % (2 * math.pi)  # нормалізуємо до [0, 2π)
        hdg = int((math.degrees(hdg_rad) % 360) * 100)  # курс у cdeg
        yaw = hdg_rad  # yaw у радіанах для attitude

        # Обмежуємо time_boot_ms
        time_boot_ms = int((time.monotonic() - start_time) * 1000) % 4294967295

        master.mav.global_position_int_send(
            time_boot_ms,      # час у мс від старту
            lat_sim,           # широта (у 1e7 градусів)
            lon_sim,           # довгота (у 1e7 градусів)
            alt,               # висота над морем (мм)
            alt,               # відносна висота (мм)
            vx, vy,            # vx, vy (см/с, базові швидкості)
            vz,                # vz
            hdg                # динамічний курс (cdeg)
        )

        # ATTITUDE (імітація невеликого похитування з динамічним yaw)
        master.mav.attitude_send(
            int((time.monotonic() - start_time) * 1e6) % 4294967295,  # час у мікросекундах
            math.sin(t / 10.0) * 0.1,  # roll
            math.cos(t / 10.0) * 0.1,  # pitch
            yaw,                       # динамічний yaw (радіани)
            0, 0, 0                    # rollspeed, pitchspeed, yawspeed
        )

        # Логування для дебагінгу
        print(f"time_boot_ms: {time_boot_ms}, lat: {lat_sim/1e7:.7f}, lon: {lon_sim/1e7:.7f}, hdg: {hdg/100:.1f}, vx: {vx}, vy: {vy}, alt: {alt/1000:.1f}m")
        t += 0.1
        time.sleep(0.100)  # Затримка 100 мс

except KeyboardInterrupt:
    print("Програма зупинена користувачем")
    master.close()  # Закриваємо з'єднання
    exit(0)
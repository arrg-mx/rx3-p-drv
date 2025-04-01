#!/usr/bin/env python3
# coding: utf-8

from rosboard_drv.rosboard_drv import RosBoardDrv
import time


def main():
    bot = RosBoardDrv(com="/dev/ttyUSB0", debug=True, logpath="./logs")

    bot.create_receive_threading()
    time.sleep(0.1)
    bot.set_beep(500)
    time.sleep(0.1)

    version = bot.get_version()
    print("version=", version)

    s_id, value = bot.get_uart_servo_value(1)
    print("value:", s_id, value)

    bot.set_uart_servo_torque(1)
    time.sleep(0.1)

    state = bot.set_uart_servo_offset(1)
    print("state=", state)

    s_id, value = bot.get_uart_servo_value(1)
    angle = bot.get_uart_servo_angle(1)
    print(f"Servo ID({s_id}): angle {angle}, value {value}")

    for i in range(1, 6):
        servo_id = i
        s_angle = 0
        read_servo_ang = bot.get_uart_servo_angle(servo_id)
        read_servo_val = bot.get_uart_servo_value(servo_id)
        print(f"Testing servo [{servo_id}]...")
        if read_servo_ang < 0:
            print(f"Servo [{servo_id}]: Not found.")
        else:
            print(
                f"Servo[{servo_id}] -> current [angle, value]: [{read_servo_ang}, {read_servo_val}]"
            )
            bot.set_uart_servo_angle(servo_id, s_angle)
            time.sleep(1)
            s_angle = 90
            print(f"Servo[{servo_id}] -> go to: {s_angle}")
            bot.set_uart_servo_angle(servo_id, s_angle)
            time.sleep(0.5)
            read_servo_ang = bot.get_uart_servo_angle(servo_id)
            read_servo_val = bot.get_uart_servo_value(servo_id)
            print(
                f"Servo[{servo_id}] -> current [angle, value]: [{read_servo_ang}, {read_servo_val}]"
            )
            time.sleep(1)
            s_angle = 45
            print(f"Servo[{servo_id}] -> go to: {s_angle}")
            bot.set_uart_servo_angle(servo_id, s_angle)
            time.sleep(0.5)
            read_servo_ang = bot.get_uart_servo_angle(servo_id)
            read_servo_val = bot.get_uart_servo_value(servo_id)
            print(
                f"Servo[{servo_id}] -> current [angle, value]: [{read_servo_ang}, {read_servo_val}]"
            )
            time.sleep(1)
            s_angle = 180
            print(f"Servo[{servo_id}] -> go to: {s_angle}")
            bot.set_uart_servo_angle(servo_id, s_angle)
            time.sleep(0.5)
            read_servo_ang = bot.get_uart_servo_angle(servo_id)
            read_servo_val = bot.get_uart_servo_value(servo_id)
            print(
                f"Servo[{servo_id}] -> current [angle, value]: [{read_servo_ang}, {read_servo_val}]"
            )
            time.sleep(1)
            s_angle = 0
            print(f"Servo[{servo_id}] -> go to: {s_angle}")
            bot.set_uart_servo_angle(servo_id, s_angle)
            time.sleep(0.5)
            read_servo_ang = bot.get_uart_servo_angle(servo_id)
            read_servo_val = bot.get_uart_servo_value(servo_id)
            print(
                f"Servo[{servo_id}] -> current [angle, value]: [{read_servo_ang}, {read_servo_val}]"
            )
            time.sleep(1)
            print(f"Servo[{servo_id}] -> set_torque: False")
            bot.set_uart_servo_torque(False)
            time.sleep(1)
            print(f"Servo[{servo_id}] -> set_torque: True")
            bot.set_uart_servo_torque(True)

    print("Cooling down 3 sec....")
    time.sleep(3)
    dcmotor_pwd = 0
    print("Testing DC motors...")
    print(f"Setting all DC motor to {dcmotor_pwd}%")
    bot.set_motor(dcmotor_pwd, dcmotor_pwd, 0, 0)
    dcmotor_pwd = 45
    print(f"Setting DC motor(1) to {dcmotor_pwd}%")
    bot.set_motor(dcmotor_pwd, 0, 0, 0)
    time.sleep(2)
    print("Setting all DC motor to 0%")
    bot.set_motor(0, 0, 0, 0)
    print(f"Setting DC motor(2) to {dcmotor_pwd}%")
    bot.set_motor(0, dcmotor_pwd, 0, 0)
    time.sleep(2)
    print("Setting all DC motor to 0%")
    bot.set_motor(0, 0, 0, 0)
    print(f"Setting DC motor(1 and 2) to {dcmotor_pwd}% and -{dcmotor_pwd}%")
    bot.set_motor(dcmotor_pwd, -dcmotor_pwd, 0, 0)
    time.sleep(2)
    print("Setting all DC motor to 0%")
    bot.set_motor(0, 0, 0, 0)
    print("Testing DC motors... Finished")
    time.sleep(2)
    print("Cooling down 3 seg....")
    time.sleep(3)
    print("Testing car motion...")
    bot.set_car_motion(0, 0, -3.5)
    bot.set_car_run(6, 50)
    try:
        seconds = 3
        print(f"Getting data for ({seconds}) secs...")
        starttime = time.time()
        while True:
            now = time.time()
            if now > starttime + seconds:
                break

            ax, ay, az = bot.get_accelerometer_data()
            gx, gy, gz = bot.get_gyroscope_data()
            mx, my, mz = bot.get_magnetometer_data()
            print(
                "ACC: %3.3f, %3.3f, %3.3f, GYRO: %3.3f, %3.3f, %3.3f"
                % (ax, ay, az, gx, gy, gz)
            )

            roll, pitch, yaw = bot.get_imu_attitude_data()
            print("roll:%f, pitch:%f, yaw:%f" % (roll, pitch, yaw))
            m1, m2, m3, m4 = bot.get_motor_encoder()
            print("encoder:", m1, m2, m3, m4)

            v = bot.get_motion_data()
            print("v:", v)

            pid = bot.get_motion_pid()
            print(pid)

            vx, vy, vz = bot.get_motion_data()
            print("V:", vx, vy, vz)
            time.sleep(0.1)

        bot.set_car_motion(0, 0, 0)

    except KeyboardInterrupt:
        bot.set_car_motion(0, 0, 0)
        pass
    exit(0)


if __name__ == "__main__":
    main()

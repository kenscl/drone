import serial
import numpy as np
import matplotlib.pyplot as plt
import time

port = '/dev/ttyUSB0'  
baudrate = 115200 

ser = serial.Serial(port, baudrate)

last_acc = list() 
last_gyro = np.array([0., 0., 0.])
last_mag = np.array([0., 0., 0.])
acc = list() 
gyro = list()
mag = list()
time_stamp_gyro = list()
time_stamp_acc = list()
time_stamp_mag = list()
start = time.time()

acc_x_data, acc_y_data, acc_z_data = [], [], []
gyro_x_data, gyro_y_data, gyro_z_data = [], [], []
mag_x_data, mag_y_data, mag_z_data = [], [], []

plot_gyro = False 
plot_acc = True
plot_mag = False

try:
    try:
        while (True):
            data = ser.readline().decode('utf-8').strip().replace("\x00", "")
            try:
                sensor, x, y, z = data.split(",")
                now = time.time() - start
                if str(sensor) == str('LSM9DS1_acc'):
                    last_acc = np.array([float(x), float(y), float(z)])
                    acc_x_data.append(last_acc[0])
                    acc_y_data.append(last_acc[1])
                    acc_z_data.append(last_acc[2])
                    time_stamp_acc.append(now)

                elif str(sensor) == str('LSM9DS1_gyro'):
                    last_gyro = np.array([float(x), float(y), float(z)])
                    gyro_x_data.append(last_gyro[0])
                    gyro_y_data.append(last_gyro[1])
                    gyro_z_data.append(last_gyro[2])
                    time_stamp_gyro.append(now)

                elif str(sensor) == str('LSM9DS1_mag'):
                    last_mag = np.array([float(x), float(y), float(z)])
                    mag_x_data.append(last_mag[0])
                    mag_y_data.append(last_mag[1])
                    mag_z_data.append(last_mag[2])
                    time_stamp_mag.append(now)

                print("\033c", end="") # clear terminal
                print("Time: ", now)
                print("Accelerometer: ", last_acc)
                print("Gyroscope: ", last_gyro)
                print("Magnetometer: ", last_mag)
            except ValueError:
                pass
    except KeyboardInterrupt:
        pass 
    ser.close()

    plt.figure(figsize=(12, 8))
    if (plot_acc):
        plt.plot(time_stamp_acc, acc_x_data, label="Accelerometer X", color='r')
        plt.plot(time_stamp_acc, acc_y_data, label="Accelerometer Y", color='g')
        plt.plot(time_stamp_acc, acc_z_data, label="Accelerometer Z", color='b')
    if (plot_gyro):
        plt.plot(time_stamp_gyro, gyro_x_data, label="Gyroscope X", color='c')
        plt.plot(time_stamp_gyro, gyro_y_data, label="Gyroscope Y", color='m')
        plt.plot(time_stamp_gyro, gyro_z_data, label="Gyroscope Z", color='y')
    if (plot_mag):
        plt.plot(time_stamp_mag, mag_x_data, label="Magnetometer X", color='k')
        plt.plot(time_stamp_mag, mag_y_data, label="Magnetometer Y", color='orange')
        plt.plot(time_stamp_mag, mag_z_data, label="Magnetometer Z", color='purple')

    plt.title('Sensor Data (Accelerometer, Gyroscope, Magnetometer)')
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor Values')

    plt.legend()

    plt.show()

except UnicodeDecodeError:
    print("Error decoding data.")
    pass

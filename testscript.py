import serial
import time
import tkinter as tk

"""
===============================================================
Description             | Bytes 1           | Bytes 2
===============================================================
Harsh Braking           | 1                 | 0
Harsh Acceleration      | 2                 | 0
Cornering               | 3                 | 0
Swaying                 | 4                 | 0
4G Status               | 5                 | 0 - 1
G-Sensor Status         | 6                 | 0 - 1
Camera Status           | 7                 | 0 - 1
GPS Status              | 8                 | 0 - 1
Speeding                | 9                 | 1 - 255
LDW Left                | 10                | 1
LDW Right               | 11                | 1
PEDESTRIAN Red          | 12                | 1
PEDESTRIAN AMBER        | 13                | 1
PEDESTRIAN GREEN        | 14                | 1
CAR RED                 | 15                | 0  - 20
CAR AMBER               | 16                | 20 - 40
CAR GREEN               | 17                | 0
CAR Flashing            | 18                | 0
PED Flashing            | 19                | 0
Software Version        | 20                | 0  - 255
Error Codes             | 21                | 0  - 255
Health Status           | 22                | 0  - 255
gps_connected.mp3       | 30                | 0
gps_signal_lost.mp3     | 31                | 0
4g_connected.mp3        | 32                | 0
4g_disconnected.mp3     | 33                | 0
beep_ping.mp3           | 34                | 0
lane_beep_3.mp3         | 35                | 0
fcw_near_2.mp3          | 36                | 0
fcw_medium_3.mp3        | 37                | 0
pedestrian.mp3          | 38                | 0
Heartbeat               | 41                | 0  - 255
================================================================
"""



ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600
)

init_soft_version=bytes([20,10])
init_health_stat=bytes([22,10])
heartbeat=bytes([41, 1])
   
ser.write(init_soft_version)
time.sleep(1)
ser.write(init_health_stat)  
ser.write(heartbeat) 

# Test Case Command
test_case = [
    [5, 0],
    [6, 0],
    [7, 0],
    [8, 0],
    [21, 0],
    [1, 0],
    [2, 0],
    [3, 0],
    [4, 0],
    [5, 1],
    [6, 1],
    [7, 1],
    [8, 1],
    [9, 1],
    [10, 1],
    [11, 1],
    [12, 1],
    [13, 1],
    [14, 1],
    [15, 10],
    [16, 30],
    [17, 0],
    [18, 0],
    [19, 1],
    [30, 0],
    [31, 0],
    [32, 0],
    [33, 0],
    [34, 0],
    [35, 0],
    [36, 0],
    [37, 0],
    [38, 0],
    [21, 20],
    [21, 42],
    [21, 64],
    [21, 68],
    [21, 47],
    [21, 18],
]

test_case2 = [
    [12, 1],
    [13, 1],
    [14, 1],
    [19, 1],
]

while 1:
    ser.write(init_soft_version)
    time.sleep(1)
    ser.write(init_health_stat)  
    ser.write(heartbeat)            # send a heartbeat for every loop

    for command in test_case:
        data=bytes(command)
        ser.write(heartbeat)            # send a heartbeat for every loop
        ser.write(data)
        time.sleep(2)
    
    print("Test completed...")
    input("Press enter to continue.....\n")

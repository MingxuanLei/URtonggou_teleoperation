from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

IP = "192.168.3.15"

print("连接UR5e，RTDE频率=50Hz")

rtde_r = RTDEReceive(IP, 125)

i = 0

while True:
    q = rtde_r.getActualQ()

    if i % 10 == 0:
        print(i, q)

    i += 1
    time.sleep(0.02)
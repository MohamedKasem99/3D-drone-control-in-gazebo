import socket
import sys
from time import sleep
import matplotlib.pyplot as plt
counter = []
trail_errors = []

txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
txSocket.setblocking(0)
# 6. Try sending to NodeMCU and wait for ACKnowledgement
for _ in range(10):
    while True:
        try:
            print("Sending to UDP Node")
            # 6.1 Send a space to initiate conversation
            txSocket.sendto("  ".encode(),("192.168.43.90",8888))
            sleep(.2)
            # 6.2 Wait for acknowledgment signal ir try again
            data, addr = txSocket.recvfrom(1024)
            data = data.decode().split(',')
            if data == ["acknowledged\r\n"]:
                print("Received acknowledgement")
                counter = []
                break
        except socket.error as msg:
            pass
    print("Success")
    # 7. Read Data and populate the Imu and publish it
    while True:
        try:
            # 7.1 recive and decode and split by comme ","
            data, addr = txSocket.recvfrom(1024)
            data = data.decode().strip().split(',')
            if data == ["acknowledged"]:
                print("Received acknowledgement")
                counter = []
                continue
            counter.append(data[0])
            if len(counter) == 10000:
                break
        except BlockingIOError:
            pass
        except:
            print(f"some error happend : {sys.exc_info()[0].__name__}: {sys.exc_info()[1].args}")
            pass
    errors = 0
    for i, c in enumerate(counter):
        if int(c) != i + 1 + errors:
            errors += int(counter[i]) - (int(counter[i-1]) + 1 )
    trail_errors.append(errors)
    
plt.figure(figsize=(8,6))
plt.plot(trail_errors, label = "Errors ")
mean_err = sum(trail_errors)/len(trail_errors)
plt.plot([mean_err]*len(trail_errors), label = f"Mean error = {mean_err}/10000 = {mean_err*100/10000}%")
plt.gca().set_ylabel("Errors per 10000 samples")
plt.gca().set_xlabel("Trials")
plt.gca().set_xticks(list(range(0,10)))
plt.gca().set_xticklabels(list(range(1,11)))
plt.gca().legend();
plt.savefig("text.jpg")
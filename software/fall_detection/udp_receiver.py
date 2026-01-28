import socket
import time

HOST = "0.0.0.0"
PORT = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"Listening on UDP {HOST}:{PORT} ...")

while True:
    data, addr = sock.recvfrom(2048)
    t_recv_us = time.time_ns() // 1000
    line = data.decode(errors="ignore").strip()
    print(t_recv_us, addr, line)
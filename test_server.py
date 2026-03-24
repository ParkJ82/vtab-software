import socket
import struct

HOST = "127.0.0.1"
PORT = 9999

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"Listening on {HOST}:{PORT}...")

conn, addr = server.accept()
print(f"Connected by {addr}")

try:
    while True:
        data = conn.recv(4)
        if not data:
            break

        if len(data) < 4:
            print("Received incomplete packet:", data)
            continue

        x, y = struct.unpack("!HH", data)
        print(f"x={x}, y={y}")
finally:
    conn.close()
    server.close()
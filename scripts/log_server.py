import socket
import struct
import pickle


def handle_client(conn):
    # Receive data size
    data_size = struct.unpack(">I", conn.recv(4))[0]

    # Receive data
    data = b""
    while len(data) < data_size:
        packet = conn.recv(data_size - len(data))
        if not packet:
            return None
        data += packet

    payload = pickle.loads(data)
    print("Received:", payload)

    # Send response
    response = pickle.dumps("Acknowledged")
    conn.sendall(struct.pack(">I", len(response)) + response)


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8080))
    server.listen(5)
    print("Server listening on port 8080")

    while True:
        conn, addr = server.accept()
        print(f"Connected by {addr}")
        handle_client(conn)
        conn.close()


if __name__ == "__main__":
    main()

import socket
import time
import threading

def serve_file(file_path, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('localhost', port))
        s.listen(1)
        print(f"Server ready on port {port}, waiting for client...")

        conn, addr = s.accept()
        with conn:
            print(f"Client connected from {addr} on port {port}")
            with open(file_path, 'r') as f:
                while True:
                    sim_line = f.readline()
                    if not sim_line:  # restart file when end is reached
                        f.seek(0)
                        sim_line = f.readline()

                    conn.sendall(sim_line.encode('utf-8'))
                    print(f"[{port}] Sent: {sim_line.strip()}")
                    time.sleep(1)

def main():
    # Start two threads: one for nmea.txt and one for err.txt
    t1 = threading.Thread(target=serve_file, args=('nmea.txt', 4848), daemon=True)
    t2 = threading.Thread(target=serve_file, args=('err.txt', 6868), daemon=True)

    t1.start()
    t2.start()

    # Keep main thread alive
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()

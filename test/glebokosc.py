import socket
import sys

def connect_and_listen(host, port):
    """
    Łączy się z serwerem NMEA i nasłuchuje komunikatów dotyczących głębokości.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((host, port))
            print(f"Połączono z serwerem NMEA na {host}:{port}")
            buffer = b''
            while True:
                data = s.recv(1024)
                if not data:
                    print("Serwer zamknął połączenie.")
                    break
                buffer += data
                while b'\r\n' in buffer:
                    line, buffer = buffer.split(b'\r\n', 1)
                    nmea_message = line.decode('ascii', errors='ignore').strip()
                    # Sprawdzamy, czy komunikat zaczyna się od $SDDPT lub $SDDBT
                    if nmea_message.startswith('$SDDPT') or nmea_message.startswith('$SDDBT'):
                        print(nmea_message)
    except ConnectionRefusedError:
        print(f"Błąd: Połączenie odrzucone. Upewnij się, że serwer NMEA działa na {host}:{port}.")
    except socket.timeout:
        print(f"Błąd: Przekroczono limit czasu połączenia z {host}:{port}.")
    except Exception as e:
        print(f"Wystąpił błąd: {e}")

if __name__ == "__main__":
    # Domyślne wartości oparte na konfiguracji NMEA Simulatora (tryb AP)
    default_host = "192.168.4.1"
    default_port = 10110

    host = sys.argv[1] if len(sys.argv) > 1 else default_host
    port = int(sys.argv[2]) if len(sys.argv) > 2 else default_port

    print(f"Próba połączenia z {host}:{port}. Aby zmienić, uruchom: python glebokosc.py <IP> <PORT>")
    connect_and_listen(host, port)
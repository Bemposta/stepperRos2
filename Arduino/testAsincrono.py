import serial
import threading
import time
import re

# ---------------------------------------------------
# SerialReader con hilo dedicado
# ---------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port="COM3", baudrate=115200, reattempt_delay=2, callback=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.reattempt_delay = reattempt_delay
        self.callback = callback

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self.ser = None
        self.buffer = ""

    # Conexión automática
    def connect(self):
        while not self._stop_event.is_set():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                print(f"[SerialReader] Conectado a {self.port}")
                return
            except Exception as e:
                print(f"[SerialReader] Error de conexión: {e}. Reintentando...")
                time.sleep(self.reattempt_delay)

    # Hilo principal
    def run(self):
        self.connect()
        while not self._stop_event.is_set():
            try:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode(errors="ignore")
                    self.buffer += chunk

                    # Procesar líneas completas
                    while "\n" in self.buffer:
                        line, self.buffer = self.buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            if self.callback:
                                self.callback(line)
                            else:
                                with self._lock:
                                    self.last_line = line
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError):
                print("[SerialReader] Desconectado. Reconectando...")
                self.connect()

    # Parada limpia
    def stop(self):
        self._stop_event.set()
        if self.ser:
            self.ser.close()
        print("[SerialReader] Detenido.")

# ---------------------------------------------------
# Expresión regular para parsear línea
# formato: millis,flag_L,   pulsos_L,       vel_L,flag_R,   pulsos_R,       vel_R
#          now,   isRunning,currentPosition,speed,isRunning,currentPosition,speed
# ---------------------------------------------------
packet_regex = re.compile(
    r"^"            # inicio
    r"(\d+),"       # millis
    r"([01]),"      # flag_L
    r"(-?\d+),"     # pulsos_L
    r"(-?\d+),"     # vel_L
    r"([01]),"      # flag_R
    r"(-?\d+),"     # pulsos_R
    r"(-?\d+)"      # vel_R
    r"$"            # fin
)

def parse_packet(line):
    match = packet_regex.match(line)
    if match:
        millis = int(match.group(1))
        flag_L = int(match.group(2))
        pulsos_L = int(match.group(3))
        vel_L = int(match.group(4))
        flag_R = int(match.group(5))
        pulsos_R = int(match.group(6))
        vel_R = int(match.group(7))
        return millis, flag_L, pulsos_L, vel_L, flag_R, pulsos_R, vel_R
    else:
        print(f"[Warning] Paquete corrupto: '{line}'")
        return None

# ---------------------------------------------------
# Callback para procesar líneas recibidas
# ---------------------------------------------------
def on_line_received(line):
    result = parse_packet(line)
    if result:
        millis, flag_L, pulsos_L, vel_L, flag_R, pulsos_R, vel_R = result
        print(f"millis={millis}, L={flag_L},{pulsos_L},{vel_L}, R={flag_R},{pulsos_R},{vel_R}")

# ---------------------------------------------------
# Ejemplo de uso
# ---------------------------------------------------
if __name__ == "__main__":
    reader = SerialReader(port="COM3", baudrate=115200, callback=on_line_received)
    reader.start()

    try:
        while True:
            # Aquí podrías poner el control de tu robot
            time.sleep(0.01)
    except KeyboardInterrupt:
        reader.stop()

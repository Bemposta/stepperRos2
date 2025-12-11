import serial
import time
import re


class SerialReader:
    """
    Lector serie robusto para mensajes del tipo:
    'L=1 R=0\n'
    """

    def __init__(self, port="COM3", baud=115200, timeout=0):
        self.port = port
        self.baud = baud
        self.ser = None
        self.regex = re.compile(r"^(\d+),(\d+),([01]),(\d+),([01])$")

    # ----------------------------------------------------------
    # Conexión segura
    # ----------------------------------------------------------
    def connect(self):
        while True:
            try:
                print(f"[SerialReader] Conectando a {self.port}...")
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                self.ser.reset_input_buffer()
                print("[SerialReader] Conectado.")
                return
            except serial.SerialException:
                print("[SerialReader] No se pudo abrir el puerto. Reintentando en 1s...")
                time.sleep(1)

    # ----------------------------------------------------------
    # Lectura robusta de línea (non-blocking safe)
    # ----------------------------------------------------------
    def read_line(self):
        if self.ser is None:
            self.connect()

        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return None  # timeout sin datos
            return line

        except serial.SerialException:
            print("[SerialReader] ¡Error de conexión! Reconectando...")
            self.connect()
            return None

    # ----------------------------------------------------------
    # Limpieza rápida del buffer
    # ----------------------------------------------------------
    def flush(self):
        if self.ser:
            try:
                self.ser.reset_input_buffer()
            except:
                pass

    # ----------------------------------------------------------
    # Procesado y validación del paquete L y R
    # ----------------------------------------------------------
    def parse_line(self, line):
        """
        Devuelve (L, R) como enteros, o None si está corrupto
        """
        match = self.regex.search(line)
        if match:
            millis = int(match.group(1))
            L_pulses = int(match.group(2))
            L_flag = int(match.group(3))
            R_pulses = int(match.group(4))
            R_flag = int(match.group(5))
            print(millis, L_pulses, L_flag, R_pulses, R_flag)
        else:
            print(f"[SerialReader] Paquete corrupto: '{line}'")

    # ----------------------------------------------------------
    # Lectura continua con reconexión y filtrado
    # ----------------------------------------------------------
    def read_LR(self):
        """
        Devuelve L, R ó None si no hay datos válidos.
        """
        line = self.read_line()
        if line:
            self.parse_line(line)


# ==============================================================
# EJEMPLO DE USO
# ==============================================================

if __name__ == "__main__":
    reader = SerialReader(port="COM3", baud=115200)
    reader.connect()
    reader.flush()
    while True:
        data = reader.read_LR()
        # Ajusta si quieres reducir carga de CPU
        print(".", end="", flush=True)
        time.sleep(0.01)

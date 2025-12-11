import serial
import threading
import time
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.time import Time as RclpyTime

#============================================================================================================
# SerialReader con hilo dedicado
class SerialReader(threading.Thread):
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, reattempt_delay=2, callback=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.reattempt_delay = reattempt_delay
        self.callback = callback

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self.ser = None
        self.buffer = ""
        # formato: millis,flag_L,pulsos_L,vel_L,flag_R,pulsos_R,vel_R
        self.packet_regex = re.compile(
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
                                self.callback(self.parse_packet(line))
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

    def parse_packet(self, line):
        match = self.packet_regex.match(line)
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
            return line
            
    def sendMOV(self, numV: float, numW: float):
        if self.ser is None:
            raise RuntimeError("Serial no inicializado")
        comando = f"MOV V {numV} W {numW}\n"
        self.ser.write(comando.encode('utf-8'))
        

#============================================================================================================
class StepperMotorControl(Node):
    def __init__(self):
        super().__init__('stepper_motor_control')
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.publisher_arduino = self.create_publisher(String, '/arduino', 1)
        self.subscription_cmd_vel   # evitar advertencia de variable no usada
        self.publisher_arduino      # evitar advertencia de variable no usada 
        self.reader = SerialReader(port="/dev/ttyACM0", baudrate=115200, callback=self.serial_reciver_callback)
        self.reader.start()
        self.get_logger().info('StepperMotorControl iniciakizado')

    def cmd_vel_callback(self, msg: Twist):
        #self.get_logger().info(f"msg={msg}")
        v = msg.linear.x     # Velocidad lineal en eje X
        w = msg.angular.z    # Velocidad angular en eje Z
        self.reader.sendMOV(v, w)
        
    # Callback para procesar líneas recibidas
    def serial_reciver_callback(self, result):
        if not isinstance(result, tuple):
            self.get_logger().info(f'[Warning] Paquete corrupto en SerialPort leido: {result}.')
            return
        millis, flag_L, pulsos_L, vel_L, flag_R, pulsos_R, vel_R = result
        msg = String()
        msg.data = f"millis={millis}, L={flag_L},{pulsos_L},{vel_L}, R={flag_R},{pulsos_R},{vel_R}"
        self.publisher_arduino.publish(msg)
            
    def closeSerial(self):
        self.reader.stop()

#============================================================================================================
def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("¡¡ KeyboardInterrupt !!")
        pass
    node.closeSerial()
    node.destroy_node()
    rclpy.shutdown()

#============================================================================================================
if __name__ == '__main__':
    main()


import serial
import threading
import time
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.time import Time as RclpyTime
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time

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
            r"(-?\d+),"     # pulsos_L
            r"(-?\d+)"      # pulsos_R
            r"$"            # fin
        )

    # Conexión automática
    def connect(self):
        while not self._stop_event.is_set():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                if self.ser.is_open:
                    time.sleep(0.3)
                    self.ser.reset_input_buffer()   # limpia RX
                    self.ser.reset_output_buffer()  # opcional, limpia TX
                    self.buffer = ""  # limpia buffer interno del parser también
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
                
    def parse_packet(self, line):
        match = self.packet_regex.match(line)
        if match:
            millis = int(match.group(1))
            pulsos_L = int(match.group(2))
            pulsos_R = int(match.group(3))
            return millis, pulsos_L, pulsos_R
        else:
            return line

    # Parada limpia
    def stop(self):
        self._stop_event.set()
        if self.ser:
            self.ser.close()
        print("[SerialReader] Detenido.")
            
    def sendMOV(self, numV: float, numW: float):
        if self.ser is None:
            raise RuntimeError("Serial no inicializado")
        comando = f"MOV V {numV} W {numW}\n"
        self.ser.write(comando.encode('utf-8'))
        
#============================================================================================================
class StepperMotorControl(Node):
    def __init__(self):
        super().__init__('stepper_motor_control')
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.subscription_cmd_vel   # evitar advertencia de variable no usada
        self.odom_pub               # evitar advertencia de variable no usada
        self.reader = SerialReader(port="/dev/ttyACM0", baudrate=115200, callback=self.serial_reciver_callback)
        self.reader.start()
        self.get_logger().info('StepperMotorControl inicializado')
        self.posX = 0
        self.posY = 0
        self.theta = 0
        self.lastEncoderL = 0
        self.lastEncoderR = 0
        
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
        millis, pulsos_L, pulsos_R = result
        dL = pulsos_L - self.lastEncoderL
        dR = pulsos_R - self.lastEncoderR
        self.lastEncoderL = pulsos_L
        self.lastEncoderR = pulsos_R
        self.posX, self.posY, self.theta, v, w = self.pulses_to_odometry(dL, dR, millis, self.posX, self.posY, self.theta)
        odom_msg = self.make_odom_msg(self.posX, self.posY, self.theta, v, w)
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(odom_msg)
        
    def pulses_to_odometry(self, delta_pulses_L, delta_pulses_R, dt, x, y, theta):
        L = 0.15
        R = 0.04
        encoder_ppr = 2400.0
        v_L = 2*3.1416*R*delta_pulses_L/(encoder_ppr*dt)
        v_R = 2*3.1416*R*delta_pulses_R/(encoder_ppr*dt)
        v = (v_R + v_L)/2
        w = (v_R - v_L)/L
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        return x, y, theta , v, w

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def make_odom_msg(self, x, y, theta, v, w, frame_id="odom", child_frame_id="base_link"):
        odom = Odometry()
        # Encabezado del mensaje
        odom.header.stamp = Time()  # se reemplaza después en el publicador
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id
        # Pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(theta)
        # Velocidades (twist)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w
        return odom

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


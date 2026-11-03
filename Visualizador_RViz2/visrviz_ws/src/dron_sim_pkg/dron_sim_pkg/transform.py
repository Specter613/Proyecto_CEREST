import rclpy
from rclpy.node import Node
import math
from pymavlink import mavutil
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class MavlinkTfBridge(Node):
    def __init__(self):
        super().__init__('mavlink_tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Conexión MAVLink por UDP
        # Ajusta el puerto 14550 al que esté usando tu red local/emisora
        udp_port = 'udp:0.0.0.0:14550' 
        self.get_logger().info(f'Conectando a MAVLink en {udp_port}...')
        self.mav_connection = mavutil.mavlink_connection(udp_port)

        # Variables para almacenar el último estado conocido
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Timer para leer MAVLink a 50Hz (0.02 segundos)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy # w
        q[1] = sr * cp * cy - cr * sp * sy # x
        q[2] = cr * sp * cy + sr * cp * sy # y
        q[3] = cr * cp * sy - sr * sp * cy # z
        return q

    def timer_callback(self):
        # Leer mensaje sin bloquear el hilo principal
        msg = self.mav_connection.recv_match(
            type=['ATTITUDE', 'LOCAL_POSITION_NED'], 
            blocking=False
        )

        if msg:
            msg_type = msg.get_type()
            
            # Actualizar rotación
            if msg_type == 'ATTITUDE':
                self.roll = msg.roll
                self.pitch = msg.pitch
                self.yaw = msg.yaw
            
            # Actualizar posición (si tu telemetría la envía)
            elif msg_type == 'LOCAL_POSITION_NED':
                # Convertir de MAVLink (NED) a ROS (ENU)
                self.x = msg.y      
                self.y = msg.x      
                self.z = -msg.z     

        self.publish_transform()

    def publish_transform(self):
        t = TransformStamped()

        # Metadatos del TF
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'      # Marco de referencia estático
        t.child_frame_id = 'base_link'   # El eslabón principal de tu dron

        # Posición (Traslación)
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = float(self.z)

        # Rotación (Cuaternión)
        # Nota: Dependiendo de tu dron, podrías necesitar adaptar el RPY de NED a ENU
        q = self.euler_to_quaternion(self.roll, -self.pitch, -self.yaw)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        # Publicar
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
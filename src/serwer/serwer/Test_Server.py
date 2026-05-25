import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32  # Dodano import dla wiadomości z kątem serwa
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from flask import Flask, Response, request
import threading
import time
import sys

app = Flask(__name__)
latest_frame = None
ros_node = None  # Globalna referencja do węzła ROS2 dla Flaska
TOPIC = '/camera/image_raw/compressed'

def get_matched_qos(node: Node, topic: str, timeout_sec: float = 10.0) -> QoSProfile:
    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout_sec
    while node.get_clock().now().nanoseconds / 1e9 < deadline:
        publishers = node.get_publishers_info_by_topic(topic)
        if publishers:
            pub_qos = publishers[0].qos_profile

            reliability = pub_qos.reliability
            durability  = pub_qos.durability

            # depth=0 jest nieprawidłowe dla KEEP_LAST — używamy 1 jako bezpieczne minimum
            depth = pub_qos.depth if pub_qos.depth > 0 else 1

            node.get_logger().info(
                f"[QoS] Publisher: reliability={reliability}, "
                f"durability={durability}, depth={depth}"
            )

            return QoSProfile(
                reliability=reliability,
                durability=durability,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=depth,
            )
        time.sleep(0.5)

    node.get_logger().warn(
        f"[QoS] Brak publishera na {topic} po {timeout_sec}s. Używam fallback RELIABLE."
    )
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

class FlaskROSBridge(Node):
    def __init__(self):
        super().__init__('flask_ros_bridge')
        self.first_frame_received = False

        # --- SEKCJA SUBU (Kamera) ---
        self._keepalive_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            lambda msg: None,
            1,
        )
        self.get_logger().info("[Keepalive] Subskrypcja /camera/image_raw aktywna.")

        qos = get_matched_qos(self, TOPIC)
        self.subscription = self.create_subscription(
            CompressedImage,
            TOPIC,
            self.image_callback,
            qos,
        )
        self.get_logger().info("Subskrypcja /compressed aktywna. Czekam na klatki...")

        self.servo_pub = self.create_publisher(Int32, '/servo_command', 10)
        self.get_logger().info("Publisher /servo_command aktywny.")

    def image_callback(self, msg):
        global latest_frame
        latest_frame = bytes(msg.data)
        if not self.first_frame_received:
            self.get_logger().info("[OK] Pierwsza klatka odebrana!")
            self.first_frame_received = True

    def publish_servo_angle(self, angle: int):
        """Metoda wywoływana przez Flaska w celu wysłania komendy do ROS"""
        msg = Int32()
        msg.data = angle
        self.servo_pub.publish(msg)
        self.get_logger().info(f"[SCR] Komenda na serwo wysłana do ROS: {angle} stopni")


def ros2_thread(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = FlaskROSBridge()
    rclpy.spin(ros_node)
    
    # Czyszczenie przy wyłączaniu
    ros_node.destroy_node()
    rclpy.shutdown()


def generate_frames():
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.03)


@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/servo', methods=['GET'])
def control_servo():
    global ros_node
    angle = request.args.get('angle', type=int)
    
    if angle is not None:
        if ros_node is not None:
            ros_node.publish_servo_angle(angle)
            return f"OK: {angle} stopni wyslano", 200
        else:
            print("[BŁĄD] Próba wysłania komendy, ale węzeł ROS nie jest gotowy.")
            return "Błąd: Węzeł ROS nie zainicjowany", 500
            
    return "Brak kąta", 400


def main(args=None):

    t = threading.Thread(target=ros2_thread, args=(args,), daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main(sys.argv)
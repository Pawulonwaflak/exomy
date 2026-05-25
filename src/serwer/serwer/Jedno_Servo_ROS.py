#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from st3215.scservo_sdk import *


# ── Konfiguracja sprzętu ─────────────────────────────────────────────────────
SERVO_ID    = 1
BAUDRATE    = 1_000_000
DEVICENAME  = '/dev/ttyACM0'

SCS_MOVING_SPEED = 3500
SCS_MOVING_ACC   = 40

# Przelicznik: kąt 0–180° → pozycja 0–4095
ANGLE_MIN = 0
ANGLE_MAX = 180
POS_MIN   = 0
POS_MAX   = 4095


def angle_to_position(angle_deg: int) -> int:
    """Przelicza kąt [0–180°] na jednostki pozycji serwa [0–4095]."""
    angle_clamped = max(ANGLE_MIN, min(ANGLE_MAX, angle_deg))
    return int((angle_clamped - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN) * (POS_MAX - POS_MIN) + POS_MIN)


# ── Node ROS 2 ───────────────────────────────────────────────────────────────
class ServoCommandNode(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('servo_command_node')

        self.port_handler   = port_handler
        self.packet_handler = packet_handler

        # Ustaw tryb pozycyjny dla serwa ID=1
        self.packet_handler.unLockEprom(SERVO_ID)
        self.packet_handler.write1ByteTxRx(SERVO_ID, SMS_STS_MODE, 0)
        self.packet_handler.LockEprom(SERVO_ID)
        self.get_logger().info(f'Serwo ID={SERVO_ID} ustawione w tryb pozycyjny.')

        # Subskrypcja topicu
        self.subscription = self.create_subscription(
            Int32,
            'servo_command',
            self.servo_command_callback,
            10
        )
        self.get_logger().info("Subskrybuję topic '/servo_command' (std_msgs/Int32, kąt 0–180°).")

    def servo_command_callback(self, msg: Int32):
        angle = msg.data
        position = angle_to_position(angle)

        self.get_logger().info(
            f'Otrzymano kąt: {angle}° → pozycja serwa: {position} '
            f'(prędkość={SCS_MOVING_SPEED}, acc={SCS_MOVING_ACC})'
        )

        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            SERVO_ID,
            position,
            SCS_MOVING_SPEED,
            SCS_MOVING_ACC
        )

        if scs_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'Błąd komunikacji: {self.packet_handler.getTxRxResult(scs_comm_result)}'
            )
        elif scs_error != 0:
            self.get_logger().warn(
                f'Błąd pakietu: {self.packet_handler.getRxPacketError(scs_error)}'
            )
        else:
            self.get_logger().info(f'Komenda wysłana pomyślnie → pozycja {position}.')


# ── Main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    # Inicjalizacja portu szeregowego
    port_handler   = PortHandler(DEVICENAME)
    packet_handler = sms_sts(port_handler)

    if not port_handler.openPort():
        print('[BŁĄD] Nie można otworzyć portu szeregowego.')
        return

    if not port_handler.setBaudRate(BAUDRATE):
        print('[BŁĄD] Nie można ustawić baudrate.')
        port_handler.closePort()
        return

    print(f'Port {DEVICENAME} otwarty, baudrate={BAUDRATE}.')

    # Uruchomienie ROS 2
    rclpy.init(args=args)
    node = ServoCommandNode(port_handler, packet_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymano przez użytkownika.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        port_handler.closePort()
        print('Port zamknięty. Koniec programu.')


if __name__ == '__main__':
    main()
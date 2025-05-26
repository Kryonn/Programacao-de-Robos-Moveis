#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import time
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)


        self.bridge = CvBridge()
        self.giro_inicio = 0.0
        self.giro = False
        self.state = "Busca"
        self.dir = "Direita"
        self.kp = 0.005
        self.error = 0.0
        self.bandeira_identificada = False

        # Estado interno
        self.obstaculo_a_frente = False

    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Índices de -30° a +30° (equivalente a 330 até 30)
        indices_frente = list(range(330, 360)) + list(range(0, 31))

        # Filtra distancias
        distancias = [msg.ranges[i] for i in indices_frente]

        if distancias and min(distancias) < 0.5:
            indice = distancias.index(min(distancias))
            indice_real = indices_frente[indice]
            if indice <= 30:
                self.dir = "Direita"
            else:
                self.dir = "Esquerda"
            self.obstaculo_a_frente = True
            self.get_logger().info('Obstáculo detectado a {:.2f}m à frente'.format(min(distancias)))
        else:
            self.obstaculo_a_frente = False

    def imu_callback(self, msg: Imu):
        # Mensagens da IMU!
        pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape

        # Define a cor-alvo em BGR
        target_color = np.array([57, 66, 0])  # OBS: OpenCV usa BGR

        # Cria máscara para cor exata
        mask = cv2.inRange(frame, target_color, target_color)

        # Mostra a máscara em uma janela para debug
        cv2.imshow('Mascara de Blobs #004239', mask)
        cv2.waitKey(1)  # Tempo mínimo para a janela atualizar (1 ms)

        # Detecta contornos (blobs)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_identificada = False

        # Conta e localiza blobs
        if contours:
            self.get_logger().info(f'{len(contours)} blob(s) encontrados com cor #004239:')
            for i, cnt in enumerate(contours):
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    self.get_logger().info(f'  Blob {i+1}: posição (x={cx}, y={cy})')
                    self.error = cx - width / 2
                    self.bandeira_identificada = True


    def move_robot(self):
        twist = Twist()
        match self.state:
            case "Busca":
                twist.linear.x = 0.1
                if self.obstaculo_a_frente:
                    self.state = "Giro_inicio"
                else:
                    if self.bandeira_identificada:
                        self.state = "Busca_bandeira"
            case "Giro_inicio":
                self.giro = True
                self.giro_inicio = time.time()
                self.state = "Giro"
            case "Giro":
                twist.linear.x = 0.1
                if self.dir == "Direita":
                    twist.angular.z = 0.5
                else:
                    twist.angular.z = -0.5

                tempo_decorrido = time.time() - self.giro_inicio
                tempo_necessario = (np.pi / 4) / abs(twist.angular.z)

                if tempo_decorrido >= tempo_necessario:
                    self.giro = False
                    twist.angular.z = 0.0
                    self.state = "Giro_fim"
            case "Giro_fim":
                if not self.obstaculo_a_frente:
                    self.state = "Busca"
                else:
                    self.state = "Giro_inicio"
            case "Busca_bandeira":
                twist.linear.x = 0.0
                twist.angular.z = -self.kp * self.error
                if not self.bandeira_identificada:
                    self.state = "Busca"
                else:
                    if self.error == 0.0:
                        self.state = "Andar_bandeira"
            case "Andar_bandeira":
                twist.linear.x = 0.1
                if self.obstaculo_a_frente:
                    self.state = "Giro"



        self.cmd_vel_pub.publish(twist)




def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

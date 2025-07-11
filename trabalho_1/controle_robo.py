#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import time
from rclpy.node import Node

from cv_bridge import CvBridge
import heapq
from scipy.spatial.transform import Rotation as R


from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        self.caminho_a_estrela = []  # Lista de posições do caminho
        self.index_alvo = 0          # Índice do ponto atual no caminho

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)
        self.create_subscription(Pose, '/model/prm_robot/pose', self.pose_callback, 10)

        # Espera o robô carregar
        # Levanta a garra no início do programa
        time.sleep(2)
        # self.controlar_garra([-np.pi, -0.5, 0.5]) 

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Variáveis gerais
        self.bridge = CvBridge()            # Ponte entre a imagem ROS e OpenCv
        self.yaw_ant = 0                    # Ângulo antes do giro
        self.dir = "Direita"                # Direção do giro
        self.kp = 0.005                     # Ganho proporcional
        self.error = 0.0                    # Erro direcional

        self.alinhamento_grosso_concluido = False
        self.dist_garra  = 0.5              # Distancia máxima para identificar como garra
        self.dist_obstaculo  = 0.6          # Distancia mínima para obstáculo identificado
        self.angulo_necessario = 40         # Giro do robô
        self.giro_inicio = 0
        self.frontal_scan_angle_deg = 20 
        # Estados
        self.state = "Busca"                # "Busca", "Giro_inicio", "Giro", "Giro_fim", 
                                            # "Alinha_bandeira", "Andar_bandeira".

        # Flags
        self.giro = False                   # Condição de início do giro 
        self.bandeira_identificada = False  # Condição de busca da bandeira
        self.obstaculo_a_frente = False     # Condição de giro
        self.chegou_bandeira = False         
        self.estado_timer = None
        self.sub_state_garra = "ocioso"  # Estados: ocioso, abaixando, fechando, levantando
        self.timer_garra = None  
        self.map_size = 200  # Ex: 200x200 células
        self.resolution = 0.1  # Cada célula = 10 cm
        self.grid_map = np.zeros((self.map_size, self.map_size))
        self.simular_bandeira = False
        self.garra_abaixada = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pos_inicial_salva = False
        self.robot_init_x = 0.0
        self.robot_init_y = 0.0
        self.bandeira_world_x = 0.0
        self.bandeira_world_y = 0.0
        self.ultima_scan = None
        self.bandeira_mapeada = False
        self.flag_recalcular = False
        self.odom_received = False
        self.bandeira_posicoes = []
        # self.contador = 0
        self.last_recheck_time = self.get_clock().now().nanoseconds 
        self.recheck_interval_ns = 10 * 1_000_000_000 # 10 segundos em nanossegundos

        self.last_invalid_distance_time = 0.0 # Tempo em que a última distância inválida foi detectada
        self.invalid_distance_cooldown_s = 5.0 # Cooldown em segundos antes de tentar remapear/realinhar novamente
         
        # Limiar para área de blob
        self.min_blob_area = 700  # Área mínima para considerar um blob de bandeira (já existente)
        self.max_blob_area = 16000 # Área máxima para considerar bandeira "muito próxima" - AJUSTAR ESSE VALOR SE NECESSÁRIO
        
        self.afasta_inicio = None

        self.tentativas_desvio = 0      # Conta tentativas consecutivas de desvio
        self.max_tentativas_quina = 3   # Após N tentativas, considera que está em quina/preso
        self.time_last_desvio = None    # Para evitar desvios em sequência muito rápidos
        self.timeout_desvio = 1.0       # Segundos mínimos entre desvios consecutivos
        self.estado_antes_desvio = None
         
    def controlar_garra(self, posicoes, duracao_ns=2e9):
        """
        Controla a garra enviando posições [junta_vertical, garra_esquerda, garra_direita]
        por uma determinada duração em nanosegundos.
        """
        self.get_logger().info(f'Movendo a garra para: {posicoes}')
        command_gripper = Float64MultiArray()
        command_gripper.data = posicoes

        inicio = self.get_clock().now()
        while (self.get_clock().now() - inicio).nanoseconds < duracao_ns:
            self.gripper_pub.publish(command_gripper)
        
        self.get_logger().info('Comando da garra finalizado.')

    def is_in_map(self, grid_x, grid_y):
        return 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size


    # verifica distancia dos obstaculos
    def scan_callback(self, msg: LaserScan):
        if not self.odom_received:
        # Ignora scans até receber odometria
            return
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Para checar obstáculo à frente:
        angle_deg_cada_lado = self.frontal_scan_angle_deg
        indices_frente = list(range(360 - angle_deg_cada_lado, 360)) + list(range(0, angle_deg_cada_lado + 1))
        metade = len(indices_frente) // 2
        indices_esq = indices_frente[:metade]
        indices_dir = indices_frente[metade:]
        
        leituras_esq = [msg.ranges[i] for i in indices_esq if not np.isnan(msg.ranges[i]) and msg.ranges[i] > self.dist_garra]
        leituras_dir = [msg.ranges[i] for i in indices_dir if not np.isnan(msg.ranges[i]) and msg.ranges[i] > self.dist_garra]

        media_esq = np.mean(leituras_esq) if leituras_esq else float('inf')
        media_dir = np.mean(leituras_dir) if leituras_dir else float('inf')
        min_esq = min(leituras_esq) if leituras_esq else float('inf')
        min_dir = min(leituras_dir) if leituras_dir else float('inf')
       
        if min(min_esq, min_dir) < self.dist_obstaculo:
            agora = time.time()
            if self.time_last_desvio is None or (agora - self.time_last_desvio > self.timeout_desvio):
                self.tentativas_desvio += 1
                self.time_last_desvio = agora
            self.dir = "Direita" if media_dir > media_esq else "Esquerda"
            self.obstaculo_a_frente = True
            self.get_logger().info(
                f"Obstáculo à frente. Média Esq: {media_esq:.2f}, Média Dir: {media_dir:.2f}. Virando para {self.dir}. Tentativas: {self.tentativas_desvio}"
            )
        else:
            self.obstaculo_a_frente = False
            self.tentativas_desvio = 0  # Caminho livre, zera o contador
            self.time_last_desvio = None


        #  mapeando obstáculos no grid
        angle = msg.angle_min
        distancias = [d if (d > self.dist_garra) else float('inf') for d in msg.ranges]

        for dist in distancias:
            if np.isnan(dist) or dist == float('inf'):
                angle += msg.angle_increment
                continue

            # Coordenadas relativas ao robô
            obs_x = dist * np.cos(angle)
            obs_y = dist * np.sin(angle)

            # Coordenadas absolutas no mundo
            world_x = self.robot_x + obs_x * np.cos(self.robot_yaw) - obs_y * np.sin(self.robot_yaw)
            world_y = self.robot_y + obs_x * np.sin(self.robot_yaw) + obs_y * np.cos(self.robot_yaw)

            # Convertendo para grid
            grid_x, grid_y = self.world_to_grid(world_x, world_y)

            if self.is_in_map(grid_x, grid_y):
                self.grid_map[grid_y, grid_x] = 1.0  # 1.0 = obstáculo

            angle += msg.angle_increment
        self.ultima_scan = msg


    def show_grid(self):
        # Cria uma imagem colorida (map_size x map_size x 3)
        grid_img = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)

        # Itera sobre todas as células
        for y in range(self.map_size):
            for x in range(self.map_size):
                val = self.grid_map[y, x]
                if not self.caminho_a_estrela == None and (x, y) in self.caminho_a_estrela:
                    color = (0, 0, 255)  # Vermelho - caminho
                elif val == 0.0:
                    color = (0, 0, 0)  # Preto - livre
                elif val == 0.2:
                    color = (0, 0, 255)  # Vermelho - caminho
                elif val == 1.0:
                    color = (255, 255, 255)  # Branco - obstáculo
                elif val == 0.5:
                    color = (128, 128, 128)  # Cinza - posição atual
                elif val == 0.75:
                    color = (0, 255, 0)  # Verde - bandeira
                elif val == 0.9:
                    color = (255, 0, 0)  # Azul - posição inicial
                elif val == 0.95:
                    color = (255, 255, 0)  # Azul claro - zona inflada
                else:
                    color = (50, 50, 50)  # Default (livre escuro)

                grid_img[y, x] = color

        # Redimensiona para visualizar melhor
        grid_img = cv2.resize(grid_img, (1200, 800), interpolation=cv2.INTER_NEAREST)

        cv2.imshow('Mapa Colorido do Grid', grid_img)
        cv2.waitKey(1)


    def odom_callback(self, msg: Odometry):
        pass

    def camera_callback(self, msg: Image):
        if self.simular_bandeira:
            self.simular_bandeira_detectada()
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        target_color = np.array([227, 73, 0])

        mask = cv2.inRange(frame, target_color, target_color)
        cv2.imshow('Mascara de Blobs #004239', mask)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.bandeira_identificada = False

        if contours and not self.chegou_bandeira:
            # Pega o maior contorno válido (caso tenha mais de um)
            cnt_maior = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt_maior)
            if area >= self.min_blob_area:
                M = cv2.moments(cnt_maior)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    self.error = cx - width / 2
                    self.bandeira_identificada = True


    def simular_bandeira_detectada(self):
        """
        Função de simulação para testes, que força a identificação de uma bandeira
        e o mapeamento de sua posição no grid.

        Para usar a simulação, basta chamar esta função no início do código
        ou dentro de outro método como substituição temporária à detecção real.
        """

        # Calcula a posição global da bandeira
        bandeira_x = 0
        bandeira_y = 0

        # Converte para grid
        grid_x, grid_y = self.world_to_grid(bandeira_x, bandeira_y)

        # Valida e registra a bandeira simulada
        if self.is_in_map(grid_x, grid_y) and not self.bandeira_mapeada:
            # Preenche a lista de posições com a mesma posição várias vezes, simulando estabilidade
            self.bandeira_posicoes = [(grid_x, grid_y) for _ in range(4)]

            # Calcula a média das posições simuladas
            media_x = int(sum(pos[0] for pos in self.bandeira_posicoes) / 4)
            media_y = int(sum(pos[1] for pos in self.bandeira_posicoes) / 4)

            # Marca no grid a posição média
            if self.is_in_map(media_x, media_y):
                self.grid_map[media_y, media_x] = 0.75  # Marca a bandeira na posição média
                self.get_logger().info(f'Bandeira simulada mapeada na média ({media_x}, {media_y})')

                # Atualiza as variáveis de estado
                self.bandeira_x = media_x
                self.bandeira_y = media_y

                self.bandeira_world_x, self.bandeira_world_y = self.grid_to_world(media_x, media_y)

                self.bandeira_mapeada = True
                self.state = "Planejar_Caminho"


    def angulo_delta(self, yaw_atual, yaw_anterior):
        delta = yaw_atual - yaw_anterior
        while delta > 180:
            delta -= 360
        while delta < -180:
            delta += 360
        return delta



    def move_robot(self):
        twist = Twist()
        self.get_logger().info(f'{self.state}')
        
        # Transição dos estados
        match self.state:
            case "Alinha_bandeira":
                twist.linear.x = 0.0
                twist.angular.z = -self.kp * self.error

                if not self.bandeira_identificada:
                    self.state = "Busca"
                elif abs(self.error) < 5:
                    self.state = "Mapeia_bandeira"

            case "Mapeia_bandeira":
                twist.linear.x = 0.0
                twist.angular.z = 0.0 
                print(f'{self.bandeira_identificada,abs(self.error) < 5 , self.ultima_scan is not None}')

                if self.bandeira_identificada and abs(self.error) < 5 and self.ultima_scan is not None:
                    num_ranges = len(self.ultima_scan.ranges)
                    centro = num_ranges // 2
                    offset = 2
                    indices = range(centro - offset, centro + offset + 1)
                    distancias_validas = [
                        self.ultima_scan.ranges[i]
                        for i in indices
                        if not np.isnan(self.ultima_scan.ranges[i]) and self.ultima_scan.ranges[i] != float('inf')
                    ]
                    print("Setor central LIDAR:", [self.ultima_scan.ranges[i] for i in indices])
                    if distancias_validas:
                        distancia = min(distancias_validas)
                        bandeira_x = self.robot_x + distancia * np.cos(self.robot_yaw)
                        bandeira_y = self.robot_y + distancia * np.sin(self.robot_yaw)
                        grid_x, grid_y = self.world_to_grid(bandeira_x, bandeira_y)
                        self.get_logger().info(f'entrei')

                        if self.is_in_map(grid_x, grid_y):
                            self.bandeira_posicoes.append((grid_x, grid_y))
                            self.get_logger().info(f'entrei')


                    if len(self.bandeira_posicoes) > 0: 
                        self.bandeira_x, self.bandeira_y = self.bandeira_posicoes[-1]
                        self.grid_map[self.bandeira_y, self.bandeira_x] = 0.75
                        self.bandeira_mapeada = True
                        self.bandeira_posicoes.clear()
                        self.state = "Validar_Posicao_Bandeira"
                elif not self.bandeira_identificada:
                    self.state = "Busca" 
          
            case "Validar_Posicao_Bandeira":
                current_time_ns = self.get_clock().now().nanoseconds
                if (current_time_ns / 1e9 - self.last_invalid_distance_time) > self.invalid_distance_cooldown_s:
                    if self.ultima_scan is not None:
                        num_ranges = len(self.ultima_scan.ranges)
                        centro = num_ranges // 2
                        offset = 5
                        indices = range(centro - offset, centro + offset + 1)
                        distancias_validas = [
                            self.ultima_scan.ranges[i]
                            for i in indices
                            if not np.isnan(self.ultima_scan.ranges[i]) and self.ultima_scan.ranges[i] != float('inf')
                        ]
                        if distancias_validas:
                            distancia_nova = min(distancias_validas)
                            bandeira_x_nova = self.robot_x + distancia_nova * np.cos(self.robot_yaw)
                            bandeira_y_nova = self.robot_y + distancia_nova * np.sin(self.robot_yaw)
                            grid_x_nova, grid_y_nova = self.world_to_grid(bandeira_x_nova, bandeira_y_nova)

                            if (abs(grid_x_nova - self.bandeira_x) > 2 or abs(grid_y_nova - self.bandeira_y) > 2):
                                self.get_logger().info(f"Bandeira remapeada para ({grid_x_nova}, {grid_y_nova}). Pos. anterior: ({self.bandeira_x}, {self.bandeira_y}). Recalculando caminho.")
                                if self.is_in_map(grid_x_nova, grid_y_nova):
                                    self.bandeira_x = grid_x_nova
                                    self.bandeira_y = grid_y_nova
                                    self.grid_map[self.bandeira_y, self.bandeira_x] = 0.75
                                    self.flag_recalcular = True
                                    self.state = "Planejar_Caminho"
                                else:
                                    self.get_logger().info("Bandeira remapeada para fora do escopo. Continuando Seguir_Caminho.")
                                    self.state = "Seguir_Caminho"
                            else:
                                self.get_logger().info("Bandeira centralizada e posição confirmada. Continuando Seguir_Caminho.")
                                self.state = "Seguir_Caminho"
                        else:
                            self.get_logger().warn('Distância LIDAR inválida durante remapeamento. Voltando para Seguir_Caminho.')
                            self.last_invalid_distance_time = current_time_ns / 1e9
                            self.state = "Seguir_Caminho"
                    else:
                        self.get_logger().warn('Scan não disponível para validação. Voltando para Seguir_Caminho.')
                        self.state = "Seguir_Caminho"
                else:
                    self.get_logger().info("Aguardando cooldown para nova validação da bandeira.")
                    self.state = "Seguir_Caminho"

            case "Afasta":
                # Se ainda não iniciou o recuo, marca o instante de início
                if self.afasta_inicio is None:
                    self.afasta_inicio = self.get_clock().now()
                    if self.estado_antes_desvio is None:
                        # Salva o estado anterior ao desvio
                        self.estado_antes_desvio = self.state_anterior if hasattr(self, "state_anterior") else "Busca"  

                
                duracao = 1.0  # segundos de recuo

                # Calcula o tempo decorrido
                tempo_decorrido = (self.get_clock().now() - self.afasta_inicio).nanoseconds / 1e9

                if tempo_decorrido < duracao:
                    twist.linear.x = -0.3
                    twist.angular.z = 0.0
                else: 
                    self.afasta_inicio = None
                    # Aumenta o ângulo se muitas tentativas (pode ser 90° normalmente, ou 135°/180° para sair de quina)
                    if self.tentativas_desvio >= self.max_tentativas_quina:
                        self.angulo_necessario = 150   # Tenta um giro maior para escapar da quina
                    else:
                        self.angulo_necessario = 90

                    self.giro_inicio = time.time()
                    self.state = "Giro"

            # Giro: faz o robô girar para o lado que foi definido
            case "Giro":
                twist.linear.x = 0.0
                twist.angular.z = 0.45 if self.dir == "Direita" else -0.45
               
                tempo_decorrido = time.time() - self.giro_inicio
                tempo_necessario = np.radians(self.angulo_necessario) / abs(twist.angular.z)

                if tempo_decorrido >= tempo_necessario:
                    self.get_logger().info("Fim do Giro")
                    twist.angular.z = 0.0

                    self.avanco_curto_inicio = time.time()
                    self.state = "Avanco_Curto"
                    
            case "Avanco_Curto":
                twist.linear.x = 0.25
                twist.angular.z = 0.0
                duracao_avanco = 0.6

                if self.obstaculo_a_frente:
                    self.get_logger().info("Obstáculo durante avanço curto, parando e buscando novo caminho.")
                    self.tentativas_desvio = 0
                    if self.bandeira_mapeada:
                        self.flag_recalcular = True
                    if self.estado_antes_desvio:
                        self.state = self.estado_antes_desvio
                        self.estado_antes_desvio = None
                    else:
                        self.state = "Busca"
                elif time.time() - self.avanco_curto_inicio >= duracao_avanco:
                    self.tentativas_desvio = 0 
                    if self.estado_antes_desvio:
                        self.state = self.estado_antes_desvio
                        self.estado_antes_desvio = None
                    else:
                        self.state = "Busca"
            # Busca: anda para frente até encontrar um obstáculo ou a bandeira
            case "Busca":
                twist.linear.x = 0.50
                current_time_ns = self.get_clock().now().nanoseconds

                if self.obstaculo_a_frente:
                    self.state = "Afasta"
                elif self.bandeira_identificada and (current_time_ns / 1e9 - self.last_invalid_distance_time) > self.invalid_distance_cooldown_s:
                    self.state = "Alinha_bandeira"
               
            case "Andar_bandeira":
                twist.linear.x = 0.2
                if self.obstaculo_a_frente:
                    self.state = "Afasta"

            case "Planejar_Caminho":
                self.flag_recalcular = False
                self.caminho_a_estrela = self.planejar_caminho_astar()

                if not self.caminho_a_estrela == None:
                    self.get_logger().warn(f'Caminho: {self.caminho_a_estrela}') # imprime caminho
                    self.state = "Seguir_Caminho"
                    self.index_alvo = 0
                    self.bandeira_mapeada = True
                else:
                    self.get_logger().warn("Caminho A* não encontrado.")
                    current_time_ns = self.get_clock().now().nanoseconds
                    self.last_invalid_distance_time = current_time_ns / 1e9 # Atualiza o tempo do cooldown
                    self.state = "Busca"

            case "Seguir_Caminho":
                if self.obstaculo_a_frente:
                    self.get_logger().warn("Obstáculo detectado durante caminho planejado. Recalculando.")
                    self.flag_recalcular = True
                    self.state = "Afasta"
                    return
                if self.caminho_a_estrela == None:
                    self.state = "Planejar_Caminho"
                elif self.index_alvo >= len(self.caminho_a_estrela) - 4 and not self.garra_abaixada:
                    self.controlar_garra([0.1, -0.5, 0.5])
                    self.garra_abaixada = True
                elif self.index_alvo >= len(self.caminho_a_estrela) - 2:
                    self.state = "Aproximacao_Final_Bandeira"
                elif self.obstaculo_a_frente:
                        self.state = "Afasta"
                    # flag que indica a necessidade de recalcular o caminho
                elif self.flag_recalcular:
                    self.get_logger().warn("Recalculando caminho.")
                    self.state = "Planejar_Caminho"
                else:
                    # Posição alvo no grid
                    grid_x, grid_y = self.caminho_a_estrela[self.index_alvo]
                    target_x, target_y = self.grid_to_world(grid_x, grid_y)

                    # Diferença para o alvo
                    dx = target_x - self.robot_x
                    dy = target_y - self.robot_y
                    dist = np.hypot(dx, dy)
                    angulo_alvo = np.arctan2(dy, dx)
                    erro_ang = angulo_alvo - self.robot_yaw

                    # Normaliza ângulo para [-pi, pi]
                    erro_ang = np.arctan2(np.sin(erro_ang), np.cos(erro_ang))

                    if dist <= 0.35:  # Se já chegou na célula alvo
                        self.get_logger().warn(f'indice alcançado: {self.index_alvo}')
                        self.grid_map[grid_x, grid_y] = 0.0
                        self.index_alvo += 1
                    else:
                            # Controlador de perseguição suave (anda e gira ao mesmo tempo)
                        angulo_alvo = np.arctan2(dy, dx)
                        erro_ang = angulo_alvo - self.robot_yaw
                        erro_ang = np.arctan2(np.sin(erro_ang), np.cos(erro_ang)) # Normaliza

                        twist.linear.x = 0.45  # Velocidade moderada e constante
                        twist.angular.z = 0.7 * erro_ang # Velocidade angular proporcional ao erro

            case "Parado":
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            case "Aproximacao_Final_Bandeira":
                # --- 1. Parâmetros de Controle (Ajuste estes valores para o seu robô) ---
                DISTANCIA_ALVO_LIDAR = 0.40      # Distância ideal do LIDAR para a captura (40cm)
                ANGULO_ALINHADO_RAD = np.deg2rad(2)  # Tolerância para considerar o robô alinhado (2 graus)
                KP_ANGULAR = 0.7               # Ganho do controlador de ângulo
                KP_LINEAR = 0.8                # Ganho do controlador de distância
                MAX_VEL_ANGULAR = 0.3          # Velocidade máxima de giro (rad/s)
                MAX_VEL_LINEAR = 0.35          # Velocidade máxima de avanço (m/s)

                # --- 2. Verificações de Segurança ---
                if not self.ultima_scan or not self.caminho_a_estrela:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().error("Aproximação final: Faltam dados do LIDAR ou do caminho A*.")
                    self.state = "Planejar_Caminho"
                    self.alinhamento_grosso_concluido = False # Reseta a flag ao sair
                    return

                # --- 3. Lógica de Alinhamento em Duas Fases ---

                # FASE 1: Alinhamento Grosso com o Mapa (A*)
                if not self.alinhamento_grosso_concluido:
                    self.get_logger().info("FASE 1: Alinhamento com o alvo do mapa...")
                    
                    ultimo_ponto_grid = self.caminho_a_estrela[-1]
                    target_world_x, target_world_y = self.grid_to_world(ultimo_ponto_grid[0], ultimo_ponto_grid[1])
                    
                    dx_mapa = target_world_x - self.robot_x
                    dy_mapa = target_world_y - self.robot_y
                    angulo_alvo_mapa = np.arctan2(dy_mapa, dx_mapa)
                    
                    angulo_erro_rad = angulo_alvo_mapa - self.robot_yaw
                    angulo_erro_rad = np.arctan2(np.sin(angulo_erro_rad), np.cos(angulo_erro_rad))

                    # Se o alinhamento com o mapa ainda não está bom, continue girando
                    if abs(angulo_erro_rad) > ANGULO_ALINHADO_RAD:
                        twist.linear.x = 0.0
                        twist.angular.z = np.clip(KP_ANGULAR * angulo_erro_rad, -MAX_VEL_ANGULAR, MAX_VEL_ANGULAR)
                    else:
                        # Alinhamento grosso concluído! Mude a flag para passar para a próxima fase.
                        self.get_logger().info("FASE 1 CONCLUÍDA. Passando para a FASE 2 (Alinhamento fino com LIDAR).")
                        self.alinhamento_grosso_concluido = True
                        twist.angular.z = 0.0 # Para o robô por um instante antes de começar a próxima fase
                
                # FASE 2: Alinhamento Fino com LIDAR e Aproximação Final
                else:
                    self.get_logger().info("FASE 2: Alinhamento fino com LIDAR e aproximação...")
                    
                    # Encontra o ponto mais próximo no LIDAR para o alinhamento fino
                    indices_frente = list(range(355, 360)) + list(range(0, 6))
                    min_dist = float('inf')
                    min_dist_index = -1
                    for i in indices_frente:
                        dist = self.ultima_scan.ranges[i]
                        if not np.isinf(dist) and not np.isnan(dist) and dist > 0.1:
                            if dist < min_dist:
                                min_dist = dist
                                min_dist_index = i
                    
                    if min_dist_index == -1:
                        self.get_logger().warn("FASE 2: Nenhum alvo detectado pelo LIDAR para alinhamento fino.")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        # Calcula o erro angular em relação ao ponto mais próximo do LIDAR
                        angulo_erro_graus = min_dist_index - 360 if min_dist_index > 180 else min_dist_index
                        angulo_erro_rad_lidar = np.deg2rad(angulo_erro_graus)
                        
                        # Se o alinhamento fino ainda não está bom, continue girando
                        if abs(angulo_erro_rad_lidar) > ANGULO_ALINHADO_RAD:
                            self.get_logger().info(f"Ajustando ângulo com LIDAR... Erro de {angulo_erro_graus:.1f}°")
                            twist.linear.x = 0.0
                            twist.angular.z = np.clip(KP_ANGULAR * angulo_erro_rad_lidar, -MAX_VEL_ANGULAR, MAX_VEL_ANGULAR)
                        else:
                            # Alinhamento fino concluído! Agora, controle a distância.
                            self.get_logger().info("Alinhamento fino concluído. Controlando distância...")
                            twist.angular.z = 0.0
                            
                            erro_dist_lidar = min_dist - DISTANCIA_ALVO_LIDAR
                            
                            if abs(erro_dist_lidar) < 0.05:
                                self.get_logger().info("Distância de captura alcançada! Preparando para capturar.")
                                twist.linear.x = 0.0
                                self.state = "Capturar_Bandeira"
                                self.alinhamento_grosso_concluido = False # Reseta a flag para a próxima vez
                            else:
                                twist.linear.x = np.clip(KP_LINEAR * erro_dist_lidar, 0.0, MAX_VEL_LINEAR)

            case "Capturar_Bandeira":
                if self.sub_state_garra == "ocioso":
                    self.get_logger().info("Iniciando sequência de captura da bandeira...")
                    
                    # Para o robô completamente
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                    # Inicia o PRIMEIRO passo da sequência
                    self.get_logger().info("Abaixando a garra...")
                      # 1. Abaixa a garra (aberta)
                    self.sub_state_garra = "abaixando"
                    
                    # Cria o PRIMEIRO timer para a duração do primeiro movimento
                    # Após 2 segundos, o _callback_timer_garra será chamado pela primeira vez
                    self.timer_garra = self.create_timer(2.0, self._callback_timer_garra) 

            case "Planejar_Retorno":
                self.get_logger().info("Preparando rota de retorno invertendo o caminho de ida.")
                if self.caminho_a_estrela:
                    caminho_de_volta = list(reversed(self.caminho_a_estrela))
                    
                    self.caminho_a_estrela = caminho_de_volta[4:]
                    
                    self.index_alvo = 0
                    self.state = "Retornar_Para_Base"
                    self.get_logger().info("Retorno iniciado. Seguindo rota invertida.")

            case "Retornar_Para_Base":
                if self.index_alvo >= len(self.caminho_a_estrela):
                    self.get_logger().info("Chegou à base! Iniciando depósito.")
                    self.state = "Depositar_Bandeira"
                else:
                    grid_x, grid_y = self.caminho_a_estrela[self.index_alvo]
                    target_x, target_y = self.grid_to_world(grid_x, grid_y)
                    
                    dx = target_x - self.robot_x
                    dy = target_y - self.robot_y
                    dist = np.hypot(dx, dy)
                    
                    if dist <= 0.15: # Usando um limiar um pouco maior para o retorno
                        self.index_alvo += 1
                    else:
                        angulo_alvo = np.arctan2(dy, dx)
                        erro_ang = angulo_alvo - self.robot_yaw
                        erro_ang = np.arctan2(np.sin(erro_ang), np.cos(erro_ang))
                        
                        twist.linear.x = 0.25 # Velocidade de retorno
                        twist.angular.z = 0.7 * erro_ang
                        
            case "Depositar_Bandeira":
                # Para o robô
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)

                # Sequência de depósito
                self.controlar_garra([0.1, 0.5, -0.5])  # 1. Abaixa a garra (fechada)
                self.controlar_garra([0.1, -0.5, 0.5])  # 2. Abre a garra para soltar

                self.get_logger().info("Bandeira depositada.")
                self.state = "Parado"

        self.cmd_vel_pub.publish(twist)
        self.show_grid()

    def _callback_timer_garra(self):
        """
        Callback chamado pelo timer para avançar na sequência de captura da garra.
        """
        # É uma boa prática cancelar o timer logo no início para evitar chamadas múltiplas
        if self.timer_garra is not None:
            self.timer_garra.cancel()

        # Avança para o próximo sub-estado
        if self.sub_state_garra == "abaixando":
            self.get_logger().info("Garra abaixou. Fechando...")
            self.controlar_garra([0.1, 0.5, -0.5])   # 2. Fecha a garra
            self.sub_state_garra = "fechando"
            # Crie um novo timer para a duração do fechamento
            self.timer_garra = self.create_timer(1.5, self._callback_timer_garra) # Duração de 1.5s

        elif self.sub_state_garra == "fechando":
            self.get_logger().info("Garra fechou. Levantando...")
            self.controlar_garra([-1.57, 0.5, -0.5]) # 3. Levanta a garra (use radianos, -pi/2)
            self.sub_state_garra = "levantando"
            # Crie um novo timer para a duração da elevação
            self.timer_garra = self.create_timer(2.0, self._callback_timer_garra) # Duração de 2.0s

        elif self.sub_state_garra == "levantando":
            # A sequência terminou!
            self.get_logger().info("Bandeira capturada! Planejando retorno.")
            self.bandeira_capturada = True
            self.sub_state_garra = "ocioso" # Reseta o sub-estado para a próxima vez
            
            # Muda o estado principal do robô
            self.state = "Planejar_Retorno"

    def planejar_caminho_astar(self):
        atual_x, atual_y = self.world_to_grid(self.robot_x, self.robot_y)
        start = (atual_x, atual_y)
        goal = (self.bandeira_x, self.bandeira_y)
        inflated_grid_for_astar = self.inflate_map(self.grid_map, 2)
        self.grid_map = inflated_grid_for_astar # debug

        caminho = self.a_star(inflated_grid_for_astar, start, goal)
        if not caminho == None:
            return [x for i,x in enumerate(caminho) if i%4 == 0][2:] #ignora os primeiros pixels para não entrar em loop de recalcular sem se mover
        else:
            return None


    def world_to_grid(self, x, y):
        map_center = self.map_size // 2
        grid_x = int(map_center + (x / self.resolution))
        grid_y = int(map_center + (y / self.resolution))
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        map_center = self.map_size // 2
        world_x = (grid_x - map_center) * self.resolution
        world_y = (grid_y - map_center) * self.resolution
        return world_x, world_y

    def inflate_map(self, grid, inflation_radius_cells):
        rows, cols = grid.shape
        inflated_grid = np.copy(grid) 

        obstacle_indices_y, obstacle_indices_x = np.where(grid == 1.0)

        for r, c in zip(obstacle_indices_y, obstacle_indices_x):
            for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
                for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                   

                    nr, nc = r + dr, c + dc 

                    if 0 <= nr < rows and 0 <= nc < cols:
                        if inflated_grid[nr, nc] == 0.0:
                            inflated_grid[nr, nc] = 0.95  # Marca a célula inflada como semi-obstáculo (0.95)

        return inflated_grid


    def pose_callback(self, msg: Pose):
        pos = msg.position
        ori = msg.orientation

        self.robot_x = pos.x
        self.robot_y = pos.y

          # Converte orientação quaternion → yaw
        q = (ori.x, ori.y, ori.z, ori.w)
        _, _, self.robot_yaw = euler_from_quaternion(q)

        # Converte posição do mundo → grid
        grid_x, grid_y = self.world_to_grid(self.robot_x, self.robot_y)

        # Marca a célula do robô no grid
        if self.is_in_map(grid_x, grid_y):
            self.grid_map[grid_y, grid_x] = 0.5  # 0.5 = posição atual do robô



        # Log informativo
        # self.get_logger().info(
        #     f'Pose recebida -> Posição: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}, '
        #     f'Orientação (quaternion): x={ori.x:.2f}, y={ori.y:.2f}, z={ori.z:.2f}, w={ori.w:.2f}, '
        #     f'Yaw: {np.degrees(self.robot_yaw):.2f}°'
        # )
        if not self.pos_inicial_salva:
            self.robot_init_x = self.robot_x
            self.robot_init_y = self.robot_y
            self.pos_inicial_salva = True
            # self.get_logger().info(f'Posição inicial salva: ({self.robot_init_x:.2f}, {self.robot_init_y:.2f})')

        if self.pos_inicial_salva:
            init_grid_x, init_grid_y = self.world_to_grid(self.robot_init_x, self.robot_init_y)
            if self.is_in_map(init_grid_x, init_grid_y):
                self.grid_map[init_grid_y, init_grid_x] = 0.9

        self.odom_received = True

    def a_star(self, grid, start, goal):
        def heuristic(a, b):
            # Distância de Manhattan (adequada para grids 4-direções)
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # Direções possíveis (4-vizinhança)
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        open_set = []
        heapq.heappush(open_set, (fscore[start], start))
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
                
            close_set.add(current)
            
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                # Verifica se está dentro do grid
                if not (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]):
                    continue
                    
                # Verifica se não é obstáculo (0 = livre, 1 = obstáculo)
                x, y = neighbor
                if grid[y, x] >= 0.95:
                    continue
                    
                # Calcula custo temporário
                tentative_g_score = gscore[current] + 1  # Custo entre células adjacentes = 1
                
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue
                    
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (fscore[neighbor], neighbor))
        
        return None  # Caminho não encontrado
  


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
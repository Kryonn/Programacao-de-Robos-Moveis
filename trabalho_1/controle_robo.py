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
        self.levanta_garra()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Variáveis gerais
        self.bridge = CvBridge()            # Ponte entre a imagem ROS e OpenCv
        self.yaw_ant = 0                    # Ângulo antes do giro
        self.dir = "Direita"                # Direção do giro
        self.kp = 0.005                     # Ganho proporcional
        self.error = 0.0                    # Erro direcional

        self.dist_garra  = 0.5              # Distancia máxima para identificar como garra
        self.dist_obstaculo  = 0.6          # Distancia mínima para obstáculo identificado
        self.angulo_necessario = 40         # Giro do robô
        self.giro_inicio = 0
        self.frontal_scan_angle_deg = 15 
        # Estados
        self.state = "Busca"                # "Busca", "Giro_inicio", "Giro", "Giro_fim", 
                                            # "Alinha_bandeira", "Andar_bandeira".

        # Flags
        self.giro = False                   # Condição de início do giro 
        self.bandeira_identificada = False  # Condição de busca da bandeira
        self.obstaculo_a_frente = False     # Condição de giro
        
        self.map_size = 200  # Ex: 200x200 células
        self.resolution = 0.1  # Cada célula = 10 cm
        self.grid_map = np.zeros((self.map_size, self.map_size))
        self.simular_bandeira = True
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pos_inicial_salva = False
        self.robot_init_x = 0.0
        self.robot_init_y = 0.0
        self.ultima_scan = None
        self.bandeira_mapeada = False
        self.flag_recalcular = False
        self.odom_received = False
        self.bandeira_posicoes = []
        # self.contador = 0
         

    def levanta_garra(self):
        self.get_logger().info('LEVANTANDO A GARRA\n')
        command_gripper = Float64MultiArray()
        command_gripper.data = [-(np.pi), -0.5, 0.5]

        # Tempo inicial
        inicio = self.get_clock().now()

        # Envia o comando pelo tempo em 'duracao'
        duracao = 5e9   # n segundos = n * 10^9 ns

        while (self.get_clock().now() - inicio).nanoseconds < duracao:  
            self.gripper_pub.publish(command_gripper)

        self.get_logger().info('FINALIZA LEVANTAR A GARRA\n')
        

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
       
        leituras_validas = [
            (msg.ranges[i], i) for i in indices_frente 
            if not np.isnan(msg.ranges[i]) and msg.ranges[i] > self.dist_garra
        ]

        if leituras_validas:
            # Encontra a leitura com a menor distância dentro do cone
            menor_distancia, indice_original = min(leituras_validas, key=lambda item: item[0])

            if menor_distancia < self.dist_obstaculo:
                self.obstaculo_a_frente = True
                
                self.dir = "Direita" if 0 <= indice_original <= angle_deg_cada_lado else "Esquerda"
                
                self.get_logger().info('Obstáculo detectado a {:.2f}m à frente'.format(menor_distancia) + f', na {self.dir}')
            else:
                self.obstaculo_a_frente = False
        else:
            self.obstaculo_a_frente = False

        distancias_frente = [msg.ranges[i] for i in indices_frente if (not np.isnan(msg.ranges[i]) and msg.ranges[i] > self.dist_garra)]

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

    
    # def imu_callback(self, msg: Imu):
        # Extraindo o quaternion da mensagemAdd commentMore actions
        # orientation_q = msg.orientation
        # quat = [
        #     orientation_q.x,
        #     orientation_q.y,
        #     orientation_q.z,
        #     orientation_q.w
        # ]

        # # Conversão para Euler usando SciPy
        # r = R.from_quat(quat)
        # roll, self.pitch, self.robot_yaw = r.as_euler('xyz', degrees=True)

        # self.robot_yaw = (yaw + 360) % 360


        # Exibindo resultados
        # self.get_logger().info('IMU Data Received:')
        # self.get_logger().info(
        #     f'Orientation (Euler): Roll={roll:.2f}°, '
        #     f'Pitch={pitch:.2f}°, Yaw={yaw:.2f}°'
        # )
        # self.get_logger().info(
        #     f'Angular velocity: [{msg.angular_velocity.x:.2f}, '
        #     f'{msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}] rad/s'
        # )
        # self.get_logger().info(
        #     f'Linear acceleration: [{msg.linear_acceleration.x:.2f}, '
        #     f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}] m/s²'
        # )
        # pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    def camera_callback(self, msg: Image):
        if self.simular_bandeira:
            self.simular_bandeira_detectada()
            return
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Extraindo os valores relacionados ao tamanho da imagem gerada
        height, width, _ = frame.shape

        # Define a cor-alvo em BGR
        target_color = np.array([227, 73, 0])  # [57, 66, 0] é o BGR da bandeira(verde escuro)

        # Cria máscara para cor exata
        mask = cv2.inRange(frame, target_color, target_color)

        # Mostra a máscara em uma janela para debug
        cv2.imshow('Mascara de Blobs #004239', mask)
        cv2.waitKey(1)

        # Detecta contornos (blobs)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Atualizando a flag
        self.bandeira_identificada = False

        # Verifica se a bandeira foi identificada
        if contours:
            # self.get_logger().info(f'{len(contours)} blob(s) encontrados com cor #004239:')
            for i, cnt in enumerate(contours):
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])       # Posição horizontal da bandeira
                    cy = int(M['m01'] / M['m00'])       # Posição vertical da bandeira
                    # self.get_logger().info(f'  Blob {i+1}: posição (x={cx}, y={cy})')
                    self.error = cx - width / 2         # Cálculo do erro do giro
                    self.bandeira_identificada = True   # Atualizando a flag

                    # Quando a bandeira estiver centralizada
                    if abs(self.error) < 5 and not self.bandeira_mapeada:  # Limiar ajustável (em pixels)
                        if self.ultima_scan is not None:
                            # Pega a distância à frente (ângulo 0°)
                            num_ranges = len(self.ultima_scan.ranges)
                            indice_frente = num_ranges // 2  # Assume 0° bem no centro
                            distancia = self.ultima_scan.ranges[indice_frente]

                            if not np.isnan(distancia) and distancia != float('inf'):
                                # Calcula a posição global da bandeira
                                bandeira_x = self.robot_x + distancia * np.cos(self.robot_yaw)
                                bandeira_y = self.robot_y + distancia * np.sin(self.robot_yaw)

                                # Converte para grid
                                grid_x, grid_y = self.world_to_grid(bandeira_x, bandeira_y)

                                if self.is_in_map(grid_x, grid_y):
                                    if len(self.bandeira_posicoes) < 4:
                                        self.bandeira_posicoes.append((grid_x, grid_y))

                                    # Se já tiver posições suficientes, calcula a média
                                    if len(self.bandeira_posicoes) == 4:
                                        media_x = int(sum(pos[0] for pos in self.bandeira_posicoes) / 4)
                                        media_y = int(sum(pos[1] for pos in self.bandeira_posicoes) / 4)

                                        # Marca no grid a posição média
                                        if self.is_in_map(media_x, media_y):
                                            self.grid_map[media_y, media_x] = 0.75  # Marca bandeira na posição média
                                            # self.get_logger().info(f'Bandeira mapeada na média ({media_x}, {media_y})')
                                            self.bandeira_x = media_x
                                            self.bandeira_y = media_y
                                            self.bandeira_mapeada = True
                                            self.state = "Planejar_Caminho"
                            else:
                                self.get_logger().warn('Distância LIDAR inválida para bandeira detectada.')

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
        self.get_logger().info(f'{self.state}') # imprime o estado atual do robô
        
        # self.contador += 1

        # Transição dos estados
        match self.state:
            case "Afasta":
                # Tempo inicial
                inicio = self.get_clock().now()

                # Envia o comando pelo tempo em 'duracao'
                duracao = 2e9   # n segundos = n * 10^9 ns

                while (self.get_clock().now() - inicio).nanoseconds < duracao:  
                    twist.linear.x = -0.2
                    self.cmd_vel_pub.publish(twist)

                self.giro_inicio = time.time()
                self.state = "Giro"

            # Giro: faz o robô girar para o lado que foi definido
            case "Giro":
                twist.linear.x = 0.15

                if self.dir == "Direita":
                    twist.angular.z = 0.3
                else:
                    twist.angular.z = -0.3

                tempo_decorrido = time.time() - self.giro_inicio
                tempo_necessario =  np.radians(self.angulo_necessario)/ abs(twist.angular.z)


                # Terminou o primeiro giro
                if tempo_decorrido >= tempo_necessario:
                    self.get_logger().info("Fim do Giro")

                    twist.angular.z = 0.0
                    
                    if not self.obstaculo_a_frente:
                        # Se a bandeira já foi mapeada, segue o caminho
                        if (self.bandeira_mapeada):
                            self.flag_recalcular = True
                            self.state = "Seguir_Caminho"
                        # Senão, continua a busca
                        else:
                            self.state = "Busca"
                    else:
                        # Se ainda há obstáculo, atualiza o tempo de inicio e continua no giro
                        self.giro_inicio = time.time()
                        self.state = "Giro"
        
            # Busca: anda para frente até encontrar um obstáculo ou a bandeira
            case "Busca":
                # [DEBUG] espera um pouco para simular a bandeira
                # if self.contador >= 10:
                #     self.simular_bandeira = True

                twist.linear.x = 0.5
                if self.obstaculo_a_frente:
                    self.state = "Afasta"

                else:
                    if self.bandeira_identificada:
                        self.state = "Alinha_bandeira"

            # Alinha_bandeira: gira em direção à bandeira
            case "Alinha_bandeira":
                twist.linear.x = 0.0
                twist.angular.z = -self.kp * self.error
                if not self.bandeira_identificada:
                    self.state = "Busca"
                else:
                    if abs(self.error) < 2:
                        self.state = "Andar_bandeira"

            # Andar_bandeira: anda até a bandeira
            case "Andar_bandeira":
                twist.linear.x = 0.2
                if self.obstaculo_a_frente:
                    self.state = "Afasta"

            case "Planejar_Caminho":
                time.sleep(3)
                if self.bandeira_mapeada:
                    self.flag_recalcular = False
                    self.caminho_a_estrela = self.planejar_caminho_astar()

                    if not self.caminho_a_estrela == None:
                        self.get_logger().warn(f'Caminho: {self.caminho_a_estrela}') # imprime caminho
                        # for a, b in self.caminho_a_estrela:
                        #     self.grid_map[b, a] = 0.2
                        self.state = "Seguir_Caminho"
                        self.index_alvo = 0
                    else:
                        self.get_logger().warn("Caminho A* não encontrado.")
                        self.state = "Busca"

            case "Seguir_Caminho":
                if self.caminho_a_estrela == None:
                    self.state = "Planejar_Caminho"

                elif self.index_alvo >= len(self.caminho_a_estrela):
                    self.get_logger().info("Chegou à bandeira!")
                    self.state = "Parado"
                else:
                    if self.obstaculo_a_frente:
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

                            twist.linear.x = 0.25  # Velocidade moderada e constante
                            twist.angular.z = 0.7 * erro_ang # Velocidade angular proporcional ao erro

            case "Parado":
                twist.linear.x = 0.0
                twist.angular.z = 0.0


        self.cmd_vel_pub.publish(twist)
        self.show_grid()
        # self.get_logger().warn(f'{self.state}') # imprime o estado atual do robô

    def planejar_caminho_astar(self):
        atual_x, atual_y = self.world_to_grid(self.robot_x, self.robot_y)
        start = (atual_x, atual_y)
        goal = (self.bandeira_x, self.bandeira_y)
        inflated_grid_for_astar = self.inflate_map(self.grid_map, 2)
        self.grid_map = inflated_grid_for_astar # debug

        caminho = self.a_star(inflated_grid_for_astar, start, goal)
        if not caminho == None:
            return [x for i,x in enumerate(caminho) if i%3 == 0][2:] #ignora os primeiros pixels para não entrar em loop de recalcular sem se mover
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


# pkill -f gazebo && pkill -f roslaunch && pkill -f rosmaster
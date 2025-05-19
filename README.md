# PRM - Programação de Robôs Móveis

**Disciplina SSC0712**  
Oferecida para os cursos de Engenharia de Computação e áreas afins na **USP São Carlos**

Este repositório contém o material da disciplina *Programação de Robôs Móveis*, focada no desenvolvimento de soluções em robótica móvel utilizando **ROS 2 Humble** e o simulador **Gazebo Fortress**.

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz / Gazebo GUI
- [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard)

---

## 🚀 Como utilizar o pacote

### 1. Clonar o repositório

Acesse a pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/Kryonn/Programacao-de-Robos-Moveis trabalho1
````

### 2. Instalar dependências

Instale as dependências do pacote com:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Certifique-se de ter rodado previamente `sudo rosdep init` e `rosdep update`, se for a primeira vez usando o `rosdep`.

### 3. Compilar o workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select trabalho_1
```

### 4. Atualizar o ambiente do terminal

```bash
source install/local_setup.bash
```

---

## 🧪 Executando a simulação

### 1. Iniciar o mundo no Gazebo

```bash
ros2 launch trabalho_1 inicia_simulacao.launch.py
```

### 2. Carregar o robô no ambiente

Em um **novo terminal** (não se esqueça de `source install/local_setup.bash`):

```bash
ros2 launch trabalho_1 carrega_robo.launch.py
```

### 3. Controle automático (demonstração)

Em outro terminal:

```bash
ros2 run trabalho_1 controle_robo
```

### 4. **Controle manual (alternativa ao passo 3)**

Você pode controlar o robô usando o teclado, como alternativa ao controle automático:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Instalar `teleop_twist_keyboard` (caso não esteja disponível)

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

> **Importante**: execute **o passo 3 *ou* o passo 4**, dependendo se deseja usar o controle automático ou manual.

```python
#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam', self.camera_callback, 10)

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False
        self.state = "Busca"
        self.giro_dir = True
        self.deslocamento_bandeira = 0.0
        self.bandeira_detectada = False

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
        # Mensagens com imagens da câmera!
        # Convertendo a imagem para BRG
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convertendo a imagem para HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definindo a faixa de cores da qual a cor da bandeira está inserida
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Criando as máscaras
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Enviando o sinal, caso a bandeira tenha sido econtrada
        self.bandeira_detectada = False
        self.deslocamento_bandeira = 0.0  # Zera o deslocamento

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                centro_bandeira = x + w / 2
                largura_imagem = frame.shape[1]
                centro_imagem = largura_imagem / 2

                # deslocamento normalizado entre -1 (esquerda) e 1 (direita)
                self.deslocamento_bandeira = (centro_bandeira - centro_imagem) / centro_imagem

                self.bandeira_detectada = True
                break
    
    def state_transition(self):
        match self.state:
            case "Busca":
                if self.bandeira_detectada:
                    self.state = "Busca_bandeira"
                else:
                    if self.obstaculo_a_frente:
                        self.state = "Giro"        
            case "Giro":
                if self.bandeira_detectada:
                    self.state = "Busca_bandeira"
                else:
                    if not self.obstaculo_a_frente:
                        self.state = "Busca"
            case "Busca_bandeira":
                if self.obstaculo_a_frente:
                    self.state = "Contorno"
            case "Contorno":
                if not self.obstaculo_a_frente and self.bandeira_detectada:
                    self.state = "Busca_bandeira"


    def move_robot(self):
        twist = Twist()
        match self.state:
            case "Busca":
                twist.linear.x = 0.1
            case "Giro":
                twist.linear.x = 0.0
                if self.giro_dir == "Esquerda":
                    twist.angular.z = 0.3
                    
                else:
                    twist.angular.z = -0.3
            case "Busca_bandeira":
                if self.obstaculo_a_frente:
                    self.state = "Contorno"
                else:
                    twist.linear.x = 0.1
                    ganho = 0.5
                    twist.angular.z = -self.deslocamento_bandeira * ganho
            case "Contorno":
                if self.obstaculo_a_frente:
                    if self.deslocamento_bandeira > 0:
                        twist.angular.z = -0.1
                    else:
                        twist.angular.z = 0.1
                else:
                    self.state = "Busca"
                    


        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

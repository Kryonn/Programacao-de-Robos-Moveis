# PRM - Programação de Robôs Móveis

**Disciplina SSC0712**  
Oferecida para os cursos de Engenharia de Computação e áreas afins na **USP São Carlos**

Este repositório contém o Trabalho 2 da disciplina *Programação de Robôs Móveis*, cujo objetivo é desenvolver soluções em robótica móvel utilizando **ROS 2 Humble** e o simulador **Gazebo Fortress**. Nosso robô utiliza sensores como lidar e câmera para identificar uma bandeira, mapear o ambiente em um grid e navegar até ela.

---

## 📦 Tecnologias Utilizadas

- **ROS 2 Humble**
- **Gazebo Fortress**
- **Python**
- **RViz / Gazebo GUI**
- [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard)

---

## ⚙️ Instruções para Uso

### 1. Clonar o Repositório

Navegue até a pasta `src` do seu workspace ROS 2 e clone o projeto:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/Kryonn/Programacao-de-Robos-Moveis trabalho_1
```

### 2. Instalar Dependências

Volte para a raiz do workspace e instale as dependências necessárias:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **Nota:** Caso seja a primeira vez usando `rosdep`, execute antes:

```bash
sudo rosdep init
rosdep update
```

Se houver erros, tente executar:

```bash
sudo apt update
sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
```

### 3. Compilar o Workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select trabalho_1
```

> **Dica:** Caso não tenha o `colcon` instalado:

```bash
sudo apt install python3-colcon-common-extensions -y
```

### 4. Atualizar o Ambiente do Terminal

Após a compilação, execute:

```bash
source install/local_setup.bash
```

---

## ▶️ Executando a Simulação
### Script simplificado
Para facilitar, você pode usar o script automático que compila o projeto e executa os três componentes da simulação (Gazebo, robô e controle de movimentação):
```bash
cd ~/ros2_ws/src/trabalho_1
./run.sh
```
Ele automatiza todas as etapas para você. Se preferir, também pode rodar manualmente cada parte da simulação, seguindo os passos abaixo:

### 1. Iniciar o Mundo no Gazebo

```bash
ros2 launch trabalho_1 inicia_simulacao.launch.py
```

### 2. Carregar o Robô no Ambiente

Em um novo terminal (não esqueça de rodar `source install/local_setup.bash`):

```bash
ros2 launch trabalho_1 carrega_robo.launch.py
```

### 3. Controlar o Robô

Em outro terminal, execute:

```bash
ros2 run trabalho_1 controle_robo
```

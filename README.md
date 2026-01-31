# Visão de Braço Robótico Humanóide (Humanoid Robotic Arm Vision)

Este projeto implementa um sistema completo de controlo para um braço robótico humanóide, utilizando Visão Computacional (MediaPipe) para detetar movimentos da mão/braço humano e replicá-los no robô.

O sistema está dividido em 3 módulos principais: **PC (Debugger)**, **Raspberry Pi (Cérebro)** e **Arduino Mega (Driver)**.

---

## 📂 Estrutura do Projeto

O código está organizado por hardware e, para cada componente, existem versões em **Inglês** (padrão) e **Português** (sufixo `_PT`).

```
HumanoidRoboticArmVision/
├── code/
│   ├── PC/                         # Código para COMPUTADOR
│   │   ├── VisionDebugger_PC.py    # (Inglês) Teste de visão no PC
│   │   └── VisionDebugger_PC_PT.py # (Português) Versão traduzida
│   │
│   ├── RPi/                        # Código para RASPBERRY PI 5
│   │   └── Final_RPI/
│   │       ├── ArmController.py    # (Inglês) Controlador principal
│   │       └── ArmController_PT.py # (Português) Versão traduzida
│   │
│   └── arduino/                    # Código para ARDUINO MEGA
│       └── Final_Ard/
│           ├── MotorDriver/        # (Inglês) Sketch Arduino
│           └── MotorDriver_PT/     # (Português) Sketch Arduino
│
└── REQUIREMENTS.txt                # Lista de bibliotecas necessárias
```

---

## 🚀 Módulos e Funcionalidades

### 1. PC: Vision Debugger (`code/PC`)

- **Função**: Ferramenta de desenvolvimento para testar a detecção da IA sem precisar do robô ligado.
- **Hardware**: Webcam padrão.
- **Tecnologia**: Usa `mediapipe.tasks` (API Nova) com aceleração GPU (se disponível).
- **Ficheiros Extra Necessários**: `pose_landmarker_lite.task` e `hand_landmarker.task` devem estar na mesma pasta.

### 2. Raspberry Pi: Arm Controller (`code/RPi`)

- **Função**: O "cérebro" do robô. Captura vídeo, processa a IA e envia comandos para o Arduino.
- **Hardware**: Raspberry Pi 5 + Pi Camera.
- **Tecnologia**: Usa `mp.solutions` (API Legada/Padrão) para compatibilidade e facilidade de instalação no Linux. Inclui comunicação Serial.

### 3. Arduino: Motor Driver (`code/arduino`)

- **Função**: Recebe ângulos do Raspberry Pi e controla os servos.
- **Hardware**: Arduino Mega 2560 + Driver PCA9685 (I2C).
- **Bibliotecas**: `Adafruit_PWMServoDriver`.

---

## 🛠️ Requisitos e Instalação

Consulte o ficheiro `REQUIREMENTS.txt` para versões detalhadas.

### PC (Windows)

```bash
pip install opencv-python mediapipe numpy
# Certifique-se que os ficheiros .task estão na pasta PC/
```

### Raspberry Pi 5

```bash
# Instalar MediaPipe (ignorar aviso de sistema gerido externamente)
pip install mediapipe --break-system-packages
pip install opencv-python pyserial numpy
```

### Arduino

- Instalar a biblioteca "Adafruit PWM Servo Driver Library" através do Gestor de Bibliotecas do Arduino IDE.

---

## 🎮 Como Usar

1.  **Arduino**: Carregue o código `MotorDriver.ino` (ou `_PT`) para o Arduino Mega.
2.  **Ligações**: Conecte o Arduino ao Raspberry Pi via USB.
3.  **Raspberry Pi**: Execute o script:
    ```bash
    python3 code/RPi/Final_RPI/ArmController_PT.py
    ```
4.  **PC (Opcional)**: Se quiser apenas testar a visão no seu computador:
    ```bash
    python code/PC/VisionDebugger_PC_PT.py
    ```

---

## 🌍 Idiomas

Todo o código principal foi traduzido.

- Use os ficheiros sem sufixo para **Inglês** (comentários e variáveis em EN).
- Use os ficheiros `_PT` para **Português de Portugal** (comentários didáticos e variáveis em PT).

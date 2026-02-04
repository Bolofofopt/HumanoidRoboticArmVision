# Braço Robótico Humanoide Controlado por Visão Computacional (RIA-G7)

> **Projeto Integrado - TeSP Robótica e Inteligência Artificial** > Escola Superior Náutica Infante D. Henrique

![Status](https://img.shields.io/badge/Status-Concluído-success)
![Python](https://img.shields.io/badge/Python-3.x-blue)
![C++](https://img.shields.io/badge/C%2B%2B-Arduino-blue)
![Hardware](https://img.shields.io/badge/Raspberry_Pi-5-red)

## 📋 Sobre o Projeto

Este repositório contém o código-fonte e a documentação de um **braço robótico antropomórfico** desenvolvido para mimetizar os movimentos do membro superior humano em tempo real.

O sistema substitui controladores físicos tradicionais por algoritmos de **Visão Computacional**, permitindo uma interação natural "homem-máquina" onde o corpo do operador funciona como o comando. A estrutura mecânica foi produzida integralmente via manufatura aditiva (Impressão 3D em PLA), baseada no projeto *InMoov*.

## ⚙️ Funcionalidades e Cinemática

O robô possui um total de **7 Graus de Liberdade (DoF)**:

* **Mão Robótica (5 DoF):** Controlo independente dos 5 dedos (aberto/fechado) utilizando geometria vetorial.
* **Cotovelo / Pitch (1 DoF):** Movimento de extensão e flexão baseado na altura relativa do pulso.
* **Base / Yaw (1 DoF):** Rotação da base (Esquerda/Centro/Direita) controlada pela angulação do ombro.
* **Rotação do Pulso (Roll):** Ajuste da orientação da mão com compensação dinâmica de eixos.

---

## 🛠️ Arquitetura de Hardware

O projeto utiliza uma **arquitetura de processamento distribuído** para garantir baixa latência e estabilidade de sinal.

### Diagrama de Blocos
A estrutura divide-se em três unidades de processamento:

| Unidade | Função Principal | Comunicação |
| :--- | :--- | :--- |
| **Raspberry Pi 5 (8GB)** | Processamento de imagem (MediaPipe), IA e cálculo de ângulos. | UART (GPIO 14) -> Arduino MEGA |
| **Arduino MEGA 2560** | **Mestre:** Recebe coordenadas, controla servos PWM e coordena o sistema. | I2C (Servos) / UART (Uno) |
| **Arduino Uno** | **Escravo:** Dedicado exclusivamente ao controlo preciso do Motor de Passo da Base. | Sinais Digitais (Driver) |

### Lista de Componentes Chave
* **Atuadores:**
    * 1x Servo DS5160 (60kgf.cm) - Cotovelo.
    * 6x Servos MG996R (10kgf.cm) - Dedos e Pulso.
    * 1x Motor de Passo NEMA 17 (17HS4401S) - Base.
* **Drivers:**
    * PCA9685 (PWM I2C de 16 canais).
    * CNC Shield V3 + Driver A4988.
* **Energia:** Fontes independentes para Lógica (5V), Servos (7V) e Motor de Passo (12V) para isolamento de ruído.

---

## 💻 Arquitetura de Software

### Visão Computacional (Python)
O núcleo de inteligência corre no Raspberry Pi utilizando a framework **MediaPipe** da Google.
* **Deteção Robusta:** Em vez de usar a distância euclidiana (que falha com a profundidade), o algoritmo calcula o **ângulo** entre três pontos articulares para determinar se um dedo está fletido.
* **Multithreading:** A captura de vídeo é separada do processamento para manter uma taxa de ~20 FPS.
* **Bibliotecas:** OpenCV, MediaPipe, PySerial.

### Firmware (C++)
* **Arduino MEGA:** Faz o *parsing* da string recebida, converte ângulos em sinais PWM e gere a comunicação I2C.
* **Arduino Uno:** Implementa uma máquina de estados para controlar a aceleração e direção do motor de passo sem bloquear o processador principal.

---

## 📡 Protocolo de Comunicação

A comunicação entre o PC/Raspberry Pi e o Arduino MEGA é feita via **UART** através de uma string formatada com marcadores de início (`$`) e fim (`\n`).

**Estrutura da Trama:**
```text
$<Base>,<Flexão>,<D1>,<D2>,<D3>,<D4>,<D5>,<Rotação>\n
```

## 🚀 Instalação e Execução
Montagem: Siga o esquema elétrico detalhado (ver diagrama Cirkit Designer).

Arduino:
* Carregue o firmware Slave no Arduino Uno.
* Carregue o firmware Master no Arduino MEGA.
* Raspberry Pi / PC:
*     Instale as dependências: pip install opencv-python mediapipe pyserial.
*     Execute o script principal em Python.

Nota: O sistema suporta modo headless para operação remota via Raspberry Pi Connect.

## 📈 Resultados e Limitações
**Desempenho:** O sistema atinge uma taxa de atualização estável (15-20 FPS), adequada para telepresença.

Limitações Atuais:
* Movimentos discretos (estados binários) para os dedos.
* Falta de feedback sensorial (haptics).
* Ausência do grau de liberdade Forward Pitch no ombro.

## 👥 Autores (Grupo RIA-G7)
- Henrique Abrantes (15196)

- Christian Rodrigues (15202)

- Rodrigo Maria (15217)

## 📚 Referências
Este projeto baseia-se no trabalho de G. Langevin (InMoov) e documentação técnica do MediaPipe e Arduino. Para detalhes completos, consulte o relatório final no repositório.

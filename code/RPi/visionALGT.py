import cv2
import mediapipe as mp # type:ignore
import math
import statistics
import time
import serial # type:ignore
from picamera2 import Picamera2 # type:ignore
import numpy as np

# ==============================================================================
#                           CONFIGURAÇÕES GERAIS
# ==============================================================================

# Definições da Resolução e Taxa de Atualização da Câmara
# 640x480 é ideal para o RPi 5 processar o MediaPipe a 30 FPS estáveis.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS_TARGET = 30

# Configuração da Porta Série (Comunicação com o Arduino)
# /dev/ttyAMA0 é geralmente a porta UART no header GPIO do RPi.
SERIAL_PORT = "/dev/ttyAMA0" 
BAUD_RATE = 115200
SEND_INTERVAL = 0.1  # Controla a frequência de envio (100ms = 10 mensagens/s)

# Limiares (Thresholds) para considerar um dedo "Aberto"
# Se o ângulo do dedo for superior a X graus, conta como aberto (1).
FINGER_THRESHOLDS = {
    "polegar": 150.0,
    "indicador": 160.0,
    "medio": 150.0,
    "anelar": 150.0,
    "mindinho": 140.0
}

# Limiar de diferença de altura (Y) para detetar flexão do braço
Y_DIFF_THRESHOLD = 0.155

# Intervalos de ângulos para classificar a orientação da base (Yaw)
ORIENT_RANGES = {
    "left": (0, 70),     # 0 a 70 graus -> Esquerda
    "front": (70, 125),  # 70 a 125 graus -> Frente
    "right": (125, 180)  # 125 a 180 graus -> Direita
}

# Índices dos landmarks do MediaPipe Hands para cada dedo
# Cada lista contém [Base, Articulação Central, Ponta]
FINGER_IDX = {
    "polegar": [1, 2, 4],
    "indicador": [5, 6, 8],
    "medio": [9, 10, 12],
    "anelar": [13, 14, 16],
    "mindinho": [17, 18, 20]
}

# ==============================================================================
#                       INICIALIZAÇÃO DE BIBLIOTECAS
# ==============================================================================

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Estilos de desenho para o esqueleto na imagem
drawing_spec_thin = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
drawing_spec_conn = mp_drawing.DrawingSpec(thickness=1)

# Inicializar Comunicação Série
# Utiliza um bloco try-except para não crashar se o Arduino não estiver ligado.
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Aguarda o reset do Arduino
    print(f"Ligado ao Arduino em {SERIAL_PORT}")
except Exception as e:
    print(f"Nao foi possivel ligar ao Arduino: {e}")
    print(f"   Verifica a porta. Podes usar: ls /dev/tty*")
    arduino = None

# ==============================================================================
#                  FUNÇÕES AUXILIARES (LÓGICA MATEMÁTICA)
# ==============================================================================

def angulo_3pts(a, b, c):
    """
    Calcula o ângulo (em graus) formado no ponto 'b' pelos segmentos ab e bc.
    Utiliza o produto escalar (dot product) de vetores.
    """
    # Vetor BA e Vetor BC
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    
    # Produto Escalar
    num = ax*cx + ay*cy
    # Produto das magnitudes
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-9 # +1e-9 evita divisão por zero
    
    # Cálculo do cosseno e conversão para graus
    cosv = max(-1.0, min(1.0, num/den))
    return math.degrees(math.acos(cosv))

def pair_hand_to_pose_side(hand_landmarks, pose_landmarks):
    """
    Determina se a mão detetada é a Esquerda ou a Direita.
    O MediaPipe Hands às vezes troca as mãos; esta função corrige isso comparando
    a distância da mão aos pulsos (wrists) detetados pelo MediaPipe Pose.
    """
    if pose_landmarks is None:
        return None
        
    # Centro da mão
    hand_cx = statistics.mean([lm.x for lm in hand_landmarks.landmark])
    hand_cy = statistics.mean([lm.y for lm in hand_landmarks.landmark])
    
    # Landmarks dos pulsos no corpo (15=Esq, 16=Dir)
    lw = pose_landmarks.landmark[15]
    rw = pose_landmarks.landmark[16]
    
    # Distância Euclidiana
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    
    return "Left" if dl < dr else "Right"

def classify_arm_orientation(angle_deg):
    """
    Classifica a orientação do braço em zonas para controlar a base.
    Retorna: (Texto para Display, Valor Numérico para Arduino)
    """
    a = angle_deg
    if ORIENT_RANGES["left"][0] <= a <= ORIENT_RANGES["left"][1]:
        return "Esq", 0  # Zona Esquerda
    if ORIENT_RANGES["front"][0] < a <= ORIENT_RANGES["front"][1]:
        return "Frente", 1 # Zona Central
    if ORIENT_RANGES["right"][0] < a <= ORIENT_RANGES["right"][1]:
        return "Dir", 2  # Zona Direita
    return "Indet", 1  # Por defeito assume Frente

# ==============================================================================
#                             LOOP PRINCIPAL
# ==============================================================================

# Inicializar Picamera2 (Método eficiente para RPi 5)
print("A inicializar Picamera2...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
    controls={"FrameRate": FPS_TARGET}
)
picam2.configure(config)
picam2.start()

print("Camera iniciada. A aguardar estabilizacao...")
time.sleep(2)

# Instanciar modelos do MediaPipe
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,       # 0 é mais rápido, 2 é mais preciso
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,          # Apenas uma mão para controlar o robô
    model_complexity=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

# Variáveis de controlo de tempo
frame_count = 0
start_time = time.time()
last_send = time.time()

print("Sistema iniciado. Pressiona Ctrl+C para sair.")

try:
    while True:
        # 1. Captura de Imagem (Array NumPy)
        frame = picam2.capture_array()
        
        # 2. Processamento MediaPipe
        # O MediaPipe espera RGB, e a Picamera já fornece RGB, logo não precisamos converter BGR.
        pose_res = pose.process(frame)
        hands_res = hands.process(frame)
        
        # Copia do frame para desenhar por cima (Output)
        out = frame
        
        # 3. Lógica de Deteção da Mão
        if hands_res.multi_hand_landmarks:
            hand_lm = hands_res.multi_hand_landmarks[0]
            
            # Tenta identificar se é mão esquerda ou direita usando o corpo como referência
            side = pair_hand_to_pose_side(hand_lm, pose_res.pose_landmarks) if pose_res.pose_landmarks else "Right"
            
            # Valores por defeito
            orient_val = 1      # 1 = Frente
            flexion_val = 0     # 0 = Estendido
            ang_braco = None
            orient_text = "N/A"
            estado_braco = "N/A"
            
            # 4. Lógica de Deteção do Corpo (Braço)
            if pose_res.pose_landmarks:
                plm = pose_res.pose_landmarks.landmark
                
                # Seleciona os pontos (Ombro, Cotovelo, Pulso) consoante o lado
                if side == "Left":
                    sh = plm[11]; el = plm[13]; wr = plm[15]
                else: # Right
                    sh = plm[12]; el = plm[14]; wr = plm[16]
                
                # Calcula ângulo do braço (Geometria 2D)
                ang_braco = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
                orient_text, orient_val = classify_arm_orientation(ang_braco)
                
                # Calcula Flexão (baseado na diferença de altura Y)
                # Nota: Y cresce de cima para baixo na imagem.
                y_diff = sh.y - wr.y
                
                if y_diff > Y_DIFF_THRESHOLD:
                    estado_braco = "Flexao" # Mão acima do limiar
                    flexion_val = 1
                elif abs(y_diff) <= Y_DIFF_THRESHOLD:
                    estado_braco = "Estendido" # Mão nivelada
                    flexion_val = 0
                else:
                    estado_braco = "Baixo" # Mão muito em baixo
                    flexion_val = 0
                
                # Desenha informações do braço no ecrã
                # Nota: Usamos sem acentos (Braco) para evitar erros gráficos no OpenCV
                cv2.putText(out, f"Braco: {int(ang_braco)}deg {orient_text}", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(out, f"Estado: {estado_braco} ({round(y_diff, 2)})", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            # 5. Processar Dedos da Mão
            estados = {}
            for fname, ids in FINGER_IDX.items():
                # Obtém coordenadas dos 3 pontos do dedo
                a = hand_lm.landmark[ids[0]]
                b = hand_lm.landmark[ids[1]]
                c = hand_lm.landmark[ids[2]]
                
                # Calcula ângulo de abertura do dedo
                ang = angulo_3pts((a.x, a.y), (b.x, b.y), (c.x, c.y))
                thresh = FINGER_THRESHOLDS.get(fname, 150.0)
                
                # Determina se está aberto ou fechado
                aberto = ang > thresh
                estados[fname] = (aberto, ang)
            
            # 6. Desenhar estado dos dedos no ecrã
            y0 = 75
            for i, (name, (aberto, ang)) in enumerate(estados.items()):
                # Azul se aberto, Vermelho se fechado
                color = (255, 0, 0) if aberto else (0, 0, 255)
                # Formata texto: "Pol:A 170" (Polegar Aberto 170 graus)
                # Usamos name[:3] para abreviar (Pol, Ind, Med...)
                txt = f"{name[:3].capitalize()}:{'A' if aberto else 'F'} {int(ang)}"
                cv2.putText(out, txt, (10, y0 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            
            # Desenha o esqueleto da mão
            mp_drawing.draw_landmarks(
                out, hand_lm, mp_hands.HAND_CONNECTIONS,
                drawing_spec_thin, drawing_spec_conn
            )
            
            # 7. Envio de Dados via Serial (Arduino)
            # Formato da mensagem: $orientacao,flexao,indicador,medio,anelar,mindinho,polegar
            # Ordem deve corresponder à esperada pelo Arduino
            dedos = [
                1 if estados["indicador"][0] else 0,
                1 if estados["medio"][0] else 0,
                1 if estados["anelar"][0] else 0,
                1 if estados["mindinho"][0] else 0,
                1 if estados["polegar"][0] else 0
            ]
            
            # Cria a string (apenas um \n no final)
            msg = f"${orient_val},{flexion_val},{','.join(map(str, dedos))}\n\n"
            
            # Envia a cada SEND_INTERVAL segundos para não saturar o buffer
            if time.time() - last_send > SEND_INTERVAL and arduino:
                try:
                    arduino.write(msg.encode())
                    last_send = time.time()
                    # Log na consola (pode ter acentos aqui)
                    print(f"Enviado: {msg.strip()} | {orient_text} | Ang:{int(ang_braco) if ang_braco else 'N/A'} | {estado_braco}")
                except Exception as e:
                    print(f"Erro ao enviar serial: {e}")
        
        else:
            # Se não detetar mão, mostra aviso
            cv2.putText(out, "Sem mao detectada", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        
        # Desenha esqueleto do corpo se detetado
        if pose_res.pose_landmarks:
            mp_drawing.draw_landmarks(
                out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                drawing_spec_thin, drawing_spec_conn
            )
        
        # 8. Cálculo e Display de FPS (Frames Por Segundo)
        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            print(f"FPS medio: {fps:.1f}")
        
        # Mostrar imagem
        cv2.imshow("Braco+Mao RPi", out)
        if cv2.waitKey(1) & 0xFF == 27:  # Tecla ESC para sair
            break

except KeyboardInterrupt:
    print("\nInterrompido pelo utilizador")

finally:
    # Limpeza de recursos ao fechar
    picam2.stop()
    if arduino:
        arduino.close()
    cv2.destroyAllWindows()
    print("Sistema encerrado correctamente")
import cv2
import mediapipe as mp
import math
import statistics
import time
import serial
from picamera2 import Picamera2
import numpy as np

# ------------------ CONFIGURAÇÃO OTIMIZADA PARA RPI ------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS_TARGET = 30

# Configuração Série
SERIAL_PORT = "/dev/ttyAMA0"  # Verifica se é esta a porta correta no RPi 5
BAUD_RATE = 115200
SEND_INTERVAL = 0.1  # Enviar a cada 100ms

# ---------- Parâmetros Ajustáveis (Do Código Utilizador) ----------
# Limiares para dedos (ângulo mínimo para considerar aberto)
TH_POLEGAR = 150.0
TH_INDICADOR = 160.0
TH_MEDIO = 150.0
TH_ANELAR = 150.0
TH_MINDINHO = 140.0

# Limiares para orientação do braço (baseado em ângulo)
TH_BRACO_ESQUERDA = 70    # 0-70: braço para o corpo (esquerda)
TH_BRACO_CENTRO_MAX = 130 # 70-130: braço para a frente (centro)
# 130-180: braço para fora (direita)

# Limiar para flexão do braço (diferença em Y entre pulso e ombro)
TH_FLEXAO_Y = 0.1  # Se diff_y > este valor, braço fletido
# ---------------------------------------------------------------

# Inicializar MediaPipe
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

drawing_spec_thin = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
drawing_spec_conn = mp_drawing.DrawingSpec(thickness=1)

# Definição de dedos
FINGERS = {
    "polegar": [1, 2, 4],
    "indicador": [5, 6, 8],
    "medio": [9, 10, 12],
    "anelar": [13, 14, 16],
    "mindinho": [17, 18, 20]
}

# Limiares por dedo
FINGER_THRESHOLDS = {
    "polegar": TH_POLEGAR,
    "indicador": TH_INDICADOR,
    "medio": TH_MEDIO,
    "anelar": TH_ANELAR,
    "mindinho": TH_MINDINHO
}

# -------- INICIALIZAR SÉRIE --------
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Ligado ao Arduino em {SERIAL_PORT}")
except Exception as e:
    print(f"Não foi possível ligar ao Arduino: {e}")
    print(f"   Verifica a porta. Podes usar: ls /dev/tty*")
    arduino = None

# -------- FUNÇÕES DE LÓGICA (Do Utilizador) --------

def angulo_3pts(a, b, c):
    """Ângulo em b dado a,b,c. a,b,c são (x,y) ou tuplas."""
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-8
    cosv = max(-1.0, min(1.0, num/den))
    ang = math.degrees(math.acos(cosv))
    return ang

def finger_angle_open(hand_landmarks, ids):
    """Devolve ângulo na articulação intermédia para medir se dedo está estendido."""
    a = hand_landmarks.landmark[ids[0]]
    b = hand_landmarks.landmark[ids[1]]
    c = hand_landmarks.landmark[ids[2]]
    return angulo_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def hand_center(hand_landmarks):
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    return (statistics.mean(xs), statistics.mean(ys))

def pair_hand_to_pose_side(hand_cx, hand_cy, pose_landmarks):
    """Devolve 'Right' ou 'Left' segundo pulso da pose mais próximo."""
    if pose_landmarks is None:
        return None
    lw = pose_landmarks.landmark[15]
    rw = pose_landmarks.landmark[16]
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    return "Left" if dl < dr else "Right"

def determinar_orientacion_brazo(angulo):
    """Determina orientação do braço segundo o seu ângulo."""
    if angulo <= TH_BRACO_ESQUERDA:
        return "Esquerda (para o corpo)"
    elif angulo <= TH_BRACO_CENTRO_MAX:
        return "Centro (frente)"
    else:
        return "Direita (para fora)"

def determinar_flexion_brazo(hombro_y, muneca_y):
    """Determina se o braço está fletido ou estendido."""
    diff_y = abs(hombro_y - muneca_y)
    if diff_y > TH_FLEXAO_Y:
        return "Fletido", diff_y
    else:
        return "Estendido", diff_y

def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation):
    """
    Calcula a rotação da mão com lógica estrita definida pelo utilizador.
    [-1.0 (Palma) ... 1.0 (Dorso)]
    """
    thumb = hand_landmarks.landmark[4]
    pinky = hand_landmarks.landmark[20]
    
    diff = 0
    t_val = 0
    p_val = 0
    axis_used = "Indef"
    
    # Determinar contexto
    is_flexed = (flex_status == "Fletido")
    
    if is_flexed:
        # CASO 1: Fletido -> Eixo X
        # Lógica Utilizador: Palma se Thumb.X > Pinky.X -> Mapping Inverso -> -1
        t_val = thumb.x
        p_val = pinky.x
        diff = -(t_val - p_val) 
        axis_used = "X (Fletido)"
        
    else:
        # Estendido
        if "Direita" in arm_orientation:
            # CASO 3: Estendido + Direita -> Eixo Y
            # Fix: Invertido relativamente à lógica anterior.
            # Antes: -(T-P). Agora: (T-P).
            t_val = thumb.y
            p_val = pinky.y
            diff = (t_val - p_val)
            axis_used = "Y (Ext-Dir)"
            
        else:
            # CASO 2: Estendido + Corpo/Esq -> Eixo Y
            # Fix: Invertido relativamente à lógica anterior.
            # Antes: (T-P). Agora: -(T-P).
            t_val = thumb.y
            p_val = pinky.y
            diff = -(t_val - p_val)
            axis_used = "Y-Inv (Ext-Corpo)"

    # Normalização
    MAX_DIFF = 0.22
    rotation_value = max(-1.0, min(1.0, diff / MAX_DIFF))
    
    orientation = "Indefinido"
    if rotation_value < -0.3:
        orientation = "Palma (Camara)"
    elif rotation_value > 0.3:
        orientation = "Dorso (Camara)"
    else:
        orientation = "A rodar / Canto"
        
    return diff, rotation_value, orientation, axis_used

# -------- INICIALIZAÇÃO DO LOOP DA CÂMARA E MODELOS --------

# Inicializar Picamera2
print("A inicializar Picamera2...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
    controls={"FrameRate": FPS_TARGET}
)
picam2.configure(config)
picam2.start()

print("Câmara iniciada. A aguardar estabilização...")
time.sleep(2)

# Inicializar modelos com parâmetros do utilizador
pose = mp_pose.Pose(
    min_detection_confidence=0.5, 
    min_tracking_confidence=0.5,
    model_complexity=1
)
hands = mp_hands.Hands(
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=1
)

# Variáveis para FPS e envio
frame_count = 0
start_time = time.time()
last_send = time.time()

print("Sistema iniciado. Pressiona Ctrl+C para sair.")

try:
    while True:
        frame = picam2.capture_array()
        
        # Processar
        pose_res = pose.process(frame)
        hands_res = hands.process(frame)
        
        out = frame
        h, w = frame.shape[:2]

        # Variáveis para armazenar informação
        ang_brazo = None
        orientacion_brazo = "Indefinido" 
        estado_flexion = "Estendido"
        side = None
        hombro_y = 0
        muneca_y = 0
        diff_y = 0
        
        hand_rotation_val = 0
        hand_orientation_str = ""
        
        # 1. PROCESSAR POSE
        if pose_res.pose_landmarks:
            plm = pose_res.pose_landmarks.landmark
            
            # Determinar que braço usar usando a mão detetada se existir
            if hands_res.multi_hand_landmarks:
                hand_lm_check = hands_res.multi_hand_landmarks[0]
                hc_x, hc_y = hand_center(hand_lm_check)
                side = pair_hand_to_pose_side(hc_x, hc_y, pose_res.pose_landmarks)
            
            if side is None:
                side = "Right"
            
            if side == "Left":
                sh = plm[11]; el = plm[13]; wr = plm[15]
            else:
                sh = plm[12]; el = plm[14]; wr = plm[16]
            
            ang_brazo = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
            orientacion_brazo = determinar_orientacion_brazo(ang_brazo)
            hombro_y = sh.y; muneca_y = wr.y
            estado_flexion, diff_y = determinar_flexion_brazo(hombro_y, muneca_y)

            # Debug Info Braço
            cv2.putText(out, f"Braco ({side}): {int(ang_brazo)} deg", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(out, f"Orient: {orientacion_brazo}", 
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(out, f"Flex: {estado_flexion}", 
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 0), 1)

            # Desenhar pose
            mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      drawing_spec_thin, drawing_spec_conn)
        
        # 2. PROCESSAR MÃO
        estados_dedos = {}
        found_hand = False
        
        if hands_res.multi_hand_landmarks:
            found_hand = True
            hand_lm = hands_res.multi_hand_landmarks[0] # Tomamos primeira mão
            
            # Calcular Rotação
            _, hand_rotation_val, hand_orientation_str, _ = calculate_hand_rotation(
                hand_lm, estado_flexion, orientacion_brazo)
            
            # Calcular Dedos
            for name, ids in FINGERS.items():
                ang = finger_angle_open(hand_lm, ids)
                umbral = FINGER_THRESHOLDS[name]
                abierto = ang > umbral
                estados_dedos[name] = (abierto, ang)
            
            # Debug Visual Rotação
            bar_x, bar_y, bar_w, bar_h = 300, 30, 150, 15
            cv2.rectangle(out, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
            norm_pos = int((hand_rotation_val + 1) / 2 * bar_w)
            cv2.circle(out, (bar_x + norm_pos, bar_y + int(bar_h/2)), 6, (0, 255, 255), -1)
            cv2.putText(out, f"Rot: {hand_rotation_val:.2f}", (bar_x, bar_y-5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Debug Visual Dedos
            y0 = 100
            ordered_keys = ["indicador", "medio", "anelar", "mindinho", "polegar"]
            for i, k in enumerate(ordered_keys):
                abierto, ang = estados_dedos[k]
                color = (0, 255, 0) if abierto else (0, 0, 255)
                cv2.putText(out, f"{k}: {int(ang)}", (10, y0 + i*20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS,
                                      drawing_spec_thin, drawing_spec_conn)

        # 3. ENVIAR POR SÉRIE
        if time.time() - last_send > SEND_INTERVAL and arduino and found_hand:
            # Preparar valores para Arduino
            # Orientação: para controlar o stepper do Arduino através do Mega
            # Mapeamos strings a int: Esquerda->0, Centro->1, Direita->2
            val_orient = 1 # Default Centro
            if "Esquerda" in orientacion_brazo: val_orient = 0
            elif "Direita" in orientacion_brazo: val_orient = 2
            
            # Flexão
            val_flex = 1 if estado_flexion == "Fletido" else 0
            
            # Dedos (ordem específica: Indice, Medio, Anelar, Mindinho, Polegar)
            # Nota: O código original usava map em join. Aqui fazemos a lista explícita.
            d_list = []
            ordered_keys = ["indicador", "medio", "anelar", "mindinho", "polegar"]
            for k in ordered_keys:
                if k in estados_dedos:
                    d_list.append(1 if estados_dedos[k][0] else 0)
                else:
                    d_list.append(0) # Default fechado se falhar algo
            
            # Rotação
            # Converter -1.0 a 1.0 em 0 a 180
            # -1 -> 0 deg (Palma)
            #  1 -> 180 deg (Dorso)
            rot_deg = int((hand_rotation_val + 1.0) * 90)
            rot_deg = max(0, min(180, rot_deg))
            
            # Construir mensagem
            # Formato: $orient,flex,d1,d2,d3,d4,d5,rot\n\n
            dedos_str = ",".join(map(str, d_list))
            msg = f"${val_orient},{val_flex},{dedos_str},{rot_deg}\n"
            
            try:
                # O '\n' extra ajuda a limpar buffer às vezes, mas \n simples chega se Arduino usa \n terminator
                arduino.write(msg.encode()) 
                last_send = time.time()
                # print(f"TX: {msg.strip()}")
            except Exception as e:
                print(f"Erro na Série: {e}")

        # Mostrar vídeo
        cv2.imshow("RPi Hand Tracking", out)
        if cv2.waitKey(1) & 0xFF == 27:
            break
            
        # Stats
        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            print(f"FPS: {frame_count/elapsed:.1f} | Rot: {hand_rotation_val:.2f}")

except KeyboardInterrupt:
    print("\nA parar...")

finally:
    picam2.stop()
    if arduino:
        arduino.close()
    cv2.destroyAllWindows()
    print("Fim.")

import cv2
import mediapipe as mp
import math
import statistics
import time
import serial
from picamera2 import Picamera2
import numpy as np

# ------------------ CONFIGURACAO OPTIMIZADA PARA RPI ------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS_TARGET = 30

# Configuracao Serial
SERIAL_PORT = "/dev/ttyUSB0"  # Alterar conforme a tua configuracao
# Alternativas comuns: "/dev/ttyACM0", "/dev/serial0"
BAUD_RATE = 115200
SEND_INTERVAL = 0.1  # Enviar a cada 100ms

# Limiares dos dedos
FINGER_THRESHOLDS = {
    "polegar": 150.0,
    "indicador": 160.0,
    "medio": 150.0,
    "anelar": 150.0,
    "mindinho": 140.0
}

Y_DIFF_THRESHOLD = 0.155

ORIENT_RANGES = {
    "left": (0, 70),
    "front": (70, 125),
    "right": (125, 180)
}
# ----------------------------------------------------------------

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

drawing_spec_thin = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
drawing_spec_conn = mp_drawing.DrawingSpec(thickness=1)

# -------- INICIALIZAR SERIAL --------
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Ligado ao Arduino em {SERIAL_PORT}")
except Exception as e:
    print(f"Nao foi possivel ligar ao Arduino: {e}")
    print(f"   Verifica a porta. Podes usar: ls /dev/tty*")
    arduino = None

def angulo_3pts(a, b, c):
    """Calcula angulo no ponto b formado por a-b-c."""
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-9
    cosv = max(-1.0, min(1.0, num/den))
    return math.degrees(math.acos(cosv))

def pair_hand_to_pose_side(hand_landmarks, pose_landmarks):
    """Emparelha mao com lado do corpo."""
    if pose_landmarks is None:
        return None
    hand_cx = statistics.mean([lm.x for lm in hand_landmarks.landmark])
    hand_cy = statistics.mean([lm.y for lm in hand_landmarks.landmark])
    lw = pose_landmarks.landmark[15]
    rw = pose_landmarks.landmark[16]
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    return "Left" if dl < dr else "Right"

def classify_arm_orientation(angle_deg):
    """Classifica orientacao do braco e devolve (texto, valor_numerico)."""
    a = angle_deg
    if ORIENT_RANGES["left"][0] <= a <= ORIENT_RANGES["left"][1]:
        return "Lado esq", 0
    if ORIENT_RANGES["front"][0] < a <= ORIENT_RANGES["front"][1]:
        return "Frente", 1
    if ORIENT_RANGES["right"][0] < a <= ORIENT_RANGES["right"][1]:
        return "Lado dir", 2
    return "Indet", 1  # Por defeito frente

FINGER_IDX = {
    "polegar": [1, 2, 4],
    "indicador": [5, 6, 8],
    "medio": [9, 10, 12],
    "anelar": [13, 14, 16],
    "mindinho": [17, 18, 20]
}

# Inicializar Picamera2
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

# Inicializar modelos
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

# Variaveis para FPS e envio
frame_count = 0
start_time = time.time()
last_send = time.time()

print("Sistema iniciado. Pressiona Ctrl+C para sair.")

try:
    while True:
        frame = picam2.capture_array()
        
        pose_res = pose.process(frame)
        hands_res = hands.process(frame)
        
        out = frame
        
        if hands_res.multi_hand_landmarks:
            hand_lm = hands_res.multi_hand_landmarks[0]
            side = pair_hand_to_pose_side(hand_lm, pose_res.pose_landmarks) if pose_res.pose_landmarks else "Right"
            
            # Valores por defeito
            orient_val = 1  # Frente
            flexion_val = 0  # Estendido
            ang_braco = None
            orient_text = "N/A"
            estado_braco = "N/A"
            
            if pose_res.pose_landmarks:
                plm = pose_res.pose_landmarks.landmark
                if side == "Left":
                    sh = plm[11]; el = plm[13]; wr = plm[15]
                else:
                    sh = plm[12]; el = plm[14]; wr = plm[16]
                
                ang_braco = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
                orient_text, orient_val = classify_arm_orientation(ang_braco)
                
                y_diff = sh.y - wr.y
                if y_diff > Y_DIFF_THRESHOLD:
                    estado_braco = "Flexao"
                    flexion_val = 1
                elif abs(y_diff) <= Y_DIFF_THRESHOLD:
                    estado_braco = "Estendido"
                    flexion_val = 0
                else:
                    estado_braco = "Baixo"
                    flexion_val = 0
                
                cv2.putText(out, f"Braco:{int(ang_braco)}graus {orient_text}", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(out, f"Estado:{estado_braco} ({round(y_diff, 2)})", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            # Processar dedos
            estados = {}
            for fname, ids in FINGER_IDX.items():
                a = hand_lm.landmark[ids[0]]
                b = hand_lm.landmark[ids[1]]
                c = hand_lm.landmark[ids[2]]
                ang = angulo_3pts((a.x, a.y), (b.x, b.y), (c.x, c.y))
                thresh = FINGER_THRESHOLDS.get(fname, 150.0)
                aberto = ang > thresh
                estados[fname] = (aberto, ang)
            
            # Desenhar estado dos dedos
            y0 = 75
            for i, (name, (aberto, ang)) in enumerate(estados.items()):
                color = (255, 0, 0) if aberto else (0, 0, 255)
                txt = f"{name[:3]}:{'A' if aberto else 'F'} {int(ang)}"
                cv2.putText(out, txt, (10, y0 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            
            # Desenhar landmarks
            mp_drawing.draw_landmarks(
                out, hand_lm, mp_hands.HAND_CONNECTIONS,
                drawing_spec_thin, drawing_spec_conn
            )
            
            # ========== ENVIAR DADOS POR SERIAL ==========
            # Formato: $orientacao,flexao,indicador,medio,anelar,mindinho,polegar
            dedos = [
                1 if estados["indicador"][0] else 0,
                1 if estados["medio"][0] else 0,
                1 if estados["anelar"][0] else 0,
                1 if estados["mindinho"][0] else 0,
                1 if estados["polegar"][0] else 0
            ]
            
            msg = f"${orient_val},{flexion_val},{','.join(map(str, dedos))}\n\n"
            
            # Enviar a cada SEND_INTERVAL segundos
            if time.time() - last_send > SEND_INTERVAL and arduino:
                try:
                    arduino.write(msg.encode())
                    last_send = time.time()
                    print(f"Enviado: {msg.strip()} | {orient_text} | Ang:{int(ang_braco) if ang_braco else 'N/A'}graus | {estado_braco}")
                except Exception as e:
                    print(f"Erro ao enviar serial: {e}")
        
        else:
            cv2.putText(out, "Sem mao detectada", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        
        if pose_res.pose_landmarks:
            mp_drawing.draw_landmarks(
                out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                drawing_spec_thin, drawing_spec_conn
            )
        
        # Contador FPS
        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            print(f"FPS medio: {fps:.1f}")
        
        # Mostrar (comentar se usares headless)
        cv2.imshow("Braco+Mao RPi", out)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

except KeyboardInterrupt:
    print("\nInterrompido pelo utilizador")

finally:
    picam2.stop()
    if arduino:
        arduino.close()
    cv2.destroyAllWindows()
    print("Sistema encerrado correctamente")

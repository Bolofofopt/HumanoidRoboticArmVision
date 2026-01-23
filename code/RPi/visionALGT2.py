import cv2
import mediapipe as mp # type: ignore
import math
import statistics

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Inicializar captura e modelos
cap = cv2.VideoCapture(1)

# ---------- parâmetros ajustáveis ----------
# Limiares para dedos (ângulo mínimo para considerar aberto)
TH_POLEGAR = 150.0
TH_INDICADOR = 160.0
TH_MEDIO = 150.0
TH_ANELAR = 150.0
TH_MINDINHO = 140.0

# Limiares para orientação do braço (baseado no ângulo)
TH_BRACO_ESQUERDA = 70    # 0-70: braço para o corpo (esquerda)
TH_BRACO_CENTRO_MAX = 130 # 70-130: braço para a frente (centro)
# 130-180: braço para fora (direita)

# Limiar para flexão do braço (diferença em Y entre pulso e ombro)
TH_FLEXAO_Y = 0.1  # Se diff_y > este valor, braço fletido
# -------------------------------------------

# Definição de dedos usando os índices de landmarks do MediaPipe
FINGERS = {
    "polegar": [2, 3, 4],
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
    """Devolve o ângulo na articulação intermédia para medir se o dedo está estendido."""
    a = hand_landmarks.landmark[ids[0]]
    b = hand_landmarks.landmark[ids[1]]
    c = hand_landmarks.landmark[ids[2]]
    return angulo_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def hand_center(hand_landmarks):
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    return (statistics.mean(xs), statistics.mean(ys))

def pair_hand_to_pose_side(hand_cx, hand_cy, pose_landmarks):
    """Devolve 'Right' ou 'Left' conforme o pulso da pose mais próxima."""
    if pose_landmarks is None:
        return None
    lw = pose_landmarks.landmark[15]
    rw = pose_landmarks.landmark[16]
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    return "Left" if dl < dr else "Right"

def determinar_orientacao_braco(angulo):
    """Determina a orientação do braço conforme o seu ângulo."""
    if angulo <= TH_BRACO_ESQUERDA:
        return "Esquerda (para o corpo)"
    elif angulo <= TH_BRACO_CENTRO_MAX:
        return "Centro (em frente)"
    else:
        return "Direita (para fora)"

def determinar_flexao_braco(ombro_y, pulso_y):
    """Determina se o braço está fletido ou estendido."""
    diff_y = abs(ombro_y - pulso_y)
    if diff_y > TH_FLEXAO_Y:
        return "Fletido", diff_y
    else:
        return "Estendido", diff_y



# Validar índices para evitar erro se não houver landmarks suficientes
def get_landmarks_safe(hand_landmarks, idx):
    if idx < len(hand_landmarks.landmark):
        return hand_landmarks.landmark[idx]
    return None


def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation):
    """
    Calcula a rotação da mão com lógica estrita definida pelo utilizador.
    
    Casos:
    1. Fletido:
       - Eixo X.
       - Palma = Polegar.X > Mindinho.X -> Mapeamento Inverso -> -1
    
    2. Estendido + Corpo (Esquerda/Centro):
       - Eixo Y.
       - INVERTIDO (Fix): Palma = Polegar.Y > Mindinho.Y -> Mapeamento Inverso -> -1
       
    3. Estendido + Direita:
       - Eixo Y.
       - INVERTIDO (Fix): Palma = Polegar.Y < Mindinho.Y -> Mapeamento Normal -> -1
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
        # Lógica Utilizador: Palma se Polegar.X > Mindinho.X -> Inverso
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
            axis_used = "Y (Est-Dir)"
            
        else:
            # CASO 2: Estendido + Corpo/Esq -> Eixo Y
            # Fix: Invertido relativamente à lógica anterior.
            # Antes: (T-P). Agora: -(T-P).
            t_val = thumb.y
            p_val = pinky.y
            diff = -(t_val - p_val)
            axis_used = "Y-Inv (Est-Corpo)"

    # Normalização
    MAX_DIFF = 0.15
    rotation_value = max(-1.0, min(1.0, diff / MAX_DIFF))
    
    orientation = "Indefinido"
    if rotation_value < -0.3:
        orientation = "Palma (Câmara)"
    elif rotation_value > 0.3:
        orientation = "Dorso (Câmara)"
    else:
        orientation = "A Rodar / Canto"
        
    # Retornamos valores extra para debug
    return diff, rotation_value, orientation, axis_used, t_val, p_val


# Inicializar captura e modelos
# Ajustamos parâmetros de deteção para melhorar a deteção de mãos
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

cv2.namedWindow("Deteção Braço+Mão", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Deteção Braço+Mão", 1280, 720)

print("\n" + "="*60)
print("A INICIAR DETEÇÃO DE ROTAÇÃO ADAPTATIVA - Pressione ESC para sair")
print("="*60 + "\n")

frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    h, w = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    pose_res = pose.process(rgb)
    hands_res = hands.process(rgb)

    out = frame.copy()

    # Variáveis para armazenar informação do braço
    ang_braco = None
    orientacao_braco = "Indefinido" # Default string
    estado_flexao = "Estendido"     # Default string
    side = None
    ombro_y = None
    pulso_y = None
    diff_y = None
    
    # Variáveis rotação mão
    hand_rotation_val = 0
    hand_orientation_str = ""
    raw_diff = 0
    axis_mode_str = ""


    # PROCESSAR POSE (MANTIDO)
    if pose_res.pose_landmarks:
        plm = pose_res.pose_landmarks.landmark
        
        # Determinar que braço usar
        if hands_res.multi_hand_landmarks:
            hand_lm = hands_res.multi_hand_landmarks[0]
            hc_x, hc_y = hand_center(hand_lm)
            side = pair_hand_to_pose_side(hc_x, hc_y, pose_res.pose_landmarks)
        
        if side is None:
            side = "Right"
        
        if side == "Left":
            sh = plm[11]; el = plm[13]; wr = plm[15]
        else:
            sh = plm[12]; el = plm[14]; wr = plm[16]
        
        ang_braco = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
        orientacao_braco = determinar_orientacao_braco(ang_braco)
        ombro_y = sh.y; pulso_y = wr.y
        estado_flexao, diff_y = determinar_flexao_braco(ombro_y, pulso_y)
        

        # Info Braço
        cv2.putText(out, f"Braco ({side}): {int(ang_braco)} deg", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(out, f"Orientacao: {orientacao_braco}", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(out, f"Flexao: {estado_flexao} (diff_y: {diff_y:.3f})", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
        cv2.putText(out, f"Ombro_Y: {ombro_y:.3f} | Pulso_Y: {pulso_y:.3f}", 
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        # Desenhar pose (Restaurado)
        mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # PROCESSAR MÃO
    estados_dedos = {}
    if hands_res.multi_hand_landmarks:
        # Usamos a primeira mão detetada
        hand_lm = hands_res.multi_hand_landmarks[0]
        

        # 1. Calcular Rotação (PROCESSAR COM ESTADO DO BRAÇO)
        if estado_flexao is None: estado_flexao = "Estendido"
        if orientacao_braco is None: orientacao_braco = "Indefinido"

        raw_diff, hand_rotation_val, hand_orientation_str, axis_mode_str, t_val, p_val = calculate_hand_rotation(
            hand_lm, estado_flexao, orientacao_braco)
        
        # 2. Calcular estado de dedos
        for name, ids in FINGERS.items():
            ang = finger_angle_open(hand_lm, ids)
            umbral = FINGER_THRESHOLDS[name]
            abierto = ang > umbral
            estados_dedos[name] = (abierto, ang)
        
        # 3. Desenhar Info Rotação
        bar_x, bar_y, bar_w, bar_h = 300, 50, 200, 20
        cv2.rectangle(out, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
        
        norm_pos = int((hand_rotation_val + 1) / 2 * bar_w)
        cv2.circle(out, (bar_x + norm_pos, bar_y + int(bar_h/2)), 8, (0, 255, 255), -1)
        
        cv2.putText(out, f"Rot: {hand_rotation_val:.2f} ({hand_orientation_str})", 
                    (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Info Debug Completa
        cv2.putText(out, f"Modo: {axis_mode_str}", 
                    (bar_x, bar_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        # Mostrar valores brutos comparados
        cv2.putText(out, f"T:{t_val:.3f} vs P:{p_val:.3f} | Diff:{raw_diff:.3f}", 
                    (bar_x, bar_y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # 4. Desenhar estado dos dedos
        y0 = 160
        cv2.putText(out, "Estado dos dedos:", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y0 += 30
        for i, (name, (aberto, ang)) in enumerate(estados_dedos.items()):
            color = (0, 255, 0) if aberto else (0, 0, 255)
            # Capitalize name for display
            display_name = name.capitalize()
            txt = f"{display_name}: {'ABERTO' if aberto else 'FECHADO'} ({int(ang)}deg)"
            cv2.putText(out, txt, (20, y0 + i*26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS)
    else:
        cv2.putText(out, "MÃO NÃO DETETADA", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # IMPRIMIR NA CONSOLA
    if frame_count % 15 == 0:
        print("\n" + "-"*60)
        print(f"Frame: {frame_count}")
        # Info Braço
        if ang_braco is not None:
             print(f"BRAÇO: {estado_flexao} | {orientacao_braco}")
        
        # Info Mão
        if hands_res.multi_hand_landmarks:
             print(f"ROTAÇÃO DA MÃO ({axis_mode_str}):")
             print(f"  Val: {hand_rotation_val:.2f} | T: {t_val:.3f} P: {p_val:.3f} Diff: {raw_diff:.3f}")

             print(f"  Estado: {hand_orientation_str}")
             print(f"  Modo Eixo: {axis_mode_str}")
             
             if estados_dedos:
                print("DEDOS:")
                for name, (aberto, ang) in estados_dedos.items():
                    estado = "ABERTO" if aberto else "FECHADO"
                    print(f"  {name}: {estado} (ang={ang:.1f}°)")
        else:
            print("MÃO: Não detetada")
        print("-"*60)

    cv2.imshow("Deteção Braço+Mão", out)

    key = cv2.waitKey(1) & 0xFF
    if key == 27: # ESC
        break

cap.release()
cv2.destroyAllWindows()
print("\nPrograma terminado.")

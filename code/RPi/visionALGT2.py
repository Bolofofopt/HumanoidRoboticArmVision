import cv2
import mediapipe as mp # type: ignore
import math
import statistics

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Inicializar captura y modelos
cap = cv2.VideoCapture(1)

# ---------- parámetros ajustables ----------
# Umbrales para dedos (ángulo mínimo para considerar abierto)
TH_PULGAR = 150.0
TH_INDICE = 160.0
TH_MEDIO = 150.0
TH_ANULAR = 150.0
TH_MENIQUE = 140.0

# Umbrales para orientación del brazo (basado en ángulo)
TH_BRAZO_IZQUIERDA = 70    # 0-70: brazo hacia el cuerpo (izquierda)
TH_BRAZO_CENTRO_MAX = 130  # 70-130: brazo al frente (centro)
# 130-180: brazo hacia afuera (derecha)

# Umbral para flexión del brazo (diferencia en Y entre muñeca y hombro)
TH_FLEXION_Y = 0.1  # Si diff_y > este valor, brazo flexionado
# -------------------------------------------

# Definición de dedos usando los índices de landmarks de MediaPipe
FINGERS = {
    "pulgar": [2, 3, 4],
    "indice": [5, 6, 8],
    "medio": [9, 10, 12],
    "anular": [13, 14, 16],
    "menique": [17, 18, 20]
}

# Umbrales por dedo
FINGER_THRESHOLDS = {
    "pulgar": TH_PULGAR,
    "indice": TH_INDICE,
    "medio": TH_MEDIO,
    "anular": TH_ANULAR,
    "menique": TH_MENIQUE
}

def angulo_3pts(a, b, c):
    """Ángulo en b dado a,b,c. a,b,c son (x,y) o tuplas."""
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-8
    cosv = max(-1.0, min(1.0, num/den))
    ang = math.degrees(math.acos(cosv))
    return ang

def finger_angle_open(hand_landmarks, ids):
    """Devuelve ángulo en la articulación intermedia para medir si dedo está extendido."""
    a = hand_landmarks.landmark[ids[0]]
    b = hand_landmarks.landmark[ids[1]]
    c = hand_landmarks.landmark[ids[2]]
    return angulo_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def hand_center(hand_landmarks):
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    return (statistics.mean(xs), statistics.mean(ys))

def pair_hand_to_pose_side(hand_cx, hand_cy, pose_landmarks):
    """Devuelve 'Right' o 'Left' según muñeca de pose más cercana."""
    if pose_landmarks is None:
        return None
    lw = pose_landmarks.landmark[15]
    rw = pose_landmarks.landmark[16]
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    return "Left" if dl < dr else "Right"

def determinar_orientacion_brazo(angulo):
    """Determina orientación del brazo según su ángulo."""
    if angulo <= TH_BRAZO_IZQUIERDA:
        return "Izquierda (hacia cuerpo)"
    elif angulo <= TH_BRAZO_CENTRO_MAX:
        return "Centro (al frente)"
    else:
        return "Derecha (hacia afuera)"

def determinar_flexion_brazo(hombro_y, muneca_y):
    """Determina si el brazo está flexionado o extendido."""
    diff_y = abs(hombro_y - muneca_y)
    if diff_y > TH_FLEXION_Y:
        return "Flexionado", diff_y
    else:
        return "Extendido", diff_y



# Validar indices para evitar error si no hay suficientes landmarks
def get_landmarks_safe(hand_landmarks, idx):
    if idx < len(hand_landmarks.landmark):
        return hand_landmarks.landmark[idx]
    return None


def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation):
    """
    Calcula la rotación de la mano con lógica estricta definida por usuario.
    
    Casos:
    1. Flexionado:
       - Eje X.
       - Palma = Thumb.X > Pinky.X -> Mapping Inverso -> -1
    
    2. Extendido + Cuerpo (Izquierda/Centro):
       - Eje Y.
       - INVERTIDO (Fix): Palma = Thumb.Y > Pinky.Y -> Mapping Inverso -> -1
       
    3. Extendido + Derecha:
       - Eje Y.
       - INVERTIDO (Fix): Palma = Thumb.Y < Pinky.Y -> Mapping Normal -> -1
    """
    thumb = hand_landmarks.landmark[4]
    pinky = hand_landmarks.landmark[20]
    
    diff = 0
    t_val = 0
    p_val = 0
    axis_used = "Indef"
    
    # Determinar contexto
    is_flexed = (flex_status == "Flexionado")
    
    if is_flexed:
        # CASO 1: Flexionado -> Eje X
        # Lógica Usuario: Palma si Thumb.X > Pinky.X -> Inverso
        t_val = thumb.x
        p_val = pinky.x
        diff = -(t_val - p_val) 
        axis_used = "X (Flex)"
        
    else:
        # Extendido
        if "Derecha" in arm_orientation:
            # CASO 3: Extendido + Derecha -> Eje Y
            # Fix: Invertido respecto a lógica anterior.
            # Antes: -(T-P). Ahora: (T-P).
            t_val = thumb.y
            p_val = pinky.y
            diff = (t_val - p_val)
            axis_used = "Y (Ext-Der)"
            
        else:
            # CASO 2: Extendido + Cuerpo/Izq -> Eje Y
            # Fix: Invertido respecto a lógica anterior.
            # Antes: (T-P). Ahora: -(T-P).
            t_val = thumb.y
            p_val = pinky.y
            diff = -(t_val - p_val)
            axis_used = "Y-Inv (Ext-Cuerpo)"

    # Normalización
    MAX_DIFF = 0.15
    rotation_value = max(-1.0, min(1.0, diff / MAX_DIFF))
    
    orientation = "Indefinido"
    if rotation_value < -0.3:
        orientation = "Palma (Camara)"
    elif rotation_value > 0.3:
        orientation = "Dorso (Camara)"
    else:
        orientation = "Girando / Canto"
        
    # Retornamos valores extra para debug
    return diff, rotation_value, orientation, axis_used, t_val, p_val


# Inicializar captura y modelos
# Ajustamos parámetros de detección para mejorar la detección de manos
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

cv2.namedWindow("Deteccion Brazo+Mano", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Deteccion Brazo+Mano", 1280, 720)

print("\n" + "="*60)
print("INICIANDO DETECCIÓN ROTACIÓN ADAPTATIVA - Presiona ESC para salir")
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

    # Variables para almacenar información del brazo
    ang_brazo = None
    orientacion_brazo = "Indefinido" # Default string
    estado_flexion = "Extendido"     # Default string
    side = None
    hombro_y = None
    muneca_y = None
    diff_y = None
    
    # Variables rotación mano
    hand_rotation_val = 0
    hand_orientation_str = ""
    raw_diff = 0
    axis_mode_str = ""


    # PROCESAR POSE (MANTENIDO)
    if pose_res.pose_landmarks:
        plm = pose_res.pose_landmarks.landmark
        
        # Determinar qué brazo usar
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
        
        ang_brazo = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
        orientacion_brazo = determinar_orientacion_brazo(ang_brazo)
        hombro_y = sh.y; muneca_y = wr.y
        estado_flexion, diff_y = determinar_flexion_brazo(hombro_y, muneca_y)
        

        # Info Brazo
        cv2.putText(out, f"Brazo ({side}): {int(ang_brazo)} deg", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(out, f"Orientacion: {orientacion_brazo}", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(out, f"Flexion: {estado_flexion} (diff_y: {diff_y:.3f})", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
        cv2.putText(out, f"Hombro_Y: {hombro_y:.3f} | Muneca_Y: {muneca_y:.3f}", 
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        # Dibujar pose (Restaurado)
        mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # PROCESAR MANO
    estados_dedos = {}
    if hands_res.multi_hand_landmarks:
        # Tomamos la primera mano detectada
        hand_lm = hands_res.multi_hand_landmarks[0]
        

        # 1. Calcular Rotación (PROCESAR CON ESTADO DEL BRAZO)
        if estado_flexion is None: estado_flexion = "Extendido"
        if orientacion_brazo is None: orientacion_brazo = "Indefinido"

        raw_diff, hand_rotation_val, hand_orientation_str, axis_mode_str, t_val, p_val = calculate_hand_rotation(
            hand_lm, estado_flexion, orientacion_brazo)
        
        # 2. Calcular estado de dedos
        for name, ids in FINGERS.items():
            ang = finger_angle_open(hand_lm, ids)
            umbral = FINGER_THRESHOLDS[name]
            abierto = ang > umbral
            estados_dedos[name] = (abierto, ang)
        
        # 3. Dibujar Info Rotación
        bar_x, bar_y, bar_w, bar_h = 300, 50, 200, 20
        cv2.rectangle(out, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
        
        norm_pos = int((hand_rotation_val + 1) / 2 * bar_w)
        cv2.circle(out, (bar_x + norm_pos, bar_y + int(bar_h/2)), 8, (0, 255, 255), -1)
        
        cv2.putText(out, f"Rot: {hand_rotation_val:.2f} ({hand_orientation_str})", 
                    (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Debug Info Completa
        cv2.putText(out, f"Modo: {axis_mode_str}", 
                    (bar_x, bar_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        # Mostrar valores crudos comparados
        cv2.putText(out, f"T:{t_val:.3f} vs P:{p_val:.3f} | Diff:{raw_diff:.3f}", 
                    (bar_x, bar_y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # 4. Dibujar estado de dedos
        y0 = 160
        cv2.putText(out, "Estado de dedos:", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y0 += 30
        for i, (name, (abierto, ang)) in enumerate(estados_dedos.items()):
            color = (0, 255, 0) if abierto else (0, 0, 255)
            txt = f"{name}: {'ABIERTO' if abierto else 'CERRADO'} ({int(ang)}deg)"
            cv2.putText(out, txt, (20, y0 + i*26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS)
    else:
        cv2.putText(out, "MANO NO DETECTADA", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # IMPRIMIR EN CONSOLA
    if frame_count % 15 == 0:
        print("\n" + "-"*60)
        print(f"Frame: {frame_count}")
        # Info Brazo
        if ang_brazo is not None:
             print(f"BRAZO: {estado_flexion} | {orientacion_brazo}")
        
        # Info Mano
        if hands_res.multi_hand_landmarks:
             print(f"MANO ROTACION ({axis_mode_str}):")
             print(f"  Val: {hand_rotation_val:.2f} | T: {t_val:.3f} P: {p_val:.3f} Diff: {raw_diff:.3f}")

             print(f"  Estado: {hand_orientation_str}")
             print(f"  Modo Eje: {axis_mode_str}")
             
             if estados_dedos:
                print("DEDOS:")
                for name, (abierto, ang) in estados_dedos.items():
                    estado = "ABIERTO" if abierto else "CERRADO"
                    print(f"  {name}: {estado} (ang={ang:.1f}°)")
        else:
            print("MANO: No detectada")
        print("-"*60)

    cv2.imshow("Deteccion Brazo+Mano", out)

    key = cv2.waitKey(1) & 0xFF
    if key == 27: # ESC
        break

cap.release()
cv2.destroyAllWindows()
print("\nPrograma finalizado.")

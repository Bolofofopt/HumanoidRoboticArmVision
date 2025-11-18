import cv2
import mediapipe as mp # type: ignore
import math
import statistics
import serial # type: ignore
import time

# ------------------ CONFIG ------------------
CAM_INDEX = 0
WINDOW_W, WINDOW_H = 1280, 720
SERIAL_PORT = "COM6"
BAUD_RATE = 9600
SEND_INTERVAL = 0.1

FINGER_THRESHOLDS = {
    "pulgar": 150.0,
    "indice": 150.0,
    "medio": 140.0,
    "anular": 150.0,
    "meñique": 140.0
}

Y_DIFF_THRESHOLD = 0.145
ORIENT_RANGES = {
    "left": (0, 70),
    "front": (70, 130),
    "right": (130, 180)
}

# --------------------------------------------
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# -------- SERIAL SETUP --------
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"✅ Conectado al Arduino en {SERIAL_PORT}")
except Exception as e:
    print(f"⚠️ No se pudo conectar al Arduino: {e}")
    arduino = None

# -------- FUNCIONES --------
def angulo_3pts(a, b, c):
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-9
    cosv = max(-1.0, min(1.0, num/den))
    return math.degrees(math.acos(cosv))

def pair_hand_to_pose_side(hand_landmarks, pose_landmarks):
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
    a = angle_deg
    if ORIENT_RANGES["left"][0] <= a <= ORIENT_RANGES["left"][1]:
        return "left", 0
    if ORIENT_RANGES["front"][0] < a <= ORIENT_RANGES["front"][1]:
        return "front", 1
    if ORIENT_RANGES["right"][0] < a <= ORIENT_RANGES["right"][1]:
        return "right", 2
    return "unknown", 1

FINGER_IDX = {
    "pulgar": [1, 2, 4],
    "indice": [5, 6, 8],
    "medio": [9, 10, 12],
    "anular": [13, 14, 16],
    "meñique": [17, 18, 20]
}

# -------- MAIN LOOP --------
cap = cv2.VideoCapture(CAM_INDEX)
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.6)

cv2.namedWindow("Brazo+Mano Control", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Brazo+Mano Control", WINDOW_W, WINDOW_H)

last_send = time.time()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pose_res = pose.process(rgb)
    hands_res = hands.process(rgb)
    out = frame.copy()

    msg = None

    if hands_res.multi_hand_landmarks:
        hand_lm = hands_res.multi_hand_landmarks[0]
        side = pair_hand_to_pose_side(hand_lm, pose_res.pose_landmarks) if pose_res.pose_landmarks else "Right"

        ang_brazo = 0
        orient_text, orient_val = "unknown", 1
        brazo_estado = "Desconocido"
        y_diff = 0

        if pose_res.pose_landmarks:
            plm = pose_res.pose_landmarks.landmark
            if side == "Left":
                sh = plm[11]; el = plm[13]; wr = plm[15]
            else:
                sh = plm[12]; el = plm[14]; wr = plm[16]

            ang_brazo = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
            orient_text, orient_val = classify_arm_orientation(ang_brazo)

            y_diff = sh.y - wr.y
            brazo_estado = "Flexionado" if y_diff > Y_DIFF_THRESHOLD else "Extendido"

            # Mostrar ángulo, orientación y estado del brazo
            cv2.putText(out, f"Brazo {side}: {int(ang_brazo)}° ({orient_text})",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            color_estado = (0, 0, 255) if brazo_estado == "Flexionado" else (0, 255, 255)
            cv2.putText(out, f"Estado: {brazo_estado} (ΔY={y_diff:.3f})",
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_estado, 2)

        estados = {}
        for fname, ids in FINGER_IDX.items():
            a = hand_lm.landmark[ids[0]]
            b = hand_lm.landmark[ids[1]]
            c = hand_lm.landmark[ids[2]]
            ang = angulo_3pts((a.x, a.y), (b.x, b.y), (c.x, c.y))
            abierto = ang > FINGER_THRESHOLDS[fname]
            estados[fname] = 1 if abierto else 0
            color = (255, 0, 0) if abierto else (0, 0, 255)
            cv2.putText(out, f"{fname}: {'O' if abierto else 'X'} ({int(ang)}°)",
                        (20, 100 + list(FINGER_IDX.keys()).index(fname) * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS)
        if pose_res.pose_landmarks:
            mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # -------------------- Enviar datos --------------------
        dedos = [
            estados["indice"],
            estados["medio"],
            estados["anular"],
            estados["meñique"],
            estados["pulgar"]
        ]
        cotovelo_val = 1
        base_val = orient_val  # 0=izq,1=frente,2=derecha
        msg = f"${base_val},{cotovelo_val},{','.join(map(str, dedos))}\\n"

        if time.time() - last_send > SEND_INTERVAL and arduino:
            arduino.write(msg.encode())
            last_send = time.time()
            print(f"→ Enviado: {msg.strip()} | Ori:{orient_text} | Ang:{int(ang_brazo)}° | {brazo_estado}")

    cv2.imshow("Brazo+Mano Control", out)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()

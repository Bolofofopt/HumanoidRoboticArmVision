"""
======================================================================================
NOME DO FICHEIRO: visionALGT2.py
======================================================================================

DESCRIÇÃO PARA PRINCIPIANTES:
Este programa é uma versão para executar no Computador (PC).
Serve para testar a "visão" (inteligência artificial) sem precisar do robô ligado.
Usa a webcam do computador para ver a mão e o braço e mostra no ecrã o que está a perceber.

O que este programa faz:
1. Liga a webcam do computador.
2. Usa o MediaPipe para detetar o esqueleto do braço e da mão.
3. Calcula se o braço está fletido, para onde aponta, e se a mão está rodada.
4. Mostra tudo num vídeo no ecrã com desenhos coloridos.
5. Escreve os dados na janela preta (consola) para análise técnica.

LINGUAGEM: Python
IDIOMA DOS COMENTÁRIOS: Português de Portugal (PT-PT)
======================================================================================
"""

# --- BIBLIOTECAS ---
import cv2                  # "OpenCV": Controla a câmara e janelas de vídeo
import mediapipe as mp      # "MediaPipe": A Inteligência Artificial que deteta pessoas
import math                 # Matemática (cálculo de ângulos)
import statistics           # Estatística (médias)

# Atalhos para usar o MediaPipe mais facilmente
mp_pose = mp.solutions.pose       # Módulo do corpo
mp_hands = mp.solutions.hands     # Módulo das mãos
mp_drawing = mp.solutions.drawing_utils # Para desenhar as linhas verdes e vermelhas

# --- INICIALIZAR CÂMARA ---
# O número '1' indica qual a câmara a usar. 
# Num portátil, geralmente '0' é a webcam integrada e '1' seria uma externa.
# Se der erro, experimente mudar para '0'.
cap = cv2.VideoCapture(1)

# ---------- PARÂMETROS AJUSTÁVEIS (Configurações) ----------

# Limiares dos DEDOS (Ângulo mínimo para considerar "Aberto")
TH_POLEGAR = 150.0
TH_INDICADOR = 160.0 # O indicador precisa de estar bem esticado
TH_MEDIO = 150.0
TH_ANELAR = 150.0
TH_MINDINHO = 140.0

# Limiares do BRAÇO (Angulo do ombro)
# Define para onde o braço está a apontar no espaço.
TH_BRACO_ESQUERDA = 70    # 0-70 graus: Para a esquerda (cruzado sobre o corpo)
TH_BRACO_CENTRO_MAX = 130 # 70-130 graus: Para a frente (centro)
                          # 130-180 graus: Para a direita (aberto)

# Limiar de FLEXÃO (Altura Ombro vs Pulso)
# Se a diferença de altura for grande (> 0.1), consideramos que o braço está levantado/fletido.
TH_FLEXAO_Y = 0.1  
# -----------------------------------------------------------

# Definição dos pontos que formam cada dedo (segundo o mapa do MediaPipe)
FINGERS = {
    "polegar": [2, 3, 4],     # Diferente do outro script? Atenção: MediaPipe Polegar é 1,2,3,4.
    "indicador": [5, 6, 8],
    "medio": [9, 10, 12],
    "anelar": [13, 14, 16],
    "mindinho": [17, 18, 20]
}

# Mapa de limiares
FINGER_THRESHOLDS = {
    "polegar": TH_POLEGAR,
    "indicador": TH_INDICADOR,
    "medio": TH_MEDIO,
    "anelar": TH_ANELAR,
    "mindinho": TH_MINDINHO
}

# --- FUNÇÕES MATEMÁTICAS E LÓGICA ---

def angulo_3pts(a, b, c):
    """
    Calcula o ângulo formado por 3 pontos (ex: ombro-cotovelo-pulso).
    Imagine um compasso: 'b' é o bico do compasso.
    """
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-8
    cosv = max(-1.0, min(1.0, num/den))
    ang = math.degrees(math.acos(cosv))
    return ang

def finger_angle_open(hand_landmarks, ids):
    """Calcula se um dedo está dobrado ou esticado."""
    a = hand_landmarks.landmark[ids[0]]
    b = hand_landmarks.landmark[ids[1]]
    c = hand_landmarks.landmark[ids[2]]
    return angulo_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def hand_center(hand_landmarks):
    """Encontra o centro da mão."""
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    return (statistics.mean(xs), statistics.mean(ys))

def pair_hand_to_pose_side(hand_cx, hand_cy, pose_landmarks):
    """
    Decide se a mão detetada é a Esquerda ou a Direita, comparando
    a distância aos pulsos detetados no corpo.
    """
    if pose_landmarks is None:
        return None
    lw = pose_landmarks.landmark[15] # 15 = Pulso esquerdo
    rw = pose_landmarks.landmark[16] # 16 = Pulso direito
    
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    
    return "Left" if dl < dr else "Right"

def determinar_orientacao_braco(angulo):
    """Traduz o ângulo do braço em texto legível (Esq/Centro/Dir)."""
    if angulo <= TH_BRACO_ESQUERDA:
        return "Esquerda (para o corpo)"
    elif angulo <= TH_BRACO_CENTRO_MAX:
        return "Centro (em frente)"
    else:
        return "Direita (para fora)"

def determinar_flexao_braco(ombro_y, pulso_y):
    """Vê se o braço está levantado (fletido)."""
    diff_y = abs(ombro_y - pulso_y)
    if diff_y > TH_FLEXAO_Y:
        return "Fletido", diff_y
    else:
        return "Estendido", diff_y

# --- LÓGICA DE ROTAÇÃO DA MÃO ---
def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation):
    """
    Calcula a rotação do pulso (pronação/supinação).
    
    Como funciona:
    Mede a distância horizontal (X) ou vertical (Y) entre o Polegar e o Mindinho.
    Essa distância muda conforme rodamos a mão.
    
    A lógica é "inteligente" (adaptativa):
    - Se o braço está dobrado (fletido), usa a distância horizontal.
    - Se o braço está esticado, usa a distância vertical (mas inverte o sinal dependendo do lado).
    """
    thumb = hand_landmarks.landmark[4]
    pinky = hand_landmarks.landmark[20]
    
    diff = 0
    t_val = 0
    p_val = 0
    axis_used = "Indef"
    
    # Verificar se está fletido
    is_flexed = (flex_status == "Fletido")
    
    if is_flexed:
        # CASO 1: Braço Fletido -> Usar eixo X
        t_val = thumb.x
        p_val = pinky.x
        diff = -(t_val - p_val) 
        axis_used = "X (Fletido)"
        
    else:
        # Braço Estendido
        if "Direita" in arm_orientation:
            # CASO 3: Estendido para a Direita -> Eixo Y normal
            t_val = thumb.y
            p_val = pinky.y
            diff = (t_val - p_val)
            axis_used = "Y (Est-Dir)"
            
        else:
            # CASO 2: Estendido para o Centro/Esq -> Eixo Y invertido
            t_val = thumb.y
            p_val = pinky.y
            diff = -(t_val - p_val)
            axis_used = "Y-Inv (Est-Corpo)"

    # Normalização (Converter para escala -1 a 1)
    MAX_DIFF = 0.15 # Este valor define a sensibilidade
    rotation_value = max(-1.0, min(1.0, diff / MAX_DIFF))
    
    # Classificação em texto
    orientation = "Indefinido"
    if rotation_value < -0.3:
        orientation = "Palma (Câmara)"
    elif rotation_value > 0.3:
        orientation = "Dorso (Câmara)"
    else:
        orientation = "A Rodar / Canto"
        
    return diff, rotation_value, orientation, axis_used, t_val, p_val


# --- INICIALIZAÇÃO DE MODELOS ---
# Configura a "sensibilidade" da deteção
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

# Configurar a janela visual
cv2.namedWindow("Deteção Braço+Mão", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Deteção Braço+Mão", 1280, 720)

print("\n" + "="*60)
print("A INICIAR DETEÇÃO DE ROTAÇÃO ADAPTATIVA - Pressione ESC para sair")
print("="*60 + "\n")

frame_count = 0

# --- CICLO PRINCIPAL ---
while cap.isOpened():
    ret, frame = cap.read() # Ler imagem da câmara
    if not ret:
        break

    frame_count += 1
    # Converte cores: OpenCV usa BGR (Azul,Verde,Vermelho), mas MediaPipe gosta de RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # PROCESSAR (Pedir à AI para analisar a imagem)
    pose_res = pose.process(rgb)
    hands_res = hands.process(rgb)

    out = frame.copy() # Cópia para desenhar

    # Variáveis de estado (Valores predefinidos)
    ang_braco = None
    orientacao_braco = "Indefinido" 
    estado_flexao = "Estendido"    
    side = None
    ombro_y = None
    pulso_y = None
    diff_y = None
    
    hand_rotation_val = 0
    hand_orientation_str = ""
    raw_diff = 0
    axis_mode_str = ""

    # 1. PROCESSAR CORPO (Se detetado)
    if pose_res.pose_landmarks:
        plm = pose_res.pose_landmarks.landmark
        
        # Tentar adivinhar lado pela mão
        if hands_res.multi_hand_landmarks:
            hand_lm = hands_res.multi_hand_landmarks[0]
            hc_x, hc_y = hand_center(hand_lm)
            side = pair_hand_to_pose_side(hc_x, hc_y, pose_res.pose_landmarks)
        
        if side is None:
            side = "Right"
        
        # Selecionar pontos do esqueleto
        if side == "Left":
            sh = plm[11]; el = plm[13]; wr = plm[15]
        else:
            sh = plm[12]; el = plm[14]; wr = plm[16]
        
        # Cálculos de geometria do braço
        ang_braco = angulo_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
        orientacao_braco = determinar_orientacao_braco(ang_braco)
        ombro_y = sh.y; pulso_y = wr.y
        estado_flexao, diff_y = determinar_flexao_braco(ombro_y, pulso_y)
        
        # Escrever info no ecrã (canto superior esquerdo)
        cv2.putText(out, f"Braco ({side}): {int(ang_braco)} deg", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(out, f"Orientacao: {orientacao_braco}", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(out, f"Flexao: {estado_flexao} (diff_y: {diff_y:.3f})", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
        cv2.putText(out, f"Ombro_Y: {ombro_y:.3f} | Pulso_Y: {pulso_y:.3f}", 
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        # Desenhar esqueleto
        mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # 2. PROCESSAR MÃO (Se detetada)
    estados_dedos = {}
    if hands_res.multi_hand_landmarks:
        # Usamos a primeira mão detetada
        hand_lm = hands_res.multi_hand_landmarks[0]
        
        # Garantir que temos dados do braço (se não houver corpo, assume padrão)
        if estado_flexao is None: estado_flexao = "Estendido"
        if orientacao_braco is None: orientacao_braco = "Indefinido"

        # Calcular Rotação
        raw_diff, hand_rotation_val, hand_orientation_str, axis_mode_str, t_val, p_val = calculate_hand_rotation(
            hand_lm, estado_flexao, orientacao_braco)
        
        # Calcular Dedos
        for name, ids in FINGERS.items():
            ang = finger_angle_open(hand_lm, ids)
            umbral = FINGER_THRESHOLDS[name]
            abierto = ang > umbral
            estados_dedos[name] = (abierto, ang)
        
        # Desenhar Barra Visual de Rotação
        bar_x, bar_y, bar_w, bar_h = 300, 50, 200, 20
        cv2.rectangle(out, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
        
        norm_pos = int((hand_rotation_val + 1) / 2 * bar_w)
        cv2.circle(out, (bar_x + norm_pos, bar_y + int(bar_h/2)), 8, (0, 255, 255), -1)
        
        cv2.putText(out, f"Rot: {hand_rotation_val:.2f} ({hand_orientation_str})", 
                    (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.putText(out, f"Modo: {axis_mode_str}", 
                    (bar_x, bar_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        # Mostrar valores brutos (Debug avançado)
        cv2.putText(out, f"T:{t_val:.3f} vs P:{p_val:.3f} | Diff:{raw_diff:.3f}", 
                    (bar_x, bar_y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Lista de Dedos
        y0 = 160
        cv2.putText(out, "Estado dos dedos:", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y0 += 30
        for i, (name, (aberto, ang)) in enumerate(estados_dedos.items()):
            color = (0, 255, 0) if aberto else (0, 0, 255)
            display_name = name.capitalize()
            txt = f"{display_name}: {'ABERTO' if aberto else 'FECHADO'} ({int(ang)}deg)"
            cv2.putText(out, txt, (20, y0 + i*26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS)
    else:
        cv2.putText(out, "MÃO NÃO DETETADA", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # --- IMPRIMIR DADOS NA CONSOLA (A cada 15 frames) ---
    if frame_count % 15 == 0:
        print("\n" + "-"*60)
        print(f"Frame (Imagem): {frame_count}")
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
                    print(f"  {name}: {estado} (ang={ang:.1f}o)")
        else:
            print("MÃO: Não detetada")
        print("-"*60)

    # Mostrar janela
    cv2.imshow("Deteção Braço+Mão", out)

    # Tecla de saída (ESC)
    key = cv2.waitKey(1) & 0xFF
    if key == 27: 
        break

# Limpeza final
cap.release()
cv2.destroyAllWindows()
print("\nPrograma terminado.")

"""
======================================================================================
                        FILE NAME: VisionDebugger_PC.py
======================================================================================

DESCRIPTION:
This program is a version to run on a Computer (PC).
It serves to test the "vision" (artificial intelligence) without needing the arm connected.
It uses the computer's webcam to see the hand and arm and shows on screen what it perceives.

What this program does:
1. Turns on the computer webcam.
2. Uses MediaPipe (New 'Tasks' API) to detect the arm and hand skeleton.
3. Calculates if arm is bent, where it points, and if hand is rotated.
4. Shows everything in a video on screen with colorful drawings.
5. Writes data in the black window (console) for technical analysis.

FINAL OPTIMIZED Version.
- Uses video "Thread" to eliminate lags/delays.
- Resolution adjusted to 640x480 (Standard for high speed).
- GPU activated (Uses graphics card to be faster).
- Lite Model (Lightweight version of artificial intelligence).

LANGUAGE: Python
======================================================================================
"""

# --- LIBRARIES (Tools the program uses) ---
import cv2                  # "OpenCV": Controls camera and video windows
import mediapipe as mp      # "MediaPipe": The Artificial Intelligence that detects people
import math                 # Math (angle detection)
import statistics           # Statistics (averages)
import time                 # Clock (to measure time and calculate FPS)
import os                   # Operating System (to find files on PC)
import threading            # "Multitasking": Allows reading camera and processing at the same time

# --- NEW MEDIAPIPE TASKS API ---
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ======================================================================================
#                      >>> CONFIGURATION AREA (EDIT HERE) <<<
# ======================================================================================

# 1. CAMERA (Webcam)
CAMERA_INDEX = 1        # Which camera to use? (0 = Main, 1 = Secondary/External)
CAMERA_WIDTH = 1280     # Image width (Recommended: 640 for speed, 1280 for quality)
CAMERA_HEIGHT = 720     # Image height (Recommended: 480 for speed, 720 for quality)

# 2. FINGER THRESHOLDS (Minimum angle to consider "Open")
# If finger angle is larger than this, program says "OPEN"
TH_THUMB = 150.0 
TH_INDEX = 160.0
TH_MIDDLE = 150.0
TH_RING = 150.0
TH_PINKY = 140.0

# 3. ARM THRESHOLDS
TH_ARM_LEFT = 70          # Degrees to consider facing left
TH_ARM_CENTER_MAX = 130   # Up to how many degrees is center?
TH_FLEXION_Y = 0.1        # How high arm needs to be to be "flexed" (bent)

# 4. FINGER MAP (MediaPipe Points)
# Tells program which skeleton "points" form each finger
FINGERS = {
    "thumb": [1, 2, 4], 
    "index": [5, 6, 8], 
    "middle": [9, 10, 12], 
    "ring": [13, 14, 16], 
    "pinky": [17, 18, 20]
}

# ======================================================================================
#                           >>> END OF CONFIGURATION <<<
# ======================================================================================

# --- INTERNAL CONSTANTS (Fixed values program uses) ---
FINGER_THRESHOLDS = {
    "thumb": TH_THUMB,
    "index": TH_INDEX,
    "middle": TH_MIDDLE,
    "ring": TH_RING,
    "pinky": TH_PINKY
}

# Colors for drawings (BGR Format: Blue, Green, Red)
COLOR_POSE_BONE = (0, 255, 0)      # Green
COLOR_POSE_JOINT = (0, 0, 255)     # Red
COLOR_HAND_BONE = (255, 255, 0)    # Cyan (Yellow+Green ?)
COLOR_HAND_JOINT = (255, 0, 0)     # Blue
THICKNESS = 2                      # Line thickness for drawings

# --- MODEL CONFIGURATION (Where AI files are) ---
# Program looks for ".task" files in same folder where this .py file is
script_dir = os.path.dirname(os.path.abspath(__file__))
POSE_MODEL_PATH = os.path.join(script_dir, 'pose_landmarker_lite.task') 
HAND_MODEL_PATH = os.path.join(script_dir, 'hand_landmarker.task')

# --- CLASS DEFINITION (Custom tools) ---

class CameraStream:
    """
    SPECIAL CLASS: FAST CAMERA READER
    This tool reads the camera in a 'separate line' (Thread).
    This serves so the program never waits for the camera.
    Eliminates image 'Lag' or 'Delay'.
    """
    def __init__(self, src=0, width=640, height=480):
        # Starts connection to camera
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # Try to tell Windows not to keep old images in memory
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Reads first image
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # Starts background task
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Infinite loop that only does one thing: Read latest image
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Returns last captured image
        return self.frame

    def stop(self):
        # Stops camera
        self.stopped = True
        self.stream.release()

# --- HELPER FUNCTIONS (Small useful commands) ---

def draw_landmarks_custom(image, landmarks, connections, color_joint, color_bone):
    """
    Function to draw skeleton (dots and lines) on image.
    """
    h, w, _ = image.shape # Image Height and Width
    # Draw Points (Dots)
    for lm in landmarks:
        cx, cy = int(lm.x * w), int(lm.y * h)
        cv2.circle(image, (cx, cy), 4, color_joint, -1)
    # Draw Connections (Lines)
    if connections:
        for s, e in connections:
            if s < len(landmarks) and e < len(landmarks):
                cv2.line(image, (int(landmarks[s].x*w), int(landmarks[s].y*h)), (int(landmarks[e].x*w), int(landmarks[e].y*h)), color_bone, THICKNESS)

def calculate_angle_3pts(a, b, c):
    """
    Math: Calculates angle between 3 points.
    (Example: Shoulder -> Elbow -> Wrist to know if arm is straight)
    """
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-8
    return math.degrees(math.acos(max(-1.0, min(1.0, num/den))))

def calculate_finger_angle(hand_landmarks_list, ids):
    """Checks how much a finger is bent."""
    a = hand_landmarks_list[ids[0]]
    b = hand_landmarks_list[ids[1]]
    c = hand_landmarks_list[ids[2]]
    return calculate_angle_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def get_hand_center(lm_list): 
    """Finds center of hand (average of all points)."""
    return (statistics.mean([l.x for l in lm_list]), statistics.mean([l.y for l in lm_list]))

def determine_hand_side(hc_x, hc_y, plm):
    """
    Tries to guess if hand is Left or Right.
    Compares distance of hand to body wrists.
    """
    return "Left" if math.hypot(hc_x - plm[15].x, hc_y - plm[15].y) < math.hypot(hc_x - plm[16].x, hc_y - plm[16].y) else "Right"

def determine_arm_orientation(angle):
    """Tells where the arm is pointing based on the shoulder angle."""
    if angle <= TH_ARM_LEFT:
        return "Left (Body)" 
    elif angle <= TH_ARM_CENTER_MAX:
        return "Center (Front)" 
    else:
        return "Right (Out)"

def determine_arm_flexion(shoulder_y, wrist_y):
    """
    Tells if the arm is raised (flexed).
    Compares the height (Y) of the shoulder and the wrist.
    """
    diff_y = abs(shoulder_y - wrist_y)
    if diff_y > TH_FLEXION_Y:
        return "Flexed", diff_y
    else:
        return "Extended", diff_y

def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation, side="Right"):
    """
    Calculates if hand is Palm or Back (Dorsum).
    Measures distance between Thumb and Pinky.
    If distance is positive it's palm, negative it's back (or vice-versa).
    """
    thumb = hand_landmarks[4]  # Point at tip of thumb
    pinky = hand_landmarks[20] # Point at tip of pinky
    
    diff = 0
    t_val = 0
    p_val = 0
    axis_used = "Indef"
    
    # Logic changes depending on whether arm is bent or extended.
    is_flexed = (flex_status == "Flexed")
    
    if is_flexed:
        # CASE 1: Arm Flexed (Bent)
        # We use difference in X axis (Horizontal)
        t_val = thumb.x
        p_val = pinky.x
        
        # CORRECT LOGIC FOR MIRRORED IMAGE (Selfie Mode):
        if side == "Right":
            # Right Hand (on screen Right): Palm means Thumb(Left) < Pinky(Right) -> Diff is Neg
            diff = -(t_val - p_val) 
        else:
            # Left Hand (on screen Left): Palm means Thumb(Right) > Pinky(Left) -> Diff is Pos -> Invert
            diff = (t_val - p_val)
            
        axis_used = "X (Bent)"
        
    else:
        # Arm Extended
        if "Right" in arm_orientation:
            # CASE 3: Extended to Right -> Use Y axis (Vertical)
            t_val = thumb.y
            p_val = pinky.y
            diff = (t_val - p_val)
            axis_used = "Y (Ext-Right)"
            
        else:
            # CASE 2: Extended to Center/Body -> Use Inverted Y axis
            t_val = thumb.y
            p_val = pinky.y
            diff = -(t_val - p_val)
            axis_used = "Y-Inv (Ext-Body)"

    # Normalization: Converts raw pixel difference to a standard value between -1 and 1
    MAX_DIFF = 0.22
    rotation_value = max(-1.0, min(1.0, diff / MAX_DIFF))
    
    # Classify by text for Debugging
    orientation = "Undefined"
    if rotation_value < -0.3:
        orientation = "Palm"
    elif rotation_value > 0.3:
        orientation = "Back"
    else:
        orientation = "Rotating / Edge"
        
    return diff, rotation_value, orientation, axis_used


# --- INITIALIZE MODELS AND CAMERA (Prepare everything before starting) ---

# 1. Load Artificial Intelligence (MediaPipe)
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker, PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarker, mp.tasks.vision.PoseLandmarkerOptions
HandLandmarker, HandLandmarkerOptions = mp.tasks.vision.HandLandmarker, mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

try:
    print("Trying to activate Graphics Card (GPU) to be faster...")
    # Tries to configure to use GPU
    base_p, base_h = BaseOptions(model_asset_path=POSE_MODEL_PATH, delegate=BaseOptions.Delegate.GPU), BaseOptions(model_asset_path=HAND_MODEL_PATH, delegate=BaseOptions.Delegate.GPU)
    detector_pose = PoseLandmarker.create_from_options(PoseLandmarkerOptions(base_options=base_p, running_mode=VisionRunningMode.VIDEO, num_poses=1))
    detector_hand = HandLandmarker.create_from_options(HandLandmarkerOptions(base_options=base_h, running_mode=VisionRunningMode.VIDEO, num_hands=2))
    print(">>> SUCCESS! GPU ACTIVATED! <<<")
except Exception as e:
    print(f"WARNING: GPU failed ({e}). Computer will use Processor (CPU)...")
    base_p, base_h = BaseOptions(model_asset_path=POSE_MODEL_PATH, delegate=BaseOptions.Delegate.CPU), BaseOptions(model_asset_path=HAND_MODEL_PATH, delegate=BaseOptions.Delegate.CPU)
    detector_pose = PoseLandmarker.create_from_options(PoseLandmarkerOptions(base_options=base_p, running_mode=VisionRunningMode.VIDEO, num_poses=1))
    detector_hand = HandLandmarker.create_from_options(HandLandmarkerOptions(base_options=base_h, running_mode=VisionRunningMode.VIDEO, num_hands=2))

# 2. Start Camera (Uses our 'Fast Reader')
vs = CameraStream(src=CAMERA_INDEX, width=CAMERA_WIDTH, height=CAMERA_HEIGHT).start()
time.sleep(2.0) # Waits 2 seconds for camera to "warm up" (adjust light)

# Create window where we see the video
cv2.namedWindow("Detecao OTIMIZADA", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detecao OTIMIZADA", 1280, 720) 

print("\n" + "="*60)
print("STARTING - NO DELAY MODE")
print("="*60 + "\n")

# --- MAP OF CONNECTIONS (Which points connect to which) ---
POSE_CONNECTIONS_ARMS = [
    (11, 13), (13, 15), (12, 14), (14, 16), # Arms
    (11, 12), (23, 24), (11, 23), (12, 24)  # Torso
]

# Hand map (if MediaPipe doesn't have it, we use this manual one)
HAND_CONNECTIONS = mp.solutions.hands.HAND_CONNECTIONS if hasattr(mp.solutions, 'hands') else [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), 
    (6, 7), (7, 8), (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16), (0, 17), (17, 18), 
    (18, 19), (19, 20), (5, 9), (9, 13), (13, 17)
]

# --- MAIN LOOP (Where magic happens) ---
# Program gets stuck here repeating this infinitely
prev_time = time.time()
frame_count = 0

while True:
    # 1. Read New Image
    frame = vs.read()
    if frame is None: continue 
    
    # 2. Calculate FPS (Program speed)
    curr_time = time.time()
    dt = curr_time - prev_time
    if dt > 0:
        fps = 1 / dt
    else:
        fps = 0
    prev_time = curr_time
    frame_count += 1
    
    # 3. Prepare Image for AI (Convert colors)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    timestamp_ms = int(time.time() * 1000)

    # 4. EXECUTE DETECTION (Ask AI where body and hand are)
    pose_result = detector_pose.detect_for_video(mp_image, timestamp_ms)
    hand_result = detector_hand.detect_for_video(mp_image, timestamp_ms)
    
    out = frame.copy() # Make a copy to draw over
    cv2.putText(out, f"FPS: {int(fps)}", (out.shape[1]-150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    arm_angle, arm_orientation, flexion_state = None, "Undefined", "Extended"
    
    # 5. If BODY detected:
    if pose_result.pose_landmarks:
        plm = pose_result.pose_landmarks[0]
        # Try to discover side
        side = "Right"
        if hand_result.hand_landmarks:
            side = determine_hand_side(*get_hand_center(hand_result.hand_landmarks[0]), plm) if hand_result.hand_landmarks else "Right"
        
        # Select correct points for Left or Right
        idx_s, idx_e, idx_w = (11, 13, 15) if side == "Left" else (12, 14, 16)
        arm_angle = calculate_angle_3pts((plm[idx_s].x, plm[idx_s].y), (plm[idx_e].x, plm[idx_e].y), (plm[idx_w].x, plm[idx_w].y))
        
        arm_orientation = determine_arm_orientation(arm_angle)
        # Check if elbow is bent
        flexion_state, diff_y = ("Flexed", abs(plm[idx_s].y - plm[idx_w].y)) if abs(plm[idx_s].y - plm[idx_w].y) > TH_FLEXION_Y else ("Extended", abs(plm[idx_s].y - plm[idx_w].y))
        
        # Write on screen and draw
        cv2.putText(out, f"Arm: {int(arm_angle)} deg {arm_orientation}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(out, f"Flex: {flexion_state}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        draw_landmarks_custom(out, plm, POSE_CONNECTIONS_ARMS, COLOR_POSE_JOINT, COLOR_POSE_BONE)

    # 6. If HAND detected:
    if hand_result.hand_landmarks:
        hand_lm = hand_result.hand_landmarks[0]
        # Calculate Hand Rotation (Palm/Back)
        _, hand_rotation_val, hand_orientation_str, _ = calculate_hand_rotation(
            hand_lm, flexion_state, arm_orientation, side)
        
        # Check each finger
        y0 = 80
        for name, ids in FINGERS.items():
            is_open = calculate_finger_angle(hand_lm, ids) > FINGER_THRESHOLDS[name]
            cv2.putText(out, f"{name[:3]}: {'OPN' if is_open else 'CLS'}", (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if is_open else (0, 0, 255), 2)
            y0 += 20
            
        # Draw rotation bar
        cv2.rectangle(out, (10, y0+10), (160, y0+25), (100, 100, 100), 2)
        cv2.circle(out, (10 + int((hand_rotation_val + 1) / 2 * 150), y0+17), 6, (0, 255, 255), -1)
        cv2.putText(out, f"{hand_orientation_str}", (10, y0+45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        draw_landmarks_custom(out, hand_lm, HAND_CONNECTIONS, COLOR_HAND_JOINT, COLOR_HAND_BONE)

    # 7. Show final image
    cv2.imshow("Detecao OTIMIZADA", out)
    
    # If ESC key (code 27) pressed, exit program
    if (cv2.waitKey(1) & 0xFF) == 27: break

# Final cleanup
vs.stop()
cv2.destroyAllWindows()

"""
======================================================================================
                            FILE NAME: ArmController.py
======================================================================================

BEGINNER DESCRIPTION:
This program runs on a Raspberry Pi 5 (a small computer).
It uses a camera to see the movement of a person's arm and hand and
translates these movements into commands that the robot can understand.

What this program does:
1. Captures video from the camera.
2. Uses "Artificial Intelligence" (MediaPipe) to detect where the fingers, elbow, and shoulder are.
3. Performs mathematical calculations to determine if the arm is extended or bent, and where it is pointing.
4. Sends this information via a cable (Serial) to the Arduino, which controls the motors.

LANGUAGE: Python
======================================================================================
"""

# --- IMPORT LIBRARIES (Pre-made tools) ---
import cv2                  # "OpenCV": Tool to process images and video
import mediapipe as mp      # "MediaPipe": Google's tool that detects hands and bodies
import math                 # Math tools (angles, distances)
import statistics           # To calculate averages
import time                 # To control time (pauses, measure delays)
import serial               # To talk to the Arduino via USB/Serial cable
from picamera2 import Picamera2 # Specific library to control the Raspberry Pi 5 camera (Pi Camera Module 3)
import numpy as np          # Tool for advanced mathematical calculations

# ------------------ OPTIMIZED CONFIGURATION FOR RASPBERRY PI ------------------
# We define the image size. Smaller images are processed faster.
FRAME_WIDTH = 640   # Width in pixels
FRAME_HEIGHT = 480  # Height in pixels
FPS_TARGET = 30     # We aim for 30 images per second (normal video smoothness)

# --- COMMUNICATION CONFIGURATION (SERIAL) ---
# Here we say where the Arduino is connected and how fast to talk to it.
SERIAL_PORT = "/dev/ttyAMA0"  # Internal Raspberry Pi port (GPIO Pins)
BAUD_RATE = 115200            # Speed of conversation (bits per second). Must be the same on Arduino!
SEND_INTERVAL = 0.1           # We only send data every 0.1 seconds (10 times/second) to avoid clogging

# ---------- ADJUSTABLE PARAMETERS (Tuning) ----------
# Thresholds: Reference values. If we pass this value, it is considered "Open" or "Closed".

# Fingers: Minimum angle to consider the finger OPEN.
TH_THUMB = 150.0
TH_INDEX = 160.0
TH_MIDDLE = 150.0
TH_RING = 150.0
TH_PINKY = 140.0

# Arm: Angles to know where the arm is pointing.
TH_ARM_LEFT = 70          # 0 to 70 degrees: Arm facing the body (Left)
TH_ARM_CENTER_MAX = 130   # 70 to 130 degrees: Arm in front
                          # 130 to 180 degrees: Arm outwards (Right)

# Flexion: Vertical height difference (Y) between wrist and shoulder to know if arm is raised.
TH_FLEXION_Y = 0.1  # If difference is greater than 0.1, we consider it "Flexed" (Bent)
# ---------------------------------------------------------------

# --- INITIALIZE MEDIAPIPE (The Artificial Intelligence) ---
mp_pose = mp.solutions.pose       # Module to detect body (shoulders, elbows)
mp_hands = mp.solutions.hands     # Module to detect hands/fingers
mp_drawing = mp.solutions.drawing_utils # Useful for drawing the "skeletons" on the screen

# Visual configuration of the lines we will draw on the screen
drawing_spec_thin = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
drawing_spec_conn = mp_drawing.DrawingSpec(thickness=1)

# Finger Definition: Dictionary that says which points (landmarks) form each finger
FINGERS = {
    "thumb": [1, 2, 4],     # Specific points of the hand that form the thumb, usually 2, 3, 4
    "index": [5, 6, 8],
    "middle": [9, 10, 12],
    "ring": [13, 14, 16],
    "pinky": [17, 18, 20]
}

# Associating thresholds to each finger
FINGER_THRESHOLDS = {
    "thumb": TH_THUMB,
    "index": TH_INDEX,
    "middle": TH_MIDDLE,
    "ring": TH_RING,
    "pinky": TH_PINKY
}

# -------- START CONNECTION TO ARDUINO --------
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Wait 2 seconds for Arduino to wake up properly
    print(f"Connected to Arduino on {SERIAL_PORT}")
except Exception as e:
    # If it fails (cable disconnected?), notify but do not stop the program
    print(f"Could not connect to Arduino: {e}")
    print(f"   Check the port. You can use: ls /dev/tty*")
    arduino = None

# -------- LOGIC FUNCTIONS (The Brain of the Program) --------

def calculate_angle_3pts(a, b, c):
    """
    Calculates the angle between 3 points (like an elbow: shoulder, elbow, wrist).
    Inputs: Points a, b, c (x,y coordinates).
    Returns: Angle in degrees.
    """
    # Vector math (calculation of differences and dot products)
    ax, ay = a[0]-b[0], a[1]-b[1]
    cx, cy = c[0]-b[0], c[1]-b[1]
    num = ax*cx + ay*cy
    den = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-8
    cosv = max(-1.0, min(1.0, num/den))
    ang = math.degrees(math.acos(cosv))
    return ang

def calculate_finger_angle(hand_landmarks, ids):
    """
    Measures the angle at the middle 'hinge' of the finger.
    """
    a = hand_landmarks.landmark[ids[0]]
    b = hand_landmarks.landmark[ids[1]]
    c = hand_landmarks.landmark[ids[2]]
    return calculate_angle_3pts((a.x,a.y),(b.x,b.y),(c.x,c.y))

def get_hand_center(hand_landmarks):
    """Calculates the center point of the hand (average of all points)."""
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    return (statistics.mean(xs), statistics.mean(ys))

def determine_hand_side(hand_cx, hand_cy, pose_landmarks):
    """
    Figures out if the hand we are seeing is the RIGHT or LEFT one.
    Checks which wrist (shoulder/body landmarks) is closer.
    """
    if pose_landmarks is None:
        return None
    lw = pose_landmarks.landmark[15] # Body's left wrist
    rw = pose_landmarks.landmark[16] # Body's right wrist
    
    # Calculate distance (Pythagorean Theorem)
    dl = math.hypot(hand_cx - lw.x, hand_cy - lw.y)
    dr = math.hypot(hand_cx - rw.x, hand_cy - rw.y)
    
    return "Left" if dl < dr else "Right"

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

def calculate_hand_rotation(hand_landmarks, flex_status, arm_orientation):
    """
    Tries to guess if the palm is facing the camera or the back of the hand.
    Uses the position of the Thumb and Pinky.
    
    Returns a value between -1.0 (Palm to camera) and 1.0 (Back of hand).
    """
    thumb = hand_landmarks.landmark[4]  # Point at tip of thumb
    pinky = hand_landmarks.landmark[20] # Point at tip of pinky
    
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
        diff = -(t_val - p_val) 
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

# -------- INITIALIZE LOOP OF CAMERA AND MODELS --------

# 1. Configure and start camera
print("Initializing Picamera2...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
    controls={"FrameRate": FPS_TARGET}
)
picam2.configure(config)
picam2.start()

print("Camera started. Waiting for stabilization...")
time.sleep(2)

# 2. Configure detection models (MediaPipe)
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

# Control variables
frame_count = 0             # Counts how many frames passed
start_time = time.time()    # Saves start time
last_send = time.time()     # Saves time of last send to Arduino

print("System started. Press Ctrl+C to exit.")

try:
    # --- INFINITE LOOP (Repeats for every camera image) ---
    while True:
        # 1. Capture image
        frame = picam2.capture_array()
        
        # 2. Process with Artificial Intelligence
        pose_res = pose.process(frame)   # Looks for body
        hands_res = hands.process(frame) # Looks for hands
        
        # Create a copy of the image to draw on (out)
        out = frame
        h, w = frame.shape[:2]

        # Prepare empty variables (reset)
        arm_angle = None
        arm_orientation = "Undefined" 
        flexion_state = "Extended"
        side = None
        shoulder_y = 0
        wrist_y = 0
        diff_y = 0
        
        hand_rotation_val = 0
        hand_orientation_str = ""
        
        # --- PART A: BODY ANALYSIS (POSE) ---
        if pose_res.pose_landmarks:
            plm = pose_res.pose_landmarks.landmark
            
            # Tries to discover which arm is active using the detected hand
            if hands_res.multi_hand_landmarks:
                hand_lm_check = hands_res.multi_hand_landmarks[0]
                hc_x, hc_y = get_hand_center(hand_lm_check)
                side = determine_hand_side(hc_x, hc_y, pose_res.pose_landmarks)
            
            # If unsure, assume Right
            if side is None:
                side = "Right"
            
            # Select correct points (Shoulder, Elbow, Wrist) depending on side
            if side == "Left": # Points 11, 13, 15 are left side
                sh = plm[11]; el = plm[13]; wr = plm[15]
            else:              # Points 12, 14, 16 are right side
                sh = plm[12]; el = plm[14]; wr = plm[16]
            
            # Calculate angles and logic
            arm_angle = calculate_angle_3pts((sh.x, sh.y), (el.x, el.y), (wr.x, wr.y))
            arm_orientation = determine_arm_orientation(arm_angle)
            shoulder_y = sh.y; wrist_y = wr.y
            flexion_state, diff_y = determine_arm_flexion(shoulder_y, wrist_y)

            # Write text on screen so we see what it's thinking
            cv2.putText(out, f"Arm ({side}): {int(arm_angle)} deg", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(out, f"Orient: {arm_orientation}", 
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(out, f"Flex: {flexion_state}", 
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 0), 1)

            # Draw skeleton on screen
            mp_drawing.draw_landmarks(out, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      drawing_spec_thin, drawing_spec_conn)
        
        # --- PART B: HAND ANALYSIS ---
        finger_states = {}
        found_hand = False
        
        if hands_res.multi_hand_landmarks:
            found_hand = True
            hand_lm = hands_res.multi_hand_landmarks[0] # Use first hand found
            
            # Calculate Hand Rotation (Palm/Back)
            _, hand_rotation_val, hand_orientation_str, _ = calculate_hand_rotation(
                hand_lm, flexion_state, arm_orientation)
            
            # Calculate if each finger is open or closed
            for name, ids in FINGERS.items():
                ang = calculate_finger_angle(hand_lm, ids)
                threshold = FINGER_THRESHOLDS[name]
                is_open = ang > threshold # True if Angle > Threshold
                finger_states[name] = (is_open, ang)
            
            # --- DEBUG DRAWINGS (Visual Aid) ---
            # Rotation Bar
            bar_x, bar_y, bar_w, bar_h = 300, 30, 150, 15
            cv2.rectangle(out, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
            norm_pos = int((hand_rotation_val + 1) / 2 * bar_w) # Position point on bar
            cv2.circle(out, (bar_x + norm_pos, bar_y + int(bar_h/2)), 6, (0, 255, 255), -1)
            cv2.putText(out, f"Rot: {hand_rotation_val:.2f}", (bar_x, bar_y-5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Finger list in corner
            y0 = 100
            ordered_keys = ["index", "middle", "ring", "pinky", "thumb"]
            for i, k in enumerate(ordered_keys):
                is_open, ang = finger_states[k]
                color = (0, 255, 0) if is_open else (0, 0, 255) # Green if open, Red if closed
                cv2.putText(out, f"{k}: {int(ang)}", (10, y0 + i*20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Draw hand connections
            mp_drawing.draw_landmarks(out, hand_lm, mp_hands.HAND_CONNECTIONS,
                                      drawing_spec_thin, drawing_spec_conn)

        # --- PART C: SEND COMMANDS TO ARDUINO ---
        # Only send if enough time passed (SEND_INTERVAL) and if connected to Arduino
        if time.time() - last_send > SEND_INTERVAL and arduino and found_hand:
            
            # 1. Convert Base/Shoulder to numbers (0, 1, 2)
            val_orient = 1 # Center (Default)
            if "Left" in arm_orientation: val_orient = 0
            elif "Right" in arm_orientation: val_orient = 2
            
            # 2. Convert Flexion to number (0 or 1)
            val_flex = 1 if flexion_state == "Flexed" else 0
            
            # 3. Convert finger list to 0s and 1s
            d_list = []
            ordered_keys = ["index", "middle", "ring", "pinky", "thumb"]
            for k in ordered_keys:
                if k in finger_states:
                    d_list.append(1 if finger_states[k][0] else 0)
                else:
                    d_list.append(0) 
            
            # 4. Convert rotation (-1.0 to 1.0) to degrees (0 to 180) for servo
            rot_deg = int((hand_rotation_val + 1.0) * 90)
            rot_deg = max(0, min(180, rot_deg)) # Force limits between 0 and 180
            
            # 5. Create text message
            # Format: "$orient,flex,d1,d2,d3,d4,d5,rot\n"
            dedos_str = ",".join(map(str, d_list))
            msg = f"${val_orient},{val_flex},{dedos_str},{rot_deg}\n"
            
            try:
                arduino.write(msg.encode()) # Send via cable
                last_send = time.time()
                # print(f"TX: {msg.strip()}") # (Optional: See what we sent)
            except Exception as e:
                print(f"Serial Error: {e}")

        # --- PART D: SHOW IMAGE ---
        cv2.imshow("RPi Hand Tracking", out)
        
        # If ESC key (code 27) pressed, exit program
        if cv2.waitKey(1) & 0xFF == 27:
            break
            
        # --- STATISTICS (FPS) ---
        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            # Print speed (FPS) and rotation to console
            print(f"FPS: {frame_count/elapsed:.1f} | Rot: {hand_rotation_val:.2f}")

except KeyboardInterrupt:
    print("\nStopping (Ctrl+C pressed)...")

finally:
    # Final cleanup: Turn off camera, close windows and Arduino connection
    picam2.stop()
    if arduino:
        arduino.close()
    cv2.destroyAllWindows()
    print("Done.")

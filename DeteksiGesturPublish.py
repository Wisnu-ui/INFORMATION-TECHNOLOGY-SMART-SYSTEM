import paho.mqtt.client as paho
import time
import numpy as np
import cv2
import mediapipe as mp
from paho import mqtt

# MQTT Settings
broker = "7d63c3b70100485297e42154ae5a22cb.s1.eu.hivemq.cloud"
port = 8883
username = "sirobo"
password = "B@dak430"
topic = "XY"

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(
    min_detection_confidence=0.2,
    min_tracking_confidence=0.1,
    model_complexity=0,
    static_image_mode=False
)

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

# Camera coordinate limits
CAMERA_X_MIN = 250
CAMERA_X_MAX = 950
CAMERA_Y_MIN = 250
CAMERA_Y_MAX = 650

# Initialize Kalman Filter
kalman = cv2.KalmanFilter(4, 2)
kalman.statePre = np.zeros((4, 1), np.float32)
kalman.statePost = np.zeros((4, 1), np.float32)
kalman.transitionMatrix = np.array([
    [1, 0, 1, 0],
    [0, 1, 0, 1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
], np.float32)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-4
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-3
kalman.errorCovPost = np.eye(4, dtype=np.float32)

# MQTT Callback functions
def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT with result code: " + str(rc))

def on_publish(client, userdata, mid, properties=None):
    pass

# Initialize MQTT client
client = paho.Client(protocol=paho.MQTTv5)
client.username_pw_set(username, password)
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client.max_inflight_messages_set(1)
client.max_queued_messages_set(1)
client.on_connect = on_connect
client.on_publish = on_publish
client.connect(broker, port, 60)
client.loop_start()

def is_hand_closed(hand_landmarks):
    """Detect if hand is closed (gripping) based on finger positions"""
    index_tip = hand_landmarks.landmark[8]
    middle_tip = hand_landmarks.landmark[12]
    ring_tip = hand_landmarks.landmark[16]
    pinky_tip = hand_landmarks.landmark[20]
    
    index_base = hand_landmarks.landmark[5]
    middle_base = hand_landmarks.landmark[9]
    ring_base = hand_landmarks.landmark[13]
    pinky_base = hand_landmarks.landmark[17]
    
    index_dist = np.sqrt((index_tip.x - index_base.x)**2 + (index_tip.y - index_base.y)**2)
    middle_dist = np.sqrt((middle_tip.x - middle_base.x)**2 + (middle_tip.y - middle_base.y)**2)
    ring_dist = np.sqrt((ring_tip.x - ring_base.x)**2 + (ring_tip.y - ring_base.y)**2)
    pinky_dist = np.sqrt((pinky_tip.x - pinky_base.x)**2 + (pinky_tip.y - pinky_base.y)**2)
    
    return all(dist < 0.1 for dist in [index_dist, middle_dist, ring_dist, pinky_dist])

def map_hand_to_robot_coordinates(hand_x, hand_y, frame_width, frame_height):
    # Clamp hand coordinates to camera frame limits
    hand_x = max(CAMERA_X_MIN, min(CAMERA_X_MAX, hand_x))
    hand_y = max(CAMERA_Y_MIN, min(CAMERA_Y_MAX, hand_y))
    
    # Map X camera coordinate (250-950) to E axis (330-0)
    robot_e = int(np.interp(hand_x, 
                           [CAMERA_X_MIN, CAMERA_X_MAX], 
                           [330, 0]))
    
    # Map Y camera coordinate (250-650) to Z axis (200 to -100)
    robot_z = int(np.interp(hand_y, 
                           [CAMERA_Y_MIN, CAMERA_Y_MAX], 
                           [200, -100]))
    
    return robot_e, robot_z

def main():
    measurement = np.zeros((2, 1), dtype=np.float32)
    last_send_time = time.time()
    SEND_INTERVAL = 0.3  # 300ms interval
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        frame = cv2.flip(frame, 1)
        
        # Draw white frame for accessible area
        cv2.rectangle(frame, 
                     (CAMERA_X_MIN, CAMERA_Y_MIN), 
                     (CAMERA_X_MAX, CAMERA_Y_MAX), 
                     (255, 255, 255), 2)
        
        cv2.putText(frame, 'Robot Control Area', 
                   (CAMERA_X_MIN, CAMERA_Y_MIN - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                if hand_info.classification[0].label == 'Right':
                    # Draw landmarks
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style()
                    )

                    # Check hand state
                    hand_state = is_hand_closed(hand_landmarks)
                    state_text = "Gripping" if hand_state else "Opening"
                    cv2.putText(frame, f'Hand State: {state_text}', 
                              (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    # Get index finger position
                    landmark = hand_landmarks.landmark[1]
                    height, width = frame.shape[:2]
                    x = int(landmark.x * width)
                    y = int(landmark.y * height)

                    # Update Kalman Filter
                    kalman.predict()
                    measurement[0, 0] = np.float32(x)
                    measurement[1, 0] = np.float32(y)
                    kalman.correct(measurement)

                    # Get predicted position
                    predicted_x = int(kalman.statePost[0])
                    predicted_y = int(kalman.statePost[1])

                    # Map hand coordinates to robot coordinates
                    robot_e, robot_z = map_hand_to_robot_coordinates(predicted_x, predicted_y, width, height)
                    
                    # Send data every 300ms
                    if (current_time - last_send_time) >= SEND_INTERVAL:
                        # Send data: x, y, hand_state, timestamp
                        data = f"{robot_e},{robot_z},{int(hand_state)},{current_time}"
                        client.publish(topic, data)
                        last_send_time = current_time
                        print(f"Published - E: {robot_e}, Z: {robot_z}, Hand: {'Closed' if hand_state else 'Open'}")

                    # Visualize
                    cv2.circle(frame, (predicted_x, predicted_y), 10, (0, 255, 0), -1)
                    cv2.putText(frame, f'Hand: ({predicted_x}, {predicted_y})', 
                              (predicted_x + 10, predicted_y), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f'Robot: (E:{robot_e}, Z:{robot_z})', 
                              (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Hand Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main() 
import paho.mqtt.client as paho
from paho import mqtt
import time
import serial
import serial.tools.list_ports
import threading
from queue import Queue

# MQTT Settings
broker = "7d63c3b70100485297e42154ae5a22cb.s1.eu.hivemq.cloud"
port = 8883
username = "sirobo"
password = "B@dak430"
topic = "XY"

# Serial Settings
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
TIMEOUT = None

# Global variables for thread communication
is_running = True
data_queue = Queue()

# Robot control variables
GCODE_X = 0  # Fixed X position
GCODE_Y = 270  # Fixed Y position
MIN_FEED_RATE = 30     # Feed rate for small movements
MAX_FEED_RATE = 100    # Maximum feed rate for large movements
BASE_FEED_RATE = 50    # Base feed rate
MOVEMENT_RADIUS_THRESHOLD = 50  # Threshold for movement detection

def find_available_ports():
    """Find all available COM ports"""
    ports = serial.tools.list_ports.comports()
    available_ports = []
    for port in ports:
        available_ports.append(port.device)
    return available_ports

def initialize_serial():
    """Initialize serial connection with error handling"""
    try:
        # First try to close the port if it's already open
        try:
            temp_ser = serial.Serial(SERIAL_PORT)
            temp_ser.close()
        except:
            pass

        # Try to open the port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Successfully connected to {SERIAL_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {SERIAL_PORT}: {e}")
        print("\nAvailable ports:")
        available_ports = find_available_ports()
        for port in available_ports:
            print(f"- {port}")
        print("\nPlease check:")
        print("1. Is the correct port selected?")
        print("2. Is the device connected?")
        print("3. Is another program using the port?")
        print("4. Try unplugging and replugging the device")
        exit(1)

def wait_for_ok():
    while True:
        response = ser.readline()
        if "ok" in response.decode("utf-8"):
            break

def send_robot_command(e, z, feed_rate):
    cmd = f"G0 X{GCODE_X:.2f} Y{GCODE_Y:.2f} Z{z:.2f} E{e:.2f} F{feed_rate:.2f}\r"
    ser.write(cmd.encode('utf-8'))
    wait_for_ok()

def send_gripper_commands(is_closed):
    """Send appropriate M-codes based on hand state"""
    if is_closed:
        # Send M6 and M207 for closed hand
        ser.write(b'M6\r')
        wait_for_ok()
        ser.write(b'M207\r')
        wait_for_ok()
    else:
        # Send M7 and M206 for open hand
        ser.write(b'M7\r')
        wait_for_ok()
        ser.write(b'M206\r')
        wait_for_ok()

# Initialize serial connection
ser = initialize_serial()
time.sleep(1)

# MQTT Callback functions
def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT with result code: " + str(rc))
    client.subscribe(topic, qos=0)

def on_message(client, userdata, msg):
    try:
        # Parse received data: e, z, hand_state, timestamp
        e, z, hand_state, send_time = map(float, msg.payload.decode().split(','))
        current_time = time.time()
        latency = (current_time - send_time) * 1000  # Convert to milliseconds
        
        # Put data in queue for robot control
        data_queue.put((e, z, hand_state, latency))
        
    except Exception as e:
        print(f"Error processing message: {e}")

def robot_control_loop():
    global is_running
    last_position = None
    last_hand_state = None
    
    while is_running:
        try:
            if not data_queue.empty():
                e, z, hand_state, latency = data_queue.get()
                is_closed = bool(int(hand_state))
                
                # Handle gripper state
                if last_hand_state != is_closed:
                    print(f"Gripper state changed to: {'Closed' if is_closed else 'Open'}")
                    send_gripper_commands(is_closed)
                    last_hand_state = is_closed
                
                # Handle position movement
                current_position = (e, z)
                if last_position is not None:
                    dx = e - last_position[0]
                    dy = z - last_position[1]
                    movement_distance = (dx*dx + dy*dy)**0.5
                    
                    # Determine feed rate based on movement distance
                    if movement_distance > MOVEMENT_RADIUS_THRESHOLD:
                        feed_rate = MAX_FEED_RATE
                    else:
                        feed_rate = MIN_FEED_RATE
                    
                    send_robot_command(e, z, feed_rate)
                    print(f"Robot moved - E: {e:.1f}, Z: {z:.1f}, Feed Rate: {feed_rate}, Latency: {latency:.1f}ms")
                else:
                    # First movement
                    send_robot_command(e, z, BASE_FEED_RATE)
                    print(f"Initial position - E: {e:.1f}, Z: {z:.1f}")
                
                last_position = current_position
                
            time.sleep(0.01)  # Small sleep to prevent CPU overuse
            
        except Exception as e:
            print(f"Error in robot control loop: {e}")
            continue

# Initialize MQTT client
client = paho.Client(protocol=paho.MQTTv5)
client.username_pw_set(username, password)
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client.max_inflight_messages_set(1)
client.max_queued_messages_set(1)
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker, port, 60)
client.loop_start()

# Home the robot first
print("Homing robot...")
ser.write(b'G28\r')
time.sleep(5)
print(ser.readline())
print(ser.readline())
print(ser.readline())

print("Robot control system ready. Press Ctrl+C to exit.")

# Start robot control thread
robot_thread = threading.Thread(target=robot_control_loop)
robot_thread.start()

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping robot...")
    is_running = False
    robot_thread.join()
    
    ser.write(b'G0 X0 Y140 Z31\r')
    time.sleep(1)
    print("Disabling steppers in 5 seconds...")
    time.sleep(5)
    ser.write(b'M18\r')
    wait_for_ok()
    
    # Cleanup
    client.loop_stop()
    client.disconnect()
    ser.close() 
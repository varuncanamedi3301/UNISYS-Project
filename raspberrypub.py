import RPi.GPIO as IO # calling all libraries
import time
import picamera
from time import sleep
import cv2 
import matplotlib.pyplot as plt
import numpy as np
import math
import paho.mqtt.client as mqtt
import os

IO.setwarnings(False)
IO.setmode(IO.BOARD)
IO.setup(14,IO.IN) #GPIO 14 for IR sensor input
IO.setup(12,IO.OUT) # led pin 12 (for led controls)
camera = picamera.PiCamera() # initializing the raspberry pi camera module

broker = ''
port = 
publish_topic = "Gate1/camera"
subscribe_topic = "server/message1"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = ''
password = ''

def capture_photo():
    """ Capture image to 
    file new_image.jpg
    """
    camera.start_preview()
    sleep(5)
    camera.capture('/home/pi/python_code/capture/new_image.jpg')
    camera.stop_preview()     

def delete_photo():
    """ delete photo after 
    the program has been executed
    """
    if os.path.exists("/home/pi/python_code/capture/new_image.jpg"):
        os.remove("/home/pi/python_code/capture/new_image.jpg")
    else:
        print("file does not exist")

def connect_mqtt():
    """ connects to mqtt server and establishes connection 
    between the broker and client"""
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
        client = mqtt_client.Client(client_id)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client 
    
def publish(client, msg, tpc):
    """publish hog vector onto 
    the mqtt architeture with paho
    mqtt library"""
    msg_new = msg.tobytes()
    client.publish(tpc, msg)
    
def subscribe(client, topic):
    """recieve message from server """
    def on_message(client, userdata, msg):
        pl = msg.payload.decode()
        if pl == 'open':
            GPIO.output(12, GPIO.HIGH)
        else:
            GPIO.output(12, GPIO.LOW)
    client.subscribe(topic)
    client.on_message = on_message

class Hog_descriptor():
    def __init__(self, img, cell_size=16, bin_size=8):
        self.img = img
        self.img = np.sqrt(img / float(np.max(img)))
        self.img = self.img * 255
        self.cell_size = cell_size
        self.bin_size = bin_size
        self.angle_unit = 360 / self.bin_size
        assert type(self.bin_size) == int, "bin_size should be integer,"
        assert type(self.cell_size) == int, "cell_size should be integer,"

    def extract(self):
        height, width = self.img.shape
        gradient_magnitude, gradient_angle = self.global_gradient()
        gradient_magnitude = abs(gradient_magnitude)
        cell_gradient_vector = np.zeros((int(height / self.cell_size), int(width / self.cell_size), self.bin_size))
        for i in range(cell_gradient_vector.shape[0]):
            for j in range(cell_gradient_vector.shape[1]):
                cell_magnitude = gradient_magnitude[i * self.cell_size:(i + 1) * self.cell_size,
                                 j * self.cell_size:(j + 1) * self.cell_size]
                cell_angle = gradient_angle[i * self.cell_size:(i + 1) * self.cell_size,
                             j * self.cell_size:(j + 1) * self.cell_size]
                cell_gradient_vector[i][j] = self.cell_gradient(cell_magnitude, cell_angle)

        hog_image = self.render_gradient(np.zeros([height, width]), cell_gradient_vector)
        hog_vector = []
        for i in range(cell_gradient_vector.shape[0] - 1):
            for j in range(cell_gradient_vector.shape[1] - 1):
                block_vector = []
                block_vector.extend(cell_gradient_vector[i][j])
                block_vector.extend(cell_gradient_vector[i][j + 1])
                block_vector.extend(cell_gradient_vector[i + 1][j])
                block_vector.extend(cell_gradient_vector[i + 1][j + 1])
                mag = lambda vector: math.sqrt(sum(i ** 2 for i in vector))
                magnitude = mag(block_vector)
                if magnitude != 0:
                    normalize = lambda block_vector, magnitude: [element / magnitude for element in block_vector]
                    block_vector = normalize(block_vector, magnitude)
                hog_vector.append(block_vector)
        return hog_vector

    def global_gradient(self):
        gradient_values_x = cv2.Sobel(self.img, cv2.CV_64F, 1, 0, ksize=5)
        gradient_values_y = cv2.Sobel(self.img, cv2.CV_64F, 0, 1, ksize=5)
        gradient_magnitude = cv2.addWeighted(gradient_values_x, 0.5, gradient_values_y, 0.5, 0)
        gradient_angle = cv2.phase(gradient_values_x, gradient_values_y, angleInDegrees=True)
        return gradient_magnitude, gradient_angle

    def cell_gradient(self, cell_magnitude, cell_angle):
        orientation_centers = [0] * self.bin_size
        for i in range(cell_magnitude.shape[0]):
            for j in range(cell_magnitude.shape[1]):
                gradient_strength = cell_magnitude[i][j]
                gradient_angle = cell_angle[i][j]
                min_angle, max_angle, mod = self.get_closest_bins(gradient_angle)
                orientation_centers[min_angle] += (gradient_strength * (1 - (mod / self.angle_unit)))
                orientation_centers[max_angle] += (gradient_strength * (mod / self.angle_unit))
        return orientation_centers

    def get_closest_bins(self, gradient_angle):
        idx = int(gradient_angle / self.angle_unit)
        mod = gradient_angle % self.angle_unit
        if idx == self.bin_size:
            return idx - 1, (idx) % self.bin_size, mod
        return idx, (idx + 1) % self.bin_size, mod

    def render_gradient(self, image, cell_gradient):
        cell_width = self.cell_size / 2
        max_mag = np.array(cell_gradient).max()
        for x in range(cell_gradient.shape[0]):
            for y in range(cell_gradient.shape[1]):
                cell_grad = cell_gradient[x][y]
                cell_grad /= max_mag
                angle = 0
                angle_gap = self.angle_unit
                for magnitude in cell_grad:
                    angle_radian = math.radians(angle)
                    x1 = int(x * self.cell_size + magnitude * cell_width * math.cos(angle_radian))
                    y1 = int(y * self.cell_size + magnitude * cell_width * math.sin(angle_radian))
                    x2 = int(x * self.cell_size - magnitude * cell_width * math.cos(angle_radian))
                    y2 = int(y * self.cell_size - magnitude * cell_width * math.sin(angle_radian))
                    cv2.line(image, (y1, x1), (y2, x2), int(255 * math.sqrt(magnitude)))
                    angle += angle_gap
        return image

def HOG_extract():
    """extract HOG features usin the hog_descripter class
    defined above, returns list of hog features"""
    img = cv2.imread('/home/pi/python_code/capture/new_image.jpg', cv2.IMREAD_GRAYSCALE)
    resize_image = cv2.resize(img, (1050, 1610))
    hog = Hog_descriptor(resize_image, cell_size=4, bin_size=4)
    vector = hog.extract()
    return vector

while True:
    i = GPIO.input(14)
    if i==1:
        print('motion detected')
        capture_photo()
        sleep(1)
        hog_vctr = HOG_extract()
        client = connect_mqtt()
        client.loop_start()
        publish(client, hog_vctr, publish_topic)
        sleep(5)
        delete_photo()
        subscribe(client, subscribe_topic)

from collections import defaultdict
import cv2
import os
import numpy as np
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import time
from PIL import Image
import smbus2
from datetime import datetime
import tempfile
import sys
import subprocess
import threading
from telegram.ext import Application, MessageHandler, CommandHandler, filters
import requests

# GPIO Setup
BUZZER_PIN = 17
GREEN_LED_PIN = 26
RED_LED_PIN = 19
SERVO_PIN = 18
RELAY_PINS = {
    "relay1": 21,
    "relay2": 27,
    "relay3": 22,
    "relay4": 5,
}

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

ALL_OUTPUTS = list(RELAY_PINS.values())
for pin in ALL_OUTPUTS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

GPIO.output(BUZZER_PIN, GPIO.LOW)
GPIO.output(GREEN_LED_PIN, GPIO.LOW)
GPIO.output(RED_LED_PIN, GPIO.LOW)

# Servo PWM setup
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Telegram Bot Setup
BOT_TOKEN = "7038070025:AAHOoUWmqVPvFmmITJKpbWVGcdwzLDmcVJI"
BASE_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"
last_update_id = 0

def handle_telegram():
    global last_update_id
    print("[INFO] Telegram listener started...")
    while True:
        try:
            url = f"{BASE_URL}/getUpdates?offset={last_update_id + 1}&timeout=10"
            res = requests.get(url).json()

            for update in res.get("result", []):
                last_update_id = update["update_id"]
                msg = update.get("message", {}).get("text", "").lower()
                chat_id = update["message"]["chat"]["id"]
                print(f"[TELEGRAM] Received: {msg}")

                if msg == "/relayall_on":
                    for pin in RELAY_PINS.values():
                        GPIO.output(pin, GPIO.HIGH)
                    response = "All relays turned ON"

                elif msg == "/relayall_off":
                    for pin in RELAY_PINS.values():
                        GPIO.output(pin, GPIO.LOW)
                    response = "All relays turned OFF"

                elif msg.startswith("/relay"):
                    parts = msg.split("_")
                    if len(parts) == 2:
                        relay, state = parts
                        relay = relay.lstrip("/")
                        if relay in RELAY_PINS:
                            if state == "on":
                                GPIO.output(RELAY_PINS[relay], GPIO.HIGH)
                                response = f"{relay} turned ON"
                            elif state == "off":
                                GPIO.output(RELAY_PINS[relay], GPIO.LOW)
                                response = f"{relay} turned OFF"
                            else:
                                response = "Invalid state command."
                        else:
                            response = "Invalid relay name."
                    else:
                        response = "Invalid format. Use /relay1_on"
                else:
                    response = "Unknown command."

                requests.post(f"{BASE_URL}/sendMessage", data={"chat_id": chat_id, "text": response})
        except Exception as e:
            print(f"[ERROR] Telegram handler: {e}")
        time.sleep(2)

threading.Thread(target=handle_telegram, daemon=True).start()

# LCD Setup
LCD_ADDR = 0x27
LCD_WIDTH = 16
LCD_CHR = 1
LCD_CMD = 0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100
LINE_1 = 0x80
LINE_2 = 0xC0
bus = smbus2.SMBus(1)

attendance_log = defaultdict(list)

# LCD functions
def lcd_byte(bits, mode):
    high_bits = mode | (bits & 0xF0) | LCD_BACKLIGHT
    low_bits = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(LCD_ADDR, high_bits)
    lcd_toggle_enable(high_bits)
    bus.write_byte(LCD_ADDR, low_bits)
    lcd_toggle_enable(low_bits)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(LCD_ADDR, bits | ENABLE)
    time.sleep(0.0005)
    bus.write_byte(LCD_ADDR, bits & ~ENABLE)
    time.sleep(0.0005)

def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.005)

def lcd_display(message, line):
    lcd_byte(line, LCD_CMD)
    message = message.ljust(LCD_WIDTH, ' ')
    for char in message:
        lcd_byte(ord(char), LCD_CHR)

def rotate_servo():
    def angle_to_duty(angle):
        return 2 + (angle / 18)

    for angle in range(0, 181, 5):
        servo.ChangeDutyCycle(angle_to_duty(angle))
        time.sleep(0.02)
    servo.ChangeDutyCycle(0)
    time.sleep(10)
    for angle in range(180, -1, -5):
        servo.ChangeDutyCycle(angle_to_duty(angle))
        time.sleep(0.02)
    servo.ChangeDutyCycle(0)

def restart_program():
    print("[INFO] Restarting program due to camera error...")
    python = sys.executable
    os.execl(python, python, *sys.argv)

# Initialize LCD
lcd_init()
lcd_display("Welcome to", LINE_1)
lcd_display("SmartDoor System", LINE_2)
time.sleep(2)

reader = SimpleMFRC522()
face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
recognizer = cv2.face.LBPHFaceRecognizer_create()

try:
    while True:
        lcd_display("Scan your", LINE_1)
        lcd_display("RFID Card...", LINE_2)

        cam = cv2.VideoCapture(0)
        cam.set(3, 640)
        cam.set(4, 480)

        print("\n[INFO] Please scan your RFID card...")
        try:
            rfid_id, rfid_text = reader.read()
            rfid_id = str(rfid_id)
            print(f"[INFO] RFID Scanned: {rfid_id}")
            lcd_display("RFID Found", LINE_1)
            lcd_display("Processing...", LINE_2)
        except Exception as e:
            print(f"[ERROR] RFID Read Failed: {e}")
            lcd_display("RFID Read Error", LINE_1)
            lcd_display("Please Retry", LINE_2)
            GPIO.cleanup()
            break

        image_folder = os.path.join("dataset", rfid_id)
        if not os.path.exists(image_folder):
            print(f"[ERROR] No dataset folder found for RFID {rfid_id}")
            lcd_display("No Data Found", LINE_1)
            lcd_display("Access Denied", LINE_2)
            time.sleep(3)
            continue

        def get_images_and_labels(path):
            image_paths = [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.jpg')]
            face_samples = []
            ids = []
            for image_path in image_paths:
                img = Image.open(image_path).convert('L')
                img_np = np.array(img, 'uint8')
                faces = face_detector.detectMultiScale(img_np)
                for (x, y, w, h) in faces:
                    face_samples.append(img_np[y:y+h, x:x+w])
                    ids.append(1)
            return face_samples, ids

        print("[INFO] Training model from RFID-specific folder...")
        lcd_display("Training Face", LINE_1)
        lcd_display("Please Wait...", LINE_2)
        faces, ids = get_images_and_labels(image_folder)
        recognizer.train(faces, np.array(ids))

        print("[INFO] Model trained. Look at the camera...")
        lcd_display("Look at Camera", LINE_1)
        lcd_display("Verifying...", LINE_2)
        matched = False

        while True:
            ret, img = cam.read()
            if not ret or img is None:
                print("[ERROR] Failed to read from camera")
                cam.release()
                cv2.destroyAllWindows()
                time.sleep(2)
                restart_program()

            img = cv2.flip(img, -1)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_detector.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in faces:
                id_pred, confidence = recognizer.predict(gray[y:y+h, x:x+w])

                if confidence < 40:
                    name_file = os.path.join(image_folder, "name.txt")
                    person_name = open(name_file).read().strip() if os.path.exists(name_file) else "Matched"
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    print(f"[INFO] Face matched - {person_name} - Access Granted")
                    lcd_display("Your Door open", LINE_1)
                    lcd_display(f"Welcome {person_name[:9]}", LINE_2)
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
                    time.sleep(0.2)
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    time.sleep(3)
                    GPIO.output(GREEN_LED_PIN, GPIO.LOW)

                    rotate_servo()

                    GPIO.output(RELAY_PINS["relay1"], GPIO.HIGH)
                    GPIO.output(RELAY_PINS["relay3"], GPIO.HIGH)
                    GPIO.output(RELAY_PINS["relay2"], GPIO.LOW)
                    GPIO.output(RELAY_PINS["relay4"], GPIO.LOW)

                    matched = True
                    break
                else:
                    print("[WARNING] Unknown face detected - Triggering buzzer")
                    lcd_display("Unknown Face", LINE_1)
                    lcd_display("Access Denied", LINE_2)
                    for _ in range(2):
                        GPIO.output(BUZZER_PIN, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(BUZZER_PIN, GPIO.LOW)
                        time.sleep(0.2)
                    GPIO.output(RED_LED_PIN, GPIO.HIGH)
                    time.sleep(3)
                    GPIO.output(RED_LED_PIN, GPIO.LOW)
                    lcd_display("Put Correct", LINE_1)
                    lcd_display("Face", LINE_2)

            cv2.imshow("camera", img)
            if matched or cv2.waitKey(1) & 0xFF == 27:
                break

        cam.release()
        cv2.destroyAllWindows()
        time.sleep(3)

except KeyboardInterrupt:
    print("\n[INFO] Program interrupted. Exiting gracefully.")
    lcd_display("Welcome to", LINE_1)
    lcd_display("SmartDoor System", LINE_2)
    time.sleep(2)
    servo.stop()
    GPIO.cleanup()

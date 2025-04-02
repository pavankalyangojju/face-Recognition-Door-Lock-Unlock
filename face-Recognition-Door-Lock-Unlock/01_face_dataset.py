# -*- coding: utf-8 -*-
'''
Real Time Face Registration with RFID Integration, LED Indication, and I2C LCD Messages
'''

import cv2
import os
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import time
import smbus2

# GPIO and LED Setup
LED_PIN = 18
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

# I2C LCD Setup
LCD_ADDR = 0x27
LCD_WIDTH = 16
LCD_CHR = 1
LCD_CMD = 0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100
LINE_1 = 0x80
LINE_2 = 0xC0
bus = smbus2.SMBus(1)

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

lcd_init()

while True:
    lcd_display("Hi, Welcome to", LINE_1)
    lcd_display("SmartDoor System", LINE_2)
    time.sleep(2)
    lcd_display("Put RFID Card", LINE_1)
    lcd_display("", LINE_2)

    # Setup RFID Reader
    reader = SimpleMFRC522()

    # Initialize Camera
    cam = cv2.VideoCapture(0)
    cam.set(3, 640)
    cam.set(4, 480)

    # Load Haar Cascade for face detection
    face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Scan RFID Card
    print("\n[INFO] Please scan your RFID card to begin...")
    try:
        rfid_id, rfid_text = reader.read()
        print(f"\n[INFO] RFID ID: {rfid_id}")
    except Exception as e:
        print(f"[ERROR] RFID Read Error: {e}")
        lcd_display("RFID Read Error", LINE_1)
        lcd_display("Please Retry", LINE_2)
        GPIO.cleanup()
        exit()

    lcd_display("Type your name", LINE_1)
    lcd_display("In Terminal", LINE_2)

    # Ask for user's name
    name = input("Enter the name of the person: ").strip()

    # Create folder named after RFID if not exists
    rfid_folder = os.path.join('dataset', str(rfid_id))
    os.makedirs(rfid_folder, exist_ok=True)

    # Save name in a text file for future reference
    with open(os.path.join(rfid_folder, "name.txt"), "w") as f:
        f.write(name)

    print(f"\n[INFO] Registered name '{name}' for RFID {rfid_id}")
    lcd_display("Put your face", LINE_1)
    lcd_display("in front camera", LINE_2)
    time.sleep(2)
    lcd_display("Stay until", LINE_1)
    lcd_display("Light is OFF", LINE_2)

    print("[INFO] Initializing face capture. Look at the camera...")
    count = 0
    GPIO.output(LED_PIN, GPIO.HIGH)

    while True:
        ret, img = cam.read()
        img = cv2.flip(img, -1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_detector.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            count += 1
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            filename = os.path.join(rfid_folder, f"{count}.jpg")
            cv2.imwrite(filename, gray[y:y+h, x:x+w])
            cv2.imshow('image', img)

        k = cv2.waitKey(100) & 0xff
        if k == 27 or count >= 30:
            break

    GPIO.output(LED_PIN, GPIO.LOW)
    time.sleep(1)
    lcd_display("Data Saved", LINE_1)
    lcd_display("Successfully", LINE_2)
    time.sleep(3)
    lcd_display("Hi, Welcome to", LINE_1)
    lcd_display("SmartDoor System", LINE_2)

    print("\n[INFO] Exiting Program and cleaning up...")
    cam.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    break

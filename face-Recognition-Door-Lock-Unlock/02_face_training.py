import cv2
import numpy as np
from PIL import Image
import os
import csv

# Paths
dataset_path = 'dataset'
trainer_path = 'trainer'
labels_path = os.path.join(trainer_path, 'labels.csv')

if not os.path.exists(trainer_path):
    os.makedirs(trainer_path)

recognizer = cv2.face.LBPHFaceRecognizer_create()
detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

# Function to get images and labels
def getImagesAndLabels(path):
    face_samples = []
    ids = []
    labels = []

    for folder_name in os.listdir(path):
        folder_path = os.path.join(path, folder_name)
        if not os.path.isdir(folder_path):
            continue

        try:
            id = int(folder_name)
        except ValueError:
            print(f"[WARNING] Skipping folder with invalid RFID: {folder_name}")
            continue

        # Read name from name.txt
        name_file = os.path.join(folder_path, 'name.txt')
        if os.path.exists(name_file):
            with open(name_file, 'r') as f:
                name = f.read().strip()
        else:
            name = "Unknown"

        labels.append((id, name))

        for image_file in os.listdir(folder_path):
            image_path = os.path.join(folder_path, image_file)
            try:
                pil_img = Image.open(image_path).convert('L')
            except Exception as e:
                print(f"[WARNING] Skipping image {image_path}: {e}")
                continue

            img_numpy = np.array(pil_img, 'uint8')
            faces = detector.detectMultiScale(img_numpy)

            for (x, y, w, h) in faces:
                face_samples.append(img_numpy[y:y+h, x:x+w])
                ids.append(id)

    return face_samples, ids, labels

print("\n[INFO] Training faces. It will take a few seconds. Wait ...")
faces, ids, labels = getImagesAndLabels(dataset_path)
recognizer.train(faces, np.array(ids))

# Save trained model
recognizer.write(os.path.join(trainer_path, 'trainer.yml'))

# Save labels to CSV
with open(labels_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["RFID", "Name"])
    unique_labels = list({(id_, name) for id_, name in labels})  # remove duplicates
    writer.writerows(unique_labels)

print(f"\n[INFO] {len(np.unique(ids))} unique faces trained.")
print(f"[INFO] Labels saved to {labels_path}. Exiting Program.")

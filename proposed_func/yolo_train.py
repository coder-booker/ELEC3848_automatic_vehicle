

from ultralytics import YOLO
import os
import cv2
import numpy as np
# import onnx
# import onnxruntime

os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

def train():
    model = YOLO('yolov8n.pt')  # Load model
    model.train(data="D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\Tennis Model.v3i.yolov8\data.yaml", epochs=100);
    model.val();
    model.save('yolov8n_tennis.pt')  # Save model

def predict():
    model = YOLO('D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\\best.pt')  # Load model
    # model.save('D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\yolov8n_tennis.pt')  # Save model
    # Load an image
    img_path = "D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\Tennis Model.v3i.yolov8\\test\images\\0a3677732f6f0ea1_jpg.rf.01624c89313df89be36e29854c24dfa5.jpg"
    img = cv2.imread(img_path)

    # Perform prediction
    results = model(img)
    for r in results:
        # print(r)
        boxes = r.boxes[0].cpu().numpy()
        print(boxes)
        xyxy = boxes.xyxy.squeeze().tolist()
        x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
        
        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(img, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

    cv2.imwrite('output.jpg', img)

predict()
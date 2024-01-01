from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch

# Use the model
model.train(data="object_detection/dataset/data.yaml", epochs=25)  # train the model
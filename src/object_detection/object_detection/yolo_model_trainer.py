from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch

# Use the model
results = model.train(data="src/object_detection/object_detection/config.yaml", epochs=100)  # train the model

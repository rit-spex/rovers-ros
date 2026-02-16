from ultralytics.models import YOLO

# 1. Load your model from the specific path
model_path = '/home/spex-rover/SPEX/rovers-ros/src/autonomous/object_detection/object_detection/last.pt'
model = YOLO(model_path)

# 2. Export to TensorRT
# usage: export(format, device, half, workspace)
# workspace=4 tells TensorRT it can use up to 4GB of RAM during build (helps prevent crashes)
model.export(format='engine', device=0, half=True, workspace=4)
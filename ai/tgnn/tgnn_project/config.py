import os
    
BASE_DIR = os.path.join("..", "dataset")
DATASET_PATH = os.path.join(BASE_DIR, "dataset.csv")
SAVE_DIR = os.path.join("..", "save")
MODEL_PATH = os.path.join(SAVE_DIR, "model.pth")
FEATURE_SCALER_PATH = os.path.join(SAVE_DIR, "feature_scaler.pkl")
TARGET_SCALER_PATH = os.path.join(SAVE_DIR, "target_scaler.pkl")

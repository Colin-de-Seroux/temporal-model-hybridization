import os
import sys
import torch
import joblib

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from tgnn_project.tgnn_project.model import GConvGRUModel

def load_model_and_scalers():
    save_dir = os.path.join("..", "save")
    model_path = os.path.join(save_dir, "model.pth")
    feature_scaler_path = os.path.join(save_dir, "feature_scaler.pkl")
    target_scaler_path = os.path.join(save_dir, "target_scaler.pkl")

    model = GConvGRUModel(in_channels=6, out_channels=4)
    model.load_state_dict(torch.load(model_path))
    model.eval()

    feature_scaler = joblib.load(feature_scaler_path)
    target_scaler = joblib.load(target_scaler_path)

    print("✅ Model and scalers loaded.")
    return model, feature_scaler, target_scaler

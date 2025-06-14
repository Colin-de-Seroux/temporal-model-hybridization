import os
import sys
import torch
import joblib
from tgnn_project.model import GConvGRUModel
import config


def load_model_and_scalers():
    model_path = config.MODEL_PATH
    feature_scaler_path = config.FEATURE_SCALER_PATH
    target_scaler_path = config.TARGET_SCALER_PATH

    model = GConvGRUModel(in_channels=6, out_channels=4)
    model.load_state_dict(torch.load(model_path))
    model.eval()

    feature_scaler = joblib.load(feature_scaler_path)
    target_scaler = joblib.load(target_scaler_path)

    print("✅ Model and scalers loaded.")
    return model, feature_scaler, target_scaler

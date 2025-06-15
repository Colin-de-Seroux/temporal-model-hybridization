import torch
import joblib
import os
from tgnn_project.model import GConvGRUModel
from tgnn_project.dataset import create_dataset
from tgnn_project.predict import predict
import config

def main():
    dataset_path =config.PREDICT_DATASET_PATH 
    save_dir = config.SAVE_DIR 
    feature_scaler_path = config.FEATURE_SCALER_PATH
    target_scaler_path = config.TARGET_SCALER_PATH
    model_path = config.MODEL_PATH

    dataset = create_dataset(dataset_path, is_prediction=True)

    feature_scaler = joblib.load(feature_scaler_path)
    target_scaler = joblib.load(target_scaler_path)

    model = GConvGRUModel(in_channels=6, out_channels=4)
    model.load_state_dict(torch.load(model_path))
    print("✅ Model loaded successfully.")

    predictions = predict(model, dataset, feature_scaler, target_scaler)
    print("Predictions:", predictions)

if __name__ == "__main__":
    print("Prediction")
    main()

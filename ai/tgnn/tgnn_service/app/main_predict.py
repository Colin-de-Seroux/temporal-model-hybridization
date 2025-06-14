import torch
import joblib
import os
from tgnn_project.model import GConvGRUModel
from tgnn_project.data_loader import create_dataset
from tgnn_project.train import predict

def main():
    dataset_path = os.path.join("..", "dataset", "predict_dataset.csv")
    save_dir = os.path.join("..", "save")
    feature_scaler_path = os.path.join(save_dir, "feature_scaler.pkl")
    target_scaler_path = os.path.join(save_dir, "target_scaler.pkl")
    model_path = os.path.join(save_dir, "model.pth")

    dataset = create_dataset(dataset_path, is_prediction=True)

    feature_scaler = joblib.load(feature_scaler_path)
    target_scaler = joblib.load(target_scaler_path)

    model = GConvGRUModel(in_channels=6, out_channels=4)
    model.load_state_dict(torch.load(model_path))
    print("✅ Model loaded successfully.")

    predict(model, dataset, feature_scaler, target_scaler)

if __name__ == "__main__":
    print("Prediction")
    main()

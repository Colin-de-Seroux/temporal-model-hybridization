import torch
import joblib
from tgnn_project.model import GConvGRUModel
from tgnn_project.data_loader import create_dataset
from tgnn_project.train import predict

def main():
    dataset = create_dataset("dataset/predict_dataset.csv", is_prediction=True)

    feature_scaler = joblib.load("save/feature_scaler.pkl")
    target_scaler = joblib.load("save/target_scaler.pkl")

    model = GConvGRUModel(in_channels=6, out_channels=4)
    model.load_state_dict(torch.load("save/model.pth"))
    print("✅ Model loaded successfully.")

    predict(model, dataset, feature_scaler, target_scaler)

if __name__ == "__main__":
    print("Prediction")
    main()

from tgnn_project.data_loader import create_dataset
from tgnn_project.train import train_model, predict
import networkx as nx
import matplotlib.pyplot as plt
import joblib
import torch
import os


def main():
    base_dir = os.path.join("..", "dataset")
    save_dir = os.path.join("..", "save2")

    dataset_path = os.path.join(base_dir, "dataset.csv")
    save_model_path = os.path.join(save_dir, "model.pth")
    save_feature_scaler_path = os.path.join(save_dir, "feature_scaler.pkl")
    save_target_scaler_path = os.path.join(save_dir, "target_scaler.pkl")

    dataset = create_dataset(dataset_path)
    model, feature_scaler, target_scaler = train_model(dataset, epochs=30, lr=0.01)

    os.makedirs(save_dir, exist_ok=True)

    torch.save(model.state_dict(), save_model_path)
    joblib.dump(feature_scaler, save_feature_scaler_path)
    joblib.dump(target_scaler, save_target_scaler_path)
    print("✅ Model and scalers saved.")


if __name__ == "__main__":
    print("Training")
    main()


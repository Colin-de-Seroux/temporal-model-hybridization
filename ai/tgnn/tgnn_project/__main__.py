from tgnn_project.dataset import create_dataset
from tgnn_project.train import train_model
import networkx as nx
import matplotlib.pyplot as plt
import joblib
import torch
import os
import config


def main():
    save_dir = config.SAVE_DIR
    dataset_path = config.DATASET_PATH
    save_model_path = config.MODEL_PATH
    save_feature_scaler_path = config.FEATURE_SCALER_PATH
    save_target_scaler_path = config.TARGET_SCALER_PATH

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


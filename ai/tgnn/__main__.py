from tgnn_project.data_loader import create_dataset
from tgnn_project.train import train_model, predict
import networkx as nx
import matplotlib.pyplot as plt
import joblib
import torch
import os


def main():
    dataset = create_dataset("dataset/dataset.csv")
    model, feature_scaler, target_scaler = train_model(dataset, epochs=30, lr=0.01)
    
    os.makedirs('save', exist_ok=True)

    torch.save(model.state_dict(), 'save/model.pth')
    joblib.dump(feature_scaler, 'save/feature_scaler.pkl')
    joblib.dump(target_scaler, 'save/target_scaler.pkl')
    print("✅ Model and scalers saved.")


if __name__ == "__main__":
    print("Training")
    main()

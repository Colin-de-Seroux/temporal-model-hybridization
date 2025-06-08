from tgnn_project.data_loader import create_dataset
from tgnn_project.train import train_model, predict
import networkx as nx
import matplotlib.pyplot as plt

def main():
    print("Chargement du dataset...")
    dataset = create_dataset("dataset/dataset.csv")
    model, feature_scaler, target_scaler = train_model(dataset, epochs=10, lr=0.01)
    predict(model, dataset, feature_scaler, target_scaler)

if __name__ == "__main__":
    print("Démarrage du programme...")
    main()

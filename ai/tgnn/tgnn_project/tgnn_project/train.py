import torch
import numpy as np
from sklearn.preprocessing import MinMaxScaler
from torch.nn import L1Loss, MSELoss
import matplotlib.pyplot as plt
from tgnn_project.model import GConvGRUModel

def train_model(dataset, epochs=10, lr=0.01):
    all_features = np.vstack([snapshot.x for snapshot in dataset])
    all_targets = np.concatenate([snapshot.y for snapshot in dataset]).reshape(-1, 1)

    feature_scaler = MinMaxScaler()
    target_scaler = MinMaxScaler()

    feature_scaler.fit(all_features)
    target_scaler.fit(all_targets)

    model = GConvGRUModel(in_channels=6, out_channels=4)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = MSELoss()
    mae_fn = L1Loss()

    # plt.hist(all_targets, bins=100)
    # plt.title("Distribution des temps d'exécution")
    # plt.xlabel("Execution Time (s)")
    # plt.ylabel("Count")
    # plt.grid(True)
    # plt.show()

    model.train()
    for epoch in range(epochs):
        loss_total = 0
        mae_total = 0
        h = None
        for snapshot in dataset:
            x_norm = feature_scaler.transform(snapshot.x)
            y_norm = target_scaler.transform(snapshot.y.reshape(-1, 1))

            x = torch.FloatTensor(x_norm)
            y = torch.FloatTensor(y_norm)

            edge_index = torch.LongTensor(snapshot.edge_index.T if snapshot.edge_index.shape[0] != 2 else snapshot.edge_index)
            edge_weight = torch.FloatTensor(snapshot.edge_weight) if snapshot.edge_weight is not None else torch.ones(edge_index.shape[1])

            if h is None or h.size(0) != x.size(0):
                h = torch.zeros(x.size(0), model.recurrent.out_channels)

            optimizer.zero_grad()
            out, h = model(x, edge_index, edge_weight, h)
            loss = loss_fn(out, y)
            loss.backward()
            optimizer.step()
            if h is not None:
                h = h.detach()
            loss_total += loss.item()
            mae_total += mae_fn(out, y).item()
        print(f"Epoch {epoch+1}, Loss: {loss_total:.4f}, MAE: {mae_total:.4f}")

    return model, feature_scaler, target_scaler

def predict(model, dataset, feature_scaler, target_scaler):
    model.eval()
    h = None
    for t, snapshot in enumerate(dataset):
        x_norm = feature_scaler.transform(snapshot.x)
        x = torch.FloatTensor(x_norm)
        y_norm = target_scaler.transform(snapshot.y.reshape(-1, 1))
        y = torch.FloatTensor(y_norm)

        edge_index = torch.LongTensor(snapshot.edge_index.T if snapshot.edge_index.shape[0] != 2 else snapshot.edge_index)
        edge_weight = torch.FloatTensor(snapshot.edge_weight) if snapshot.edge_weight is not None else torch.ones(edge_index.shape[1])

        if h is None or h.size(0) != x.size(0):
            h = torch.zeros(x.size(0), model.recurrent.out_channels)

        out, h = model(x, edge_index, edge_weight, h)
        out_norm = out.view(-1).detach().numpy().reshape(-1, 1)
        out_original = target_scaler.inverse_transform(out_norm).flatten()
        y_norm = y.view(-1).detach().numpy().reshape(-1, 1)
        real = target_scaler.inverse_transform(y_norm).flatten()

        print(f"Prédiction (dénormalisée) : {out_original}")
        print(f"Réel                   : {real}")
        for i, (p, r) in enumerate(zip(out_original, real)):
            print(f"Noeud {i} -> Prédiction: {p:.3f}, Réel: {r:.3f}")
        print()

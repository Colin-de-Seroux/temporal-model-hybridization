import torch
from torch_geometric_temporal.signal import DynamicGraphTemporalSignal
from torch_geometric_temporal.nn.recurrent import GConvGRU
import torch.nn.functional as F
import numpy as np
import pandas as pd
# ---------------------------
#  Données : graphe + temps d’exécution des actions
# ---------------------------
df = pd.read_csv("dataset/dataset.csv")
print(df.head())

nb_nodes = 4
nb_actions = 3
nb_timestamps = 30

edge_index = torch.tensor([
    [0, 1, 2, 3],
    [1, 2, 3, 0]
])
edge_indices = [edge_index for _ in range(nb_timestamps)]
edge_weights = [torch.ones(edge_index.shape[1]) for _ in range(nb_timestamps)]

features = []
targets = []
for t in range(nb_timestamps):
    actions = torch.rand(nb_nodes, nb_actions) * 10  
    features.append(actions)
    mean_time = actions.mean(dim=1, keepdim=True)  
    targets.append(mean_time)

targets = [t.numpy() if isinstance(t, torch.Tensor) else np.array(t) for t in targets]

dataset = DynamicGraphTemporalSignal(edge_indices, edge_weights, features, targets)

features = []
targets = []
for t in range(nb_timestamps):
    actions = torch.rand(nb_nodes, nb_actions) * 10  
    features.append(actions)
    mean_time = actions.mean(dim=1, keepdim=True)
    targets.append(mean_time)

targets = [t.float() if isinstance(t, torch.Tensor) else torch.tensor(t, dtype=torch.float) for t in targets]

# ---------------------------
#  Modèle
# ---------------------------

class ActionPredictor(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.recurrent = GConvGRU(in_channels=nb_actions, out_channels=32, K=2)
        self.linear = torch.nn.Linear(32, 1)

    def forward(self, x, edge_index, edge_weight, h):
        h = self.recurrent(x, edge_index, edge_weight, h)
        out = self.linear(h)  
        return out, h

# ---------------------------
#  Entraînement
# ---------------------------

model = ActionPredictor()
optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

for epoch in range(10):
    h = None
    total_loss = 0
    print(f"\n🔁 Entraînement - Époque {epoch + 1}")
    print(dataset)
    for t, snapshot in enumerate(dataset):
        print(f"Traitement du snapshot {t}...") 
        x, edge_index, edge_weight, y = snapshot.x, snapshot.edge_index, snapshot.edge_attr, snapshot.y

        optimizer.zero_grad()
        out, h = model(x, edge_index, edge_weight, h)
        loss = F.mse_loss(out, y)
        loss.backward()
        optimizer.step()
        h = h.detach()
        total_loss += loss.item()
    print(f"[Epoch {epoch}] Loss: {total_loss:.4f}")

# ---------------------------
#  Prédiction
# ---------------------------
print("\n Prédiction sur le dernier snapshot du graphe :")
h = None 
last_snapshot = None

for snapshot in dataset:
    x, edge_index, edge_weight, y = snapshot.x, snapshot.edge_index, snapshot.edge_attr, snapshot.y
    out, h = model(x, edge_index, edge_weight, h)
    h = h.detach()
    last_snapshot = snapshot

x_last, edge_index_last, edge_weight_last, y_last = last_snapshot.x, last_snapshot.edge_index, last_snapshot.edge_attr, last_snapshot.y
with torch.no_grad():
    pred, _ = model(x_last, edge_index_last, edge_weight_last, h)


print("Temps d exécution des actions (features d entrée) :")
print(x_last)

print("\nTemps moyen réel à prédire (target réelle) :")
print(y_last.view(-1).numpy())

print("\nTemps moyen prédit par le modèle :")
print(pred.view(-1).numpy())

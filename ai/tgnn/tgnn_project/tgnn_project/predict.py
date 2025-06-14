import torch


def predict(model, dataset, feature_scaler, target_scaler):
    model.eval()
    h = None
    all_predictions = []

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
        all_predictions.append(out_original)

        # print(f"Prédiction (dénormalisée) : {out_original}")
        # print(f"Réel                   : {real}")
        # for i, (p, r) in enumerate(zip(out_original, real)):
        #     print(f"Noeud {i} -> Prédiction: {p:.3f}, Réel: {r:.3f}")
        # print()

    return [float(pred) for batch in all_predictions for pred in batch]

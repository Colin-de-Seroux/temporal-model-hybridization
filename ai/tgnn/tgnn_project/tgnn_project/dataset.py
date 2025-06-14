
import numpy as np
import pandas as pd
from torch_geometric_temporal.signal import DynamicGraphTemporalSignal

from tgnn_project.preprocess import encode_action_type


class CustomDynamicGraphTemporalSignal(DynamicGraphTemporalSignal):
    def __init__(self, edge_indices, edge_weights, features, targets):
        super().__init__(edge_indices=edge_indices, edge_weights=edge_weights, features=features, targets=targets)
        self.edge_weights = edge_weights

    def __iter__(self):
        for i in range(len(self.features)):
            yield type('Snapshot', (), {
                "x": self.features[i],
                "y": self.targets[i],
                "edge_index": self.edge_indices[i],
                "edge_weight": self.edge_weights[i]
            })


def create_dataset(csv_path, is_prediction=False):
    df = pd.read_csv(csv_path)
    df = df.sort_values(by=["ModelName", "NodeName", "ActionOrder"])
    has_execution_time = "ExecutionTime" in df.columns and not is_prediction

    unique_topics = df["Topic"].dropna().unique()
    topic_to_id = {topic: i for i, topic in enumerate(unique_topics)}

    snapshots = []

    for model_name, group in df.groupby("ModelName"):
        node_index_map = {}
        features = []
        targets = []
        node_exec_time = {}
        idx = 0

        for _, row in group.iterrows():
            node_id = f'{row["NodeName"]}-{row["ActionType"]}'
            if node_id not in node_index_map:
                node_index_map[node_id] = idx
                idx += 1

                one_hot = encode_action_type(row["ActionType"])
                topic_id = topic_to_id.get(row["Topic"], -1)
                period = row["Value"] / 1000.0 if row["ActionType"] == "timer" else row["Value"]
                feature_vector = one_hot + [topic_id, period, row["ActionOrder"]]
                features.append(feature_vector)
                if has_execution_time:
                    exec_time = row["ExecutionTime"]
                    targets.append(exec_time)
                    node_exec_time[node_id] = exec_time
                else:
                    targets.append(0.0)  
                    node_exec_time[node_id] = 0.0
               

        num_nodes = len(node_index_map)
        x = np.array(features)
        y = np.array(targets)

        edge_list = []
        edge_weights = []

        timers = group[group["ActionType"] == "timer"]
        publishers = group[group["ActionType"] == "pub"]
        subscribers = group[group["ActionType"] == "sub"]

        for _, timer in timers.iterrows():
            for _, pub in publishers.iterrows():
                if timer["NodeName"] == pub["NodeName"]:
                    src = f'{timer["NodeName"]}-timer'
                    dst = f'{pub["NodeName"]}-pub'
                    if src in node_index_map and dst in node_index_map:
                        edge_list.append((node_index_map[src], node_index_map[dst]))
                        weight = abs(node_exec_time[dst] - node_exec_time[src])
                        edge_weights.append(weight)

        for _, pub in publishers.iterrows():
            for _, sub in subscribers.iterrows():
                if pub["Topic"] == sub["Topic"]:
                    src = f'{pub["NodeName"]}-pub'
                    dst = f'{sub["NodeName"]}-sub'
                    if src in node_index_map and dst in node_index_map:
                        edge_list.append((node_index_map[src], node_index_map[dst]))
                        weight = abs(node_exec_time[dst] - node_exec_time[src])
                        edge_weights.append(weight)

        if edge_list:
            edge_index = np.array(edge_list).T
            edge_weight = np.array(edge_weights, dtype=np.float32)
        else:
            edge_index = np.empty((2, 0), dtype=np.int64)
            edge_weight = np.empty((0,), dtype=np.float32)

        snapshots.append((x, edge_index, edge_weight, y))

    features = [snap[0] for snap in snapshots]
    edge_indices = [snap[1] for snap in snapshots]
    edge_weights = [snap[2] for snap in snapshots]
    targets = [snap[3] for snap in snapshots]
    print("creation dataset")
    return CustomDynamicGraphTemporalSignal(
        edge_indices=edge_indices,
        edge_weights=edge_weights,
        features=features,
        targets=targets
    )

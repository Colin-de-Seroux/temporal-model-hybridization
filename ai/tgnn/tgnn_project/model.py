import torch
import torch.nn.functional as F
from torch_geometric_temporal.nn.recurrent import GConvGRU

class GConvGRUModel(torch.nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.recurrent = GConvGRU(
            in_channels=in_channels,
            out_channels=out_channels,
            K=2
        )
        self.linear = torch.nn.Sequential(
            torch.nn.Linear(out_channels, 1),
            torch.nn.Softplus()
        )

    def forward(self, x, edge_index, edge_weight, h):
        h = self.recurrent(x, edge_index, edge_weight, h)
        out = self.linear(F.relu(h))
        return out, h

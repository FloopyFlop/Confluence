import torch
import torch.nn as nn


class NN_classifier(nn.Module):
    def __init__(self, input_layer=81, p_mc_dropout=0.5):
        super().__init__()
        self.p_mc_dropout = p_mc_dropout
        self.linear1 = nn.Linear(input_layer, 256)
        self.linear2 = nn.Linear(256, 256)
        self.linear3 = nn.Linear(256, 64)
        self.linear4 = nn.Linear(64, 5)

    def forward(self, x, stochastic=True):
        x = nn.functional.relu(self.linear1(x))
        x = nn.functional.relu(self.linear2(x))
        x = nn.functional.relu(self.linear3(x))
        x = self.linear4(x)
        return x

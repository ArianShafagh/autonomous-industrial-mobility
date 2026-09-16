"""The neural half: a small network that scores ONE candidate action.

Input:  the feature vector of `features.action_features` (25 numbers).
Output: one number — how much value this action is expected to lead to, measured in delivered
        units (the objective of params.yaml), discounted over the look-ahead horizon.

It is deliberately tiny (two hidden layers, ~6 000 parameters). The heavy lifting of "what is
allowed" and "what is urgent" is done by the symbolic layer; the network only has to learn
judgement: is this trip worth its energy, is it better to fill the load first, is it better to
charge now than in ten minutes.
"""
import os

import torch
from torch import nn

from robofetch_ai.policies.features import FEATURE_NAMES

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "models", "ns_scorer.pt")


class ActionScorer(nn.Module):
    def __init__(self, hidden_sizes=(64, 64), n_features=len(FEATURE_NAMES)):
        super().__init__()
        layers, size = [], n_features
        for hidden in hidden_sizes:
            layers += [nn.Linear(size, hidden), nn.ReLU()]
            size = hidden
        layers.append(nn.Linear(size, 1))
        self.net = nn.Sequential(*layers)
        self.hidden_sizes = list(hidden_sizes)
        # Feature standardisation, filled at training time (kept with the weights so inference
        # never has to remember how the training data was scaled).
        self.register_buffer("mean", torch.zeros(n_features))
        self.register_buffer("std", torch.ones(n_features))

    def forward(self, x):
        return self.net((x - self.mean) / self.std).squeeze(-1)

    @torch.no_grad()
    def score(self, feature_rows):
        self.eval()
        x = torch.as_tensor(feature_rows, dtype=torch.float32)
        if x.ndim == 1:
            x = x.unsqueeze(0)
        return self(x).tolist()

    def set_normalisation(self, x):
        self.mean.copy_(x.mean(dim=0))
        self.std.copy_(x.std(dim=0).clamp_min(1e-6))

    def save(self, path=DEFAULT_MODEL, extra=None):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save({"state_dict": self.state_dict(), "hidden_sizes": self.hidden_sizes,
                    "features": FEATURE_NAMES, "extra": extra or {}}, path)
        return path

    @classmethod
    def load(cls, path=DEFAULT_MODEL):
        blob = torch.load(path, map_location="cpu", weights_only=False)
        if blob.get("features") != FEATURE_NAMES:
            raise ValueError(f"{path} was trained on different features - retrain it "
                             "(tools/ai/train_ns.py)")
        model = cls(tuple(blob["hidden_sizes"]))
        model.load_state_dict(blob["state_dict"])
        model.eval()
        model.info = blob.get("extra", {})
        return model

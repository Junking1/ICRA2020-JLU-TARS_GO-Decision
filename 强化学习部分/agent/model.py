import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from torch.autograd import Variable

class ActorCritic(nn.Module):
    def __init__(self):
        super(ActorCritic, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(135*2, 1024),
            nn.ReLU(),
            nn.Linear(1024, 256),
            nn.ReLU(),
        )
        self.head_a_m = nn.Sequential(
            nn.Linear(256, 3), # 8x4
            nn.Softmax(dim=1),
        )
        self.head_a_t = nn.Sequential(
            nn.Linear(256, 3), # 8x4
            nn.Softmax(dim=1),
        )
        self.head_v = nn.Sequential(
            nn.Linear(256, 1),
        )

    def forward(self, s):
        h = s
        wall = torch.stack([
            torch.min(s[:,0,135//2+45//2:], dim=1)[0], 
            torch.min(s[:,0,135//2-45//2:135//2+45//2], dim=1)[0], 
            torch.min(s[:,0,:135//2-45//2], dim=1)[0]
            ], dim=1) # batch, 3
        wall = F.softmax(wall, dim=1)
        enemy = torch.stack([
            torch.mean(s[:,1,135//2+45//2:], dim=1), 
            torch.mean(s[:,1,135//2-45//2:135//2+45//2], dim=1), 
            torch.mean(s[:,1,:135//2-45//2], dim=1)], dim=1) # batch, 3
        enemy = F.softmax(enemy, dim=1)
        batch, channel, seq = h.shape
        h = h.reshape([batch, -1])
        head = self.fc(h)
        a_m, a_t, v = self.head_a_m(head), self.head_a_t(head), self.head_v(head)
        a_m = F.softmax(a_m + wall + enemy, dim=1)
        a_t = F.softmax(a_t + enemy, dim=1)
        return a_m, a_t, v

if __name__=="__main__":
    x = torch.rand([1,2,135])
    model = ActorCritic()
    y = model(x)
    print(y)
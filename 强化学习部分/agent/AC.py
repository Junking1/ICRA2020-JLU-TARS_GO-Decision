import math
import random
import time
from collections import namedtuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
from torch.utils.data import DataLoader

from agent.model import ActorCritic
from utils import Action, RobotState

BATCH_SIZE = 2048
GAMMA = 0.99
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 50

Transition = namedtuple(
    'Transition', ('state', 'action', 'next_state', 'reward'))


def normal(x, mu, std):
    a = (-1*(x-mu).pow(2)/(2*std)).exp()
    b = 1/(2*std*np.pi).sqrt()
    return a*b


class ReplayMemory(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.epoch_memory = []
        self.main_memory = []
        self.position = 0
        #self.epoch_begin = 0

    def push(self, *args):
        """Saves a transition."""
        self.epoch_memory.append(Transition(*args))
        # if len(self.main_memory) < self.capacity:
        # self.main_memory.append(None)
        #self.main_memory[self.position] = Transition(*args)
        #self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, is_test=False):
        if is_test:
            return self.main_memory[0:batch_size]
        else:
            return random.sample(self.main_memory, batch_size)

    def finish_epoch(self):
        R = 0
        if len(self.main_memory) < self.capacity:
            for t in self.epoch_memory[::-1]:
                state, action, next_state, reward = t
                reward = reward[0]
                R = reward + GAMMA*R
                self.main_memory.append(Transition(
                    state, action, next_state, [R]))
        else:
            for t in self.epoch_memory[::-1]:
                state, action, next_state, reward = t
                reward = reward[0]
                R = reward + GAMMA*R
                self.main_memory[self.position] = Transition(
                    state, action, next_state, [R])
                self.position = (self.position + 1) % self.capacity
        del self.epoch_memory
        self.epoch_memory = []

    def __len__(self):
        return len(self.main_memory)

    def __getitem__(self, index):
        return self.main_memory[index]


class ActorCriticAgent():
    def __init__(self):
        # if gpu is to be used
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.device = device
        self.policy_net = ActorCritic().to(device).double()
        self.target_net = ActorCritic().to(device).double()
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.lr = 1e-5
        self.optimizer = optim.Adam([
            {"params": self.policy_net.head_a_m.parameters()},
            {"params": self.policy_net.head_a_t.parameters()},
            {"params": self.policy_net.fc.parameters()},
        ], lr=self.lr)
        self.optimizer2 = optim.Adam([
            {"params": self.policy_net.head_v.parameters()},
            {"params": self.policy_net.fc.parameters()},
        ], lr=self.lr)
        self.memory = ReplayMemory(100000)

    def preprocess(self, state: RobotState):
        return torch.tensor([state.scan]).double()  # 1x2xseq

    def run_AC(self, tensor_state):
        with torch.no_grad():
            self.target_net.eval()
            a_m, a_t, v = self.target_net(tensor_state.to(self.device))
        a_m = a_m.cpu().numpy()[0]  # left, ahead, right
        a_t = a_t.cpu().numpy()[0]  # turn left, stay, right

        return a_m, a_t

    def decode_action(self, a_m, a_t, state, mode):
        if mode == "max_probability":
            a_m = np.argmax(a_m)
            a_t = np.argmax(a_t)
        elif mode == "sample":
            #a_m += 0.01
            a_m /= a_m.sum()
            a_m = np.random.choice(range(3), p=a_m)
            #a_t += 0.01
            a_t /= a_t.sum()
            a_t = np.random.choice(range(3), p=a_t)

        action = Action()
        if a_m == 0:  # left
            action.v_n = -1.0
        elif a_m == 1:  # ahead
            action.v_t = +1.0
        elif a_m == 2:  # right
            action.v_n = +1.0

        if a_t == 0:  # left
            action.angular = +1.0
        elif a_t == 1:  # stay
            action.angular = 0.0
        elif a_t == 2:  # right
            action.angular = -1.0

        if state.detect:
            action.shoot = +1.0
        else:
            action.shoot = 0.0

        return action

    def select_action(self, state, mode):
        tensor_state = self.preprocess(state).to(self.device)
        a_m, a_t = self.run_AC(tensor_state)
        action = self.decode_action(a_m, a_t, state, mode)

        return action

    def push(self, state, next_state, action, reward):
        self.memory.push(state, action, next_state, reward)

    def make_state_map(self, state):
        return torch.cat(state, dim=0).double()  # batchx2xseq

    def sample_memory(self, is_test=False):
        device = self.device
        transitions = self.memory.sample(BATCH_SIZE, is_test)
        # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
        # detailed explanation). This converts batch-array of Transitions
        # to Transition of batch-arrays.
        batch = Transition(*zip(*transitions))

        #state_batch = torch.cat(batch.state).to(device)
        #next_state_batch = torch.cat(batch.next_state).to(device)
        state_batch = self.make_state_map(batch.state).double()
        next_state_batch = self.make_state_map(batch.next_state).double()
        action_batch = torch.tensor(batch.action).double()
        reward_batch = torch.tensor(batch.reward).double()

        return state_batch, action_batch, reward_batch, next_state_batch

    def optimize_once(self, data):
        state_batch, action_batch, reward_batch, next_state_batch = data
        device = self.device
        state_batch = state_batch.to(device)
        action_batch = action_batch.to(device)
        reward_batch = reward_batch.to(device)
        next_state_batch = next_state_batch.to(device)

        self.policy_net.train()
        state_batch = Variable(state_batch, requires_grad=True)
        a_m, a_t, value_eval = self.policy_net(state_batch)  # batch, 1, 10, 16
        ### Critic ###
        td_error = reward_batch - value_eval
        loss = nn.MSELoss()(value_eval, reward_batch)
        self.optimizer2.zero_grad()
        loss.backward(retain_graph=True)
        self.optimizer2.step()
        ### Actor ###
        #prob = x.gather(1, (action_batch[:,0:1]*32).long()) * y.gather(1, (action_batch[:,1:2]*20).long())
        prob_m = a_m.gather(1, action_batch[:, 0].long())
        prob_t = a_t.gather(1, action_batch[:, 1].long())
        log_prob = torch.log(prob_m * prob_t + 1e-6)
        exp_v = torch.mean(log_prob * td_error.detach())
        loss = -exp_v + F.smooth_l1_loss(value_eval, reward_batch)
        self.optimizer.zero_grad()
        loss.backward()
        # for param in self.model.parameters():
        # if param.grad is not None:
        #param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

        return loss.item()

    def optimize_online(self):
        if len(self.memory) < BATCH_SIZE:
            return
        data = self.sample_memory()
        loss = self.optimize_once(data)
        return loss

    def test_model(self):
        if len(self.memory) < BATCH_SIZE:
            return
        state_batch, action_batch, reward_batch, next_state_batch = self.sample_memory(
            True)
        device = self.device
        state_batch = state_batch.to(device)
        action_batch = action_batch.to(device)
        reward_batch = reward_batch.to(device)
        next_state_batch = next_state_batch.to(device)

        with torch.no_grad():
            self.target_net.eval()
            a_m, a_t, value_eval = self.target_net(
                state_batch)  # batch, 1, 10, 16
            ### Critic ###
            td_error = reward_batch - value_eval
            ### Actor ###
            #prob = x.gather(1, (action_batch[:,0:1]*32).long()) * y.gather(1, (action_batch[:,1:2]*20).long())
            prob_m = a_m.gather(1, action_batch[:, 0:1].long())
            prob_t = a_t.gather(1, action_batch[:, 1:2].long())
            log_prob = torch.log(prob_m * prob_t + 1e-6)
            exp_v = torch.mean(log_prob * td_error.detach())
            loss = -exp_v

        return loss.item()

    def save_model(self, file_path):
        torch.save(self.policy_net.state_dict(), file_path)

    def save_memory(self, file_path):
        torch.save(self.memory, file_path)

    def load_model(self, file_path):
        self.policy_net.load_state_dict(torch.load(
            file_path, map_location=self.device))
        # FIXME 开场直接加载已获得参数作为经验
        # self.update_target_net()

    def load_memory(self, file_path):
        self.memory = torch.load(file_path)

    def optimize_offline(self, num_epoch):
        def batch_state_map(transitions):
            batch = Transition(*zip(*transitions))
            state_batch = self.make_state_map(batch.state).double()
            next_state_batch = self.make_state_map(batch.next_state).double()
            action_batch = torch.tensor(batch.action).double()
            reward_batch = torch.tensor(batch.reward).double()
            return state_batch, action_batch, reward_batch, next_state_batch

        dataloader = DataLoader(self.memory.main_memory, batch_size=BATCH_SIZE,
                                shuffle=True, collate_fn=batch_state_map, num_workers=0, pin_memory=True)
        device = self.device
        for epoch in range(num_epoch):
            #print("Train epoch: [{}/{}]".format(epoch, num_epoch))
            for data in (dataloader):
                loss = self.optimize_once(data)
            #loss = self.test_model()
            #print("Test loss: {}".format(loss))
        return loss

    def decay_LR(self, decay):
        self.lr *= decay
        self.optimizer = optim.SGD(self.model.parameters(), lr=self.lr)

    def update_target_net(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())

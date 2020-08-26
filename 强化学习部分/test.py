'''
https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html
'''
import random
import argparse

import numpy as np
import torch

from agent.AC import ActorCriticAgent
from agent.hand import HandAgent
from simulator import ICRABattleField
from utils import Action, ID_R1, ID_B1

parser = argparse.ArgumentParser(
    description="Test the trained model in the ICRA 2020 Battlefield")
parser.add_argument("--seed", type=int, default=233, help="Random seed")
parser.add_argument("--enemy", type=str, default="hand",
                    help="The opposite agent type [AC, hand]")
parser.add_argument("--load_model", action = 'store_true', help = "Whether to load the trained model")
parser.add_argument("--load_model_path", type = str, default = "ICRA_save.model", help = "The path of trained model")
parser.add_argument("--epoch", type=int, default=50,
                    help="Number of epoches to test")
args = parser.parse_args()


torch.random.manual_seed(args.seed)
torch.cuda.random.manual_seed(args.seed)
np.random.seed(args.seed)
random.seed(args.seed)

agent = ActorCriticAgent()
if args.load_model:
    print("Load model")
    agent.load_model(args.load_model_path)
if args.enemy == "hand":
    agent2 = HandAgent()
elif args.enemy == "AC":
    agent2 = ActorCriticAgent()
    agent2.load_model(args.load_model_path)
else:
    print("Unknown agent!!!")
    exit()

env = ICRABattleField()
env.seed(args.seed)

for i_episode in range(args.epoch):
    print("Epoch: {}".format(i_episode))
    # Initialize the environment and state
    action = Action()
    pos = env.reset()
    if args.enemy == "hand":
        agent2.reset(pos)
    state, reward, done, info = env.step(action)
    for t in range(7*60*30):
        # Other agent
        if args.enemy == "hand":
            env.set_robot_action(ID_B1, agent2.select_action(state[ID_B1]))
        elif args.enemy == "AC":
            env.set_robot_action(ID_B1, agent2.select_action(
                state[ID_B1], mode="max_probability"))

        # Select and perform an action
        action = agent.select_action(state[ID_R1], "max_probability")

        # Step
        next_state, reward, done, info = env.step(action)
        state = next_state

        env.render()
        if done:
            break

env.close()

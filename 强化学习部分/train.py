'''
https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html
'''
import random
import time
import argparse

import matplotlib.pyplot as plt
import numpy as np
import torch

from agent.AC import ActorCriticAgent
from agent.hand import HandAgent
from simulator import ICRABattleField
from utils import Action, ID_R1, ID_B1, ID_R2, ID_B2

parser = argparse.ArgumentParser(
    description="Train the model in the ICRA 2020 Battlefield")
parser.add_argument("--seed", type=int, default=233, help="Random seed")
parser.add_argument("--enemy", type=str, default="hand",
                    help="The opposite agent type [AC, hand]")
parser.add_argument("--load_model", action='store_true', default=False,
                    help="Whether to load the trained model")
parser.add_argument("--load_model_path", type=str,
                    default="ICRA.model", help="The path of trained model")
parser.add_argument("--save_model_path", type=str,
                    default="ICRA_save.model", help="The path of trained model")
parser.add_argument("--epoch", type=int, default=1000,
                    help="Number of epoches to train")
parser.add_argument("--update_step", type=int, default=10,
                    help="After how many step, update the model?")
args = parser.parse_args()


torch.random.manual_seed(args.seed)
torch.cuda.random.manual_seed(args.seed)
np.random.seed(args.seed)
random.seed(args.seed)

agent = ActorCriticAgent()
# 新增的红方 agent
aux_agent = ActorCriticAgent()
if args.load_model:
    aux_agent.load_model(args.save_model_path)
    agent.load_model(args.save_model_path)
if args.enemy == "hand":
    agent2 = HandAgent()
    aux_agent2 = HandAgent()
elif args.enemy == "AC":
    agent2 = ActorCriticAgent()
    agent2.load_model(args.load_model_path)
    aux_agent2 = ActorCriticAgent()
    aux_agent2.load_model(args.load_model_path)


env = ICRABattleField()
if args.enemy == "person":
    env.render()
    env.viewer.window.on_key_press = env.key_press
    env.viewer.window.on_key_release = env.key_release
env.seed(args.seed)
losses = []
rewards = []
for i_episode in range(1, args.epoch + 1):
    print("Epoch: [{}/{}]".format(i_episode, args.epoch))
    # Initialize the environment and state
    action = Action()
    pos = env.reset()
    if args.enemy == "hand":
        pass
        # agent2.reset([7.5, 0.5])
    state, reward, done, info = env.step([action, action])
    
    for t in (range(2*60*30)):
        # Other agent
        if args.enemy == 'person':
            env.set_robot_action(ID_B1, env.action[0])
        if args.enemy == "hand":
            env.set_robot_action(ID_B1, agent2.select_action(state[ID_B1]))
            env.set_robot_action(ID_B2, aux_agent2.select_action(state[ID_B2]))
        elif args.enemy == "AC":
            env.set_robot_action(ID_B1, agent2.select_action(
                state[ID_B1], mode="max_probability"))
            env.set_robot_action(ID_B2, aux_agent2.select_action(
                state[ID_B2], mode="max_probability"))

        # Select and perform an action
        # 新增红方
        state_map = aux_agent.preprocess(state[ID_R2])
        a_m, a_t = aux_agent.run_AC(state_map)
        action2 = aux_agent.decode_action(a_m, a_t, state[ID_R2], "max_probability")

        state_map = agent.preprocess(state[ID_R1])
        a_m, a_t = agent.run_AC(state_map)
        action1 = agent.decode_action(a_m, a_t, state[ID_R1], "max_probability")

        # Step
        # print(action1)
        next_state, reward, done, info = env.step([action1, action2])
        # print(action1)
        # print(f"step reward: {reward}")
        tensor_next_state = agent.preprocess(next_state[ID_R1])

        # Store the transition in memory
        if state[ID_R1].health > 0 and not env.control:
            # print("memory once")
            agent.push(state_map, tensor_next_state, [a_m, a_t], [reward])
        state = next_state
        state_map = tensor_next_state

        env.render()
        # Perform one step of the optimization (on the target network)
        if done:
            break

    print("Simulation end in: {}:{:02d}, reward: {}".format(
        t//(60*30), t % (60*30)//30, env.reward))
    print(env.get_roborts()[ID_R1].get_health(),'\t', env.get_roborts()[ID_R2].get_health(), '\t', env.get_roborts()[ID_B1].get_health(), '\t', env.get_roborts()[ID_B2].get_health())
    agent.memory.finish_epoch()
    loss = agent.optimize_offline(1)
    losses.append(loss)
    rewards.append(env.reward)

    # # Update the target network, copying all weights and biases in DQN
    if i_episode % args.update_step == 0:
        agent.update_target_net()
        agent.save_model(args.save_model_path)

    if i_episode % 200 == 0:
        plt.title("Loss")
        plt.xlabel("Epoch")
        plt.ylabel("Loss")
        plt.plot(losses)
        plt.savefig("loss.pdf")

print('Complete')
env.close()

plt.title("Loss")
plt.xlabel("Epoch")
plt.ylabel("Loss")
plt.plot(losses)
plt.savefig("loss.pdf")

plt.title("Reward")
plt.xlabel("Epoch")
plt.ylabel("Final reward")
plt.plot(rewards)
plt.savefig("reward.pdf")

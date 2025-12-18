import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random

env = gym.make('CartPole-v1')
state_dim = env.observation_space.shape[0]
action_dim = env.action_space.n

class DQN(nn.Module):
    def __init__(self):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )
    def forward(self, x):
        return self.fc(x)

model = DQN()
optimizer = optim.Adam(model.parameters(), lr=0.001)
criterion = nn.MSELoss()

def select_action(state, eps):
    if random.random() < eps:
        return env.action_space.sample()
    else:
        q_values = model(torch.FloatTensor(state))
        return torch.argmax(q_values).item()
    
gamma = 0.99
epsilon = 1.0
memory = []

for episode in range(200):
    state, _ = env.reset()
    total_reward = 0
    for t in range(200):
        action = select_action(state, epsilon)
        next_state, reward, done, _, _ = env.step(action)
        memory.append((state, action, reward, next_state, done))
        if len(memory) > 1000:
            batch = random.sample(memory, 64)
            s, a, r, s_next, d = zip(*batch)
            s = torch.FloatTensor(s)
            a = torch.LongTensor(a).unsqueeze(1)
            r = torch.FloatTensor(r).unsqueeze(1)
            s_next = torch.FloatTensor(s_next)
            d = torch.FloatTensor(d).unsqueeze(1)
            q_val = model(s).gather(1, a)
            q_next = model(s_next).max(1)[0].unsqueeze(1)
            q_target = r + gamma * q_next * (1 - d)
            loss = criterion(q_val, q_target)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        state = next_state
        total_reward += reward
        if done:
            break
    epsilon = max(0.1, epsilon * 0.995)
    print(f"Episode {episode}, Reward: {total_reward}")
env.close()
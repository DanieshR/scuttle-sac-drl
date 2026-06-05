#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .off_policy_agent import OffPolicyAgent, Network
from ..common.settings import SAC_ALPHA, SAC_ALPHA_LR, LEARNABLE_TEMPERATURE

LOG_STD_MIN = -5
LOG_STD_MAX = 2
EPSILON     = 1e-6

LINEAR  = 0
ANGULAR = 1


class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super().__init__(name)
        self.fc1     = nn.Linear(state_size, hidden_size)
        self.fc2     = nn.Linear(hidden_size, hidden_size)
        self.mean    = nn.Linear(hidden_size, action_size)
        self.log_std = nn.Linear(hidden_size, action_size)
        self.apply(super().init_weights)

    def forward(self, state, visualize=False):
        x       = F.relu(self.fc1(state))
        x       = F.relu(self.fc2(x))
        mean    = self.mean(x)
        log_std = torch.clamp(self.log_std(x), LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, state):
        mean, log_std = self.forward(state)
        std    = log_std.exp()
        normal = torch.distributions.Normal(mean, std)
        x_t    = normal.rsample()                           # reparameterization trick
        action = torch.tanh(x_t)
        # log prob with tanh squashing correction
        log_prob = normal.log_prob(x_t) - torch.log(1 - action.pow(2) + EPSILON)
        log_prob = log_prob.sum(dim=-1, keepdim=True)
        return action, log_prob, torch.tanh(mean)


class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super().__init__(name)
        # Q1
        self.l1 = nn.Linear(state_size, hidden_size // 2)
        self.l2 = nn.Linear(action_size, hidden_size // 2)
        self.l3 = nn.Linear(hidden_size, hidden_size)
        self.l4 = nn.Linear(hidden_size, 1)
        # Q2
        self.l5 = nn.Linear(state_size, hidden_size // 2)
        self.l6 = nn.Linear(action_size, hidden_size // 2)
        self.l7 = nn.Linear(hidden_size, hidden_size)
        self.l8 = nn.Linear(hidden_size, 1)
        self.apply(super().init_weights)

    def forward(self, state, action):
        xs = F.relu(self.l1(state))
        xa = F.relu(self.l2(action))
        x  = torch.cat((xs, xa), dim=1)
        x  = F.relu(self.l3(x))
        q1 = self.l4(x)

        xs = F.relu(self.l5(state))
        xa = F.relu(self.l6(action))
        x  = torch.cat((xs, xa), dim=1)
        x  = F.relu(self.l7(x))
        q2 = self.l8(x)
        return q1, q2


class SAC(OffPolicyAgent):
    def __init__(self, device, simulation_speed):
        super().__init__(device, simulation_speed)

        self.target_entropy      = -float(self.action_size)
        self.learnable_temperature = LEARNABLE_TEMPERATURE
        self.log_alpha = torch.tensor(
            np.log(SAC_ALPHA), dtype=torch.float32
        ).to(device)
        if self.learnable_temperature:
            self.log_alpha.requires_grad_(True)
        self.alpha = self.log_alpha.exp().item()

        self.actor         = self.create_network(Actor,  'actor')
        self.critic        = self.create_network(Critic, 'critic')
        self.critic_target = self.create_network(Critic, 'critic_target')
        self.hard_update(self.critic_target, self.critic)

        self.actor_optimizer  = self.create_optimizer(self.actor)
        self.critic_optimizer = self.create_optimizer(self.critic)
        if self.learnable_temperature:
            self.alpha_optimizer = torch.optim.AdamW([self.log_alpha], lr=SAC_ALPHA_LR)

    def get_action(self, state, training, step, visualize=False):
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        if training:
            action, _, _ = self.actor.sample(state)
        else:
            with torch.no_grad():
                _, _, action = self.actor.sample(state)    # deterministic mean at test time
        # Return a plain Python list of floats (the DrlStep service rejects numpy
        # arrays — matches the original td3/ddpg .tolist() convention).
        return action.detach().cpu().numpy().flatten().tolist()

    def get_action_random(self):
        return np.random.uniform(-1.0, 1.0, self.action_size).tolist()

    def train(self, state, action, reward, next_state, done):
        # ------------------------------------------------------------------ #
        #  Critic update                                                       #
        # ------------------------------------------------------------------ #
        with torch.no_grad():
            next_action, next_log_prob, _ = self.actor.sample(next_state)
            q1_next, q2_next = self.critic_target(next_state, next_action)
            q_next   = torch.min(q1_next, q2_next) - self.alpha * next_log_prob
            q_target = reward + (1 - done) * self.discount_factor * q_next

        q1, q2      = self.critic(state, action)
        critic_loss = self.loss_function(q1, q_target) + self.loss_function(q2, q_target)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # ------------------------------------------------------------------ #
        #  Actor update                                                        #
        # ------------------------------------------------------------------ #
        action_new, log_prob, _ = self.actor.sample(state)
        q1_new, q2_new = self.critic(state, action_new)
        actor_loss = (self.alpha * log_prob - torch.min(q1_new, q2_new)).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # ------------------------------------------------------------------ #
        #  Alpha (temperature) update                                          #
        # ------------------------------------------------------------------ #
        if self.learnable_temperature:
            alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            self.alpha = self.log_alpha.exp().item()

        # ------------------------------------------------------------------ #
        #  Soft update critic target                                           #
        # ------------------------------------------------------------------ #
        self.soft_update(self.critic_target, self.critic, self.tau)

        return critic_loss.item(), actor_loss.item()

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

from .off_policy_agent import OffPolicyAgent, Network
from .td3 import Critic  # identical twin-Q structure

LINEAR = 0
ANGULAR = 1

LOG_STD_MIN = -20
LOG_STD_MAX = 2
EPSILON = 1e-6


class GaussianActor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(GaussianActor, self).__init__(name)
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.mean_layer    = nn.Linear(hidden_size, action_size)
        self.log_std_layer = nn.Linear(hidden_size, action_size)
        self.apply(super().init_weights)

    def forward(self, states, visualize=False):
        x = torch.relu(self.fa1(states))
        x = torch.relu(self.fa2(x))
        mean    = self.mean_layer(x)
        log_std = self.log_std_layer(x).clamp(LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, states):
        mean, log_std = self.forward(states)
        std = log_std.exp()
        dist = Normal(mean, std)
        x_t = dist.rsample()                              # reparameterisation trick
        y_t = torch.tanh(x_t)
        log_prob = dist.log_prob(x_t)
        log_prob -= torch.log(1 - y_t.pow(2) + EPSILON)  # tanh squash correction
        log_prob = log_prob.sum(dim=-1, keepdim=True)
        mean_action = torch.tanh(mean)
        return y_t, log_prob, mean_action


class SAC(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # Auto-tuned entropy temperature
        self.target_entropy  = -float(self.action_size)   # heuristic: -|A|
        self.log_alpha       = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha           = self.log_alpha.exp().item()
        self.alpha_optimizer = torch.optim.AdamW([self.log_alpha], lr=self.learning_rate)

        self.last_actor_loss = 0

        self.actor           = self.create_network(GaussianActor, 'actor')
        self.actor_optimizer = self.create_optimizer(self.actor)

        self.critic          = self.create_network(Critic, 'critic')
        self.critic_target   = self.create_network(Critic, 'target_critic')
        self.critic_optimizer = self.create_optimizer(self.critic)

        self.hard_update(self.critic_target, self.critic)

    def get_action(self, state, is_training, step, visualize=False):
        state = torch.from_numpy(np.asarray(state, np.float32)).unsqueeze(0).to(self.device)
        if is_training:
            action, _, _ = self.actor.sample(state)
        else:
            _, _, action = self.actor.sample(state)   # use mean at test time
        return action.squeeze(0).detach().cpu().numpy().tolist()

    def get_action_random(self):
        return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0)] * self.action_size

    def train(self, state, action, reward, state_next, done):
        # Critic update
        with torch.no_grad():
            next_action, next_log_prob, _ = self.actor.sample(state_next)
            Q1_next, Q2_next = self.critic_target(state_next, next_action)
            Q_next   = torch.min(Q1_next, Q2_next) - self.alpha * next_log_prob
            Q_target = reward + (1 - done) * self.discount_factor * Q_next

        Q1, Q2 = self.critic(state, action)
        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=2.0, norm_type=2)
        self.critic_optimizer.step()

        # Actor update
        action_new, log_prob, _ = self.actor.sample(state)
        Q1_new, Q2_new = self.critic(state, action_new)
        loss_actor = (self.alpha * log_prob - torch.min(Q1_new, Q2_new)).mean()
        self.actor_optimizer.zero_grad()
        loss_actor.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
        self.actor_optimizer.step()

        # Temperature update
        loss_alpha = -(self.log_alpha * (log_prob.detach() + self.target_entropy)).mean()
        self.alpha_optimizer.zero_grad()
        loss_alpha.backward()
        self.alpha_optimizer.step()
        self.alpha = self.log_alpha.exp().item()

        self.soft_update(self.critic_target, self.critic, self.tau)
        self.last_actor_loss = loss_actor.detach().cpu()
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]

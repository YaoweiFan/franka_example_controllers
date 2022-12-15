from fc_agent import FCAgent
from action_selectors import MultinomialActionSelector
import torch as th
import numpy as np


# This multi-agent controller shares parameters between agents
class Controller:
    def __init__(self):
        self.n_agents = 2
        self.agent = FCAgent()
        self.last_action = np.array([[0,0,0], [0,0,0]])

    def forward(self, transition):
        agent_inputs = self._build_inputs(transition)

        # agent_outs: 2*7
        # self.hidden_states: 2*64
        agent_outs = self.agent.forward(agent_inputs)

        return agent_outs.numpy()

    def _build_inputs(self, transition):
        agent_ids = np.array([[1,0], [0,1]])
        inputs = np.c_[transition["obs"], self.last_action, agent_ids]
        return inputs


from rnn_agent import RNNAgent
from action_selectors import MultinomialActionSelector
import torch as th
import numpy as np


# This multi-agent controller shares parameters between agents
class Controller:
    def __init__(self):
        self.n_agents = 2
        self.agent = RNNAgent()
        self.action_selector = MultinomialActionSelector()

        self.hidden_states = None
        self.last_action = np.array([[0,0,0,0,0,0,0], [0,0,0,0,0,0,0]])

    def select_actions(self, transition):
        # Only select actions for the selected batch elements in bs
        avail_actions = transition["avail_actions"]
        agent_outputs = self.forward(transition)
        # agent_outputs: tensor, 2*7
        # avail_actions: 2*7
        avail_actions = th.from_numpy(avail_actions)
        chosen_actions = self.action_selector.select_action(agent_outputs, avail_actions)
        # set self.last_action to one hot vector
        self.last_action = np.array([[0,0,0,0,0,0,0], [0,0,0,0,0,0,0]])
        self.last_action[0][chosen_actions[0]] = 1
        self.last_action[1][chosen_actions[1]] = 1

        return chosen_actions.numpy()

    def forward(self, transition):
        agent_inputs = self._build_inputs(transition)

        # agent_outs: 2*7
        # self.hidden_states: 2*64
        agent_outs, self.hidden_states = self.agent.forward(agent_inputs, self.hidden_states)

        # change to tensor for softmax
        agent_outs = th.from_numpy(agent_outs)

        # Softmax the agent outputs, policy logits
        agent_outs = th.nn.functional.softmax(agent_outs, dim=-1)

        return agent_outs

    def init_hidden(self):
        self.hidden_states = np.zeros((2,64))

    def _build_inputs(self, transition):
        agent_ids = np.array([[1,0], [0,1]])
        inputs = np.c_[transition["obs"], self.last_action, agent_ids]
        return inputs


import torch as th
from torch.distributions import Categorical

class MultinomialActionSelector():

    def __init__(self):
        # 0.05
        self.epsilon = 0.01
        self.test_greedy = False

    def select_action(self, agent_inputs, avail_actions):
        masked_policies = agent_inputs.clone()

        if self.test_greedy:
            picked_actions = masked_policies.max(dim=1)[1]
            # print picked_actions
            # print picked_actions.size()
            # print "*************"
        else:
            # 2*1
            picked_actions = Categorical(masked_policies).sample().long()
            # print picked_actions
            # print picked_actions.size()

            random_numbers = th.rand(2)
            # print random_numbers
            pick_random = (random_numbers < self.epsilon).long()
            # print pick_random
            # print pick_random.size()
            random_actions = Categorical(avail_actions.float()).sample().long()
            # print random_actions
            # print random_actions.size()
            picked_actions = pick_random * random_actions + (1 - pick_random) * picked_actions
            # print picked_actions
            # print picked_actions.size()

        # picked_actions: 1*2
        return picked_actions

import os
import numpy as np

class FCAgent():
    def __init__(self):
        # 42*64
        self.fc1_weight = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc1_weight.txt'))
        # 64
        self.fc1_bias = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc1_bias.txt'))

        # 64*64
        self.fc2_weight = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc2_weight.txt'))
        # 64
        self.fc2_bias = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc2_bias.txt'))

        # 64*3
        self.fc3_weight = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc3_weight.txt'))
        # 3
        self.fc3_bias = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/continuous/params/fc3_bias.txt'))

    def relu(self, inputs):
        return (inputs > 0) * inputs

    def sigmoid(self, inputs):
        return 1 / (1 + np.exp(-inputs))
    
    def tanh(self, input):
        return (np.exp(input)-np.exp(-input))/(np.exp(input)+np.exp(-input))

    def forward(self, inputs):
        """
        inputs: 2*42
        """
        # x: 64*2
        fc1_output = self.relu(np.dot(self.fc1_weight, inputs.T) + np.c_[self.fc1_bias, self.fc1_bias])
        fc2_output = self.relu(np.dot(self.fc2_weight, fc1_output.T) + np.c_[self.fc2_bias, self.fc2_bias])
        output = np.dot(self.fc3_weight, fc2_output.T) + np.c_[self.fc3_bias, self.fc3_bias]

        return output

import numpy as np

class RNNAgent():
    def __init__(self):
        # 64*42
        self.fc1_weight = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/fc1_weight.txt'))
        # 64
        self.fc1_bias = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/fc1_bias.txt'))

        # 64*64
        self.w_ir = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_wir.txt'))
        self.w_iz = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_wiz.txt'))
        self.w_in = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_win.txt'))
        self.w_hr = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_whr.txt'))
        self.w_hz = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_whz.txt'))
        self.w_hn = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_whn.txt'))
        # 64
        self.b_ir = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_bir.txt'))
        self.b_iz = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_biz.txt'))
        self.b_in = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_bin.txt'))
        self.b_hr = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_bhr.txt'))
        self.b_hz = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_bhz.txt'))
        self.b_hn = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/gru_bhn.txt'))

        # 7*64
        self.fc2_weight = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/fc2_weight.txt'))
        # 7
        self.fc2_bias = np.loadtxt(os.path.join(os.path.dirname(__file__), 'parameters/fc2_bias.txt'))

    def relu(self, inputs):
        return (inputs > 0) * inputs

    def sigmoid(self, inputs):
        return 1 / (1 + np.exp(-inputs))
    
    def tanh(self, input):
        return (np.exp(input)-np.exp(-input))/(np.exp(input)+np.exp(-input))

    def forward(self, inputs, hidden_state):
        # inputs: 2*42, hidden_state: 2*64
        # x: 64*2
        x = self.relu(np.dot(self.fc1_weight, inputs.T) + np.c_[self.fc1_bias, self.fc1_bias] )
        # print("inputs:")
        # print(inputs)
        # print("x:")
        # print(x.T)
        # print("hidden_state:")
        # print(hidden_state)
        # GRUCell
        # r: 64*2
        r = self.sigmoid( np.dot(self.w_ir, x) + np.c_[self.b_ir, self.b_ir] \
                            + np.dot(self.w_hr, hidden_state.T) + np.c_[self.b_hr, self.b_hr] )
        # z: 64*2
        z = self.sigmoid( np.dot(self.w_iz, x) + np.c_[self.b_iz, self.b_iz] \
                            + np.dot(self.w_hz, hidden_state.T) + np.c_[self.b_hz, self.b_hz] )
        # n: 64*2
        n = self.tanh( np.dot(self.w_in, x) + np.c_[self.b_in, self.b_in] \
                      + r * (np.dot(self.w_hn, hidden_state.T) + np.c_[self.b_hn, self.b_hn]) )
        # h: 64*2
        h = (1-z) * n + z * hidden_state.T

        # q: 7*2
        q = np.dot(self.fc2_weight, h) + np.c_[self.fc2_bias, self.fc2_bias]

        return q.T, h.T

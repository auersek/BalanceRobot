import random
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

def sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))

def sigmoid_prime(z):
    return sigmoid(z) * (1 - sigmoid(z))

import json

def save_weights_to_header(network, filename="nn_weights.h"):
    weights = [w.tolist() for w in network.weights]
    biases = [b.tolist() for b in network.biases]
    
    with open(filename, "w") as f:
        f.write("// Auto-generated neural network weights and biases\n")
        f.write("#ifndef NN_WEIGHTS_H\n#define NN_WEIGHTS_H\n\n")
        
        for i, (w, b) in enumerate(zip(weights, biases)):
            f.write(f"// Layer {i}\n")
            f.write(f"const float W{i}[{len(w)}][{len(w[0])}] = {{\n")
            for row in w:
                f.write("  {" + ", ".join(f"{v:.6f}" for v in row) + "},\n")
            f.write("};\n")

            f.write(f"const float Bias{i}[{len(b)}] = {{")
            f.write(", ".join(f"{v[0]:.6f}" for v in b))
            f.write("};\n\n")
        
        f.write("#endif // NN_WEIGHTS_H\n")

class Network(object):
    def __init__(self, sizes):
        self.num_layers = len(sizes)
        self.sizes = sizes
        self.biases = [np.random.randn(y, 1) for y in sizes[1:]]
        self.weights = [np.random.randn(y, x) for x, y in zip(sizes[:-1], sizes[1:])]

    def feedforward2(self, a):
        zs = []
        activations = [a]
        activation = a
        for b, w in zip(self.biases, self.weights):
            z = np.dot(w, activation) + b
            zs.append(z)
            activation = sigmoid(z)
            activations.append(activation)
        return zs, activations

    def SGD(self, training_data, epochs, mini_batch_size, eta, test_data=None):
        for j in range(epochs):
            random.shuffle(training_data)
            mini_batches = [
                training_data[k:k+mini_batch_size]
                for k in range(0, len(training_data), mini_batch_size)
            ]
            for mini_batch in mini_batches:
                self.update_mini_batch2(mini_batch, eta)
            if test_data:
                mse = self.evaluate(test_data)
                print(f"Epoch {j + 1}: MSE = {mse:.4f}")
            else:
                print(f"Epoch {j + 1} complete")

    def update_mini_batch2(self, mini_batch, eta):
        batch_size = len(mini_batch)
        x = np.asarray([_x.ravel() for _x, _y in mini_batch]).T
        y = np.asarray([_y.ravel() for _x, _y in mini_batch]).T

        nabla_b, nabla_w = self.backprop2(x, y)
        self.weights = [w - (eta / batch_size) * nw for w, nw in zip(self.weights, nabla_w)]
        self.biases = [b - (eta / batch_size) * nb for b, nb in zip(self.biases, nabla_b)]

    def backprop2(self, x, y):
        nabla_b = [0 for _ in self.biases]
        nabla_w = [0 for _ in self.weights]

        zs, activations = self.feedforward2(x)
        delta = self.cost_derivative(activations[-1], y) * sigmoid_prime(zs[-1])
        nabla_b[-1] = delta.sum(axis=1).reshape(-1, 1)
        nabla_w[-1] = np.dot(delta, activations[-2].T)

        for l in range(2, self.num_layers):
            z = zs[-l]
            sp = sigmoid_prime(z)
            delta = np.dot(self.weights[-l + 1].T, delta) * sp
            nabla_b[-l] = delta.sum(axis=1).reshape(-1, 1)
            nabla_w[-l] = np.dot(delta, activations[-l - 1].T)

        return (nabla_b, nabla_w)

    def evaluate(self, test_data):
        predictions = [self.feedforward2(x)[1][-1] for (x, _) in test_data]
        actuals = [y for (_, y) in test_data]
        mse = np.mean([
            np.mean((pred - actual)**2) for pred, actual in zip(predictions, actuals)
        ])
        return mse

    def cost_derivative(self, output_activations, y):
        return output_activations - y

if __name__ == '__main__':
    # load data in
    df = pd.read_csv("control_data.csv")

    # columns
    required_columns = ['AccelAngle', 'GyroAngle', 'SpinAngle', 'currentspeed', 'bigTiltCurrent' 'WheelPos', 'error', 'turnerror']
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Missing column in CSV: {col}")
        
    # input and output
    X = df[['AccelAngle', 'GyroAngle', 'SpinAngle', 'currentspeed', 'bigTiltCurrent' 'WheelPos', 'error', 'turnerror']]
    y = df[['stop', 'forward', 'reverse', 'clockwise', 'anticlockwise']]

    # train and test split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # normalize
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    # reshape
    training_data = [(x.reshape(-1, 1), y.reshape(-1, 1)) for x, y in zip(X_train_scaled, y_train.to_numpy())]
    test_data = [(x.reshape(-1, 1), y.reshape(-1, 1)) for x, y in zip(X_test_scaled, y_test.to_numpy())]

    # train
    net = Network([8, 16, 5])           # possibly use 2 more hidden layers
    net.SGD(training_data, epochs=30, mini_batch_size=20, eta=1.0, test_data=test_data)
    save_weights_to_header(net)










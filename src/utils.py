import numpy as np
import matplotlib.pyplot as plt


def world_to_robot_frame(input, theta):
    """input: dim=2"""
    input = np.array(input).reshape([2, 1])
    return (np.array([[-np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), -np.cos(theta)]]) @ input).reshape(2)

def plot(input, d=1, labels=None):
    # Suppose 'data' is your n x d NumPy array
    # For example:
    # data = np.array([[1, 2, 3],
    #                  [4, 5, 6],
    #                  [7, 8, 9],
    #                  ... ])

    # Generate sample data for demonstration (remove this in your actual code)
    n = input.shape[0]
    input = input.reshape([n, d])
    # Create an array of time steps
    time_steps = np.arange(n)

    # Plot each of the 3 columns
    plt.figure(figsize=(10, 6))
    for i in range(d):
        if labels is None:
            plt.plot(time_steps, input[:, i])
        else:
            plt.plot(time_steps, input[:, i], label=labels[i])

    plt.show()

def plot1d(input):
    n = input.shape[0]
    time_steps = np.arange(n)

    # Plot each of the 3 columns
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, input, label='cost')

    # Show the plot
    plt.show()

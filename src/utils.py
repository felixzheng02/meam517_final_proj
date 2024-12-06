import numpy as np
import matplotlib.pyplot as plt


def world_to_robot_frame(input, theta):
    """input: dim=2"""
    input = np.array(input).reshape([2, 1])
    return (np.array([[-np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), -np.cos(theta)]]) @ input).reshape(2)

import numpy as np
import matplotlib.pyplot as plt

def plot(data, d=1, labels=None, plot=True):
    """
    Plots each column of the input data array over time steps.

    Parameters:
    - data: n x d NumPy array, where n is the number of samples and d is the dimensionality.
    - d: The number of dimensions/columns to plot.
    - labels: A list of labels for each data series.

    """
    n = data.shape[0]
    data = data.reshape([n, d])
    # Create an array of time steps
    time_steps = np.arange(n)

    # Plot each of the d columns
    plt.figure(figsize=(10, 6))
    for i in range(d):
        if labels is None:
            plt.plot(time_steps, data[:, i])
        else:
            plt.plot(time_steps, data[:, i], label=labels[i])

    if labels is not None:
        plt.legend()  # Add this line to display the legend with labels

    plt.xlabel('Time Steps')
    plt.ylabel('Values')
    plt.title('Plot of Data Columns Over Time')
    if plot:
        plt.show()
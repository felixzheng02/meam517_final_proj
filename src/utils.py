import numpy as np
import matplotlib.pyplot as plt


def world_to_robot_frame(input, theta):
    """input: dim=2"""
    input = np.array(input).reshape([2, 1])
    return (np.array([[-np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), -np.cos(theta)]]) @ input).reshape(2)

def plot_w_hist(input):
    # Suppose 'data' is your n x 3 NumPy array
    # For example:
    # data = np.array([[1, 2, 3],
    #                  [4, 5, 6],
    #                  [7, 8, 9],
    #                  ... ])

    # Generate sample data for demonstration (remove this in your actual code)
    n = input.shape[0]

    # Create an array of time steps
    time_steps = np.arange(n)

    # Plot each of the 3 columns
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, input[:, 0], label='fy')
    plt.plot(time_steps, input[:, 1], label='fz')
    plt.plot(time_steps, input[:, 2], label='torque')

    # Add labels and title
    plt.xlabel('Time Steps')
    plt.ylabel('Values')
    plt.title('Time Series Data')
    plt.legend()

    # Show the plot
    plt.show()

def plot_cost_hist(input):
    n = input.shape[0]
    time_steps = np.arange(n)

    # Plot each of the 3 columns
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, input, label='cost')

    # Show the plot
    plt.show()

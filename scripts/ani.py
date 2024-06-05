# # plot
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import animation
# df = pd.read_csv(r"outputTue Jun  4 17:41:42 2024_FK_clean.csv")
# df.head()
# df["T"] = df["T"] / 1000

# fig = plt.figure()
# ax = Axes3D(fig)

# iterations = len(df)

# # setting
# ax.set_xlim3d([0,50])
# ax.set_ylim3d([-50,0])
# ax.set_zlim3d([100,150])
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# # ax.grid()
# ax.set_title("3D Test")
# # ax.view_init()
# init_pose = [df.iloc[0]['x1'], df.iloc[0]['y1'], df.iloc[0]['z1']]
# print(init_pose)

# def update(i, df, init):
#     tmp = df.iloc[i]
#     data = [tmp['x1'], tmp['y1'], tmp['z1']]
#     return data


# ani = animation.FuncAnimation(fig, update, iterations, fargs=(df, init_pose), interval=50, blit=False, repeat=True)
# plt.show()


# IMPORTS
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


def generate_data(nbr_iterations, nbr_elements):
    """
    Generates dummy data.
    The elements will be assigned random initial positions and speed.
    Args:
        nbr_iterations (int): Number of iterations data needs to be generated for.
        nbr_elements (int): Number of elements (or points) that will move.
    Returns:
        list: list of positions of elements. (Iterations x (# Elements x Dimensions))
    """
    dims = (3, 1)

    # Random initial positions.
    gaussian_mean = np.zeros(dims)
    gaussian_std = np.ones(dims)
    start_positions = np.array(
        list(
            map(np.random.normal, gaussian_mean, gaussian_std, [nbr_elements] * dims[0])
        )
    ).T

    # Random speed
    start_speed = np.array(
        list(
            map(np.random.normal, gaussian_mean, gaussian_std, [nbr_elements] * dims[0])
        )
    ).T

    # Computing trajectory
    data = [start_positions]
    for iteration in range(nbr_iterations):
        previous_positions = data[-1]
        new_positions = previous_positions + start_speed
        data.append(new_positions)

    return data


def animate_scatters(iteration, data, scatters):
    """
    Update the data held by the scatter plot and therefore animates it.
    Args:
        iteration (int): Current iteration of the animation
        data (list): List of the data positions at each iteration.
        scatters (list): List of all the scatters (One per element)
    Returns:
        list: List of scatters (One per element) with new coordinates
    """
    for i in range(data[0].shape[0]):
        scatters[i]._offsets3d = (
            data[iteration][i, 0:1],
            data[iteration][i, 1:2],
            data[iteration][i, 2:],
        )
    return scatters


def main(data, save=False):
    """
    Creates the 3D figure and animates it with the input data.
    Args:
        data (list): List of the data positions at each iteration.
        save (bool): Whether to save the recording of the animation. (Default to False).
    """

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # Initialize scatters
    scatters = [
        ax.scatter(data[0][i, 0:1], data[0][i, 1:2], data[0][i, 2:])
        for i in range(data[0].shape[0])
    ]

    # Number of iterations
    iterations = len(data)

    # Setting the axes properties
    ax.set_xlim3d([-50, 50])
    ax.set_xlabel("X")

    ax.set_ylim3d([-50, 50])
    ax.set_ylabel("Y")

    ax.set_zlim3d([-50, 50])
    ax.set_zlabel("Z")

    ax.set_title("3D Animated Scatter Example")

    # Provide starting angle for the view.
    ax.view_init(25, 10)

    ani = animation.FuncAnimation(
        fig,
        animate_scatters,
        iterations,
        fargs=(data, scatters),
        interval=50,
        blit=False,
        repeat=True,
    )


    plt.show()
    if save:
        Writer = animation.writers[""]
        writer = Writer(
            fps=30,
            metadata=dict(artist="Me"),
            bitrate=1800,
            extra_args=["-vcodec", "libx264"],
        )
        ani.save("3d-scatted-animated.mp4", writer=writer)


data = generate_data(100, 2)
main(data, save=True)

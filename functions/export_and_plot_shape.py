import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np

def export_and_plot_shape(output_file):

    data = pd.read_csv(output_file)

    # Extract the position coordinates and flight modes
    x = data['px']
    y = data['py']
    z = -1*data['pz']
    modes = data['mode']

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set colormap (you can change these colors to anything you like)
    colors = {0: 'grey', 10: 'orange', 20: 'yellow', 30: 'green', 40: 'blue', 50: 'purple', 60: 'brown', 70: 'red', 80: 'pink', 90: 'cyan', 100: 'black'}

    # Define flight mode names
    mode_names = {0: 'On the ground', 10: 'Trạng thái leo ban đầu', 20: 'Giữ ban đầu sau khi leo lên', 30: 'Di chuyển đến điểm bắt đầu', 40: 'Giữ ở điểm bắt đầu', 50: 'Di chuyển đến điểm bắt đầu điều động', 60: 'Giữ ở điểm bắt đầu của thao tác', 70: 'Thao tác (quỹ đạo)', 80: 'Giữ ở điểm cuối của tọa độ quỹ đạo', 90: 'Quay về tọa độ nhà', 100: 'Landing'}

    # Plot each segment with the corresponding color
    for mode in np.unique(modes):
        ix = np.where(modes == mode)
        ax.plot(x.iloc[ix], y.iloc[ix], z.iloc[ix], color=colors[mode], label=mode_names[mode])

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Quỹ đạo của Drone')

    # Create legend
    ax.legend(loc='best')

    # Save the figure before showing it
    plt.savefig('shapes/trajectory_plot.png')

    # Then show the plot
    plt.show()

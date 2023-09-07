import numpy as np
import matplotlib.pyplot as plt


params = {'axes.labelsize' : 9,
          'font.size' : 9,
          'legend.fontsize': 9,
          'xtick.labelsize' : 8,
          'ytick.labelsize' : 8,
        #   'figure.figsize': fig_size,
          'font.family' : 'sans-serif',
          'font.sans-serif' : 'Helvetica',
          'text.usetex' : True,
          'mathtext.fontset' : 'custom',
        #   'axes.prop_cycle': plt.cycler('color', ['#EE7733', '#0077BB', '#33BBEE', '#EE3377', '#CC3311', '#009988', '#BBBBBB']),
        #   'axes.prop_cycle': plt.cycler('color', ['#4477AA', '#EE6677', '#228833', '#CCBB44', '#66CCEE', '#AA3377', '#BBBBBB']),
          'axes.prop_cycle': plt.cycler('color', ['#0C5DA5', '#00B945', '#FF9500', '#FF2C00', '#845B97', '#474747', '#9e9e9e']), # bright colors
        #   'axes.prop_cycle': plt.cycler('color', ['#4165c0', '#e770a2', '#5ac3be', '#696969', '#f79a1e', '#ba7dcd']), # very muted colors

          # Grid lines
          'axes.grid' : True,
          'axes.axisbelow' : True,
          'grid.color' : 'k',
          'grid.alpha' : 0.5,
          'grid.linewidth' : 0.5,

          # Legend
          'legend.frameon' : True,
          'legend.framealpha' : 1.0,
          'legend.fancybox' : True,
          'legend.numpoints' : 1,
          } 
plt.rcParams.update(params)



def data_over_time(time_array, data_arrays, legend_labels, x_label, y_label, title, save_path):
    """
    Parameters:
        time_array (array-like): Time values.
        data_arrays (list of array-like): List of data arrays for different variations.
        legend_labels (list of str): Labels for the legend corresponding to data arrays.
        x_label (str): Label for the x-axis.
        y_label (str): Label for the y-axis.
        title (str): Title of the plot.
        save_path (str): Path to save the vectorized plot.

    Returns:
        None
    """
    plt.figure(figsize=(3.54,3.54), dpi=600)  # Adjust the figure size as needed

    for i, data in enumerate(data_arrays):
        plt.plot(time_array, data, label=legend_labels[i])

    plt.xlabel(x_label)  # LaTeX formatted label
    plt.ylabel(y_label)  # LaTeX formatted label
    plt.legend()
    # plt.title(title)
    # plt.legend(loc='upper right')  # Placing legend within the axis boundaries
    # plt.gca().set_aspect('equal', adjustable='box')  # Keeping the aspect ratio square

    # plt.legend()

    plt.grid(True)
    plt.tight_layout()

    # Save the plot in vectorized format (PDF)
    plt.savefig(save_path, format='pdf')

    plt.show()

def data_over_data(x_arrays, y_arrays, legend_labels, x_label, y_label, title, save_path):

    plt.figure(figsize=(3.54,3.54), dpi=600)  # Adjust the figure size as needed

    for i, (x_data, y_data) in enumerate(zip(x_arrays, y_arrays)):
        plt.plot(x_data, y_data, label=legend_labels[i])

    plt.xlabel(x_label) 
    plt.ylabel(y_label) 
    plt.legend()

    plt.grid(True)
    plt.tight_layout()

    plt.savefig(save_path, format='pdf')

    plt.show()


lane_changing = False
obstacle_avoidance = False
static_obstacle = True
braking = False

# Obstacle avoidance - Overtake scenario: preprocessing data
if lane_changing:
    pass
elif obstacle_avoidance:
    save_path = './src/my_truckie/results/plots/'
    state_ego_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_state_array.npy').T[0][:2]  # Extract first two arrays: x and y 
    time_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_time_array.npy')
    state_follower_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_1_state_array.npy').T[0][:2]  # Extract first two arrays
    
    index = 10
    time_array = time_array[index:]
    time_array = time_array - time_array[0]
    state_ego_array = state_ego_array[:,index:]  # discard first state
    state_follower_array = state_follower_array[:,index:] # discard first state
    combined_state_array = np.vstack((state_ego_array, state_follower_array))
    legend_labels = ['$x$ - Ego vehicle', '$y$ - Ego vehicle', '$x$ - Follower vehicle', '$y$ - Follower vehicle']
    data_over_time(time_array, combined_state_array , legend_labels, 'time $t$ (s)', 'position (m)', 'States Over Time', save_path + '05_obstacle_avoidance_position_time.pdf')

    legend_labels = ['$(x,y)$ - Ego vehicle', '$(x,y)$ - Follower vehicle']
    data_over_data(np.vstack((state_ego_array[0], state_follower_array[0])), np.vstack((state_ego_array[1], state_follower_array[1])), legend_labels, '$x$ (m)', '$y$ (m)', 'States Over Time', save_path + '05_obstacle_avoidance_position.pdf')

    vel_ego_vehicle = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_state_array.npy').T[0][2]
    vel_ego_vehicle = vel_ego_vehicle[index:]
    vel_follower_vehicle = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_1_state_array.npy').T[0][2]
    vel_follower_vehicle = vel_follower_vehicle[index:]
    vel_array = np.vstack((vel_ego_vehicle, vel_follower_vehicle))
    legend_labels = ['Ego vehicle', 'Follower vehicle']
    data_over_time(time_array, vel_array, legend_labels, 'time $t$ (s)', 'velocity $v_0$ (m s$^{-1}$)', 'Velocity Over Time', save_path + '05_obstacle_avoidance_velocity_time.pdf')

elif static_obstacle:
    number_followers = 2
    save_path = './src/my_truckie/results/plots/'
    
    state_ego_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_state_array.npy').T[0][:2]
    time_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_time_array.npy')
    index = 10
    time_array = time_array[index:]
    time_array = time_array - time_array[0]
    state_ego_array = state_ego_array[:, index:]
    
    combined_state_arrays = [state_ego_array]  # Initialize with ego vehicle state
    
    for i in range(1, number_followers + 1):
        state_follower_array = np.load(f'/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_{i}_state_array.npy').T[0][:2]
        state_follower_array = state_follower_array[:, index:]
        combined_state_arrays.append(state_follower_array)
    
    combined_state_array = np.vstack(combined_state_arrays)
    
    legend_labels = ['$x$ - Ego vehicle', '$y$ - Ego vehicle']
    for i in range(1, number_followers + 1):
        legend_labels.extend([f'$x$ - Follower {i} vehicle', f'$y$ - Follower {i} vehicle'])
    
    data_over_time(time_array, combined_state_array, legend_labels, 'time $t$ (s)', 'position (m)', 'States Over Time', save_path + '05_obstacle_static_position_time.pdf')


    legend_labels = ['$(x,y)$ - Ego vehicle']
    for i in range(1, number_followers + 1):
        legend_labels.extend([f'$(x,y)$ - Follower {i} vehicle'])
    x_array = np.vstack((combined_state_array[0], combined_state_array[2], combined_state_array[4]))
    y_array = np.vstack((combined_state_array[1], combined_state_array[3], combined_state_array[5]))
    data_over_data(x_array, y_array, legend_labels, '$x$ (m)', '$y$ (m)', 'States Over Time', save_path + '05_obstacle_static_position.pdf')

    vel_ego_vehicle = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_state_array.npy').T[0][2]
    vel_ego_vehicle = vel_ego_vehicle[index:]
    vel_arrays = [vel_ego_vehicle]
    
    for i in range(1, number_followers + 1):
        vel_follower_vehicle = np.load(f'/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_{i}_state_array.npy').T[0][2]
        vel_follower_vehicle = vel_follower_vehicle[index:]
        vel_arrays.append(vel_follower_vehicle)
    
    vel_array = np.vstack(vel_arrays)
    
    legend_labels = ['Ego vehicle']
    for i in range(1, number_followers + 1):
        legend_labels.append(f'Follower {i} vehicle')
    
    data_over_time(time_array, vel_array, legend_labels, 'time $t$ (s)', 'velocity $v_0$ (m s$^{-1}$)', 'Velocity Over Time', save_path + '05_obstacle_static_velocity_time.pdf')

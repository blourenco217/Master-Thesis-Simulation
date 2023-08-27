import numpy as np
import matplotlib.pyplot as plt

fig_width_pt = 455.24408             
inches_per_pt = 1.0/72.27            
golden_mean = (np.sqrt(5)-1.0)/2.0       
fig_width = fig_width_pt*inches_per_pt   
fig_height = fig_width*golden_mean        
fig_size =  [fig_width,fig_height]
params = {'axes.labelsize' : 10,
          'font.size' : 10,
          'legend.fontsize': 10,
          'xtick.labelsize' : 8,
          'ytick.labelsize' : 8,
          'figure.figsize': fig_size,
          'font.family' : 'sans-serif',
          'font.sans-serif' : 'DejaVu Sans',
          'text.usetex' : True,
          'mathtext.fontset' : 'dejavusans',
        #   'axes.prop_cycle': plt.cycler('color', ['#EE7733', '#0077BB', '#33BBEE', '#EE3377', '#CC3311', '#009988', '#BBBBBB']),
          'axes.prop_cycle': plt.cycler('color', ['#4477AA', '#EE6677', '#228833', '#CCBB44', '#66CCEE', '#AA3377', '#BBBBBB']),
        #   'axes.prop_cycle': plt.cycler('color', ['#0C5DA5', '#00B945', '#FF9500', '#FF2C00', '#845B97', '#474747', '#9e9e9e']),
        #   'axes.prop_cycle': plt.cycler('color', ['#4165c0', '#e770a2', '#5ac3be', '#696969', '#f79a1e', '#ba7dcd']),
          'legend.frameon' : True,
          'legend.framealpha' : 1.0,
          'legend.fancybox' : True,
          'legend.numpoints' : 1,
          } 
plt.rcParams.update(params)


def style_plot(time_array, data_arrays, legend_labels, x_label, y_label, title, save_path):
    """
    Generates IEEE-style plots for different variations over time and saves them in a vectorized manner.

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
    plt.figure(figsize=(8, 5))  # Adjust the figure size as needed

    for i, data in enumerate(data_arrays):
        plt.plot(time_array, data, label=legend_labels[i])

    plt.xlabel(x_label)  # LaTeX formatted label
    plt.ylabel(y_label)  # LaTeX formatted label
    # plt.title(title)
    plt.legend()

    plt.grid(True)
    plt.tight_layout()

    # Save the plot in vectorized format (PDF)
    plt.savefig(save_path, format='pdf')

    plt.show()


##### TESTING #####
# Example usage
# time_array = np.linspace(0, 10, 100)  # Example time values
# data_arrays = [
#     np.sin(time_array),            # Example data array 1
#     np.cos(time_array),            # Example data array 2
#     0.5 * np.sin(2 * time_array)   # Example data array 3
# ]
# legend_labels = ['$\sin$', '$\cos$', '$0.5\sin(2x)$']
# x_label = 'Time ($x$)'
# y_label = 'Amplitude ($y$)'
# title = 'IEEE-Style Plot with Variations'
save_path = './src/my_truckie/results/plots/'


state_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_1_state_array.npy').T[0]
input_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_1_input_array.npy').T[0]
time_array = np.load('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_1_time_array.npy')


# Plot the states over time
time_indices = np.arange(time_array.size)
legend_labels = ['State 1', 'State 2', 'State 3', 'State 4', 'State 5', 'State 6']
# style_plot(time_indices, state_array, legend_labels, 'time $t$', 'Value', 'States Over Time', save_path + 'states_plot.pdf')

# Plot the inputs over time
# legend_labels = ['Input 1', 'Input 2']
# style_plot(time_indices, input_array, legend_labels, 'Time Index', 'Value', 'Inputs Over Time', save_path +'inputs_plot.pdf')

style_plot(time_array, state_array, legend_labels, 'time $t$', 'Value', 'States Over Time', save_path + 'states_plot.pdf')
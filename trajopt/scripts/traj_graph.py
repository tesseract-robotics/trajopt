import numpy as np
import matplotlib.pyplot as plt
import argparse
import pandas as pd

# parse file names from command line
parser = argparse.ArgumentParser(description="Plot before and after trajectory")
parser.add_argument('a', type=str, help='First file')
parser.add_argument('b', type=str, help='Second file')
args = parser.parse_args()

# read the files
data_a = pd.read_csv(args.a, skip_blank_lines=False)
data_b = pd.read_csv(args.b, skip_blank_lines=False)

# get the dimensions of each of the files
shape_a = data_a.shape
rows_a = shape_a[0]
cols_a = shape_a[1]

shape_b = data_b.shape
rows_b = shape_b[0]
cols_b = shape_b[1]

# check that the files have the same number of columns
if (cols_a != cols_b):
    print('The files do not have the same number of columns. Exiting');

# check that the files' columns hold the same information
if ((data_a.columns != data_b.columns).any()):
        print('The column names for these files do not match. Exiting')
	exit(-1)
	
num_dofs = cols_a - 14
cols = cols_a

# check that the robots have the same number of poses set for the path
num_poses_a = 0
num_poses_b = 0

var = data_a.iloc[0,0]
while (not np.isnan(var)):
	num_poses_a += 1
	var = data_a.iloc[num_poses_a, 0]

var = data_b.iloc[0,0]
while (not np.isnan(var)):
	num_poses_b += 1
	var = data_b.iloc[num_poses_b, 0]

if (num_poses_a != num_poses_b):
	print('These two files do not describe the same path. Exiting')
	exit(-1)

# get the data for the final trajectories in each file
lower_a = rows_a - 1 - num_poses_a;
upper_a = rows_a - 1;
dof_vals_a = np.matrix(data_a.iloc[lower_a:upper_a, 0:num_dofs])
pose_vals_a = np.matrix(data_a.iloc[lower_a:upper_a, num_dofs:(num_dofs + 7)])
err_vals_a = np.matrix(data_a.iloc[lower_a:upper_a, (num_dofs + 7):(num_dofs + 14)])

lower_b = rows_b - 1 - num_poses_b;
upper_b = rows_b - 1;
dof_vals_b = np.matrix(data_b.iloc[lower_b:upper_b, 0:num_dofs])
pose_vals_b = np.matrix(data_b.iloc[lower_b:upper_b, num_dofs:(num_dofs + 7)])
err_vals_b = np.matrix(data_b.iloc[lower_b:upper_b, (num_dofs + 7):(num_dofs + 14)])

x = np.transpose(np.array(range(0,num_poses_a), ndmin=2))

# start at figure 1 and plot the DOFs
current_fig = 1
plots_left = num_dofs

# plot DOF values
for iter in range(0, (num_dofs >> 2) + 1):

    # open a new figure and determine the number of subplots
    plt.figure(current_fig)
    num_to_plot = min(plots_left, 4)

    # plots the subplots for the figure
    for i in range(0, num_to_plot):
        index = i + iter * 4
        cur_plot = plt.subplot(num_to_plot, 1, i + 1)
        plt.plot(x, dof_vals_a[:, index], x, dof_vals_b[:, index])

        dof_name = data_a.columns[i]
        plt.legend([dof_name + '_' + args.a[:-4], dof_name + '_' + args.b[:-4]])

        ylim = cur_plot.get_ylim()
        diff = (ylim[1] - ylim[0]) * 0.1
        cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])

    # set the plot to fullscreen
    #figManager = plt.get_current_fig_manager()
    #figManager.full_screen_toggle()

    # increment the figure number and note the plots we finished
    current_fig += 1
    plots_left -= 4
	
plots_left = 7

# plot pose values
for iter in range(0, 2):

    # open a new figure and determine the number of subplots
    plt.figure(current_fig)
    num_to_plot = min(plots_left, 4)

    # plots the subplots for the figure
    for i in range(0, num_to_plot):
        index = i + iter * 4
        cur_plot = plt.subplot(num_to_plot, 1, i + 1)
        plt.plot(x, pose_vals_a[:, index], x, pose_vals_b[:, index])

        pose_name = data_a.columns[index + num_dofs]
        plt.legend([pose_name + '_' + args.a[:-4], pose_name + '_' + args.b[:-4]])

        ylim = cur_plot.get_ylim()
        diff = (ylim[1] - ylim[0]) * 0.1
        cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])

    # set the plot to fullscreen
    #figManager = plt.get_current_fig_manager()
    #figManager.full_screen_toggle();

    # increment the figure number and note the plots we finished
    current_fig += 1
    plots_left -= 4

plots_left = 7

# plot err values
for iter in range(0, 2):

    # open a new figure and determine the number of subplots
    plt.figure(current_fig)
    num_to_plot = min(plots_left, 4)

    # plots the subplots for the figure
    for i in range(0, num_to_plot):
        index = i + iter * 4
        cur_plot = plt.subplot(num_to_plot, 1, i + 1)
        plt.plot(x, err_vals_a[:, index], x, err_vals_b[:, index])

        err_name = data_a.columns[index + 7 + num_dofs]
        plt.legend([err_name + '_' + args.a[:-4], err_name + '_' + args.b[:-4]])

        ylim = cur_plot.get_ylim()
        diff = (ylim[1] - ylim[0]) * 0.1
        cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])

    # set the plot to fullscreen
    #figManager = plt.get_current_fig_manager()
    #figManager.full_screen_toggle()

    # increment the figure number and note the plots we finished
    current_fig += 1
    plots_left -= 4

plt.show()

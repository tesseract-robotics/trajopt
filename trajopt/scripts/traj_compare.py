'''
Copyright 2018 Southwest Research Institute

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

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

#%% Perform checks before processing
# check that the files have the same number of columns
if (cols_a != cols_b):
    print('The files do not have the same number of columns. Exiting');

# check that the files' columns hold the same information
if ((data_a.columns != data_b.columns).any()):
    print('The column names for these files do not match. Exiting')
    exit(-1)
	
# Get the DOF of the robot - TODO: check if joints always start with joint_a 
joint_val_col = [col for col in data_a if col.startswith('joint_a')] 
num_dofs = len(joint_val_col)
cols = cols_a

# check that the files have the same number of time steps
num_steps_a = 0
num_steps_b = 0

var = data_a.iloc[0,0]
while (not np.isnan(var)):
	num_steps_a += 1
	var = data_a.iloc[num_steps_a, 0]

var = data_b.iloc[0,0]
while (not np.isnan(var)):
	num_steps_b += 1
	var = data_b.iloc[num_steps_b, 0]

if (num_steps_a != num_steps_b):
	print('These two files do not describe the same path. Exiting')
	exit(-1)

#%% 
# get the data for the final trajectories in each file
lower_a = rows_a - 1 - num_steps_a;
upper_a = rows_a - 1;
dof_vals_a = np.matrix(data_a.iloc[lower_a:upper_a, 0:num_dofs])
pose_vals_a = np.matrix(data_a.iloc[lower_a:upper_a, num_dofs:(num_dofs + 7)])

lower_b = rows_b - 1 - num_steps_b;
upper_b = rows_b - 1;
dof_vals_b = np.matrix(data_b.iloc[lower_b:upper_b, 0:num_dofs])
pose_vals_b = np.matrix(data_b.iloc[lower_b:upper_b, num_dofs:(num_dofs + 7)])

x = np.transpose(np.array(range(0,num_steps_a), ndmin=2))

#%% Plot joint values
# start at figure 1 and plot the DOFs
current_fig = 1
plots_left = num_dofs

# Plot in a few different windows for ease of viewing
for iter in range(0, (num_dofs >> 2) + 1):  # This is non-intuitive but works 

    # open a new figure and determine the number of subplots
    plt.figure(current_fig)
    num_to_plot = min(plots_left, 4)

    # plots the subplots for the figure
    for i in range(0, num_to_plot):
        index = i + iter * 4
        cur_plot = plt.subplot(num_to_plot, 1, i + 1)
        plt.plot(x, dof_vals_a[:, index], x, dof_vals_b[:, index])

        dof_name = data_a.columns[index]
        plt.legend([dof_name + '_' + args.a[:-4], dof_name + '_' + args.b[:-4]])

        # set the y axis limits such that the maximum and minimum values are visible
        ylim = cur_plot.get_ylim()
        diff = (ylim[1] - ylim[0]) * 0.1
        cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])

    # increment the figure number and note the plots we finished
    current_fig += 1
    plots_left -= 4
	
plots_left = 7
#%% Plot cartesian pose values
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

        # set the y axis limits such that the maximum and minimum values are visible
        ylim = cur_plot.get_ylim()
        diff = (ylim[1] - ylim[0]) * 0.1
        cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])

    # increment the figure number and note the plots we finished
    current_fig += 1
    plots_left -= 4

plt.show()

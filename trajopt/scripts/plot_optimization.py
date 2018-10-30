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


''' 
This script is primarily written to plot the costs over iteration. It can also plot the final trajectory.There are 3 switches that can be used to turn on and off pieces of the code.

It currently takes an average of the cost for each iteration and plots that across iteration for each cost/constraint. 

It has no knowledge of what was a cost vs a contraint. They are both just errors to be minimized

Usage:
python plot_optimization.py <filepath.csv>

'''

import numpy as np
import matplotlib.pyplot as plt
import argparse
import pandas as pd

# Switches
plotting_joints = False
plotting_cartesian = False
plotting_costs = True

# parse file names from command line
parser = argparse.ArgumentParser(description="Plot before and after trajectory")
parser.add_argument('a', type=str, help='First file')

args = parser.parse_args()

# read the files
df = pd.read_csv(args.a, skip_blank_lines=False)

# get the dimensions of each of the files
shape_a = df.shape
rows_a = shape_a[0]
cols_a = shape_a[1]


#%% Perform checks before processing
# check that the files have the same number of columns

# Get the DOF of the robot - TODO: check if joints always start with joint_a 
joint_val_col = [col for col in df if col.startswith('joint_a')] 
num_dofs = len(joint_val_col)
cols = cols_a

# Get the number of 0time steps
num_steps_a = 0

var = df.iloc[0,0]
while (not np.isnan(var)):
	num_steps_a += 1
	var = df.iloc[num_steps_a, 0]



#%% 
# get the data for the final trajectories in each file
lower_a = rows_a - 1 - num_steps_a;
upper_a = rows_a - 1;
dof_vals_a = np.asarray(df.iloc[lower_a:upper_a, 0:num_dofs])
pose_vals_a = np.asarray(df.iloc[lower_a:upper_a, num_dofs:(num_dofs + 7)])
cost_vals_a = df.iloc[lower_a:upper_a, (num_dofs + 7):]


x = np.transpose(np.array(range(0,num_steps_a), ndmin=2))

#%% Plot joint values of the final trajectory
# start at figure 1 and plot the DOFs
current_fig = 1
if plotting_joints == True:
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
            plt.plot(x, dof_vals_a[:, index])
    
            dof_name = df.columns[index]
            plt.legend([dof_name + '_' + args.a[:-4]])
    
            # set the y axis limits such that the maximum and minimum values are visible
            ylim = cur_plot.get_ylim()
            diff = (ylim[1] - ylim[0]) * 0.1
            cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])
    
        # increment the figure number and note the plots we finished
        current_fig += 1
        plots_left -= 4
    plt.show()
    	

#%% Plot cartesian pose values of the final trajectory
if plotting_cartesian == True:
    plots_left = 7
    for iter in range(0, 2):
    
        # open a new figure and determine the number of subplots
        plt.figure(current_fig)
        num_to_plot = min(plots_left, 4)
    
        # plots the subplots for the figure
        for i in range(0, num_to_plot):
            index = i + iter * 4
            cur_plot = plt.subplot(num_to_plot, 1, i + 1)
            plt.plot(x, pose_vals_a[:, index])
    
            pose_name = df.columns[index + num_dofs]
            plt.legend([pose_name + '_' + args.a[:-4]])
    
            # set the y axis limits such that the maximum and minimum values are visible
            ylim = cur_plot.get_ylim()
            diff = (ylim[1] - ylim[0]) * 0.1
            cur_plot.set_ylim([ylim[0] - diff, ylim[1] + diff])
    
        # increment the figure number and note the plots we finished
        current_fig += 1
        plots_left -= 4
    
    plt.show()


#%% Plot Costs and Constraints

if plotting_costs == True:
    # Drop the nan rows (the ones seperating the iterations)
    df2 = df.dropna(axis=0)
    
    df2.reset_index(drop=True, inplace=True)

    
    # Take average of each cost (across time) for each iteration
    # https://stackoverflow.com/questions/36810595
    df3 = df2.groupby(np.arange(len(df2))//num_steps_a).mean()
    
    # get columns to plot
    cost_cols = list(df3)[num_dofs+7:]
    
    df3.plot(y=cost_cols)
    
    plt.show()





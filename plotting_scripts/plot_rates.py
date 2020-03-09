#!/usr/bin/python3

import sys
import os
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd

if (len(sys.argv)==2):  # Folder name should be passed
    folder_name = sys.argv[1]
else:
    folder_name = '.'

print("Reading .csv files...")
states_df = pd.read_csv(f'{folder_name}/states.csv')
ref_df = pd.read_csv(f'{folder_name}/ref.csv')
cmd_df = pd.read_csv(f'{folder_name}/input.csv')


def plot_rate(axh, rate_name):
    t_states = states_df.loc[:, 'field.header.stamp']
    t_ref = ref_df.loc[:, 'field.header.stamp']
    t_cmd = cmd_df.loc[:, 'field.header.stamp']
    if rate_name == 'p':
        field_name = 'x'
    if rate_name == 'q':
        field_name = 'y'
    if rate_name == 'r':
        field_name = 'z'
    
    state_full_name = f'field.velocity.angular.{field_name}'
    state = states_df.loc[:, state_full_name]
    ref_full_name = f'field.vector.{field_name}'
    ref = ref_df.loc[:, ref_full_name]
    cmd_full_name = f'field.vector.{field_name}'
    cmd = cmd_df.loc[:, cmd_full_name]

    axh.set_xlabel('time (s)')
    axh.set_ylabel('angular rate (rad/s)')
    state_h = axh.plot(t_states, state, color='tab:red')
    ref_h = axh.plot(t_ref, ref, color='tab:blue')
    axh.tick_params(axis='y', labelcolor='tab:red')

    axh2 = axh.twinx()
    axh2.set_ylabel('control surface command')
    cmd_h = axh2.plot(t_cmd, cmd, color='k')

    axh.grid(True)
    proxy_state = mpl.patches.Patch(color='tab:red', label=rate_name)
    proxy_ref = mpl.patches.Patch(color='tab:blue', label=f'ref_{rate_name}')
    proxy_cmd = mpl.patches.Patch(color='k', label='ctrl_input')
    axh.legend(handles = [proxy_state, proxy_ref, proxy_cmd])
    
# Create a figure
fig = plt.figure(figsize=(16,9))
fig.suptitle('Angular rates')

print("Plotting...")
# Plot subplots
axh = fig.add_subplot(3,1,1)
plot_rate(axh, 'p')
axh = fig.add_subplot(3,1,2)
plot_rate(axh, 'q')
axh = fig.add_subplot(3,1,3)
plot_rate(axh, 'r')

plt.show()
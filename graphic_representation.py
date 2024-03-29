#--------------------------------------------------------------------------------------------

import matplotlib.pyplot as plt
import time

#--------------------------------------------------------------------------------------------

def initialize_plot(fig, ax, laser_odometry):
    fig.set_size_inches(7, 7, forward=True)
    
    true_xy_laser_plot, = ax[0].plot(laser_odometry[:, 0], laser_odometry[:, 1], color='b', label="True X-Y Laser Trajectory", linestyle='solid')
    # It is initialized with "laser_odometry" but it will be reset during training
    predicted_xy_laser_plot, = ax[0].plot(laser_odometry[:, 0], laser_odometry[:, 1], color='r', label="Predicted X-Y Laser Trajectory", linestyle='solid')
    
    true_theta_laser_plot = ax[1].plot(laser_odometry[:, 2], color='b', label="True Theta Laser Trajectory", linestyle='solid')
    # It is initialized with "laser_odometry" but it will be reset during training
    predicted_theta_laser_plot, = ax[1].plot(laser_odometry[:, 2], color='r', label="Predicted Theta Laser Trajectory", linestyle='solid')
    
    ax[0].set_xlabel("x")
    ax[0].set_ylabel("y", rotation=0)
    ax[1].set_xlabel("sample index")
    ax[1].set_ylabel("θ", rotation=0)
    ax[0].legend()
    ax[1].legend()
    ax[0].set_xlim(-5, 5)
    ax[0].set_ylim(-5, 5)
    
    return predicted_xy_laser_plot, predicted_theta_laser_plot

#--------------------------------------------------------------------------------------------

def double_plot(ax, predicted_odometry_trajectory, predicted_xy_plot, predcted_theta_plot):
    x = predicted_odometry_trajectory[:, 0]
    y = predicted_odometry_trajectory[:, 1]
    theta = predicted_odometry_trajectory[:, 2]
    
    predicted_xy_plot.set_xdata(x)
    predicted_xy_plot.set_ydata(y)
    
    predcted_theta_plot.set_ydata(theta)
    
    plt.pause(0.2)
    
    ax[0].relim()
    ax[0].autoscale_view()
    ax[1].relim()
    ax[1].autoscale_view()
#--------------------------------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import HexapodBrain

def inverse_kinematics(x, y, z):
    """Placeholder IK function. Replace with your actual IK solver."""
    # Simulating an IK solver where points within a circle of radius 10 are reachable
    if np.sqrt(x**2 + y**2) <= 10:
        return [x, y, z]  # Replace with actual joint angles
    return None

def generate_reachable_points(x_range, y_range, resolution, z=0):
    """Generate a grid of reachable points in the x-y plane using the IK model."""
    x_vals = np.arange(x_range[0], x_range[1], resolution)
    y_vals = np.arange(y_range[0], y_range[1], resolution)
    reachable_points = []
    
    for x in x_vals:
        for y in y_vals:
                if HexapodBrain.write_leg(HexapodBrain.FL, HexapodBrain.Point3D(x, y, z)) == 0:
                    reachable_points.append([x, y, z])
        
    for leg in HexapodBrain.legs:
        for x in x_vals:
            for y in y_vals:
                if HexapodBrain.write_leg(leg, HexapodBrain.Point3D(x, y, z)) == 0:
                    reachable_points.append([x, y, z])
    return np.array(reachable_points)

def plot_point_cloud(points):
    """Visualize a 3D point cloud using Matplotlib."""

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', alpha=0.5)
    
    ax.scatter(HexapodBrain.FL.offset.x, HexapodBrain.FL.offset.y, HexapodBrain.FL.offset.z, c='r', marker='x', s=100, label='Leg Offset')
    ax.scatter(HexapodBrain.CL.offset.x, HexapodBrain.CL.offset.y, HexapodBrain.CL.offset.z, c='r', marker='x', s=100, label='Leg Offset')
    ax.scatter(HexapodBrain.BL.offset.x, HexapodBrain.BL.offset.y, HexapodBrain.BL.offset.z, c='r', marker='x', s=100, label='Leg Offset')
    ax.scatter(HexapodBrain.FR.offset.x, HexapodBrain.FR.offset.y, HexapodBrain.FR.offset.z, c='r', marker='x', s=100, label='Leg Offset')
    ax.scatter(HexapodBrain.CR.offset.x, HexapodBrain.CR.offset.y, HexapodBrain.CR.offset.z, c='r', marker='x', s=100, label='Leg Offset')
    ax.scatter(HexapodBrain.BR.offset.x, HexapodBrain.BR.offset.y, HexapodBrain.BR.offset.z, c='r', marker='x', s=100, label='Leg Offset')

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title("Reachable Points Visualization")
    
    plt.show()

if __name__ == "__main__":
    x_range = (-40, 40)  # Define X boundaries
    y_range = (-40, 40)  # Define Y boundaries
    resolution = 0.5  # Grid resolution
    
    points = generate_reachable_points(x_range, y_range, resolution, z=0)
    plot_point_cloud(points)
    expected_y_values = []
    x_values = points[:, 0]
    y_values = points[:, 1]
    expected_y_values = (1 * x_values)
    mask = (np.abs(y_values - (expected_y_values)) < 1e-6)
    
    expected_y_values = (-1 * x_values)
    mask = (np.abs(y_values - (expected_y_values)) < 1e-6) | mask | (np.abs(y_values) < 1e-6)
    
    filteredPoints = points[mask]

    plot_point_cloud(filteredPoints)
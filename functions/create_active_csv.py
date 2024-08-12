import csv
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from functions.trajectories import *

def create_active_csv(shape_name, diameter, direction, maneuver_time, start_x, start_y, initial_altitude, climb_rate, move_speed, hold_time, step_time, output_file='active.csv'):
    if shape_name == "eight_shape":
        shape_code = 0
        shape_fnc = eight_shape_trajectory
        shape_args = ()
    elif shape_name == "circle":
        shape_code = 1
        shape_fnc = circle_trajectory
        shape_args = ()
    elif shape_name == "square":
        shape_code = 2
        shape_fnc = square_trajectory
        shape_args = ()
    elif shape_name == "helix":
        shape_code = 3
        shape_fnc = helix_trajectory
        end_altitude = 20
        turns = 3
        shape_args = (end_altitude, turns,)
    elif shape_name == "heart_shape":
        shape_code = 4
        shape_fnc = heart_shape_trajectory
        shape_args = ()
    elif shape_name == "infinity_shape":
        shape_code = 5
        shape_fnc = infinity_shape_trajectory
        shape_args = ()
    elif shape_name == "sprial_square":
        shape_code = 6
        shape_fcn = spiral_square_trajectory
        turns = 3
        shape_args = (turns,)
    elif shape_name == "star_shape":
        shape_code = 7
        shape_fcn = star_shape_trajectory
        points = 5
        shape_args = (points,)
    elif shape_name == "zigzag":
        shape_code = 8
        shape_fcn = zigzag_trajectory
        turns = 3
        shape_args = (turns,)
    elif shape_name == "sine_wave":
        shape_code = 9
        shape_fcn = sine_wave_trajectory
        turns = 3
        shape_args = (turns,)
    else:
        raise ValueError(f"Invalid shape name: {shape_name}")

    header = ["idx", "t", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az", "yaw", "mode", "ledr", "ledg", "ledb"]
    with open(output_file, mode='w', newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)

        # Tính toán quỹ đạo leo lên của drone
        climb_time = initial_altitude / climb_rate
        climb_steps = int(climb_time / step_time)

        for i in range(climb_steps):
            t = i * step_time
            x = 0
            y = 0
            z = (climb_rate * t) * -1
            vx = 0.0
            vy = 0.0
            vz = -climb_rate
            ax = 0
            ay = 0
            az = 0
            yaw = 0
            mode = 10
            row = [i, t, x, y, z, vx, vy, vz, ax, ay, az, yaw, mode, "nan", "nan", "nan"]
            writer.writerow(row)

        # Giữ ở độ cao sau khi đã leo lên
        hold_steps = int(hold_time / step_time)

        for i in range(hold_steps):
            t = climb_time + i * step_time
            x = 0
            y = 0
            z = -1 * initial_altitude
            vx = 0.0
            vy = 0.0
            vz = 0.0
            ax = 0
            ay = 0
            az = 0
            yaw = 0
            mode = 20
            row = [climb_steps + i, t, x, y, z, vx. vy, vz, 0, 0, 0, yaw, mode, "nan", "nan", "nan"]
            writer.writerow(row)

        # Di chuyển từ vị trí hiện tại đến vị trí bắt đầu
        move_start_distance = math.sqrt(start_x**2 + start_y**2)
        move_start_time = move_start_distance / move_speed
        move_start_steps = int(move_start_time / step_time)

        for i in range(move_start_steps):
            t = climb_time + hold_time + i * step_time
            ratio = i / move_start_steps
            x = start_x * ratio
            y = start_y * ratio
            z = -1 * initial_altitude
            vx = 0.0
            vy = 0.0
            vz = 0.0
            ax = 0
            ay = 0
            az = 0
            yaw = 0
            mode = 40
            row = [climb_steps + hold_steps + move_start_steps + i, t, x, y, z, vx, vy, vz, 0, 0, 0, yaw, mode, "nan","nan", "nan"]

        hold_steps = int(hold_time / step_time)

        for i in range(hold_steps):
            t = climb_time + move_start_time + i * step_time
            x = start_x
            y = start_y
            z = -1 * initial_altitude
            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw = 0
            row = [climb_steps + move_start_steps + i, t, x, y, z, vx, vy, vz, "nan", "nan", "nan", yaw, "nan", "nan",
                   "nan"]
            writer.writerow(row)

            # Check if start position is different from first setpoint of maneuver
        if 0 != shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[0] or 0 != \
                shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[1]:
            print("different Start and Manuever")
            maneuver_start_x = \
            shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[0];
            maneuver_start_y = \
            shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[1];

            print(f"Origin Start: {start_x} , {start_y}")
            print(f"Manuever Start: {maneuver_start_x} , {maneuver_start_y}")

            # Calculate distance and time required to move to first setpoint of maneuver
            move_distance = math.sqrt((maneuver_start_x) ** 2 + (maneuver_start_y) ** 2)
            move_time = move_distance / 2.0
            move_steps = int(move_time / step_time)

            # Move drone to first setpoint of maneuver at 2 m/s
            for i in range(move_steps):
                t = climb_time + move_start_time + hold_time + i * step_time
                ratio = i / move_steps
                x = start_x + (maneuver_start_x) * ratio
                y = start_y + (maneuver_start_y) * ratio
                z = -1 * initial_altitude
                vx = move_speed * (maneuver_start_x) / move_distance
                vy = move_speed * (maneuver_start_y) / move_distance
                vz = 0.0
                yaw = 0
                row = [climb_steps + move_start_steps + hold_steps + i, t, x, y, z, vx, vy, vz, "nan", "nan", "nan",
                       yaw, "nan", "nan", "nan"]
                writer.writerow(row)

            # Hold drone at first setpoint for 2 seconds
            for i in range(hold_steps):
                t = climb_time + move_start_time + move_time + hold_time + i * step_time
                x = start_x + \
                    shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[0]
                y = start_y + \
                    shape_fcn(0, maneuver_time, diameter, direction, initial_altitude, step_time, *shape_args)[1]
                z = -1 * initial_altitude
                vx = 0.0
                vy = 0.0
                vz = 0.0
                yaw = 0
                row = [climb_steps + move_steps + move_start_steps + hold_steps + i, t, x, y, z, vx, vy, vz, "nan",
                       "nan", "nan", yaw, "nan", "nan", "nan"]
                writer.writerow(row)

            # Calculate the start time after maneuver start
            start_time = climb_time + move_start_time + move_time + hold_time + hold_time
        else:
            # Calculate the start time after maneuver start
            start_time = climb_time + move_start_time + hold_time
            move_distance = 0
            move_steps = 0
            move_time = 0

        # Calculate the total duration of the trajectory after maneuver start
        total_duration = maneuver_time + start_time
        total_steps = int(total_duration / step_time)
        maneuver_steps = int(maneuver_time / step_time)

        # Fly the shape trajectory
        for step in range(maneuver_steps):
            # Call the appropriate shape function based on the shape code

            x, y, z, vx, vy, vz = shape_fcn(step, maneuver_time, diameter, direction, initial_altitude, step_time,
                                            *shape_args)
            x += start_x
            y += start_y

            yaw = 0
            missionTime = start_time + step * step_time
            row = [climb_steps + move_start_steps + hold_steps + maneuver_steps + move_steps + hold_steps + step,
                   missionTime, x, y, z, vx, vy, vz, "nan", "nan", "nan", yaw, "nan", "nan", "nan"]
            writer.writerow(row)

        print(f"Created {output_file} with the {shape_name}.")

    # Example usage
    shape_name = "square"
    diameter = 30.0
    direction = 1
    maneuver_time = 60.0
    start_x = 0
    start_y = 0
    initial_altitude = 10
    climb_rate = 1.0
    move_speed = 2.0  # m/s
    hold_time = 2.0  # s
    step_time = 0.1  # s
    output_file = "shapes/active.csv"

    create_active_csv(
        shape_name=shape_name,
        diameter=diameter,
        direction=direction,
        maneuver_time=maneuver_time,
        start_x=start_x,
        start_y=start_y,
        initial_altitude=initial_altitude,
        climb_rate=climb_rate,
        move_speed=move_speed,
        hold_time=hold_time,
        step_time=step_time,
        output_file=output_file,
    )

    # Load the data from the active.csv file
    data = pd.read_csv(output_file)

    # Extract the position coordinates
    x = data['px']
    y = data['py']
    z = -1 * data['pz']

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(x, y, z)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Drone Trajectory')

    # Show the plot
    plt.show()
    plt.savefig('shapes/trajectory_plot.png')

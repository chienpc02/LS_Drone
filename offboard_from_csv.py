"""
Script: offboard_from_csv.py

Description:
-------------
The offboard_from_csv.py script is responsible for controlling a drone in an offboard mode using a trajectory defined in a CSV file. The script establishes a connection with the drone, reads the trajectory data from the CSV file, and commands the drone to follow the trajectory. At the end of the trajectory, the drone returns to its home position and lands.

Prerequisites:
--------------
- The script requires the MAVSDK library to be installed. Refer to the MAVSDK documentation for installation instructions.
- Ensure that the drone is properly set up for offboard control and has a valid global position estimate.

Usage:
------
1. Connect the drone to the system running this script.
2. Ensure that the drone has a valid global position estimate.
3. Run the script using the command: python offboard_from_csv.py

Inputs:
-------
The offboard_from_csv.py script expects the following inputs:
- CSV file: The trajectory data in CSV format should be located at "shapes/active.csv" relative to the script's location.

Outputs:
--------
The script controls the drone to follow the trajectory defined in the CSV file. The drone performs the desired trajectory and returns to its home position to land.

Example Usage:
--------------
1. Ensure that the drone is connected to the system and has a valid global position estimate.
2. Place the trajectory data in CSV format in the "shapes/active.csv" file.
3. Run the script using the command: python offboard_from_csv.py

Additional Information:
-----------------------
- If you are unfamiliar with using the MAVSDK library for controlling drones, refer to the video tutorial provided in the GitHub repository (alireza787b) for a step-by-step guide.

Note:
-----
- Make sure that the drone is properly configured for offboard control before running this script.
- Adjust the time resolution (0.1 seconds) in the script if needed for your application.
- Uncomment the lines to change the flight mode or include additional functionality as required.
"""




import asyncio
import csv
import os

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, AccelerationNed , OffboardError
from mavsdk.telemetry import LandedState
import subprocess
import signal


async def run():
    
    # Define a dictionary to map mode codes to their descriptions
    mode_descriptions = {
    0: "On the ground",
    10: "Trạng thái leo ban đầu",
    20: "Giữ ban đầu sau khi leo lên",
    30: "Di chuyển đến điểm bắt đầu",
    40: "Giữ ở điểm bắt đầu",
    50: "Di chuyển đến điểm bắt đầu điều động",
    60: "Giữ ở điểm bắt đầu của thao tác",
    70: "Thao tác (quỹ đạo)",
    80: "Giữ ở điểm cuối của tọa độ quỹ đạo",
    90: "Quay về tọa độ nhà",
    100: "Landing"
    }
    grpc_port = 50040
    drone = System(mavsdk_server_address="127.0.0.1", port=grpc_port)
    await drone.connect(system_address="udp://:14540")

    print("Đang chờ drone kết nối...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Đã kết nối với drone!")
            break

    print("Đang chờ drone ước lượng vị trí toàn cầu...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Ước lượng vị trí toàn cầu OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Cài đặt điểm setpoint ban đầu")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Bắt đầu chế độ điều khiển")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Khởi động chế độ offboard thất bại với mã lỗi: {error._result.result}")
        print("-- Disarming") # Tắt động cơ của drone
        await drone.action.disarm()
        return

    waypoints = []

    # Read data from the CSV file
    with open("shapes/active.csv", newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            t = float(row["t"])
            px = float(row["px"])
            py = float(row["py"])
            pz = float(row["pz"])
            vx = float(row["vx"])
            vy = float(row["vy"])
            vz = float(row["vz"])
            ax = float(row["ax"])
            ay = float(row["ay"])
            az = float(row["az"])
            yaw = float(row["yaw"])
            mode_code = int(row["mode"])  # Assuming the mode code is in a column named "mode"
        
        
            waypoints.append((t, px, py, pz, vx, vy, vz,ax,ay,az,mode_code))

    print("-- Quỹ đạo biển diễn")
    total_duration = waypoints[-1][0]  # Thời gian tổng cộng là thời gian của waypoint cuối cùng
    t = 0  # Time variable
    last_mode = 0
    while t <= total_duration:
        # Xác định waypoint hiện tại dựa trên thời gian
        current_waypoint = None
        for waypoint in waypoints:
            if t <= waypoint[0]:
                current_waypoint = waypoint
                break

        if current_waypoint is None:
            # Đã đến điểm cuối của quỹ đạo
            break

        position = current_waypoint[1:4]  # Trích xuất vị trí (px, py, pz)
        velocity = current_waypoint[4:7]  # Trích xuất vận tốc (vx, vy, vz)
        acceleration = current_waypoint[7:10]  # Trích xuất gia tốc (ax, ay, az)
        mode_code = current_waypoint[-1]
        if last_mode != mode_code:
                print(f" Mode number: {mode_code}, Description: {mode_descriptions[mode_code]}")
                last_mode = mode_code
                
        await drone.offboard.set_position_velocity_acceleration_ned(
            PositionNedYaw(*position, yaw),
            VelocityNedYaw(*velocity, yaw),
            AccelerationNed(*acceleration)
        )

        await asyncio.sleep(0.1)  # Độ phân giải thời gian là 0.1 giây
        t += 0.1

    print("-- Hoàn thành hình dạng")

    print("-- Landing") # Hạ cánh
    await drone.action.land()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    print("-- Dừng chế độ offboard")
    try:
        await drone.offboard.stop()
    except Exception as error:
        print(f"Khởi động chế độ offboard thất bại với mã lỗi: {error}")

    print("-- Disarming")
    await drone.action.disarm()


async def main():

    udp_port = 14540 

    # Start mavsdk_server 
    grpc_port = 50040 
    mavsdk_server = subprocess.Popen(["./mavsdk_server", "-p", str(grpc_port), f"udp://:{udp_port}"])
    # await asyncio.sleep(1)

    tasks = []
    tasks.append(asyncio.create_task(run()))

    await asyncio.gather(*tasks)

    # Kill  mavsdk_server 
    os.kill(mavsdk_server.pid, signal.SIGTERM)

    print("Tất cả các chương trình đã hoàn thành. Thoát khỏi chương trình.")

if __name__ == "__main__":
    asyncio.run(main())
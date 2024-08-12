import asyncio
import csv

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from mavsdk.telemetry import LandedState


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
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
            yaw = float(row["yaw"])
            waypoints.append((t, px, py, pz, vx, vy, vz))

    print("-- Performing trajectory")
    total_duration = waypoints[-1][0]  # Total duration is the time of the last waypoint
    t = 0  # Time variable

    while t <= total_duration:
        # Find the current waypoint based on time
        current_waypoint = None
        for waypoint in waypoints:
            if t <= waypoint[0]:
                current_waypoint = waypoint
                break

        if current_waypoint is None:
            # Reached the end of the trajectory
            break

        position = current_waypoint[1:4]  # Extract position (px, py, pz)
        velocity = current_waypoint[4:7]  # Extract velocity (vx, vy, vz)

        await drone.offboard.set_position_velocity_ned(
            PositionNedYaw(*position, yaw),
            VelocityNedYaw(*velocity, yaw),
        )

        await asyncio.sleep(0.1)  # Time resolution of 0.1 seconds
        t += 0.1

    print("-- Shape completed")

    print("-- Returning to home")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
    await asyncio.sleep(5)  # Adjust as needed for a stable hover

    print("-- Landing")
    await drone.action.land()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except Exception as error:
        print(f"Stopping offboard mode failed with error: {error}")

    print("-- Disarming")
    await drone.action.disarm()

    # print("-- Changing flight mode")
    # await drone.action.set_flight_mode("MANUAL")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
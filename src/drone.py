import csv
from mavsdk.system import System


class Drone:
    def __init__(self, config):
        self.hw_id = config.hw_id
        self.pos_id = config.pos_id
        self.x = config.x
        self.y = config.y
        self.ip = config.ip
        self.mavlink_port = config.mavlink_port
        self.debug_port = config.debug_port
        self.gcs_ip = config.gcs_ip
        self.grpc_port = 50040 + int(self.hw_id)
        self.drone = System(mavsdk_server_address="127.0.0.1", port=self.grpc_port)
        self.waypoints = []
        self.mode_descriptions = {
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
        self.home_position = None
        self.trajectory_offset = (0, 0, 0)
        self.altitude_offset = 0
        self.time_offset = 0

    async def connect(self):
        await self.drone.connect(system_address=f"udp://{self.mavlink_port}")
        print(f"Drone đang kết nối với UDP: {self.mavlink_port}")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone id {self.hw_id} đã kết nối Port: {self.mavlink_port} và grpc Port: {self.grpc_port}")
                break
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print(f"Ước tính vị trí toàn cầu ok {self.hw_id}")
                async for global_position in self.drone.telemetry.position():
                    self.home_position = global_position
                    print(f"Home Position of {self.hw_id} set to: {self.home_position}")
                    break
                break

    async def read_trajectory(self, filename):
        # Read data from the CSV file
        with open(filename, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                t = float(row["t"])
                px = float(row["px"]) + self.trajectory_offset[0]
                py = float(row["py"]) + self.trajectory_offset[1]
                pz = float(row["pz"]) + self.trajectory_offset[2] -  self.altitude_offset
                vx = float(row["vx"])
                vy = float(row["vy"])
                vz = float(row["vz"])
                ax = float(row["ax"])
                ay = float(row["ay"])
                az = float(row["az"])
                yaw = float(row["yaw"])
                mode_code = int(row["mode"])  # Assuming the mode code is in a column named "mode"

                self.waypoints.append((t, px, py, pz, vx, vy, vz,ax,ay,az,mode_code))
    async def perform_trajectory(self):
        print(f"Drone {self.hw_id} bắt đầu vẽ hình dạng.")
        for waypoint in self.waypoints:
            t, px, py, pz, vx, vy, vz, ax, ay, az, yaw, mode_code = waypoint

            if mode_code == 70:  # If the mode code is for maneuvering (trajectory)
                print(f"Drone {self.hw_id} maneuvering.")
                # Send the waypoint to the drone
                await self.drone.action.goto_location(px, py, pz, yaw)


        print(f"Drone {self.hw_id} đã hoàn thành hình dạng.")
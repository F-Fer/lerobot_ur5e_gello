from lerobot_teleoperator_gello import GelloConfig, Gello
from lerobot_robot_ur5e import UR5E, UR5EConfig

robot_config = UR5EConfig(
    ip="192.168.1.12",
)

teleop_config = GelloConfig(
    port="/dev/ttyUSB0",
    id="gello_teleop",
)

robot = UR5E(robot_config)
teleop_device = Gello(teleop_config)
robot.connect()
teleop_device.connect()

while True:
    action = teleop_device.get_action()
    robot.send_action(action)
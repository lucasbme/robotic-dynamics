from roboticstoolbox import ERobot
from pathlib import Path

urdf_path = Path(__file__).parent / "urdf" / "manip_desc.urdf"

robot = ERobot.URDF(urdf_path)
print(robot)

robot.plot(robot.q)
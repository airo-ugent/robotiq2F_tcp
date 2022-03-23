import time

from robotiq2f_tcp import Robotiq2F85TCP

robotiq = Robotiq2F85TCP("10.42.0.162")

# activate and wait for 4sec for calibration procedure (gripper opens and closes)
print(robotiq.activate_gripper())
time.sleep(4)

# slow down to observe better
robotiq.speed = 10
robotiq.force = 10

# set a target and observe how it takes some time before this is updated in the registers of the gripper.
robotiq.target_position = 130
print(f" target pos = {robotiq.target_position}")
time.sleep(0.2)
print(f" target pos = {robotiq.target_position}")
assert robotiq.target_position == 130

# observe current positions. If you want to wait explicitly for the gripper to reach the target pose,
# use the move_pos function.
while not robotiq._is_target_value_set(robotiq.target_position, robotiq.position):
    time.sleep(0.2)
    print(f"pos = {robotiq.position}")

time.sleep(2)
# set new target, gripper will now open fast
# this is a synchronous command
robotiq.move_to_position(255, 255, 10)

print(robotiq.is_gripper_moving())
robotiq.open()

# deactivate gripper
robotiq.deactivate_gripper()

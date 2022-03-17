# Library importation
import socket
import time


class Robotiq2F85TCP:
    """Class for interacting with the "API" of the Robotiq 2F85 URCap.
    This API is available at TCP port 63352 of the UR controller and is listed in
    https://dof.robotiq.com/discussion/2420/control-robotiq-gripper-mounted-on-ur-robot-via-socket-communication-python.

    The control sequence is gripper motor <----- gripper registers<--ModbusSerial(rs485)-- UR controller <--TCP-- remote control

    This wrapper is not really time-efficient but we don't need high control frequencies.
    Better solution would be to use ModbusTCP to address the gripper's registers directly and read/write all data at once.
    Even better(?): directly connect over USB to the gripper with a convertor.

    """

    # TODO: make speed, force, position interpretable (m/s, N, mm)
    # TODO (optional): figure out how to avoid code duplication of the polling loop in each synchronous write operation.
    # TODO: could tcp socket be reused instead of opening a new one every time?

    def __init__(self, host_ip: str, port: int = 63352) -> None:
        self.host_ip = host_ip
        self.port = port

        self._check_connection()

    def _check_connection(self):
        """validate communication with gripper is possible.

        Raises:
            ConnectionError
        """

        if not self._communicate("GET STA").startswith("STA"):
            raise ConnectionError("Could not connect to gripper")

    def _communicate(self, command: str) -> str:
        """Helper function to communicate with gripper over a tcp socket.

        Args:
            command (str): The GET/SET command string.

        Returns:
            str:
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                # open the socket
                s.connect((self.host_ip, self.port))
                # s.sendall(b'SET GTO 1\n')
                s.sendall(("" + str.strip(command) + "\n").encode())
                # s.sendall(b'SET POS 100 \n')

                data = s.recv(2 ** 10)
                return data.decode()[:-1]
            except Exception as e:
                raise (e)

    def activate_gripper(self):
        """Activates the gripper, sets target position to "Open" and sets GTO flag."""
        self._communicate("SET ACT 1")
        while not self._communicate("GET STA") == "STA 3":
            time.sleep(0.1)

        # initialize
        self._communicate("SET GTO 1")
        self.target_position = 0
        self.speed = 255
        self.force = 0

    def deactivate_gripper(self):
        self._communicate("SET ACT 0")
        while not self._communicate("GET STA") == "STA 0":
            time.sleep(0.1)

    def move_to_position(self, position: int, speed: int, grasp_force: int):
        """Synchronously move the robot to the desired position with the specified speed and grasp force.

        Args:
            position (int): 0 (open) - 255 (closed)
            speed (int): 0 (slow) - 255 (fast)
            grasp_force (int): 0 (gentle) - 255 (firm)
        """
        self.speed = speed
        self.force = grasp_force
        self.target_position = position
        while not self._is_target_value_reached(position, self.position):
            time.sleep(0.1)

    def is_object_detected(self) -> bool:
        return int(self._communicate("GET OBJ").split(" ")[1]) == 1

    @property
    def position(self) -> int:
        return int(self._communicate("GET POS").split(" ")[1])

    @position.setter
    def position(self, value: int):
        raise ValueError("cannot set position directly, use target_position")

    @property
    def target_position(self) -> int:
        return int(self._communicate("GET PRE").split(" ")[1])

    @target_position.setter
    def target_position(self, position: int):
        """Set the target position of the gripper (async)

        Args:
            position (int): target in range 0 (open) - 255 (closed)
        """

        self._communicate(f"SET  POS {position}")

    @property
    def speed(self):
        return int(self._communicate(f"GET SPE").split(" ")[1])

    @speed.setter
    def speed(self, speed: int):
        speed = self._saturate_command(speed)
        self._communicate(f"SET SPE {speed}")
        while not self._is_target_value_reached(speed, self.speed):
            time.sleep(0.1)

    @property
    def grasp_force(self):
        return int(self._communicate("GET FOR").split(" ")[1])

    @grasp_force.setter
    def grasp_force(self, force: int):
        force = self._saturate_command(force)
        self._communicate(f"SET FOR {force}")
        while not self._is_target_value_reached(force, self.force):
            time.sleep(0.1)

    @staticmethod
    def _saturate_command(cmd: int) -> int:
        return min(255, (max(cmd, 0)))

    @staticmethod
    def _is_target_value_reached(target: int, value: int) -> bool:
        """helper to compare target value to current value and overcome
        the observation that the gripper does not always halt at the exact requested value.
        """
        return abs(target - value) < 5


if __name__ == "__main__":
    """Code example / User test"""

    robotiq = Robotiq2F85TCP("10.42.0.162")

    # activate and wait for 4sec for calibration procedure (gripper opens and closes)
    print(robotiq.activate_gripper())
    time.sleep(4)

    # slow down to observe better
    robotiq.speed = 10
    robotiq.force = 10

    # set a target and observe how it takes some time before this is updated in the registers of the gripper.
    robotiq.target_position = 150
    print(f" target pos = {robotiq.target_position}")
    time.sleep(0.2)
    print(f" target pos = {robotiq.target_position}")
    assert robotiq.target_position == 150

    # observe current positions. If you want to wait explicitly for the gripper to reach the target pose,
    # use the move_pos function.
    while not robotiq._is_target_value_reached(robotiq.target_position, robotiq.position):
        time.sleep(0.2)
        print(f"pos = {robotiq.position}")

    time.sleep(2)
    # set new target, gripper will now open fast
    # this is a synchronous command
    robotiq.move_to_position(0, 255, 10)

    # use internal "object detection"
    print(robotiq.is_object_detected())
    # deactivate gripper
    robotiq.deactivate_gripper()

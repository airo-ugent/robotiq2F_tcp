# Library importation
import asyncio
import socket
import time


class Robotiq2F85TCP:
    """Class for interacting with the "API" of the Robotiq 2F85 URCap.
    This API is available at TCP port 63352 of the UR controller and wraps the Modbus registers of the gripper, as described in
    https://dof.robotiq.com/discussion/2420/control-robotiq-gripper-mounted-on-ur-robot-via-socket-communication-python.

    The control sequence is gripper motor <---- gripper registers<--ModbusSerial(rs485)-- UR controller <--TCP-- remote control

    This wrapper is not extremely time-efficient but we don't need high control frequencies.
    Better solution could be to use ModbusTCP to address the gripper's registers directly and read/write all data at once.

    Another very useful source on the relation between the register values and physical state of the gripper is:
    https://blog.robotiq.com/set-your-robotiq-gripper-by-simply-reading-this-blog-post
    """

    # TODO: could tcp socket be reused instead of opening a new one every time?
    # TODO: avoid duplication in wait for command set of the target pos/ speed /force

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
            time.sleep(0.01)

        # initialize gripper
        self._communicate("SET GTO 1")  # enable Gripper
        self.target_position = 0
        self.speed = 255
        self.force = 0

    def deactivate_gripper(self):
        self._communicate("SET ACT 0")
        while not self._communicate("GET STA") == "STA 0":
            time.sleep(0.01)

    def move_to_position(self, position: int, speed: int = None, grasp_force: int = None):
        """Synchronously move the robot to the desired position with the specified speed and grasp force.

        Args:
            position (int): 0 (open) - 230 (straight closed) - 255 ( encompassed closed if applicable)
            speed (int): 0 (slow) - 255 (fast)
            grasp_force (int): 0 (gentle) - 255 (firm)
        """
        if speed:
            self.speed = speed
        if grasp_force:
            self.force = grasp_force
        self.target_position = position
        while self.is_gripper_moving():
            time.sleep(0.05)

    async def amove_to_position(self, position: int, speed: int = None, grasp_force: int = None):
        """Asynchronously move the robot to the desired position with the specified speed and grasp force.

        Args:
            position (int): 0 (open) - 230 (straight closed) - 255 ( encompassed closed if applicable)
            speed (int): 0 (slow) - 255 (fast)
            grasp_force (int): 0 (gentle) - 255 (firm)
        """
        if speed:
            self.speed = speed
        if grasp_force:
            self.force = grasp_force
        self.target_position = position
        while self.is_gripper_moving():
            await asyncio.sleep(0.05)

    def close(self):
        self.move_to_position(255, None, None)

    def open(self):
        self.move_to_position(0, None, None)

    def is_object_detected(self) -> bool:
        return int(self._communicate("GET OBJ").split(" ")[1]) == 2

    def is_gripper_moving(self) -> bool:
        # Moving == 0 => detected OR position reached
        return int(self._communicate("GET OBJ").split(" ")[1]) == 0

    @property
    def position(self) -> int:
        """Position of the gripper fingers (symmetric).
        Takes values between 0 and 255.
        3 is actually fully open, 230 is fully closed in operating mode (straight fingers) and 255 is fully closed in encompassed mode.
        The range 3 - 230 maps approximately to 0mm - 85mm with a quasi linear relation of 0.4mm / unit
        """
        return int(self._communicate("GET POS").split(" ")[1])

    @position.setter
    def position(self, value: int):
        raise ValueError("cannot set position directly, use target_position")

    @property
    def target_position(self) -> int:
        """Target position value of the gripper. See Position for interpretation of this value."""
        return int(self._communicate("GET PRE").split(" ")[1])

    @target_position.setter
    def target_position(self, position: int):
        """Set the target position of the gripper (async)

        Args:
            position (int): target in range 0 (open) - 255 (closed)
        """
        self._communicate(f"SET  POS {position}")
        target_position = self._saturate_command(position)
        while not self._is_target_value_set(target_position, self.target_position):
            time.sleep(0.01)

    @property
    def speed(self):
        """Speed of the gripper when opening / closing.
        Takes values between 0 and 255, which maps to 20mm/s - 150mm/s.
        """
        return int(self._communicate(f"GET SPE").split(" ")[1])

    @speed.setter
    def speed(self, speed: int):
        speed = self._saturate_command(speed)
        self._communicate(f"SET SPE {speed}")
        while not self._is_target_value_set(speed, self.speed):
            time.sleep(0.01)

    @property
    def grasp_force(self):
        """Maximal force applied by gripper when closing.
         Takes values between 0 and 255, which maps to 5N - 220N for the 2F85.
        If this force is reached, the gripper will signal "Object detected".
        """
        return int(self._communicate("GET FOR").split(" ")[1])

    @grasp_force.setter
    def grasp_force(self, force: int):
        force = self._saturate_command(force)
        self._communicate(f"SET FOR {force}")
        while not self._is_target_value_set(force, self.force):
            time.sleep(0.01)

    @staticmethod
    def _saturate_command(cmd: int) -> int:
        return min(255, (max(cmd, 0)))

    @staticmethod
    def _is_target_value_set(target: int, value: int) -> bool:
        """helper to compare target value to current value and make the force / speed request synchronous"""
        return abs(target - value) < 5

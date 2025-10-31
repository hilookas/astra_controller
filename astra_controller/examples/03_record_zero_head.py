from astra_controller.head_controller import HeadController
import time

arm_controller_right = HeadController("/dev/tty_head")

arm_controller_right.set_torque(128)

time.sleep(10)
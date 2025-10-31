from astra_controller.arm_controller import ArmController
import time

arm_controller_right = ArmController("/dev/tty_puppet_right", do_init=True)

time.sleep(10)

arm_controller_left = ArmController("/dev/tty_puppet_left", do_init=True)

time.sleep(10)
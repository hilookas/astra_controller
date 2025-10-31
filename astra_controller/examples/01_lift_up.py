from astra_controller.lift_controller import LiftController

lift_controller_right = LiftController("/dev/tty_puppet_lift_right")
lift_controller_left = LiftController("/dev/tty_puppet_lift_left")

lift_controller_right.set_pos(0.7)
lift_controller_left.set_pos(0.7)
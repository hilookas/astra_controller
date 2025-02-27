import odrive_can.odrive
import threading
import math
import time
import asyncio

import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class BaseController:
    LEFT_NODE_ID = 0
    RIGHT_NODE_ID = 1

    LEFT_INSTALL_DIR = -1
    RIGHT_INSTALL_DIR = 1

    WHEEL_BASE = 0.44 # in [m]
    WHEEL_D = 0.139 # in [m]

    def __init__(self, name):
        logger.info(f"Using device {name}")

        self.state_cb = None
        self.debug_cb = None

        self.device_name = name

        self.lock = threading.Lock()

        self.last_left_vel = None
        self.last_right_vel = None
        self.last_time = None

        self.write_lock = threading.Lock()

        self.curpos = {
            self.LEFT_NODE_ID: 0,
            self.RIGHT_NODE_ID: 0
        }

        self.setpos = {
            self.LEFT_NODE_ID: 0,
            self.RIGHT_NODE_ID: 0
        }

        self.quit = threading.Event()

        # asyncio.run(self.odrive_can_thread())
        self.t = threading.Thread(target=asyncio.run, args=(self.odrive_can_thread(self.LEFT_NODE_ID),), daemon=True)
        self.t.start()

        self.t2 = threading.Thread(target=asyncio.run, args=(self.odrive_can_thread(self.RIGHT_NODE_ID),), daemon=True)
        self.t2.start()

        while True:
            with self.lock:
                if self.last_time is not None: # wait for init done (any axis)
                    break
            time.sleep(0.1)

    def feedback_cb(self, msg: odrive_can.odrive.CanMsg, caller: odrive_can.odrive.ODriveCAN):
        assert(msg.name == 'Get_Encoder_Estimates')
        node_id = msg.axis_id
        pos, vel = msg.data['Pos_Estimate'], msg.data['Vel_Estimate']

        logging.debug(f"[Node #{node_id}] pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
        
        self.curpos[node_id] = pos

        if node_id == self.LEFT_NODE_ID:
            if self.debug_cb is not None:
                self.debug_cb("curpos_left", pos)
        else:
            if self.debug_cb is not None:
                self.debug_cb("curpos_right", pos)

        this_time = time.time()
        with self.lock:
            if self.last_time is None:
                self.last_left_vel = 0
                self.last_right_vel = 0
                self.last_time = this_time
            delta_time = this_time - self.last_time

            if node_id == self.LEFT_NODE_ID:
                left_vel = vel * self.LEFT_INSTALL_DIR
                right_vel = self.last_right_vel
            else:
                left_vel = self.last_left_vel
                right_vel = vel * self.RIGHT_INSTALL_DIR

            linear_vel = (left_vel + right_vel) / 2 * (math.pi * self.WHEEL_D)
            angular_vel = (right_vel - left_vel) / 2 * (math.pi * self.WHEEL_D) / (self.WHEEL_BASE / 2)

            if node_id == self.LEFT_NODE_ID:
                self.last_left_vel = left_vel
            else:
                self.last_right_vel = right_vel
            self.last_time = this_time
            if self.state_cb is not None:
                self.state_cb(linear_vel, angular_vel, delta_time)

    async def odrive_can_thread(self, node_id):
        drv = odrive_can.odrive.ODriveCAN(node_id, self.device_name)

        await drv.start()

        logger.info("Clearing errors")
        drv.clear_errors()
        drv.check_errors()

        # Reset encoder
        drv.set_linear_count(0)

        # Set Controller Mode
        drv.set_controller_mode("POSITION_CONTROL", "TRAP_TRAJ")

        # Set gains
        drv.set_pos_gain(20.0)
        
        # Set gains
        drv.set_vel_gains(0.1, 0)
        
        drv.set_traj_vel_limit(20)
        drv.set_traj_accel_limits(1, 1)

        # set control mode
        drv.set_axis_state_no_wait("CLOSED_LOOP_CONTROL")
        
        while drv.axis_state != "CLOSED_LOOP_CONTROL":
            drv.check_errors()
            await asyncio.sleep(0.1)

        drv.feedback_callback = self.feedback_cb

        logger.info("Running base control")
        try:
            while True:
                if self.quit.is_set():
                    raise Exception("quit")
                with self.write_lock:
                    drv.set_input_pos(self.setpos[node_id])
                drv.check_errors()
                await asyncio.sleep(0.1)
        finally:
            drv.stop()

    def set_vel(self, linear_vel, angular_vel):
        left_vel = (linear_vel - angular_vel * self.WHEEL_BASE / 2) / (math.pi * self.WHEEL_D) * self.LEFT_INSTALL_DIR
        right_vel = (linear_vel + angular_vel * self.WHEEL_BASE / 2) / (math.pi * self.WHEEL_D) * self.RIGHT_INSTALL_DIR
        
        TIME_DELTA = 0.1 # TODO Better solution
        MAX_POS_DIFF_IN_TURNS = 2

        with self.write_lock:
            self.setpos[self.LEFT_NODE_ID] += left_vel * TIME_DELTA
            self.setpos[self.RIGHT_NODE_ID] += right_vel * TIME_DELTA
            
            if self.setpos[self.LEFT_NODE_ID] > self.curpos[self.LEFT_NODE_ID] + MAX_POS_DIFF_IN_TURNS:
                self.setpos[self.LEFT_NODE_ID] = self.curpos[self.LEFT_NODE_ID] + MAX_POS_DIFF_IN_TURNS
                logger.info(f"Left setpos clamped to {self.setpos[self.LEFT_NODE_ID]}")
            if self.setpos[self.LEFT_NODE_ID] < self.curpos[self.LEFT_NODE_ID] - MAX_POS_DIFF_IN_TURNS:
                self.setpos[self.LEFT_NODE_ID] = self.curpos[self.LEFT_NODE_ID] - MAX_POS_DIFF_IN_TURNS
                logger.info(f"Left setpos clamped to {self.setpos[self.LEFT_NODE_ID]}")
            if self.setpos[self.RIGHT_NODE_ID] > self.curpos[self.RIGHT_NODE_ID] + MAX_POS_DIFF_IN_TURNS:
                self.setpos[self.RIGHT_NODE_ID] = self.curpos[self.RIGHT_NODE_ID] + MAX_POS_DIFF_IN_TURNS
                logger.info(f"Right setpos clamped to {self.setpos[self.RIGHT_NODE_ID]}")
            if self.setpos[self.RIGHT_NODE_ID] < self.curpos[self.RIGHT_NODE_ID] - MAX_POS_DIFF_IN_TURNS:
                self.setpos[self.RIGHT_NODE_ID] = self.curpos[self.RIGHT_NODE_ID] - MAX_POS_DIFF_IN_TURNS
                logger.info(f"Right setpos clamped to {self.setpos[self.RIGHT_NODE_ID]}")

        if self.debug_cb is not None:
            self.debug_cb("setpos_left", self.setpos[self.LEFT_NODE_ID])
        if self.debug_cb is not None:
            self.debug_cb("setpos_right", self.setpos[self.RIGHT_NODE_ID])

    def stop(self):
        if not self.quit.is_set():
            self.quit.set()
            time.sleep(0.1)

    def __del__(self):
        self.stop()

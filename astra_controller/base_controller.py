import can.interface
import threading
import numpy as np
import struct
import math
import time

import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class BaseController:
    def __init__(self, name):
        logger.info(f"Using device {name}")

        self.state_cb = None

        self.bus = can.interface.Bus("can0", bustype="socketcan")
        # Flush CAN RX buffer so there are no more old pending messages
        while self.bus.recv(timeout=0):
            pass

        self.lock = threading.Lock()
        
        self.last_left_vel = None
        self.last_right_vel = None
        self.last_time = None

        self.write_lock = threading.Lock()

        self.quit = threading.Event()

        self.t = threading.Thread(target=self.recv_thread)
        self.t.daemon = True
        self.t.start()

        self.t = threading.Thread(target=self.request_state_thread)
        self.t.daemon = True
        self.t.start()

        while self.last_time is None: # wait for init done
            time.sleep(0.1)
    
    @staticmethod
    def split_id(id):
        node_id = id >> 5
        cmd_id = id & ((1 << 5) - 1)
        return node_id, cmd_id

    @staticmethod
    def merge_id(node_id, cmd_id):
        return (node_id << 5 | cmd_id)
    
    WHEEL_BASE = 0.44 # in [m]
    WHEEL_D = 0.139 # in [m]
    
    LEFT_INSTALL_DIR = -1
    RIGHT_INSTALL_DIR = 1
    
    LEFT_NODE_ID = 0
    RIGHT_NODE_ID = 1
    CMD_ID_HEARTBEAT = 0x01
    CMD_ID_GET_ENCODER_ESTIMATES = 0x09
    CMD_ID_SET_INPUT_VEL = 0x0d
    
    AXIS_STATE_CLOSED_LOOP_CONTROL = 0x08
    
    def request_state_thread(self):
        try:
            while not self.quit.is_set():
                self.bus.send(can.Message(
                    arbitration_id=self.merge_id(self.LEFT_NODE_ID, self.CMD_ID_GET_ENCODER_ESTIMATES),
                    is_remote_frame=True,
                    is_extended_id=False
                ))
                self.bus.send(can.Message(
                    arbitration_id=self.merge_id(self.RIGHT_NODE_ID, self.CMD_ID_GET_ENCODER_ESTIMATES),
                    is_remote_frame=True,
                    is_extended_id=False
                ))
                time.sleep(0.01)
        # except Exception:
        #     pass
        finally:
            print("thread exiting")

    def recv_thread(self):
        try:
            for msg in self.bus:
                if self.quit.is_set():
                    return

                node_id, cmd_id = self.split_id(msg.arbitration_id)
                if node_id not in [self.LEFT_NODE_ID, self.RIGHT_NODE_ID]:
                    logging.warn(f"Unknown node_id {node_id}, check your node_id setting!")
                
                if cmd_id == self.CMD_ID_HEARTBEAT:
                    error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                    if error:
                        logging.warn(f"[Node #{node_id}] Error state set to {error}")
                    if state != self.AXIS_STATE_CLOSED_LOOP_CONTROL:
                        logging.warn(f"[Node #{node_id}] Axis state {state} not set to {self.AXIS_STATE_CLOSED_LOOP_CONTROL}, check your axis state setting!")
                elif cmd_id == self.CMD_ID_GET_ENCODER_ESTIMATES:
                    pos, vel = struct.unpack('<ff', bytes(msg.data))
                    logging.info(f"[Node #{node_id}] pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
                    
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
                            self.last_left_vel = vel
                        else:
                            self.last_right_vel = vel
                        self.last_time = this_time
                        if self.state_cb is not None:
                            self.state_cb(linear_vel, angular_vel, delta_time)
        # except Exception:
        #     pass
        finally:
            print("thread exiting")

    def set_vel(self, linear_vel, angular_vel):
        left_vel = (linear_vel - angular_vel * self.WHEEL_BASE / 2) / (math.pi * self.WHEEL_D) * self.LEFT_INSTALL_DIR
        right_vel = (linear_vel + angular_vel * self.WHEEL_BASE / 2) / (math.pi * self.WHEEL_D) * self.RIGHT_INSTALL_DIR
        
        self.bus.send(can.Message(
            arbitration_id=self.merge_id(self.LEFT_NODE_ID, self.CMD_ID_SET_INPUT_VEL),
            data=struct.pack('<ff', left_vel, 0.0), # velocity, torque_feedforward
            is_extended_id=False
        ))
        
        self.bus.send(can.Message(
            arbitration_id=self.merge_id(self.RIGHT_NODE_ID, self.CMD_ID_SET_INPUT_VEL),
            data=struct.pack('<ff', right_vel, 0.0), # velocity, torque_feedforward
            is_extended_id=False
        ))
        
    def __del__(self):
        self.quit.set()
        self.bus.shutdown()

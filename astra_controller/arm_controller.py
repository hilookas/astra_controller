import serial
import threading
import numpy as np
import struct
import math
import time
import sys

import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class ArmController:
    COMM_LEN = 2 + 16
    COMM_HEAD = 0x5A
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_FEEDBACK = 0x03
    COMM_TYPE_TORQUE = 0x04
    COMM_TYPE_CONFIG_WRITE = 0x05
    COMM_TYPE_CONFIG_READ = 0x06
    COMM_TYPE_CONFIG_FEEDBACK = 0x07

    GRIPPER_GEAR_R = 0.027 / 2

    @staticmethod
    def to_si_unit(arr):
        arr = arr.copy()
        arr = -(np.array(arr) - 2048) / 4096 * (2*math.pi)
        arr[-1] *= ArmController.GRIPPER_GEAR_R
        arr[-1] += 0.03 # 标定的时候，中点要设置为夹爪张开60mm的时候（左右各30mm）
        return arr

    @staticmethod
    def to_raw_unit(arr):
        arr = arr.copy()
        arr[-1] -= 0.03
        arr[-1] /= ArmController.GRIPPER_GEAR_R
        arr = (-np.array(arr) / (2*math.pi) * 4096 + 2048).astype(int)
        return arr

    def __init__(self, name):
        logger.info(f"Using device {name}")

        self.state_cb = None
        self.pong_cb = None

        self.ser = serial.Serial(name, 921600, timeout=None)
        # self.ser.rts = False

        self.lock = threading.Lock()

        self.last_position = None
        self.last_velocity = None
        self.last_effort = None
        self.last_time = None

        self.write_lock = threading.Lock()

        self.config_cb = None
        self.config_cb_lock = threading.Lock()

        self.quit = threading.Event()

        self.t = threading.Thread(target=self.recv_thread, daemon=True)
        self.t.start()

        while self.last_position is None: # wait for init done
            time.sleep(0.1)

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == self.COMM_HEAD): # 逐步同步
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
                    continue

                data += self.ser.read(self.COMM_LEN - 1)
                assert(len(data) == self.COMM_LEN)

                if data[1] == self.COMM_TYPE_PONG:
                    if self.state_cb is not None:
                        self.pong_cb(struct.unpack('>xxHHHHHHxxxx', data))
                elif data[1] == self.COMM_TYPE_FEEDBACK:
                    position = self.to_si_unit(np.array(struct.unpack('>xxHHHHHHxxxx', data)))
                    this_time = time.time()
                    with self.lock:
                        if self.last_time is None:
                            self.last_position = position
                            self.last_velocity = np.array([0, 0, 0, 0, 0, 0])
                            self.last_effort = np.array([0, 0, 0, 0, 0, 0])
                            self.last_time = this_time - 1 # in case of dividing 0
                        delta_time = this_time - self.last_time
                        velocity = (position - self.last_position) / delta_time
                        effort = (velocity - self.last_velocity) / delta_time # without bias (gravity) and mass
                        self.last_position = position
                        self.last_velocity = velocity
                        self.last_effort = effort
                        self.last_time = this_time
                        if self.state_cb is not None:
                            self.state_cb(self.last_position, self.last_velocity, self.last_effort, self.last_time)
                elif data[1] == self.COMM_TYPE_CONFIG_FEEDBACK:
                    self.config_cb(data) # TODO 解决没有时报错
                else:
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
        # except Exception:
        #     pass
        finally:
            print("thread exiting")

    def get_pos(self): # 一帧可能会被用多次，但是绝对不会卡住
        with self.lock:
            return self.last_position, self.last_velocity, self.last_effort, self.last_time

    def write(self, encoded_data):
        with self.write_lock:
            # print(encoded_data)
            self.ser.write(encoded_data)

    JOINT_MIN = np.array([-math.pi/2, -math.pi/2, -math.pi*0.75, -math.pi/2, -math.pi*0.75, -math.pi/2])
    JOINT_MAX = np.array([math.pi/2, math.pi/2, math.pi*0.75, math.pi/2, math.pi*0.75, math.pi/2])

    def set_pos(self, pos):
        pos_protected = np.minimum(np.maximum(np.array(pos), self.JOINT_MIN), self.JOINT_MAX)
        if all(np.isclose(pos, pos_protected, atol=0.01)):
            pass
            self.write(struct.pack('>BBHHHHHHxxxx', self.COMM_HEAD, self.COMM_TYPE_CTRL, *self.to_raw_unit(pos)))
        else:
            logger.error(f"Arm reach min/max!!! {pos} {pos_protected}")

    def __del__(self):
        self.quit.set()
        self.ser.close()

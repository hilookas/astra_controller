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
    COMM_LEN = 2 + 16 + 1
    COMM_HEAD = 0x5A
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_FEEDBACK = 0x03
    COMM_TYPE_TORQUE = 0x04
    COMM_TYPE_CONFIG_WRITE = 0x05
    COMM_TYPE_CONFIG_READ = 0x06
    COMM_TYPE_CONFIG_FEEDBACK = 0x07
    COMM_TYPE_PIDTUNE = 0x08
    COMM_TYPE_INIT_JOINT = 0x09

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
        
        self.debug_cb = None
        
        self.error_cb = None

        self.quit = threading.Event()
        
        self.t = threading.Thread(target=self.recv_thread, daemon=True)
        self.t.start()
        
        self.set_torque(1)
        self.set_pid()
        
        # # set gripper mid
        # self.write(struct.pack('>BBiiii', self.COMM_HEAD, self.COMM_TYPE_INIT_JOINT, 15, 1448, 0, 0)) # 1448 / 4096 * 2 * 3.14 * 0.027 / 2 == 0.030m == 30mm
        # time.sleep(2)
        # exit()

        while self.last_position is None: # wait for init done
            time.sleep(0.1)
    
    # def set_pid(self, p=10.0, i=7.0, d=20.0, i_max=800, i_clip_thres=10.0, i_clip_coef=0.5):
    # def set_pid(self, p=8.0, i=0.5, d=0, i_max=100, i_clip_thres=100000.0, i_clip_coef=1):
    def set_pid(self, p=30, i=0, d=0, i_max=800, p2=10, p2_err_thres=2, i_clip_thres=100000.0, i_clip_coef=1):
        self.write(struct.pack('>BBffffffff', self.COMM_HEAD, self.COMM_TYPE_PIDTUNE, *[p, i, d, i_clip_thres, i_clip_coef, i_max, p2, p2_err_thres]))
    
    @staticmethod
    def checksum(data: bytes):
        checksum = 0
        for b in data[1:-1]:
            checksum = checksum + b
        checksum = checksum % 256
        return checksum == data[-1]
    
    databuf = bytearray()

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == self.COMM_HEAD): # 逐步同步
                    if data[0] == "\n".encode("ascii")[0] or data[0] == "\r".encode("ascii")[0]:
                        if len(self.databuf) > 0 and set(self.databuf) <= set("0123456789-., ".encode("ascii")):
                            try:
                                debugdata = [float(x) for x in self.databuf.decode("ascii").split(",")]
                                if self.debug_cb:
                                    self.debug_cb(debugdata)
                            except:
                                pass
                        elif len(self.databuf) == 1:
                            print(self.databuf)
                            
                        self.databuf = bytearray()
                    else:
                        self.databuf.extend(data)
                    
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
                    continue

                data += self.ser.read(self.COMM_LEN - 1)
                assert(len(data) == self.COMM_LEN)
                
                if not self.checksum(data):
                    logger.error(f"checksum failed {data.hex()}")
                    continue

                if data[1] == self.COMM_TYPE_PONG:
                    if self.pong_cb is not None:
                        self.pong_cb(struct.unpack('>HHHHHHxxxx', data[2:-1]))
                elif data[1] == self.COMM_TYPE_FEEDBACK:
                    position = self.to_si_unit(np.array(struct.unpack('>HHHHHHxxxx', data[2:-1])))
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
                    self.config_cb(data[2:-1]) # TODO 解决没有时报错
                else:
                    # sys.stdout.buffer.write(data)
                    # sys.stdout.flush()
                    pass
        finally:
            logger.info("thread exiting")

    def get_pos(self): # 一帧可能会被用多次，但是绝对不会卡住
        with self.lock:
            return self.last_position, self.last_velocity, self.last_effort, self.last_time

    def write(self, encoded_data):
        with self.write_lock:
            self.ser.write(encoded_data)

    JOINT_MIN = np.array([-math.pi/2, -math.pi/2, -math.pi*0.999, -math.pi/2, -math.pi*0.875, -math.pi/2])
    JOINT_MAX = np.array([math.pi/2, math.pi/2, math.pi*0.999, math.pi/2, math.pi*0.875, math.pi/2])

    def set_pos(self, pos):
        ok = True
        for i, (p, mn, mx) in enumerate(zip(pos, self.JOINT_MIN, self.JOINT_MAX)):
            if not (mn <= p <= mx):
                logger.error(f"Joint #{i+2} reach limit, min: {mn}, max: {mx}, current pos: {p}")
                if self.error_cb is not None:
                    self.error_cb(f"Joint #{i+2} reach limit")
                # pos[i] = np.clip(pos[i], mn, mx)
                ok = False
        if ok:
            self.write(struct.pack('>BBHHHHHHxxxx', self.COMM_HEAD, self.COMM_TYPE_CTRL, *self.to_raw_unit(pos)))
    
    def set_torque(self, torque):
        self.write(struct.pack('>BBBxxxxxxxxxxxxxxx', self.COMM_HEAD, self.COMM_TYPE_TORQUE, torque))
        
    def stop(self):
        if not self.quit.is_set():
            self.quit.set()
            self.set_torque(0)
            self.ser.flushOutput()
            self.ser.close()

    def __del__(self):
        self.stop()

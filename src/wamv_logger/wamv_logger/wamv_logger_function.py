# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

import time
import re
import os
import threading

import paramiko
import pandas as pd

class OdomAndRadioLogger(Node):

    def __init__(self, fpath: str, usr: str, pwd: str, ip_addr: str, port: int=22) -> None:
        super().__init__('odom_and_radio_logger')
        self.fpath = fpath
        self.user = usr
        self.pwd = pwd
        self.ip = ip_addr
        self.port = port
        self.columns = [
            'time', 'frequency', 'bit_rate', 'tx_power', 'link_quality',
            'signal_level', 'noise_level', 'lat', 'lon', 'heading'
        ]
        self.temp_data = {"heading": None, "llh": None, "radio": None}
        self.subscription_1 = self.create_subscription(
            NavSatFix,
            "/wamv/sensors/gps/gps/fix",
            self.logger_callback,
            10)
        self.subscription_1

    def logger_callback(self, msg: NavSatFix, echo: bool=True) -> None:
        # what are this in? lat-lon degrees?
        callback_time = time.time()
        if (echo):
            print(f"Callback time init: {msg}")
        pos_x, pos_y = msg.latitude, msg.longitude
        #rotation = msg.pose.pose.orientation
        #data_dict = self.query_radio()
        data_dict = dict()
        query_time = time.time()
        timestamp = (callback_time + query_time)/2
        if data_dict:
            data_dict['lat'] = pos_x
            data_dict['lon'] = pos_y
            #data_dict['heading'] = rotation
            data_dict['time'] = timestamp
        else:
            data_dict = {k: None for k in self.columns}
            data_dict['lat'] = pos_x
            data_dict['lon'] = pos_y
            #data_dict['heading'] = rotation
            data_dict['time'] = timestamp
        if (echo): print(data_dict)
        self.store_data(data_dict)
    
    # ref:
    # https://wiki.ros.org/microstrain_inertial_driver#Publishers
    def llh_position_callback(self, msg: NavSatFix) -> None:
        # subscribe to /ekf/llh_position
        # log lat, lon, altitude
        pass

    def ekf_antenna_heading(self, msg: PoseWithCovarianceStamped) -> None:
        # subscribe to /ekf/dual_antenna_heading
        # pose.pose.orientation gets the Z axis orientation in rads
        pass

    
    def parseRadioData(self, msg_data: str) -> dict:
        fields = {
            "protocol": re.search(r"IEEE ([\w.]+)", msg_data),
            "essid": re.search(r'ESSID:"([^"]+)"', msg_data),
            "mode": re.search(r"Mode:(\w+)", msg_data),
            "frequency": re.search(r"Frequency:([\d.]+ \w+)", msg_data),
            "access_point": re.search(r"Access Point: ([\w:]+)", msg_data),
            "bit_rate": re.search(r"Bit Rate:([\d\s\w/]+)", msg_data),
            "tx_power": re.search(r"Tx-Power=(\d+ dBm)", msg_data),
            "sensitivity": re.search(r"Sensitivity:([\d/]+)", msg_data),
            "rts_threshold": re.search(r"RTS thr:(\w+)", msg_data),
            "fragment_threshold": re.search(r"Fragment thr:(\w+)", msg_data),
            "encryption_key": re.search(r"Encryption key:(\w+)", msg_data),
            "power_management": re.search(r"Power Management:(\w+)", msg_data),
            "link_quality": re.search(r"Link Quality=(\d+/\d+)", msg_data),
            "signal_level": re.search(r"Signal level=(-?\d+ dBm)", msg_data),
            "noise_level": re.search(r"Noise level=(-?\d+ dBm)", msg_data),
            "rx_invalid_nwid": re.search(r"Rx invalid nwid:(\d+)", msg_data),
            "rx_invalid_crypt": re.search(r"Rx invalid crypt:(\d+)", msg_data),
            "rx_invalid_frag": re.search(r"Rx invalid frag:(\d+)", msg_data),
            "tx_excessive_retries": re.search(r"Tx excessive retries:(\d+)", msg_data),
            "invalid_misc": re.search(r"Invalid misc:(\d+)", msg_data),
            "missed_beacon": re.search(r"Missed beacon:(\d+)", msg_data),
        }

        temp_dict = {key: match.group(1) if match else None for key, match in fields.items()}
        formatted_dict = dict()
        for elem in self.columns:
            formatted_dict[elem] = temp_dict[elem]
        return formatted_dict
    
    def query_radio(self, echo: bool=False) -> dict:
        try:
            cmd_data = self.ssh_exec("iwconfig")
            msg_dict = self.parse(cmd_data)
            if echo:
                print(msg_dict)
    
            return msg_dict
        except paramiko.ssh_exception.SSHException:
            print(f'SSH connection for {self.ip} failed, skipping.')
            return None

    def ssh_exec(self, cmd: str) -> str:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip, self.port, self.user, self.pwd, banner_timeout=200)
        ssh = self.ssh_connect()
        _, stdout, _ = ssh.exec_command(cmd)
        ssh.close()
        return stdout.read().decode()
    
    def store_data(self, row_data: dict):
        if os.path.exists(self.fpath):
            data_frame = pd.read_csv(self.fpath)
        else:
            data_frame = pd.DataFrame(columns=self.columns)
        data_frame = pd.concat([data_frame, pd.DataFrame(row_data, index=[0])], ignore_index=True)
        data_frame.to_csv(self.fpath, index=False)



def main(args=None) -> None:
    try:
        with open('secrets.txt', mode="r") as f:
            user, passkey, ipaddr = f.readline().split(";")
    except FileNotFoundError as e:
        print(f"{e}, you might have forgotten the secrets.txt file")
        print("Format is <user;passkey;ipaddr>")
        return

    rclpy.init(args=args)

    odom_and_radio_logger = OdomAndRadioLogger("logs_wamv.csv", user, passkey, ipaddr)

    rclpy.spin(odom_and_radio_logger)

    odom_and_radio_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
#import threading

import paramiko
#import pandas as pd
import csv

DEBUG = True

class OdomAndRadioLogger(Node):

    def __init__(self, usr: str, pwd: str, ip_addr: str, port: int=22) -> None:
        super().__init__('odom_and_radio_logger')
        pid = os.getpid()
        out_dir = f'logs_{pid}'
        os.makedirs(out_dir, exist_ok=True)
        print(f'[INFO] Writing data to {os.getcwd()}/{out_dir}')
        self.radiwcfg_path = f"{out_dir}/radio_logs_iwcfg.csv"
        self.radiomca_path = f"{out_dir}/radio_logs_mca.csv"
        self.llhlog_path = f"{out_dir}/llh_logs.csv"
        self.hdnlog_path = f"{out_dir}/hdn_logs.csv"
        self.user = usr
        self.pwd = pwd
        self.ip = ip_addr
        self.port = port
        self.radio_cols = [
            'time', 'frequency', 'bit_rate', 'tx_power', 'link_quality',
            'signal_level', 'noise_level',]
        self.llg_cols = ['time', 'lat', 'lon', 'alt']
        self.hdn_cols = ['time', 'heading']
        self.mca_cols = ['time', 'rxbytes', 'txbytes', 'frequency', 'centerfreq', 'chanbw', 'txpwr', 'rxbw', 'txbw', 'signal', 'noise']
        self.mca_last_rxb = int()
        self.mca_last_txb = int()
        self.mca_last_time = int()
        # self.subscription_1 = self.create_subscription(
        #     NavSatFix,
        #     "/wamv/sensors/gps/gps/fix",
        #     self.logger_callback,
        #     10)
        # self.subscription_1

        self.llh_sub = self.create_subscription(
            NavSatFix,
            "/ekf/llh_position",
            self.llh_position_callback,
            10
        )
        self.llh_sub

        self.hdn_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/ekf/dual_antenna_heading",
            self.ekf_antenna_heading,
            10
        )
        self.hdn_sub

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
            data_dict = {k: None for k in self.radio_cols}
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
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        llh_data = {
            'time': time.time(), 'lat': lat, 'lon': lon, 'alt': alt
        }
        #print(msg)
        self.store_data(self.llhlog_path, llh_data, self.llg_cols)
        self.query_radio()
        time.sleep(1)

    def ekf_antenna_heading(self, msg: PoseWithCovarianceStamped) -> None:
        # subscribe to /ekf/dual_antenna_heading
        # pose.pose.orientation. gets the Z axis orientation in rads
        # or just write pose.pose.orienteation if
        # subscribed to /ekf/odometry_map
        heading = msg.pose.pose.orientation.z
        #heading = msg.pose.pose
        if DEBUG:
            print(f"[DEBUG] Heading data: {heading}")
        heading_data = {'time': time.time(), 'heading': heading}
        self.store_data(self.hdnlog_path, heading_data, self.hdn_cols)
        #self.query_radio()
    
    def parseMcaStatusData(self, msg_data: str) -> dict:
        fields = {
            "rxbytes": re.search(r"wlanRxBytes=(\d+)", msg_data),
            "txbytes": re.search(r"wlanTxBytes=(\d+)", msg_data),
            "frequency": re.search(r"freq=(\d+)", msg_data),
            "centerfreq": re.search(r"centerFreq=(\d+)", msg_data),
            "chanbw": re.search(r"chanbw=(\d+)", msg_data),
            "txpwr" : re.search(r"txPower=(\d+)", msg_data),
            "signal": re.search(r"signal=-(\d+)", msg_data),
            "noise": re.search(r"noise=-(\d+)", msg_data), 	
        }
        temp_dict = {key: match.group(1) if match else None for key, match in fields.items()}
        formatted_dict = dict()
        temp_dict['time'] = time.time()
        temp_dict['rxbw'] = 0
        temp_dict['txbw'] = 0
        for elem in self.mca_cols:
            formatted_dict[elem] = temp_dict[elem]
        return formatted_dict

    def estimateThorughput(self, mca_dict: dict) -> dict:
        current_time = mca_dict['time']
        current_rx = int(mca_dict['rxbytes'])
        current_tx = int(mca_dict['txbytes'])
        est_rx = (current_rx - self.mca_last_rxb) / (current_time - self.mca_last_time)
        self.mca_last_rxb = current_rx
        est_tx = (current_tx - self.mca_last_txb) / (current_time - self.mca_last_time)
        self.mca_last_txb = current_tx
        self.mca_last_time = current_time
        return {"rxbw": est_rx, "txbw": est_tx}

    def parseIwConfigData(self, msg_data: str) -> dict:
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
        temp_dict['time'] = time.time()
        for elem in self.radio_cols:
            formatted_dict[elem] = temp_dict[elem]
        return formatted_dict

    def ssh_exec(self, cmd: str) -> str:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip.strip(), self.port, self.user, self.pwd, banner_timeout=200)
        #ssh = self.ssh_connect()
        _, stdout, stderr = ssh.exec_command(cmd)
        out = stdout.read().decode()
        if DEBUG:
            print(f"[DEBUG] stderr: {stderr.read().decode()}")
            print(f"[DEBUG] stdout: {out}")
        ssh.close()
        return out

    def query_radio(self, echo: bool=False) -> None:
        try:
            cmd_data = self.ssh_exec("/usr/bin/iwconfig")
            if DEBUG:
                print(f"[DEBUG] Radio data: {cmd_data}")
            msg_dict = self.parseIwConfigData(cmd_data)
            if echo:
                print(msg_dict)

            self.store_data(self.radiwcfg_path, msg_dict, self.radio_cols)
            #print("Radio data:")
            #print(msg_dict)
            cmd_data = self.ssh_exec("/usr/bin/mca-status")
            if DEBUG:
                print(f"[DEBUG] MCA data: {cmd_data}")
            msg_dict = self.parseMcaStatusData(cmd_data)
            if echo:
                print(msg_dict)
            throughput_data = self.estimateThorughput(msg_dict)
            msg_dict.update(throughput_data)
            self.store_data(self.radiomca_path, msg_dict, self.mca_cols)

        except paramiko.ssh_exception.SSHException:
            print(f'SSH connection for {self.ip} failed, skipping.')

    def store_data(self, fpath: str, row_data: dict, cols: list):
        if os.path.exists(fpath):
            with open(fpath, 'r') as file:
                reader = csv.DictReader(file)
                data_frame = list(reader)
        else:
            data_frame = []
        data_frame.append(row_data)
        fieldnames = cols
        with open(fpath, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data_frame)
        #data_frame.to_csv(fpath, index=False)
        print(f"[INFO] Data saved to {fpath}")



def main(args=None) -> None:
    try:
        with open('secrets.txt', mode="r") as f:
            user, passkey, ipaddr = f.readline().split(";")
            print(f"[INFO] User: {user}, IP: {ipaddr}")
    except FileNotFoundError as e:
        print(f"{e}, you might have forgotten the secrets.txt file")
        print("Format is <user;passkey;ipaddr>")
        return

    rclpy.init(args=args)

    odom_and_radio_logger = OdomAndRadioLogger(user, passkey, ipaddr)

    rclpy.spin(odom_and_radio_logger)

    odom_and_radio_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
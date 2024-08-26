import os
import pandas as pd
import paramiko
import re
import time
import threading


class Logger():
    def __init__(self, fptah: str, usr: str, pwd: str, ip_addr: str, port: int=22) -> None:
        self.fptah = fptah
        self.user = usr
        self.pwd = pwd
        self.ip = ip_addr
        self.port = port
        self.columns = ['time', 'frequency', 'bit_rate', 'tx_power', 'link_quality', 'signal_level', 'noise_level']
    
    def parse(self, msg_data: str) -> dict:
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
        temp_dict['time'] = time.time()
        formatted_dict = dict()
        for elem in self.columns:
            formatted_dict[elem] = temp_dict[elem]
        return formatted_dict

    
    def log(self, msg: dict) -> None:
        # open csv file using pandas
        if os.path.exists(self.fptah):
            data_frame = pd.read_csv(self.fptah)
        else:
            data_frame = pd.DataFrame(columns=self.columns)
        # append new data to the csv file
        # using dict to map the data to the columns
        # get unix time
        msg['time'] = time.time()
        data_frame = pd.concat([data_frame, pd.DataFrame(msg, index=[0])], ignore_index=True)
        # save the new data to the csv file
        data_frame.to_csv(self.fptah, index=False)

    def ssh_connect(self) -> None:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip, self.port, self.user, self.pwd, banner_timeout=200)
        return ssh
    
    def ssh_exec(self, cmd: str) -> str:
        ssh = self.ssh_connect()
        _, stdout, _ = ssh.exec_command(cmd)
        ssh.close()
        return stdout.read().decode()
    
    def gather_data(self, cmd: str, echo: bool=False) -> None:
        try:
            cmd_data = self.ssh_exec(cmd)
            if echo:
                print(cmd_data)
            if cmd == "iwconfig":
                msg_dict = self.parse(cmd_data)
                if echo:
                    print(msg_dict)
        
            return msg_dict
        except paramiko.ssh_exception.SSHException:
            print(f'SSH connection for {self.ip} failed, skipping.')
            return None

    def run(self, cmd: str, echo: bool=False) -> None:
        msg = self.gather_data(cmd, echo)
        if msg:
            self.log(msg)

    


if __name__ == '__main__':
    logger_base = Logger('logs_base.csv', '--', '--', '--')
    logger_wamv = Logger('logs_wamv.csv', '--', '--', '--')
    n = 0
    while True:
        n += 1
        try:
            logger_base.run('iwconfig', False)
            logger_wamv.run('iwconfig', False)
            print(f'Wrote {n} times.')
        except TimeoutError:
            pass
        time.sleep(15)
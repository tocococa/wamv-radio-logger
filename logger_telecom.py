import os
import pandas as pd
import paramiko
import re
import time

# /tmp/stats/wstalist

class Logger():
    def __init__(self, fptah: str, usr: str, pwd: str, ip_addr: str, port: int=22) -> None:
        self.fptah = fptah
        self.user = usr
        self.pwd = pwd
        self.ip = ip_addr
        self.port = port
        self.iwcfg_cols = ['time', 'frequency', 'bit_rate', 'tx_power', 'link_quality', 'signal_level', 'noise_level']
        self.mca_cols = ['time', 'rxbytes', 'txbytes', 'frequency', 'centerfreq', 'chanbw', "rxbw", "txbw"]
        self.mca_last_rxb = int()
        self.mca_last_txb = int()
        self.mca_last_time = int()
    
    def parseMcaStatusData(self, msg_data: str) -> dict:
        # mca-status is a propietary command from Ubiquiti
        # see mca-status.txt for and example of the output.
        fields = {
            "rxbytes": re.search(r"wlanRxBytes=(\d+)", msg_data),
            "txbytes": re.search(r"wlanTxBytes=(\d+)", msg_data),
            "frequency": re.search(r"freq=(\d+)", msg_data),
            "centerfreq": re.search(r"centerFreq=(\d+)", msg_data),
            "chanbw": re.search(r"chanbw=(\d+)", msg_data),
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

    def parseIwcfg(self, msg_data: str) -> dict:
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
        for elem in self.iwcfg_cols:
            formatted_dict[elem] = temp_dict[elem]
        return formatted_dict

    
    def log(self, msg: dict, cols: list) -> None:
        if os.path.exists(self.fptah):
            data_frame = pd.read_csv(self.fptah)
        else:
            data_frame = pd.DataFrame(columns=cols)
        msg['time'] = time.time()
        data_frame = pd.concat([data_frame, pd.DataFrame(msg, index=[0])], ignore_index=True)
        data_frame.to_csv(self.fptah, index=False)

    def ssh_connect(self) -> None:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip, self.port, self.user, self.pwd, banner_timeout=200)
        return ssh
    
    def ssh_exec(self, cmd: str) -> str:
        ssh = self.ssh_connect()
        _, stdout, stderr = ssh.exec_command(cmd)
        print(f"stderr: {stderr.read().decode()}")
        ssh.close()
        return stdout.read().decode()
    
    def gather_data(self, cmd: str, echo: bool=False) -> None:
        try:
            cmd_data = self.ssh_exec(cmd)
            if echo:
                print(cmd_data)
            if cmd == "iwconfig":
                msg_dict = self.parseIwcfg(cmd_data)
                if echo:
                    print(msg_dict)
            if cmd == "mca-status":
                msg_dict = self.parseMcaStatusData(cmd_data)
                tmp_dict = self.estimateThorughput(msg_dict)
                msg_dict["rxbw"] = tmp_dict["rxbw"]
                msg_dict["txbw"] = tmp_dict["txbw"]
                if echo:
                    print(msg_dict)
        
            return msg_dict
        except paramiko.ssh_exception.SSHException:
            print(f'SSH connection for {self.ip} failed, skipping.')
            return None

    def run(self, cmd: str, echo: bool=False) -> None:
        msg = self.gather_data(cmd, echo)
        if msg:
            if cmd == "iwconfig":
                   self.log(msg, self.iwcfg_cols)
            if cmd == "mca-status":
                self.log(msg, self.mca_cols)


    


if __name__ == '__main__':
    pid = os.getpid()
    out_dir = f'logs_{pid}'
    os.makedirs(out_dir, exist_ok=True)
    print(f'[INFO] Writing data to {os.getcwd()}/{out_dir}')
    #logger_base = Logger(f'{out_dir}/logs_base.csv', 'pi', '1985', '192.168.1.6')
    logger_wamv_iwc = Logger(f'{out_dir}/logs_wamv.csv', 'ubnt', 'ubntwamv', '192.168.1.20')
    logger_base_iwc = Logger(f'{out_dir}/logs_base_iwc.csv', 'ubnt', 'ubntbase', '192.168.1.21')
    logger_base_mca = Logger(f'{out_dir}/logs_base_mca.csv', 'ubnt', 'ubntbase', '192.168.1.21')
    n = 0
    while True:
        n += 1
        try:
            logger_base_iwc.run('iwconfig', True)
            logger_base_mca.run('mca-status', True)
            logger_wamv_iwc.run('mca-status', True)
            print(f'[INFO] Wrote {n} times.')
        except TimeoutError:
            pass
        time.sleep(5)
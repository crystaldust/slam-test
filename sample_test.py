#!/usr/bin/env python3

import os
import psutil
import shlex
import subprocess
import signal
from multiprocessing import Process, Manager
from time import sleep


def source_envfile(envfile_path):
    command = shlex.split("env -i bash -c 'source {} && env'".format(envfile_path))
    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.decode('utf-8').partition("=")
        os.environ[key] = value.rstrip()


def init_env():
    source_envfile('/opt/ros/melodic/setup.bash')
    source_envfile('/opt/ros/eloquent/setup.bash')
    source_envfile('install/setup.bash')


init_env()


def run_cmd(cmd_str, env, output=None):
    proc = subprocess.Popen(shlex.split(cmd_str), env=env, stdout=output)
    proc.communicate()


class PerfRecordProcess(Process):
    manager = Manager()

    @classmethod
    def get_process_by_name(cls, proc_name):
        for p in psutil.process_iter(["name", "exe", "cmdline"]):
            if proc_name in p.info['name']:
                return p
        return None

    def __init__(self, proc_name, interval=2):
        super().__init__(target=self.record_perf)
        self.proc_name = proc_name
        self.is_recording = False
        self.interval = interval
        self.perf_results = PerfRecordProcess.manager.list()

    def terminate(self) -> None:
        self.is_recording = False
        with open('./{}.perf.log'.format(self.proc_name), 'w') as f:
            content_str = '\n'.join(['{0}\t{1}'.format(*t) for t in self.perf_results])
            f.write(content_str)

        # terminate the process until recording is finished
        print('finished')
        super().terminate()

    def start(self) -> None:
        self.is_recording = True
        super().start()

    def record_perf(self):
        print('record perf for ', self.proc_name)
        p = None
        n = 0
        while not p:
            p = PerfRecordProcess.get_process_by_name(self.proc_name)
            sleep(1)
            n += 1
            if n >= 10:
                break
        if not p:
            raise (Exception("Failed to find process by " + self.proc_name))

        while self.is_recording:
            mem_info = p.memory_info()
            rss_mb = round(mem_info.rss / 1024 / 1024, 2)
            cpu_percent = p.cpu_percent()
            print('rss(MiB): {}, cpu%: {}'.format(rss_mb, cpu_percent))
            self.perf_results.append((mem_info.rss, cpu_percent))
            sleep(self.interval)
        print('not recording, exit perf record func')


# Here we run the commands!
lidar_proc = subprocess.Popen(shlex.split('ros2 launch lidarslam lidarslam_eloquent.launch.py'), env=os.environ.copy(),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)

perf_record_proc = PerfRecordProcess('scanmatcher_node')
perf_record_proc.start()

playbag_proc = subprocess.Popen(shlex.split('ros2 bag play -s rosbag_v2 ./dataset/2018-05-18-14-50-12_1.bag',
                                            os.environ.copy()), stdout=subprocess.PIPE)
playbag_proc.communicate()

print('wait another 10 secs for performance monitoring')
sleep(10)
perf_record_proc.terminate()

# Notice: ros2 launch will handle SIGINT(simulate a ctrl-c) and release all resources it occupied.
lidar_proc.send_signal(signal.SIGINT)

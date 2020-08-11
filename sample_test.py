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


mp_manager = Manager()
perf_results = mp_manager.list()


def get_process_until_ready(proc_name):
    for p in psutil.process_iter(["name", "exe", "cmdline"]):
        if proc_name in p.info['name']:
            print(proc_name)
            return p
    return None


# TODO Make record_perf a customized multiprocessing.Process
# When terminate is called, finish the infinite loop and record the result
def record_perf(proc_name):
    print('record perf for ', proc_name)
    p = None
    n = 0
    while not p:
        p = get_process_until_ready(proc_name)
        sleep(1)
        n += 1
        if n >= 10:
            break
    if not p:
        raise (Exception("Failed to find process by " + proc_name))

    while True:
        meminfo = p.memory_info()
        rss_mb = round(meminfo.rss / 1024 / 1024, 2)
        cpu_percent = p.cpu_percent()
        print('rss(MiB): {}, cpu%: {}'.format(rss_mb, cpu_percent))
        perf_results.append((meminfo.rss, cpu_percent))
        sleep(2)


# Here we run the commands!
lidar_proc = subprocess.Popen(shlex.split('ros2 launch lidarslam lidarslam_eloquent.launch.py'), env=os.environ.copy(),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)


perf_record_proc = Process(target=record_perf, args=('scanmatcher_node',))
perf_record_proc.start()

playbag_proc = subprocess.Popen(shlex.split('ros2 bag play -s rosbag_v2 ./dataset/2018-05-18-14-50-12_1.bag',
                                os.environ.copy()), stdout=subprocess.PIPE)
playbag_proc.communicate()

print('wait another 10 secs for performance monitoring')
sleep(10)
perf_record_proc.terminate()

# Notice: ros2 launch will handle SIGINT(simulate a ctrl-c) and release all resources it occupied.
lidar_proc.send_signal(signal.SIGINT)

with open('./scanmatcher_node.perf.log', 'w') as f:
    content_str = '\n'.join(['{0}\t{1}'.format(*t) for t in perf_results])
    f.write(content_str)
print('finished')

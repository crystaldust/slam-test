#!/usr/bin/env python3

import os
import pprint
import shlex
import subprocess
import signal
from multiprocessing import Process, Manager
from time import sleep


def source_envfile(envfile_path):
    command = shlex.split("env -i bash -c 'source {} && env'".format(envfile_path))
    proc = subprocess.Popen(command, stdout = subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.decode('utf-8').partition("=")
        os.environ[key] = value.rstrip()

def init_env():
    source_envfile('/opt/ros/melodic/setup.bash')
    source_envfile('/opt/ros/eloquent/setup.bash')
    source_envfile('install/setup.bash')

init_env()


def get_pid(cmd_filter):
    ps_proc = subprocess.Popen(shlex.split('ps -ef'), stdout=subprocess.PIPE)
    grep_proc = subprocess.Popen(shlex.split('grep {}'.format(cmd_filter)), stdin=ps_proc.stdout, stdout=subprocess.PIPE)
    grep_v_proc = subprocess.Popen(shlex.split('grep -v grep'), stdin=grep_proc.stdout, stdout=subprocess.PIPE)
    awk_proc = subprocess.Popen(shlex.split('awk "{print $2}"'), stdin=grep_v_proc.stdout, stdout=subprocess.PIPE)

    pid = awk_proc.stdout.readline().decode('utf-8').strip()
    awk_proc.communicate()
    return pid


def run_cmd(cmd_str, env, output=None):
    proc = subprocess.Popen(shlex.split(cmd_str), env=env, stdout=output)
    proc.communicate()


# pipe to subprocessing.PIPE will hide the 'SIGINT' handling error on console screen
lidar_proc = subprocess.Popen(shlex.split('ros2 launch lidarslam lidarslam_eloquent.launch.py'), env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
def run_lidar_cmd():
    lidar_proc.communicate()


mp_manager = Manager()
perf_results = mp_manager.list()
def parse_top_result(pid):
    print('parse top result for ', pid)
    proc = subprocess.Popen(shlex.split('top -b -d 2 -p ' + pid), stdout=subprocess.PIPE)

    while True:
        output = proc.stdout.readline().decode('utf-8').strip()
        if output.find(pid) == 0:
            parts = output.split()
            res_mem, cpu_percentage = parts[5], parts[8]
            print('res(KiB): {}, res(MiB): {}, cpu%: {}'.format(res_mem, round(float(res_mem)/1024, 2), cpu_percentage))
            perf_results.append([res_mem, cpu_percentage])

# Here we run the commands!
slam_proc = Process(target=run_lidar_cmd)
slam_proc.start()
sleep(2)

top_proc = Process(target=parse_top_result, args=(get_pid('scanmatcher_node'),))
top_proc.start()

playbag_proc = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 ./dataset/2018-05-18-14-50-12_1.bag', os.environ.copy()))
playbag_proc.start()
playbag_proc.join()

print('wait 10 secs for top command')
sleep(10)
top_proc.terminate()

# Notice: the ros2 launch will handle SIGINT(simulate a ctrl-c) and release all resources it occupied.
slam_proc.terminate()
lidar_proc.send_signal(signal.SIGINT)


with open('./perf.log', 'w') as f:
    content_str = '\n'.join(['{0}\t{1}'.format(*t) for t in perf_results])
    f.write(content_str)
print('finished')

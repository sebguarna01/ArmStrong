#!/usr/bin/env python3

import yourdfpy
import xacro
import os
import time
import multiprocessing

file_name = "/home/hamoudy/catkin_ws/src/rover_control/rover_description/urdf/rover/2024_rover.xacro"


def show():
    robot = yourdfpy.URDF.load("/tmp/rover.urdf")
    robot.show()

class FileWatcher(object):

    def __init__(self):
        self._cached_stamp = 0
        self.thread= None

    def watch(self, file):
        stamp = os.stat(file).st_mtime
        if stamp != self._cached_stamp:
            self._cached_stamp = stamp

            urdf = xacro.process_file(file)
            with open("/tmp/rover.urdf", "w") as f:
                f.write(urdf.toxml())
            
            if self.thread is not None:
                self.thread.terminate()
            self.thread = multiprocessing.Process(target=show)
            self.thread.start()


if __name__ == "__main__":
    watcher = FileWatcher()

    while True:
        watcher.watch(file_name)
        time.sleep(1)
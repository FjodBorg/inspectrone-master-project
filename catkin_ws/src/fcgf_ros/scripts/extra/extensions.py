#!/usr/bin/env python3.7
import time
import rospy


class PerformanceMetrics():
    def __init__(self, parent=None):
        self.timings = dict()

    def start_time(self, name):
        rospy.loginfo(name)
        cur_time = time.time()
        self.timings[name] = - cur_time  # negative means it hasen't got the second timing yet

    def stop_time(self, name):
        cur_time = time.time()
        elapsed_time = self.timings[name]

        self.timings[name] = cur_time + elapsed_time  # eleapsed time is negative

    def print_time(self, names):

        rospy.loginfo("printing metrics for: " + str(names))
        for name in names:
            print("{:30s} took: {:2.5f} sec".format(name, self.timings[name]))

    def print_all_timings(self):
        names = [name for name in self.timings]
        self.print_time(names)

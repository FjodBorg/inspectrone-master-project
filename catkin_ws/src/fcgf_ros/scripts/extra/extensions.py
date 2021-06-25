#!/usr/bin/env python3.7
import time
import rospy


class PerformanceMetrics():
    def __init__(self, parent=None):
        self.timings = dict()
        self.id = 0
        self.f = open("timings.txt", "w")

    def start_time(self, name):
        self.id += 1
        cur_time = time.time()

        if name in self.timings:
            self.timings[name].append([self.id, -cur_time]) # negative means it hasen't got the second timing yet
        else:
            self.timings[name] = [[self.id, -cur_time]]  # negative means it hasen't got the second timing yet

    def stop_time(self, name):
        cur_time = time.time()
        for value in self.timings[name]:
            if value[1] < 0.0:  # if cur_time < 0
                elapsed_time = value[1]
                value[1] = cur_time + elapsed_time  # eleapsed time is negative
                break

    def _print_time(self, names):

        rospy.loginfo("printing metrics for: " + str(names))
        for name in names:
            values = self.timings[name]
            for index, t in values:
                print("# {:3d}:  {:30s} took: {:2.5f} sec".format(index, name, t))

    def print_all_timings(self):
        # flat out nested lists
        names = [name for name in self.timings]
        
        self._print_time(names)

    def save_all_timings(self, fitness):
        # flat out nested lists
        names = [name for name in self.timings]
        string = "\n\n####\n{:2.5f}".format(fitness)
        for name in names:
            values = self.timings[name]
            for index, t in values:
                string = string + "\n# {:3d}:  {:30s} took: {:2.5f} sec".format(index, name, t)

        self.f.write(string)

    def reset(self):
        self.id = 0
        self.timings.clear()

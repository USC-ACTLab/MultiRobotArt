import rclpy

class TimeHelper:

    def __init__(self, node):
        self.node = node
        # self.rosRate = None
        self.rateHz = None
        self.nextTime = None
        self.zeroTime = self.time()
        # self.visualizer = visNull.VisNull()

    def time(self):
        """Return current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        start = self.time()
        end = start + duration
        while self.time() < end:
            pass
            # rclpy.spin_once(self.node, timeout_sec=0)

    def sleepForRate(self, rateHz):
        """Sleep so that, if called in a loop, executes at specified rate."""
        # Note: The following ROS 2 construct cannot easily be used, because in ROS 2
        #       there is no implicit threading anymore. Thus, the rosRate.sleep() call
        #       is blocking. Instead, we simulate the rate behavior ourselves.
        # if self.rosRate is None or self.rateHz != rateHz:
        #     self.rosRate = self.node.create_rate(rateHz)
        #     self.rateHz = rateHz
        # self.rosRate.sleep()
        if self.nextTime is None or self.rateHz != rateHz:
            self.rateHz = rateHz
            self.nextTime = self.time() + 1.0 / rateHz
        while self.time() < self.nextTime:
            pass
            # rclpy.spin_once(self.node, timeout_sec=0)
        self.nextTime += 1.0 / rateHz

    def sleepUntil(self, end_time):
        while self.time() - self.zeroTime < end_time:
            pass
            # rclpy.spin_once(self.node, timeout_sec=0)

    def isShutdown(self):
        """Return True if the script should abort, e.g. from Ctrl-C."""
        return not rclpy.ok()
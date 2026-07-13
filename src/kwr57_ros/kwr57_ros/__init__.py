"""ROS 2 wrapper package for the KWR57 6-axis F/T sensor.

This package is a thin rclpy layer on top of the standalone ``kwr57_sensor``
Python library (repo root). It does not re-implement the CAN protocol; it
imports the verified driver and publishes ``geometry_msgs/WrenchStamped``.
"""

#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import time
import pigpio

from rclpy.parameter import Parameter

class RangePublisher(Node):
   def __init__(self):
      """
      Initializes the ROS2 node for publishing ultrasonic sensor range data.
      Sets up GPIO pins, initializes the pigpio library, and creates publishers.
      """
      super().__init__('range_publisher')
    
      self.declare_parameters(
         namespace='',
         parameters=[
            ('ultrasonic_frequency', Parameter.Type.DOUBLE),
            ('view_angle', Parameter.Type.DOUBLE), #in degrees for visualize in rviz2
            ('min_range', Parameter.Type.DOUBLE),
            ('max_range', Parameter.Type.DOUBLE),
            ('front_left.topic', Parameter.Type.STRING), #in degrees for visualize in rviz2
            ('front_left.Trigger_pin', Parameter.Type.INTEGER),
            ('front_left.Echo_pin', Parameter.Type.INTEGER),
            ('front.topic', Parameter.Type.STRING),
            ('front.Trigger_pin', Parameter.Type.INTEGER),
            ('front.Echo_pin', Parameter.Type.INTEGER),
            ('front_right.topic', Parameter.Type.STRING),
            ('front_right.Trigger_pin', Parameter.Type.INTEGER),
            ('front_right.Echo_pin', Parameter.Type.INTEGER)
      ])

      freq_ = self.get_parameter('ultrasonic_frequency').value
      range0_topic = self.get_parameter('front_left.topic').value
      range1_topic = self.get_parameter('front.topic').value
      range2_topic = self.get_parameter('front_right.topic').value

      self.field_of_view = self.get_parameter('view_angle').value
      self.min_range = self.get_parameter('min_range').value
      self.max_range = self.get_parameter('max_range').value

      self.range0_publisher = self.create_publisher(Range, range0_topic, 10)
      self.range1_publisher = self.create_publisher(Range, range1_topic, 10)
      self.range2_publisher = self.create_publisher(Range, range2_topic, 10)
      self.timer = self.create_timer(1/freq_, self.publish_range_data) # Timer callback every 65ms

      # Define GPIO pins for Trigger and Echo signals
      self.Trigger_0_pin = self.get_parameter('front_left.Trigger_pin').value
      self.Echo_0_pin = self.get_parameter('front_left.Echo_pin').value

      self.Trigger_1_pin = self.get_parameter('front.Trigger_pin').value
      self.Echo_1_pin = self.get_parameter('front.Echo_pin').value

      self.Trigger_2_pin = self.get_parameter('front_right.Trigger_pin').value
      self.Echo_2_pin = self.get_parameter('front_right.Echo_pin').value

      # Initialize pigpio instance
      self.pi = pigpio.pi()
      
      # Create ultrasonic sensor readers
      self.sonar0 = reader(self.pi, self.Echo_0_pin)
      self.sonar1 = reader(self.pi, self.Echo_1_pin)
      self.sonar2 = reader(self.pi, self.Echo_2_pin)

      # Set GPIO trigger pins as output
      self.pi.set_mode(self.Trigger_0_pin, pigpio.OUTPUT)
      self.pi.set_mode(self.Trigger_1_pin, pigpio.OUTPUT)
      self.pi.set_mode(self.Trigger_2_pin, pigpio.OUTPUT)
   
   def range_to_msg(self, range):
      """
      Converts a distance measurement into a ROS2 Range message.
      """
      msg = Range()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = "ultrasonic_sensor"
      msg.radiation_type = Range.ULTRASOUND  # Change to Range.INFRARED if using IR sensor
      msg.field_of_view = self.field_of_view  # Field of view in radians
      msg.min_range = self.min_range  # Minimum range in meters
      msg.max_range = self.max_range  # Maximum range in meters
      msg.range = range  # Measured range in meters

      return msg

   def publish_range_data(self):
      """
      Triggers ultrasonic sensors, reads pulse width, and publishes range data.
      """

      try:
         # Send trigger signal to all sensors
         self.pi.gpio_trigger(self.Trigger_0_pin, 10, 1)
         self.pi.gpio_trigger(self.Trigger_1_pin, 10, 1)
         self.pi.gpio_trigger(self.Trigger_2_pin, 10, 1)

         # Convert pulse width to distance in meters
         range0 = self.sonar0.pulse_width() * 0.0001715 # us * 343 /1000000 / 2 (unit in meters)
         range1 = self.sonar1.pulse_width() * 0.0001715 # us * 343 /1000000 / 2 (unit in meters)
         range2 = self.sonar2.pulse_width() * 0.0001715 # us * 343 /1000000 / 2 (unit in meters)

         # Publish range messages
         self.range0_publisher.publish(self.range_to_msg(range0))
         self.range1_publisher.publish(self.range_to_msg(range1))
         self.range2_publisher.publish(self.range_to_msg(range2))

         # For debug
         #self.get_logger().info(f'r0: {round(range0,3)} m, r1: {round(range1,3)} m, r2: {round(range2,3)} m')
      except:
         self.get_logger().warning("can't receive data from range sensor")


class reader:
   """
   A class to read PWM pulses and calculate their frequency
   and duty cycle.  The frequency is how often the pulse
   happens per second.  The duty cycle is the percentage of
   pulse high time per cycle.
   """
   def __init__(self, pi, gpio, weighting=0.0):
      """
      Instantiate with the Pi and gpio of the PWM signal
      to monitor.

      Optionally a weighting may be specified.  This is a number
      between 0 and 1 and indicates how much the old reading
      affects the new reading.  It defaults to 0 which means
      the old reading has no effect.  This may be used to
      smooth the data.
      """
      self.pi = pi
      self.gpio = gpio

      if weighting < 0.0:
         weighting = 0.0
      elif weighting > 0.99:
         weighting = 0.99

      self._new = 1.0 - weighting # Weighting for new reading.
      self._old = weighting       # Weighting for old reading.

      self._high_tick = None
      self._period = None
      self._high = None

      pi.set_mode(gpio, pigpio.INPUT)

      self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

   def _cbf(self, gpio, level, tick):
      """
      Callback function for detecting PWM signal edges.
      """
      if level == 1:

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._period is not None:
               self._period = (self._old * self._period) + (self._new * t)
            else:
               self._period = t

         self._high_tick = tick

      elif level == 0:

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._high is not None:
               self._high = (self._old * self._high) + (self._new * t)
            else:
               self._high = t

   def frequency(self):
      """
      Returns the PWM frequency.
      """
      if self._period is not None:
         return 1000000.0 / self._period
      else:
         return 0.0

   def pulse_width(self):
      """
      Returns the PWM pulse width in microseconds.
      """
      if self._high is not None:
         return self._high
      else:
         return 0.0

   def duty_cycle(self):
      """
      Returns the PWM duty cycle percentage.
      """
      if self._high is not None:
         return 100.0 * self._high / self._period
      else:
         return 0.0

   def cancel(self):
      """
      Cancels the reader and releases resources.
      """
      self._cb.cancel()

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = RangePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

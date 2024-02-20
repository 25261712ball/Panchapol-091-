#!/usr/bin/env python3
import tkinter as tk
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import rospy

class TurtleSimControlApp:
    def __init__(self, master):
        self.master = master
        master.title("TurtleSim Control")

        # Linear and Angular Velocity sliders
        self.linear_vel_label = tk.Label(master, text="Linear Velocity:")
        self.linear_vel_label.pack()
        self.linear_vel_slider = tk.Scale(master, from_=0, to=2, orient=tk.HORIZONTAL)
        self.linear_vel_slider.set(1)
        self.linear_vel_slider.pack()

        self.angular_vel_label = tk.Label(master, text="Angular Velocity:")
        self.angular_vel_label.pack()
        self.angular_vel_slider = tk.Scale(master, from_=0, to=2, orient=tk.HORIZONTAL)
        self.angular_vel_slider.set(1)
        self.angular_vel_slider.pack()

        # Motion control buttons
        self.forward_button = tk.Button(master, text="Forward", command=self.move_forward)
        self.forward_button.pack()

        self.backward_button = tk.Button(master, text="Backward", command=self.move_backward)
        self.backward_button.pack()

        self.left_turn_button = tk.Button(master, text="Turn Left", command=self.turn_left)
        self.left_turn_button.pack()

        self.right_turn_button = tk.Button(master, text="Turn Right", command=self.turn_right)
        self.right_turn_button.pack()

        # Pen control buttons
        self.pen_on_button = tk.Button(master, text="Pen On", command=self.pen_on)
        self.command_publisher = rospy.Publisher("command", String, queue_size=10)
        self.pen_on_button.pack()

        self.pen_off_button = tk.Button(master, text="Pen Off", command=self.pen_off)
        self.command_publisher = rospy.Publisher("command", String, queue_size=10)
        self.pen_off_button.pack()

        # Variable to track whether automatic clearing is enabled
        self.automatic_clearing = False

        # Initialize ROS node
        rospy.init_node('turtle_sim_control_node', anonymous=True)

        # Publisher for controlling robot motion
        self.motion_publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

        # Publisher for pen control
        self.command_publisher = rospy.Publisher("command", String, queue_size=10)

        self.led_pub = rospy.Publisher('led_state', Bool, queue_size=1)

    def move_forward(self):
        linear_velocity = self.linear_vel_slider.get()
        angular_velocity = 0.0
        self.publish_motion(linear_velocity, angular_velocity, "Forward")

    def move_backward(self):
        linear_velocity = -self.linear_vel_slider.get()
        angular_velocity = 0.0
        self.publish_motion(linear_velocity, angular_velocity, "Backward")

    def turn_left(self):
        linear_velocity = self.linear_vel_slider.get()
        angular_velocity = self.angular_vel_slider.get()
        self.publish_motion(linear_velocity, angular_velocity, "Left turn")

    def turn_right(self):
        linear_velocity = self.linear_vel_slider.get()
        angular_velocity = -self.angular_vel_slider.get()
        self.publish_motion(linear_velocity, angular_velocity, "Right turn")

    def publish_motion(self, linear_velocity, angular_velocity, command_string):
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.motion_publisher.publish(cmd)

        # Publish command string
        self.command_publisher.publish(command_string)

    def pen_on(self):
        self.automatic_clearing = False  # Stop automatic clearing when Pen On is pressed
        self.publish_motion(0.0, 0.0, "PenOn")
        self.led_pub.publish(Bool(True))

    def pen_off(self):
        self.publish_motion(0.0, 0.0, "PenOff")
        self.led_pub.publish(Bool(False))
        if not self.automatic_clearing:
            self.automatic_clearing = True  # Start automatic clearing when Pen Off is pressed
            self.clear_turtle_canvas()

    def clear_turtle_canvas(self):
        if self.automatic_clearing:
            rospy.wait_for_service('/clear')
            try:
                clear_service = rospy.ServiceProxy('/clear', Empty)
                response = clear_service()
                rospy.loginfo("Cleared the TurtleSim canvas.")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call /clear service: %s" % e)

            # Schedule the next call to clear_turtle_canvas after a delay
            self.master.after(10, self.clear_turtle_canvas)

if __name__ == "__main__":
    root = tk.Tk()
    app = TurtleSimControlApp(root)
    root.mainloop()

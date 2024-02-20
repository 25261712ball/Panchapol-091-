import tkinter as tk
from std_srvs.srv import Empty
import rospy

class TurtleSimControlApp:
    def __init__(self, master):
        self.master = master
        master.title("TurtleSim Control")
        self.pen_is_on = True

        self.pen_on_button = tk.Button(master, text="Pen On", command=self.pen_on)
        self.pen_on_button.pack()

        self.pen_off_button = tk.Button(master, text="Pen Off", command=self.pen_off)
        self.pen_off_button.pack()

        # Variable to track whether automatic clearing is enabled
        self.automatic_clearing = False

    def pen_on(self):
        self.pen_is_on = True
        # Stop automatic clearing when Pen On is pressed
        self.automatic_clearing = False

    def pen_off(self):
        if self.pen_is_on:
            # Start automatic clearing when Pen Off is pressed
            self.automatic_clearing = True
            # Call the clear_turtle_canvas function
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
    rospy.init_node('turtle_sim_control_node', anonymous=True)
    root = tk.Tk()
    app = TurtleSimControlApp(root)
    root.mainloop()

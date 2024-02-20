import tkinter as tk
from std_msgs.msg import Bool
import rospy

class GUIApp:
    def __init__(self, master):
        self.master = master
        master.title("ROSserial GUI")

        self.circle = tk.Canvas(master, width=50, height=50)
        self.circle.create_oval(5, 5, 45, 45, fill="red")
        self.circle.pack(pady=20)

        rospy.init_node('gui_node', anonymous=True)
        rospy.Subscriber('pushed', Bool, self.callback)

    def callback(self, data):
        if data.data:  # Assuming True means the switch is pressed
            self.circle.itemconfig(1, fill="green")  # Index 1 is the oval item in the canvas
        else:
            self.circle.itemconfig(1, fill="red")

if __name__ == "__main__":
    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()

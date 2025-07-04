import rclpy
from rclpy.node import Node
import tkinter as tk
import subprocess
import threading
import os

class InitialPosePromptNode(Node):
    def __init__(self):
        super().__init__('initialpose_prompt_node')
        self.get_logger().info("Initialpose-GUI gestartet.")
        threading.Thread(target=self.show_gui, daemon=True).start()

    def show_gui(self):
        def on_continue():
            self.get_logger().info("Initialpose bestätigt – starte waypoint_follower...")
            window.destroy()
            self.start_waypoint_follower()

        window = tk.Tk()
        window.title("Initialpose setzen")
        window.geometry("400x150")

        label = tk.Label(
            window,
            text="Bitte setzen Sie die Initialpose in RViz,\ndann klicken Sie auf 'Weiter'.",
            font=("Arial", 12),
            justify="center"
        )
        label.pack(pady=20)

        button = tk.Button(window, text="Weiter", command=on_continue, font=("Arial", 10))
        button.pack(pady=10)

        window.mainloop()

    def start_waypoint_follower(self):
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/ros2_ws/install/setup.bash && ros2 run turtlebot3_waypoint_nav waypoint_follower; exec bash'
            ])
        except Exception as e:
            self.get_logger().error(f"Fehler beim Start von waypoint_follower: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePromptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

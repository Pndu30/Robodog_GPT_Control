import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Thread

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit, QLabel
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt

class ControlGUI(Node):
    def __init__(self):
        super().__init__("control_gui")
        self.twist_pub = self.create_publisher(Twist, 'control', 10)
        self.cmd_pub = self.create_publisher(String, 'cmd', 10)

    def send_twist(self, x, y, z):
        t = Twist()
        t.linear.x = x
        t.linear.y = y
        t.angular.z = z
        self.twist_pub.publish(t)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

class CommandTextEdit(QTextEdit):
    def __init__(self, on_enter=None, parent=None):
        super().__init__(parent)
        self.on_enter = on_enter

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            if self.on_enter:
                self.on_enter()
        else:
            super().keyPressEvent(event)


class GUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Robodog Controller")

        self.text_box = CommandTextEdit(on_enter=self.handle_command)
        send_cmd_btn = QPushButton("Send Command")
        send_cmd_btn.clicked.connect(self.handle_command)

        forward_btn = QPushButton('Forward')
        forward_btn.clicked.connect(lambda: self.ros_node.send_twist(0.4, 0.0, 0.0))
        QShortcut(QKeySequence("W"), self, lambda: self.ros_node.send_twist(0.4, 0.0, 0.0))
        
        backward_btn = QPushButton('Backward')
        backward_btn.clicked.connect(lambda: self.ros_node.send_twist(-0.4, 0.0, 0.0))
        QShortcut(QKeySequence("S"), self, lambda: self.ros_node.send_twist(-0.4, 0.0, 0.0))
        
        right_btn = QPushButton('Right')
        right_btn.clicked.connect(lambda: self.ros_node.send_twist(0.0, 0.4, 0.0))
        QShortcut(QKeySequence("A"), self, lambda: self.ros_node.send_twist(0.0, 0.4, 0.0))   
        
        left_btn = QPushButton('Left')
        left_btn.clicked.connect(lambda: self.ros_node.send_twist(0.0, -0.4, 0.0))
        QShortcut(QKeySequence("D"), self, lambda: self.ros_node.send_twist(0.0, -0.4, 0.0))  

        rotate_left_btn = QPushButton('Rotate Left')
        rotate_left_btn.clicked.connect(lambda: self.ros_node.send_twist(0.0, 0.0, 0.4))
        QShortcut(QKeySequence("Q"), self, lambda: self.ros_node.send_twist(0.0, 0.0, 0.4))   
        
        rotate_right_btn = QPushButton('Rotate Right')
        rotate_right_btn.clicked.connect(lambda: self.ros_node.send_twist(0.0, 0.0, -0.4))
        QShortcut(QKeySequence("E"), self, lambda: self.ros_node.send_twist(0.0, 0.0, -0.4))  

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Enter GPT Command:"))
        layout.addWidget(self.text_box)
        layout.addWidget(send_cmd_btn)
        layout.addWidget(QLabel("Manual Controls:"))
        layout.addWidget(forward_btn)
        layout.addWidget(backward_btn)
        layout.addWidget(right_btn)
        layout.addWidget(left_btn)
        layout.addWidget(rotate_right_btn)
        layout.addWidget(rotate_left_btn)

        self.setLayout(layout)


    def handle_command(self):
        cmd = self.text_box.toPlainText()
        self.ros_node.send_command(cmd)


def main():
    rclpy.init()
    ros_node = ControlGUI()

    app = QApplication([])
    gui = GUI(ros_node)
    gui.show()

    Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
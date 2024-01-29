import math
from enum import Enum
from tkinter import Tk, Entry, Button, Text

from matplotlib import pyplot as plt
from matplotlib.patches import Circle

from control import Control


class SET(Enum):
    CENTRE = 1
    RADIUS = 2
    TARGET = 3
    V_MAX = 4
    END = 5


class Interaction:
    def __init__(self, robot_num=10, ax_range=50, time_interval=0.1, length=1000, step=0.2, rate=0.5):
        self.status = SET.CENTRE
        self.fig, self.ax = plt.subplots()
        self.control = Control(ax_range, time_interval, length, step, rate)
        self.robot_num = robot_num
        plt.gca().set_aspect('equal')
        self.ax.set_xlim(0, ax_range)
        self.ax.set_ylim(0, ax_range)
        self.ax.text(0.0, -5.0, "Click the mouse to confirm the center position of the circle")
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.radius = 0.0
        self.tar_x = 0.0
        self.tar_y = 0.0
        self.v_max = 0.0

    def run(self):
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        plt.show()

    def on_submit(self, text, text_box):
        if self.status == SET.V_MAX:
            self.v_max = float(text)
            text_box.destroy()
            self.control.add_robot(self.radius, self.pos_x, self.pos_y, self.tar_x, self.tar_y, self.v_max)
            if len(self.control.robot_list) < self.robot_num:
                self.status = SET.CENTRE
                for text in self.ax.texts:
                    text.remove()
                self.ax.text(0.0, -5.0, "Click the mouse to confirm the center position of the circle")
                plt.draw()
            else:
                self.status = SET.END
                plt.close('all')
                self.control.show()

    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            if self.status == SET.CENTRE:
                self.pos_x = event.xdata
                self.pos_y = event.ydata
                if self.control.check_radius(0.0, self.pos_x, self.pos_y):
                    self.status = SET.RADIUS
                    for text in self.ax.texts:
                        text.remove()
                    self.ax.text(0.0, -5.0, "Click the mouse to confirm the point on the circle.")
                    self.draw_point()
                else:
                    for text in self.ax.texts:
                        text.remove()
                    self.ax.text(0.0, -5.0, "Invalid centre position(collision).\nClick the mouse to confirm the center position of the circle.")
                    plt.draw()
            elif self.status == SET.RADIUS:
                self.radius = math.sqrt((event.xdata - self.pos_x) ** 2 + (event.ydata - self.pos_y) ** 2)
                if self.control.check_radius(self.radius, self.pos_x, self.pos_y):
                    self.status = SET.TARGET
                    for text in self.ax.texts:
                        text.remove()
                    self.ax.text(0.0, -5.0, "Click the mouse to confirm the target.")
                    self.draw_circle()
                else:
                    for text in self.ax.texts:
                        text.remove()
                    self.ax.text(0.0, -5.0, "Invalid radius(collision).\nClick the mouse to confirm the point on the circle.")
                    plt.draw()
            elif self.status == SET.TARGET:
                self.tar_x = event.xdata
                self.tar_y = event.ydata
                self.status = SET.V_MAX
                for text in self.ax.texts:
                    text.remove()
                self.ax.text(0.0, -5.0, "Type in the max velocity.")
                self.draw_arrow()
                self.input_velocity()

    def draw_point(self):
        self.ax.scatter(self.pos_x, self.pos_y, s=5)
        plt.draw()

    def draw_circle(self):
        self.ax.add_patch(Circle((self.pos_x, self.pos_y), radius=self.radius, edgecolor='blue', facecolor='orange', alpha=0.6))
        plt.draw()

    def draw_arrow(self):
        self.ax.scatter(self.tar_x, self.tar_y, s=5)
        self.ax.arrow(self.pos_x, self.pos_y, (self.tar_x - self.pos_x) * 0.98, (self.tar_y - self.pos_y) * 0.98, head_width=0.5, head_length=0.5, fc='red', ec='black')
        plt.draw()

    def input_velocity(self):
        text_box = Tk()
        text = Text(text_box)
        text.pack()
        text.insert("1.0", "Type in the max velocity.")
        entry = Entry(text_box)
        entry.pack()
        Button(text_box, text="confirm", command=lambda: self.on_submit(entry.get(), text_box)).pack()

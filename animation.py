from matplotlib import animation, pyplot as plt, patches


class Animation:
    def __init__(self, robot_list):
        self.ani = None
        self.fig, self.ax = plt.subplots()
        self.robot_list = robot_list
        self.frame_length = len(robot_list[0].trace)

    def update(self, frame):  # update function for drawing each frame
        artists = []
        self.ax.clear()
        for robot in self.robot_list:
            artists.extend(self.ax.plot(robot.x_trace_list[:frame], robot.y_trace_list[:frame]))
            artists.append(self.ax.add_patch(patches.Circle((robot.x_trace_list[frame - 1], robot.y_trace_list[frame - 1]), radius=robot.radius * 0.95, edgecolor='blue', facecolor='orange')))
        self.ax.set_xlim(-10, 20)  # 设置x轴范围
        self.ax.set_ylim(-10, 20)  # 设置y轴范围
        self.ax.set_title('Frame {}'.format(frame))  # 设置标题
        return artists

    def show(self):
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=self.frame_length, interval=1)
        plt.show()

from matplotlib import animation, pyplot as plt, patches


class Animation:
    def __init__(self, robot_list, length, ax_range):
        self.fig, self.ax = plt.subplots()
        self.ani = None
        self.robot_list = robot_list
        self.frame_length = length
        self.ax_range = ax_range

    def update(self, frame):  # update function for drawing each frame
        artists = []
        self.ax.clear()
        for robot in self.robot_list:
            artists.extend(self.ax.plot(robot.x_trace_list[:frame], robot.y_trace_list[:frame], linewidth=0.8))
            artists.append(self.ax.add_patch(patches.Circle((robot.x_trace_list[frame - 1], robot.y_trace_list[frame - 1]),
                                                            radius=robot.radius * 0.95, edgecolor='blue', facecolor='orange', alpha=0.6)))
            artists.append(self.ax.scatter(robot.x_trace_list[frame - 1], robot.y_trace_list[frame - 1], s=5))
            artists.append(self.ax.scatter(robot.pos_tar[0], robot.pos_tar[1], s=5))
            artists.append(self.ax.arrow(robot.x_trace_list[frame - 1], robot.y_trace_list[frame - 1], (robot.pos_tar[0] - robot.x_trace_list[frame - 1]) * 0.98,
                                         (robot.pos_tar[1] - robot.y_trace_list[frame - 1]) * 0.98, head_width=1.0, head_length=1.0, fc='red', ec='black', linestyle=(0, (5, 5)), linewidth=0.6))
        self.ax.set_xlim(-10, self.ax_range + 10)
        self.ax.set_ylim(-10, self.ax_range + 10)
        self.ax.set_title('Frame {}'.format(frame))
        plt.gca().set_aspect('equal')
        return artists

    def show(self):
        plt.close('all')
        self.fig, self.ax = plt.subplots()
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=self.frame_length, interval=1)
        plt.show()

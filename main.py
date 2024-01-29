import matplotlib.pyplot as plt

from interaction import Interaction

# parameter
robot_num = 5
ax_range = 50
time_interval = 0.1
frame_length = 1000
step = 0.2
rate = 0.5

interaction = Interaction(robot_num, ax_range, time_interval, frame_length, step, rate)
interaction.run()
plt.show()

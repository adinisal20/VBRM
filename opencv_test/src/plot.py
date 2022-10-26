import numpy as np
import csv
import matplotlib.pyplot as plt
from numpy import genfromtxt

my_data = genfromtxt('plots.csv', delimiter=',')

b_center_x = []
b_center_y = []
r_center_x = []
r_center_y = []
g_center_x = []
g_center_y = []
m_center_x = []
m_center_y = []


for i in range(len(my_data)):
    r_center_x.append(my_data[i, 0])
    r_center_y.append(my_data[i, 1])
    b_center_x.append(my_data[i, 2])
    b_center_y.append(my_data[i, 3])
    m_center_x.append(my_data[i, 4])
    m_center_y.append(my_data[i, 5])
    g_center_x.append(my_data[i, 6])
    g_center_y.append(my_data[i, 7])



plt.subplot(2, 2, 1)
plt.title("Trajectory for red colour")
plt.xlabel("X coordinate of red circle")
plt.ylabel("Y coordinate of red circle")
plt.plot(r_center_x, r_center_y, color="red")

plt.subplot(2, 2, 2)
plt.title("Trajectory for blue colour")
plt.xlabel("X coordinate of blue circle")
plt.ylabel("Y coordinate of blue circle")
plt.plot(b_center_x, b_center_y, color="blue")

plt.subplot(2, 2, 3)
plt.title("Trajectory for magenta colour")
plt.xlabel("X coordinate of magenta circle")
plt.ylabel("Y coordinate of magenta circle")
plt.plot(m_center_x, m_center_y, color="magenta")

plt.subplot(2, 2, 4)
plt.title("Trajectory for green colour")
plt.plot(g_center_x, g_center_y, color="green")
plt.xlabel("X coordinate of green circle")
plt.ylabel("Y coordinate of green circle")
plt.suptitle("Visual Servoing Observation Chart")
plt.show()

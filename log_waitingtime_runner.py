import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log_waitingtime.txt')
for column in data.T:
  plt.plot(data[:,0], column)

plt.gca().legend(('','No Exit phase', 'rerata Exit Strategy - Fixed Exit Phase', 'rerata Exit Strategy - Exit Phase Dinamis', 'Reinforcement Learning'))
# plt.gca().legend(('','no exit phase', 'exit to fixed phase', 'coord+preempt', 'end dwell', 'dynamic threshold timer', 'reinforcement learning'))

plt.show()





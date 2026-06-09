import numpy as np
from queue import Queue

q = Queue(maxsize=10)
q.put([1,2,3])
q.put([1,2,3])
q.put([1,2,3])
current_data = np.asarray(q.queue)
print(current_data)

np.min(current_data)
np.max(current_data)

print(current_data[:,0])
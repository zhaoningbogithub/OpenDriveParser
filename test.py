import queue

q = queue.PriorityQueue()
q.put(1)
q.put(1)
q.put(2)
q.put(3)
q.put(4)
print(q.get())
print(q.get())
print(q.get())
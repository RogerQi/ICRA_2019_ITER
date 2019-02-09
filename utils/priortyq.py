import numpy as np

def swap(a, b):
    return b, a

def bubble_up(heap, i):                                                                                  
    while i > 0:                                                                                         
        parent_i = (i - 1) >> 1                                                                          
        if heap[i] < heap[parent_i]:
            heap[i], heap[parent_i] = swap(heap[i], heap[parent_i])
            try:
                heap[i].index = i
                heap[parent_i].index = parent_i
            except:
                pass
        else:
            break
        i = parent_i

def heappush(heap, item):
    heap.append(item)
    try:
        item.index = len(heap) - 1
        bubble_up(heap, item.index)
    except:
        bubble_up(heap, len(heap)-1)

def heappop(heap):
    heap[0], heap[-1] = swap(heap[0], heap[-1])
    try:
        heap[0].index = 0
    except:
        pass
    ret = heap.pop()
    i = 0
    while i < len(heap):
        p1 = (i << 1) + 1
        p2 = (i << 1) + 2
        if p1 >= len(heap):
            break
        elif p2 >= len(heap):
            if heap[i] > heap[p1]:
                heap[i], heap[p1] = swap(heap[i], heap[p1])
                try:
                    heap[i].index = i
                    heap[p1].index = p1
                except:
                    pass
                i = p1
            else:
                break
        else:
            tmp = [i, p1, p2]
            idx = np.argmin([heap[i], heap[p1], heap[p2]])
            new_idx = tmp[idx]
            if i == new_idx:
                break
            heap[i], heap[new_idx] = swap(heap[i], heap[new_idx])
            try:
                heap[i].index = i
                heap[new_idx].index = new_idx
            except:
                pass
            i = new_idx
    return ret

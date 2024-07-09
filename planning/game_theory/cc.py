
import numpy as np 


a = np.random.rand(10, 6)
print(a[a>0].__len__())
print(a[a>0])
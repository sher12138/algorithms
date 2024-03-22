# 快排

## 1.1 快排的基本思想与过程

**分解**：构建哨兵，使得左边的都小于哨兵，右边的都大于哨兵 A[q],, A[:q], A[q+1:]

**解决**：通过递归调用快排，解决左右两边的问题

**合并**：原址排序，不需要额外的空间和合并

```python
def quick_sort(A, p, r):
    if p < r:
        q = partition(A, p, r) # 找一个 q 
        quick_sort(A, p, q-1)  # 递归调用
        quick_sort(A, q+1, r) # 递归调用


def partition(A, p, r):
    x = A[r] 
    i = p - 1   # i 是小于x的已找到且排序的最后一个元素的索引  初始时没有， -1 位置，后续 [i+1,j] 元素一定大于，因为如果小于，就给了，j后面的元素未知。
    for j in range(p, r):
        if A[j] <= x:
            i += 1 
            A[i], A[j] = A[j], A[i]
    A[i+1], A[r] = A[r], A[i+1]  # 哨兵移位
    return i + 1   # 返回哨兵的位置
```

证明：  

**循环不变式**：
    - p<=k<=i A[k] < x, i+1<=k<=j A[k] >= x, k = r A[k] = x, 循环不变式成立
**初始化**：i = p - 1, 没有元素，循环不变式成立

**保持**：如果A[j] <= x, 则交换A[i+1]和A[j]，i += 1, 保持循环不变式. 每次交换 i 更新，新元素值小于x，j更新，新元素值大于x，循环不变式成立

**终止**：j = r, 交换A[i+1]和A[r]，则A[i+1] <= x, A[i+1]的左边都小于x，右边都大于x


## 1.2 快排的性能分析
核心在于划分是否平衡， 子问题的数量决定时间 



## 1.3 随机快排 

增加随机函数，随机选择一个元素作为哨兵，避免最坏情况的发生。

```python
import random
def random_partition(A, p, r):
    i = random.randint(p, r)
    A[i], A[r] = A[r], A[i]  # 随机数数进行替换末尾，而不是直接替换哨兵位置！！！！！！
    return partition(A, p, r)
```

## 1.4 尾递归快排

尾递归优化，减少栈空间的使用

```python
def tail_recursive_quick_sort(A, p, r):
    while p < r:
        q = partition(A, p, r)
        tail_recursive_quick_sort(A, p, q-1)
        p = q + 1
```

## 1.5 三数取中快排

```python
def median_partition(A, p, r):
    mid = (p + r) // 2
    if A[p] > A[mid]:
        A[p], A[mid] = A[mid], A[p]
    if A[p] > A[r]:
        A[p], A[r] = A[r], A[p]
    if A[mid] > A[r]:
        A[mid], A[r] = A[r], A[mid]

    A[mid], A[r-1] = A[r-1], A[mid]
    return partition(A, p, r)
```

## 1.6 区间模糊排序 
要求排序结果满足，存在即可。所以等于重叠区间，可以不排序。
```python
def fuzzy_sort(A, p, r, k):
    if p < r:
        q = partition(A, p, r)
        if q - p > k:
            fuzzy_sort(A, p, q-1, k)
        if r - q > k:
            fuzzy_sort(A, q+1, r, k)
```
## 1.7 基于快排的选择算法

选择第i小的元素，可以通过快排的思想，找到哨兵的位置，然后判断哨兵的位置与i的大小关系，递归调用左边或者右边的快排。

```python
def select(A, p, r, i):
    if p == r:  # 此时 i 必然等于1 
        return A[p] # ? 数组只有一个元素，
    q = partition(A, p, r)  # 分冶 
    k = q - p + 1 # 哨兵的位置前面元素数 哨兵大于的数 
    if i == k:
        return A[q]
    elif i < k:
        return select(A, p, q-1, i)
    else:
        return select(A, q+1, r, i-k)
``` 

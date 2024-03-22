# 堆排序
O（nlgn）
堆排序是一种不稳定的排序算法，时间复杂度为O（nlgn）。

堆：只需要保证父节点与子节点满足一定的关系。
*堆排序一次只能保证拿到一个最值，最大/最小，通过重建堆来获取新的最值。*

优先队列： 小根堆构造优先队列，核心就是不断取出代价小的以满足 优先概率 
堆排序： 通过不断重建堆来获取最值，核心就是不断取出最值以满足排序概率

## 1.1 堆的定义 
堆是具有下列性质的完全二叉树：
* 每个节点的值都大于或等于其左右孩子节点的值，称为大顶堆
* 每个节点的值都小于或等于其左右孩子节点的值，称为小顶堆
* 堆总是一棵完全二叉树，所以堆可以用数组来表示，对于数组中的第i个元素，其左孩子的索引为2i+1，右孩子的索引为2i+2，父节点的索引为(i-1)/2


### 1.1.1 完全二叉树

定义： 除了最后一层，其他层都是满的，最后一层的节点都靠左排列。
数组建立的完全二叉树(堆),叶子节点的下标是从 n/2+1 到n，非叶子节点的下标是从0到n/2-1。

## 1.2 堆的构建与调整

MAX-HEAPIFY ： 用于维护最大堆性质的重要过程，时间复杂度为O（lgn）。
BUILD-MAX-HEAP： 从无序的输入数据数组中构建一个最大堆，时间复杂度为O（n）。
HEAPSORT： 堆排序，时间复杂度为O（nlgn）。
MAX-HEAP-INSERT、HEAP-EXTRACT-MAX、HEAP-INCREASE-KEY、HEAP-MAXIMUM： 用于实现优先队列的基本操作，时间复杂度为O（lgn）。


**MAX-HEAPIFY: 用于维护最大堆性质的重要过程，时间复杂度为O（lgn）。**
输入为一个数组A和一个下标i，通过逐级下降的方式，将i节点的值与其左右孩子节点的值进行比较，找出最大值，然后将最大值与i节点的值进行交换，然后递归调用MAX-HEAPIFY，直到i节点的值大于其左右孩子节点的值。
```python
# 假定A[i]的左右子树都是最大堆，但A[i]可能小于其左右子树的值。从而想办法去更新A[i]的值。
def max_heapify(A, i, heap_size):
    l = 2 * i + 1 
    r = 2 * i + 2 
    largest = i 
    # 找出左右节点中的最大值。
    if l < heap_size and A[l] > A[largest]:
        largest = l
    if r < heap_size and A[r] > A[largest]:
        largest = r
    # 如果最大值不是i节点的值，交换i节点和最大值的值，然后递归调用max_heapify
    if largest != i:
        A[i], A[largest] = A[largest], A[i]
        max_heapify(A, largest, heap_size)  # 换给那个索引，则修正那颗子树
    
```
**建堆** -- O(n) 紧确界
自底向上方法：已知MAX-HEAPIFY 可以将一个左右子树合理的二叉树，调整为最大堆。那么，我们可以从最后一个非叶子节点开始，逐个调用MAX-HEAPIFY，直到根节点，就可以将一个无序的输入数据数组构建成一个最大堆。
```python
def build_max_heap(A):
    heap_size = len(A)
    for i in range(heap_size//2 - 1, -1, -1):
        max_heapify(A, i, heap_size)

```
证明：
**循环不变式**：在每次循环开始时，结点i+1 ,... n 都是一个最大堆的根节点。
**初始化**： i = n//2, i+1 ,... 都是叶子节点，叶子节点都是最大堆。

**保持**：结点i的左右子树都是最大堆，调用max_heapify后，结点i+1 ,... n 都是一个最大堆的根节点。

**终止**：i = 0, 每个结点的左右子树都是最大堆，调用max_heapify后，整个树都是最大堆。树的根节点为 1 

## 1.3 堆排序
对与一个长度为n的数组，构建大根堆，通过不断将根节点与最后一个节点交换，然后将堆的大小减1，再调用MAX-HEAPIFY，直到堆的大小为1，就可以得到一个有序的数组。
```python
def heap_sort(A):
    build_max_heap(A)
    heap_size = len(A)
    for i in range(len(A)-1,0,-1):
        A[0], A[i] = A[i], A[0]
        heap_size -= 1 # 不断修正堆的大小
        max_heapify(A, 0, heap_size)
```

# 优先队列 

## 2.1 优先队列的定义
优先队列是一种用来维护由一组元素构成的集合S的数据结构，其中每个元素都有一个相关的值，称为关键字（key）。一个最大优先队列支持以下操作：
* INSERT(S, x)：将元素x插入集合S中。 增加关键字 
* MAXIMUM(S)：返回S中具有最大关键字的元素。 查找关键字 
* EXTRACT-MAX(S)：去掉并返回S中具有最大关键字的元素。  删除并弹出关键字 
* INCREASE-KEY(S, x, k)：将元素x的关键字值增加到k，这里假设k的值不小于x的原关键字值。  修改关键字 

**EXTRACT-MAX**:
从最大堆中去掉并返回具有最大关键字的元素。

```python
def heap_extract_max(A):
    if len(A) < 1:
        raise ValueError('heap underflow')
    max = A[0]
    A[0] = A[-1]
    A.pop()
    max_heapify(A, 0, len(A))
    return max
```


**INCREASE-KEY**:
```python 

# 仅增加关键字，如果是减小，应该还需要往子节点调整
def heap_increase_key(A, i, key):
    if key < A[i]:
        raise ValueError('new key is smaller than current key')
    A[i] = key 
    while i > 0 and A[i//2] < A[i]:
        A[i], A[i//2] = A[i//2], A[i]
        i = i//2
```

**INSERT-KEY**:
```python
# 利用heap_incrrease_key 来实现插入
def max_heap_insert(A, key):
    A.append(float('-inf'))
    heap_increase_key(A, len(A)-1, key)
```


# young 矩阵 

## 3.1 young 矩阵的定义
young 矩阵是一种特殊的矩阵，它的每一行和每一列都是按照升序排列的。young 矩阵的一个重要特性是，矩阵的最小值总是在矩阵的左下角，最大值总是在矩阵的右上角。

## 3.2 young 矩阵的构建

* EXTRACT-MIN：去掉并返回矩阵中的最小值。 同时维持矩阵性质不变 
* INSERT：将元素x插入矩阵中。 同时维持矩阵性质不变 


**EXTRACT-MIN**  已知最小值是位于位置 O（1，1） 删完以后，需要调整矩阵的性质。
```python
# 用户维护当元素 i,j 不满足性质时，调整矩阵的性质
def adjust_matrix(A, i, j):
    m, n = len(A), len(A[0])
    # 递归写法，先找到下一阶替换的值 
    replace_i, replace_j = i, j
    if i+1 < m and A[i+1][j] < A[replace_i][replace_j]:
        replace_i, replace_j = i+1, j
    if j+1 < n and A[i][j+1] < A[replace_i][replace_j]:
        replace_i, replace_j = i, j+1
    
    if replace_i != i or replace_j != j:
        A[i][j], A[replace_i][replace_j] = A[replace_i][replace_j], A[i][j]
        adjust_matrix(A, replace_i, replace_j)

def extract_min(A):
    min = A[0][0]
    A[0][0] = float('inf')
    adjust_matrix(A, 0, 0)
    return min


```

**INSERT**  已知最大值是位于位置 O（m，n） 插入以后，需要调整矩阵的性质。
只能是插在最大值的右下角，然后逐步调整，调整方式应该是从右下角开始逐层网上，每个节点同时会成为两个方向的子节点，挑选更大的记性替换，一直替换到无法找到更大的

```python

def insert(A, x):
    m, n = len(A), len(A[0])
    if A[m-1][n-1] < x:
        raise ValueError('the matrix is full')
    A[m-1][n-1] = x
    i, j = m-1, n-1
    while i > 0 and j > 0:
        if A[i][j-1] > A[i-1][j]:
            if A[i][j-1] > A[i][j]:
                A[i][j], A[i][j-1] = A[i][j-1], A[i][j]
                j -= 1
        else:
            if A[i-1][j] > A[i][j]:
                A[i][j], A[i-1][j] = A[i-1][j], A[i][j]
                i -= 1
    while i > 0:
        A[i][j], A[i-1][j] = A[i-1][j], A[i][j]
        i -= 1
    while j > 0:
        A[i][j], A[i][j-1] = A[i][j-1], A[i][j]
        j -= 1
```


## 3.3 young 矩阵的查找
判断一个给定的数是否在young矩阵中,
    - 如果从左上角，那么两条路都能走，
    - 如果从左下角或者右上角，大于/小于都只能走一条路，于是 O（m+n）
```python
def find(A,x):
    m, n = len(A), len(A[0])
    i, j = 0, n-1   # 右上角开始 
    while i < m and j >=0:
        if A[i][j] == x:return i,j 
        elif A[i][j] > x: j -= 1 
        else: i += 1
    return -1,-1
```


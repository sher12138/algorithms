'''
Segment_tree creates a segment tree with a given array and function,
allowing queries to be done later in log(N) time
function takes 2 values and returns a same type value
'''
class SegmentTree:
    def __init__(self,arr,function):
        self.segment = [0 for x in range(3*len(arr)+3)]
        self.arr = arr
        self.fn = function
        self.make_tree(0,0,len(arr)-1)

    def make_tree(self,i,l,r):
        if l==r:
            self.segment[i] = self.arr[l]
        elif l<r:
            self.make_tree(2*i+1,l,int((l+r)/2))
            self.make_tree(2*i+2,int((l+r)/2)+1,r)
            self.segment[i] = self.fn(self.segment[2*i+1],self.segment[2*i+2])

    def __query(self,i,L,R,l,r):
        if l>R or r<L or L>R or l>r:
            return None
        if L>=l and R<=r:
            return self.segment[i]
        val1 = self.__query(2*i+1,L,int((L+R)/2),l,r)
        val2 = self.__query(2*i+2,int((L+R+2)/2),R,l,r)
        print(L,R," returned ",val1,val2)
        if val1 != None:
            if val2 != None:
                return self.fn(val1,val2)
            return val1
        return val2
        

    def query(self,L,R):
        return self.__query(0,0,len(self.arr)-1,L,R)

'''
Example -
mytree = SegmentTree([2,4,5,3,4],max)
mytree.query(2,4)
mytree.query(0,3) ...

mytree = SegmentTree([4,5,2,3,4,43,3],sum)
mytree.query(1,8)
...

'''


"""
每个节点表示一个区间，叶子节点表示单位区间
非叶子节点[a,b]  --> [a,(a+b) /2 ], [(a+b) /2 +1, b]  二分的方式获取子节点 
同层节点代表区间相互不会重叠 
加起来连续 

4倍内存空间占用 --> 4n + 1 

"""

class SegmentTree:
    def __init__(self,arr,function) -> None:
        self.fn = function
        self.arr = arr 
        self.segment = [0 for x in range(4*len(arr)+1)]
        self.make_tree(0,0,len(arr)-1)
    
    def make_tree(self,i,l,r):  # 
        if l == r:
            self.segment[i] = self.arr[l]
        else:
            self.make_tree(2*i+1,l,int((l+r)/2))
            self.make_tree(2*i+2,int((l+r)/2)+1,r)
            self.segment[i] = self.fn(self.segment[2*i+1],self.segment[2*i+2]) # 左右节点值的计算父节点值 
    
    def __query(self,i,l,r,L,R,):
        # 查询某个区间的值 
        if l>R or r<L or L>R or l>r:
            return None
        if L >= l and R <= r:  # 查询区间超过了当前节点区间，直接返回当前节点值
            return self.segment[i]
        val1 = self.__query(2*i+1,L,int((L+R)/2),l,r)  # 判断查询范围L,R是否在 l,r 内，是的话，直接范围节点值，
        val2 = self.__query(2*i+2,int((L+R+2)/2),R,l,r)
        self.segment[i] = self.fn(self.segment[2*i+1],self.segment[2*i+2]) # 查询时更新 父节点值
        if val1 != None:
            if val2 != None:
                self.segment[i] = self.fn(self.segment[2*i+1],self.segment[2*i+2]) # 查询时更新 父节点值
                return self.segment[i]
            self.segment[i] = val1
            return self.segment[i]
        self.segment[i] = val2
        return self.segment[i]

    def query(self,L,R):
        return self.__query(0,0,len(self.arr)-1,L,R)  # 查询区间 L,R的值，从根节点开始查询
    
    # 节点更新：递归查询的过程中，更新节点值, 
    # lazy update: 递归查询的过程中，更新节点值，但是不更新父节点值，等到查询的时候，再更新父节点值
    def update(self,i,val):
        self.arr[i] = val
        self.__update(0,0,len(self.arr)-1,i,val)

    def __update(self,node,l,r,i,val):
        if l == r:
            self.segment[node] = val
        else:
            mid = int((l+r)/2)
            if i <= mid:
                self.__update(2*node+1,l,mid,i,val)
            else:
                self.__update(2*node+2,mid+1,r,i,val)
            # self.segment[node] = self.fn(self.segment[2*node+1],self.segment[2*node+2])
    


# 二分查找 

"""
1. 有序结构 

2. 二分查找的模板
"""

# pattern one 
# while 可相等，意味着左右都可能成为mid， 因此，mid 需要+、- 1
def binary_search(nums, target):
    left, right = 0, len(nums) - 1
    while left <= right:
        mid = left + (right - left) // 2
        if nums[mid] == target:
            return mid
        elif nums[mid] < target:
            left = mid + 1
        else:
            right = mid - 1
    return -1

# pattern two 
# while 不可相等，意味着right 永远不可能成为mid，因此，mid 不需要 -1 
def binary_search(nums, target):
    left, right = 0, len(nums)
    while left < right:
        mid = left + (right - left) // 2
        if nums[mid] == target:
            return mid
        elif nums[mid] < target:
            left = mid + 1
        else:
            right = mid
    return -1

# 
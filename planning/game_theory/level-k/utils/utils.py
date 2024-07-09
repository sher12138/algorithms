class Math:
    @staticmethod
    def standard(level_k):
        # 基于给的概率，归一化为概率和为1 
        sum_p = sum(level_k)
        for i in range(len(level_k)):
            level_k[i] /= sum_p
        return level_k

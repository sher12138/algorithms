# 该脚本基于正则表达式开发，用于读取和解析planning日志
## 相关依赖
- 请确保python环境已安装matplotlib，若未安装，输入下列指令安装matplotlib
    - `pip3 install matplotlib`
## 运行方式
- `python3 planning_log_parse.py --log [日志文件] --rule [规则文件] --func [画图方式]`
## 运行示例
-  `python3 planning_log_parse.py --log planning.spd.log.INFO.20230606-103645.573767 --rule rule.txt --func 3`
## 规则文件
- 规则字段从第一行开始向下读取，直到空白行为止
- 第一行用于变量名解析
    - 这一行需要将所有想要读取的变量名全部替换成`(.*?)`
- 后续行用于变量数据解析
    - 后续行每一行只读取一个变量值，想要读取的数值替换成`(.*?)`，其余数值替换成`.*?`、
- 涉及正则表达的相关字符需要在该字符前加`\`，如`[`需要替换成`\[`
- 示例参考rule.txt
## 画图方式
1. 所有变量的时序图
2. 用户自己选择的部分变量的时序图
3. 用户自己选择的两个变量组成的二维状态空间图
## [使用示例](https://r3c0qt6yjw.feishu.cn/docx/QfgJdedEAoExQyx7gZ3cf5tlnmd)

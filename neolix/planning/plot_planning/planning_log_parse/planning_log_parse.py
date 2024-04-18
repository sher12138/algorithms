# 内置库
import argparse
import re
# 外部库
import matplotlib.pyplot as plt


class Parse:
    def __init__(self, log_address, rule_address):
        self.log_address = log_address
        self.rule_address = rule_address
        self.vars_re = []
        self.vars_list = []
        self.vars_size = []
        self.vars_name = []

    def read_rule(self):
        rule_address = self.rule_address
        with open(rule_address, "r") as f:
            while True:
                read_in = f.readline().strip()
                if read_in != "":
                    var_re = re.compile(read_in)
                    self.vars_re.append(var_re)
                else:
                    break

    def parse_log(self):
        log_address = self.log_address
        with open(log_address, "r") as f:
            log = f.read()
        # rule.txt第一行，确定想要观察的变量名
        self.vars_name = list(self.vars_re[0].findall(log)[0])

        # rule.txt剩下行，摘出每个变量对应的数据，存入list
        for i in range(1, len(self.vars_re)):
            var_re = self.vars_re[i]
            var_list = [float(i) for i in var_re.findall(log)]
            var_size = len(var_list)
            print("{} size : {} ".format(self.vars_name[i - 1], var_size))
            self.vars_list.append(var_list)
            self.vars_size.append(var_size)

    def plot_all_over_time(self):
        plt.grid()
        for i in range(len(self.vars_list)):
            time = list(range(self.vars_size[i]))
            plt.plot(
                time, self.vars_list[i], label='var {} - {}'.format(i + 1, self.vars_name[i]))
        plt.xlabel("times")
        plt.legend()
        plt.show()

    def plot_chosen_over_time(self):
        print("\nAll variables: ")
        for i in range(len(self.vars_name)):
            index = i + 1
            name = self.vars_name[i]
            print("\t{} - {}".format(index, name))
        input_str = input(
            "Please enter the variables index (separate with spaces):\n")
        interest_index_list = [int(i) - 1 for i in input_str.split(" ")]
        plt.grid()
        for i in range(len(self.vars_list)):
            if i in interest_index_list:
                time = list(range(self.vars_size[i]))
                plt.plot(
                    time, self.vars_list[i], label='var {} - {}'.format(i + 1, self.vars_name[i]))
        plt.xlabel("times")
        plt.legend()
        plt.show()

    def plot_choose_two(self):
        print("\nAll variables: ")
        for i in range(len(self.vars_name)):
            index = i + 1
            name = self.vars_name[i]
            print("\t{} - {}".format(index, name))
        input_str = input(
            "Please enter 2 variables index (separate with spaces):\n")
        interest_index_list = [int(i) - 1 for i in input_str.split(" ")]
        index1 = interest_index_list[0]
        index2 = interest_index_list[1]
        list1 = self.vars_list[index1]
        list2 = self.vars_list[index2]
        plt.grid()
        plt.plot(list1, list2, "-*")
        plt.xlabel("{}".format(self.vars_name[index1]))
        plt.ylabel("{}".format(self.vars_name[index2]))
        plt.show()

    def plot_choose_twos(self):
        print("\nAll variables: ")
        for i in range(len(self.vars_name)):
            index = i + 1
            name = self.vars_name[i]
            print("\t{} - {}".format(index, name))
        all_list_tuple = []
        all_index_tuple = []
        while True:
            input_str = input(
                "Please enter 2 variables index (separate with spaces):\n")
            if input_str == "":
                break
            interest_index_list = [int(i) - 1 for i in input_str.split(" ")]
            index1 = interest_index_list[0]
            index2 = interest_index_list[1]
            list1 = self.vars_list[index1]
            list2 = self.vars_list[index2]
            all_list_tuple.append((list1, list2))
            all_index_tuple.append((index1, index2))
        plt.grid()
        for i in range(len(all_index_tuple)):
            index1 = all_index_tuple[i][0]
            index2 = all_index_tuple[i][1]
            list1 = all_list_tuple[i][0]
            list2 = all_list_tuple[i][1]
            label_str = self.vars_name[index1] + " - " + self.vars_name[index2]
            plt.plot(list1, list2, "-*", label=label_str)
        plt.legend()
        # plt.gca().set_aspect(1)
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', type=str, default=None)
    parser.add_argument('--rule', type=str, default="rule.txt")
    parser.add_argument('--func', type=str, default="1")
    args = parser.parse_args()

    log_address = args.log
    rule_address = args.rule
    func = args.func

    # log_address = "planning.spd.log.INFO.20230606-103645.573767"
    # rule_address = "rule.txt"
    # func = "1"
    # print("log_address: ", log_address)
    # print("rule_address: ", rule_address)
    # print("func: ", func)

    # init
    parse_demo = Parse(log_address, rule_address)
    parse_demo.read_rule()
    parse_demo.parse_log()

    while True:
        # use
        if func == "1":
            parse_demo.plot_all_over_time()
        elif func == "2":
            parse_demo.plot_chosen_over_time()
        elif func == "3":
            parse_demo.plot_choose_two()
        elif func == "4":
            parse_demo.plot_choose_twos()
        else:
            print("error func index!")
            break

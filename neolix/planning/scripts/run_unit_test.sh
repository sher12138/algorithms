#!/bin/bash

CURR_DIR=$(dirname $(readlink -f "$0"))
echo "CURR_DIR:" $CURR_DIR

export LD_LIBRARY_PATH=/home/caros/cyberrt/lib:${CURR_DIR}/../../gears/x86_64/lib/:${CURR_DIR}/../../gears/x86_64/lib/tensorrt:${LD_LIBRARY_PATH}
export GLOG_minloglevel=2
export GLOG_log_dir=${CURR_DIR}/../log

echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "GLOG_minloglevel: $GLOG_minloglevel"
echo "GLOG_log_dir: $GLOG_log_dir"
mkdir -p $GLOG_log_dir

#${CURR_DIR}/../build_x86_64/unit_test/planning_test --gtest_print_time --gtest_repeat=1000 ---gtest_filter=PlanningConfigTest.*

echo "${CURR_DIR}/../build_x86_64/unit_test/planning_test/planning_test"
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/planning_test/planning_test 
${CURR_DIR}/../build_x86_64/unit_test/planning_test/planning_test

echo "${CURR_DIR}/../build_x86_64/unit_test/common_test/common_test"
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/common_test/common_test 
${CURR_DIR}/../build_x86_64/unit_test/common_test/common_test


echo "${CURR_DIR}/../build_x86_64/unit_test/aeb_test/aeb_test"
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/aeb_test/aeb_test 
${CURR_DIR}/../build_x86_64/unit_test/aeb_test/aeb_test

echo "${CURR_DIR}/../build_x86_64/unit_test/world_model_test/world_model_test"
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/world_model_test/world_model_test
${CURR_DIR}/../build_x86_64/unit_test/world_model_test/world_model_test

echo "${CURR_DIR}/../build_x86_64/unit_test/navigation_test/navigation_test"
rm -f /home/caros/adu_data/map/topo_map.bin
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/navigation_test/navigation_test
${CURR_DIR}/../build_x86_64/unit_test/navigation_test/navigation_test

#echo "${CURR_DIR}/../build_x86_64/unit_test/planning_rl_test/planning_rl_test"
#gdb --ex=r --args ${CURR_DIR}/../build_x86_64/unit_test/planning_rl_test/planning_rl_test 
#${CURR_DIR}/../build_x86_64/unit_test/planning_rl_test/planning_rl_test

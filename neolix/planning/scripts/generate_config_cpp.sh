#!/bin/bash
set -x
CURR_DIR=`dirname $0`
echo "scripts dir:" ${CURR_DIR}
ARCH_NAME=`uname -p`
if [ ${ARCH_NAME} != "x86_64" ]; then
    exit 
fi
echo "auto generate *.h and *.cpp from json config files"

python3 scripts/json_to_cpp.py conf/fsm_config.json src/planning/config planning
python3 scripts/json_to_cpp.py conf/plan_config.json src/planning/config planning
python3 scripts/json_to_cpp.py conf/planning_research_config.json src/planning/config planning
python3 scripts/json_to_cpp.py conf/scene_special_config.json src/planning/config planning
python3 scripts/json_to_cpp.py conf/world_model_config.json src/planning/world_model/config world_model

python3 scripts/json_to_cpp.py conf/aeb_config.json src/aeb/config aeb
python3 scripts/dynamic_json_to_cpp.py conf/dynamic_aeb_config.json src/aeb/config aeb

python3 scripts/json_to_cpp.py conf/config_server_config.json src/config_server/config config_server

python3 scripts/json_to_cpp.py conf/navigation_config.json src/planning/navigation/config planning

#python3 scripts/json_to_cpp.py conf/topics_config.json src/planning_rl/config planning_rl
python3 scripts/json_to_cpp.py conf/ego_attribuate.json src/planning_rl/config planning_rl
python3 scripts/json_to_cpp.py conf/rl_model_config.json src/planning_rl/config planning_rl
python3 scripts/json_to_cpp.py conf/post_predict.json src/planning_rl/config planning_rl

set +x

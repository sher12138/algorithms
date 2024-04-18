## 1. set up enviroment
## 1.1 set up git hooks
```
sudo apt install clang-format
bash scripts/hooks/setup_git_hooks.sh
```

## 2. compile project
### 2.1 use build.sh to compile
```
bash build.sh build_pc
```

### 2.2 use cross_build.sh to compile
#### 2.2.1 Prepare gears/ and opt/ in the same directory of planning. 
  
Since cross compiling needs to use x86_64 and aarch64 libraries in the x86_64 platform. It need to compile x86_64 and aarch64 depency libraries before cross compiling planning. [x86_64平台交叉aarch64库说明](https://r3c0qt6yjw.feishu.cn/wiki/wikcnCjBp4en41HYDapBKYhisJb)

The directories should be:
```
/home/caros/planning
/home/caros/gears/x86_64
/home/caros/gears/aarch64
/home/caros/x86_64/opt
/home/caros/aarch64/opt
/home/caros/opt (soft link, point to /home/caros/x86_64/opt or /home/caros/aarch64/opt)
```

#### 2.2.2 cross compile planning  

the usage of cross_build.sh is 
```
bash cross_build.sh [x86_64|aarch64|clean] [release|debug] [unit_test_off|unit_test_on]
```

eg. compile x86_64 target in x86_64 platform or compile aarch64 target in aarch64 platform
```
bash cross_build.sh 
```

eg. compile x86_64 target in x86_64 platform and set unit test or debug
```
bash cross_build.sh x86_64 [release|debug] [unit_test_off|unit_test_on]
```
eg. compile and run unit test

Note: since planning data_center need to load config.pb.txt(installed by control project) to initialize, it should compile control project before planning.

```
bash cross_build.sh x86_64 unit_test_on 
./scripts/run_unit_test.sh
```

cross compile aarch64 in x86_64 platform
```
bash cross_build.sh aarch64 [release|debug]
```

## 3. How to launch planning on vehicle;
firstly, check the planning/src/common/planning_gflags.cpp, make sure the config files location is correct.
The default file posiiton is "/home/caros/cyberrt/conf"
and the map file location is "/home/caros/adu_data/map".
Maybe we should mv the maps to cyberrt/conf in the future.

secondly, use
 cyber_launch start planning/launch/planning.launch
to start planning, and use
 cyber_launch stop planning/launch/planning.launch
to stop planning.

thirdly, be aware that planning relies on other modules, such as gateway, perception, prediction, localization, routing, control, canbus. 
If planning cannot work correcyly, check these modules.

## 4. How to launch planning based on cyber_record.
**we can use pnc_record_combiner.py script in planning/scripts folder merge several record files into a complete record file with channels that pnc team interested**
```
./scripts/pnc_record_combiner.py  origin_folder1 origin_forder2 ...
```
firstly use the cyber_recorder command.
  cyber_recorder play -f /Path/to/File

secondly, if we want to debug planning based on bags, we need specific output of the bag.

example1:

cyber_recorder play -f data/bag/20191113182742.record.cyber -c /pnc/control -c /pnc/carstatus -c /pnc/prediction -c /perception/obstacles -c /perception/lms_objects -c /adu/perception/traffic_light -c /openapi/global_state -c /router/routing -k /pnc/planning -k /pnc/decision -k /pnc/refer_line -k /pnc/global_state -l

example2:

cyber_recorder play -f data/bag/20191113182742.record.cyber -k /pnc/planning -k /pnc/decision -k /pnc/refer_line -k /pnc/global_state -l

we need to launch planning_adapter and planning
cyber_launch start planning/launch/planning.launch

gdb command:
gdb --args mainboard -d dag_streaming_planning.dag -p planning -s CYBER_DEFAULT



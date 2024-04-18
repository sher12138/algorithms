#!/bin/bash
# set -x

function print_usage_and_exit() {
  echo -e  "$B $TAIL$TAIL$TAIL$TAIL $E"
  echo -e  "Usage:$G bash $0 $E[x86_64|aarch64|clean] [release|debug] [unit_test_off|unit_test_on]"
  exit 1
}

function clear() {  
  rm -rf build build_x86_64 build_aarch64 *INFO*
  echo -e "$G -> Clear successfully! $TAIL $E"
  exit
}

echo "==== set up base directory variables ===="
SRC_DIR=$(dirname $(readlink -f "$0"))
echo "SRC_DIR:" $SRC_DIR
cd $SRC_DIR/..
WORKSPACE_DIR=`pwd`
echo "WORKSPACE_DIR:" $WORKSPACE_DIR
cd $SRC_DIR

echo "==== set up build variables ===="
ARCH_NAME=`uname -p`
INSTALL_DIR="$WORKSPACE_DIR/opt"
if [ $# -eq 0 ]; then
    echo "not assign target platform, set host arch as default target platform"
    TARGET_PLATFROM=${ARCH_NAME}
elif [ $1 == "x86_64" ] || [ $1 == "aarch64" ]; then
    TARGET_PLATFROM="$1"
elif [ $1 == "clean" ]; then
    clear
else 
    print_usage_and_exit
fi

if [ -d "/home/caros/cyberrt" ] && [ ! -L "/home/caros/cyberrt" ]; then
    echo "/home/caros/cyberrt already exist, use it as default opt"
    INSTALL_DIR="/home/caros/cyberrt"
else
    echo "/home/caros/cyberrt is not a directory!"
    echo "==== set up soft link for cyberrt and opt ===="
    INSTALL_DIR="$WORKSPACE_DIR/${TARGET_PLATFROM}/opt"
    mkdir -p $INSTALL_DIR
    unlink /home/caros/cyberrt 
    unlink /home/caros/opt
    ln -sf $INSTALL_DIR /home/caros/cyberrt
    ln -sf $INSTALL_DIR /home/caros/opt
    ls -l /home/caros/cyberrt
    ls -l /home/caros/opt
fi


echo install_dir: ${INSTALL_DIR}
cd $INSTALL_DIR
source $INSTALL_DIR/setup.bash
export LD_LIBRARY_PATH=${WORKSPACE_DIR}/gears/$TARGET_PLATFROM/lib/:${WORKSPACE_DIR}/gears/$TARGET_PLATFROM/lib/tensorrt:${LD_LIBRARY_PATH}
env |grep LD_LIB
cd $SRC_DIR

BUILD_DIR="${SRC_DIR}/build_${TARGET_PLATFROM}"
BUILD_TYPE="Release"
UNIT_TEST_FLAG="OFF"
echo host_arch_name: ${ARCH_NAME}, target_platform: ${TARGET_PLATFROM}, bild_dir: ${BUILD_DIR}

if [ $# -ge 2 ]; then
    if [ $2 == "unit_test_off" ];then
        UNIT_TEST_FLAG="OFF"
    elif [ $2 == "unit_test_on" ];then
        UNIT_TEST_FLAG="ON"
    elif [ $2 == "release" ];then
        BUILD_TYPE="Release"
    elif [ $2 == "debug" ];then
        BUILD_TYPE="Debug"
    else 
        print_usage_and_exit
    fi
fi
if [ $# -ge 3 ]; then
    if [ $3 == "unit_test_off" ];then
        UNIT_TEST_FLAG="OFF"
    elif [ $3 == "unit_test_on" ];then
        UNIT_TEST_FLAG="ON"
    elif [ $3 == "release" ];then
        BUILD_TYPE="Release"
    elif [ $3 == "debug" ];then
        BUILD_TYPE="Debug"
    else 
        print_usage_and_exit
    fi
fi
echo build_type: ${BUILD_TYPE}, build_unit_test: ${UNIT_TEST_FLAG}

THEAD_NUM=$(nproc)
if [ $THEAD_NUM -eq 1 ]; then
    THEAD_NUM=15
fi

function build() {
  echo "==== start build ===="
  mkdir -p ${BUILD_DIR}
  if [ $ARCH_NAME == "x86_64" ] && [ $TARGET_PLATFROM == "aarch64" ]; then
    cmake   -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
            -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
            -DBUILD_SRC=ON \
            -DBUILD_UNIT_TEST=OFF \
            -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64_tool_chain.cmake \
            -B${BUILD_DIR} -H.
    make -C ${BUILD_DIR} -j${THEAD_NUM} install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  else
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DBUILD_SRC=ON -DBUILD_UNIT_TEST=${UNIT_TEST_FLAG} \
    -B${BUILD_DIR} -H.
    multi_thread_num=${THEAD_NUM}
    if [ "$multi_thread_num" -gt 24 ]; then
        multi_thread_num=24
    fi
    make -C ${BUILD_DIR} -j${multi_thread_num} install  || {
        echo "$R -> Build failed! $E"
        exit 1
    }
  fi
  echo -e "$G -> Build ${BUILD_TYPE} successfully! $TAIL $E"
  return $?
}

function delete_include_lib() {
    rm -f $INSTALL_DIR/lib/libplanning_common.so 
}
 
delete_include_lib

if [ ${ARCH_NAME} == "x86_64" ]; then
    bash scripts/generate_config_cpp.sh
fi
build 
echo -e "$Y -> Everything is done, enjoy! $TAIL $E"
echo -e "==== Build $SRC_DIR is done! ===="

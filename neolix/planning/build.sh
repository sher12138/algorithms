#!/bin/bash
#set -x

SRC_DIR=$(dirname $(readlink -f "$0"))
INSTALL_DIR="$SRC_DIR/../opt/"
BUILD_DIR="${WORKSPACE_DIR}/build"

ARCH_NAME=`uname -p`

function clear(){  
  rm -rf build
  echo -e "$G -> Clear successfully! $TAIL $E"
  return $?
}

function build() {
  mkdir -p build
  cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SRC=ON -DBUILD_UNIT_TEST=OFF \
    -Bbuild -H.

  make -C build -j$(nproc) install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  echo -e "$G -> Build release successfully! $TAIL $E"
  return $?
}

function build_pc() {
  mkdir -p build

  cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SRC=ON -DBUILD_UNIT_TEST=OFF \
  -Bbuild -H.
  make -C build -j$(nproc) install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  echo -e "$G -> Build release successfully! $TAIL $E"
  return $?
}

function build_debug() {
  mkdir -p build
  cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_SRC=ON \
        -DBUILD_UNIT_TEST=OFF \
        -Bbuild -H.
  make -C build -j8 install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  echo -e "$G -> Build Debug successfully! $TAIL $E"
  return $?
}

function build_src() {
  mkdir -p build
  cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SRC=ON -DBUILD_UNIT_TEST=OFF \
  -Bbuild -H.
  make -C build -j8 install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  echo -e "$G -> Build release successfully! $TAIL $E"
  return $?
}

function build_unit_test() {
  mkdir -p build
  cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SRC=OFF -DBUILD_UNIT_TEST=ON \
  -Bbuild -H.
  make -C build -j8 install  || {
    echo "$R -> Build failed! $E"
    exit 1
  }
  echo -e "$G -> Build test successfully! $TAIL $E"
  ./build/unit_test/planning_test
  return $?
}

function print_usage_and_exit() {
  echo -e  "$B $TAIL$TAIL$TAIL$TAIL $E"
  echo -e  "    Usage:$G bash $0 $E[$RW build_option $E] [$RW install_path $E]"
  echo -e  " "
  echo -e  "    [$RW build_option $E] can be:$E"   
  echo -e  "$Y      clear:        remove build cache $E"  
  echo -e  "$Y      build:        build src/unit_tests with release symbols $E"
  echo -e  "$Y      build_pc:     build src/unit_tests with release symbols on x86$E"
  echo -e  "$Y      build_debug:  build src/unit_tests with debug symbols $E"
  echo -e  "$Y      build_src:  build src with release symbols $E"
  echo -e  "$Y      build_unit_test:  build src/unit_tests, run unit_tests with release symbols $E"
  echo -e  "$B $TAIL$TAIL$TAIL$TAIL $E"
  exit -1
}

main() {
  if [ $# -eq 2 ]; then
    INSTALL_DIR=$2
  fi
  source $INSTALL_DIR/setup.bash
  if [ $# -eq 0 ]; then
  print_usage_and_exit
  else
    case $1 in
      clear)
      clear
      ;;      
      build)
      build
      ;;
      build_pc)
      build_pc
      ;;
      build_debug)
      build_debug
      ;;
      build_src)
      build_src
      ;;
      build_unit_test)
      build_unit_test
      ;;
      *)
      echo -e  " Your command is invalid: $R bash $0 $1 $E"
      print_usage_and_exit
      ;;
    esac
  fi
  echo -e "$Y -> Everything is done, enjoy! $TAIL $E"
}

bash ./scripts/generate_config_cpp.sh
main $@  

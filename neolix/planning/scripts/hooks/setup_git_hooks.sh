#!/bin/bash

curr_dir=`dirname $0`
echo "scripts dir:" ${curr_dir}
git_config_dir=${curr_dir}/../../.git
echo "git_config_dir:" ${git_config_dir}
cp ${curr_dir}/pre-commit ${git_config_dir}/hooks/
chmod +x ${git_config_dir}/hooks/pre-commit
ls -la ${git_config_dir}/hooks/pre-commit
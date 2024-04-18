import os
import sys
import json

def analysis_header(data, mem_dir, file_name_ext, class_name):
    header_file = open(os.path.join(mem_dir, f"{file_name_ext}.h"), "w")
    header_file.write("#pragma once\n")

    header_file.write("#include <iostream>\n")

    header_file.write("namespace neodrive {\n")
    header_file.write("namespace prediction {\n")

    header_file.write(f"class {class_name} \n")
    header_file.write("{\n")

    header_file.write("public:\n")
    for key, item in data.items():
        try:
            if isinstance(item, int):
                x = item
            else:
                x = int(eval(item))
            header_file.write(f"static const int {key} = {x}; \n")
        except:
            x = item
            header_file.write(f'std::string {key} = "{x}"; \n')

    header_file.write("public:\n")
    header_file.write(f"static {class_name} *Instance();\n")
    header_file.write(f"static void DeleteInstance();\n")

    header_file.write("private:\n")
    header_file.write(f"{class_name}();\n")
    header_file.write(f"~{class_name}();\n")
    header_file.write(f"{class_name}(const {class_name} &signal);\n")

    header_file.write(f"const {class_name} &operator=(const {class_name} &signal);\n")
    header_file.write("private:\n")
    header_file.write(f"static {class_name} *instance;\n")

    header_file.write("};\n")

    header_file.write("}\n")
    header_file.write("}\n")


def analysis_source(mem_dir, file_name_ext, class_name):
    source_file = open(os.path.join(mem_dir, f"{file_name_ext}.cc"), "w")
    source_file.write(f'#include "{file_name_ext}.h"\n')
    source_file.write("\n")

    source_file.write("namespace neodrive {\n")
    source_file.write("namespace prediction {\n")

    source_file.write(f"{class_name}* {class_name}::instance = new (std::nothrow) {class_name}();")
    source_file.write(f"{class_name}::{class_name}()")
    source_file.write("{}\n")

    source_file.write(f"{class_name}::~{class_name}()")
    source_file.write("{}\n")

    source_file.write(f"{class_name}* {class_name}::Instance()\n")
    source_file.write("{\n")
    source_file.write("if (instance == NULL){\n")
    source_file.write(f"instance = new (std::nothrow) {class_name}();\n")
    source_file.write("}\n")
    # //            if (instance == NULL){
    # //                instance = new (std::nothrow) ScenePostprocessingConfig();
    # //
    # //            }
    source_file.write("return instance;")
    source_file.write("}\n")
    source_file.write(f"void {class_name}::DeleteInstance()\n")
    source_file.write("{\n")
    source_file.write("if (instance){\n")
    source_file.write("delete instance;\n")
    source_file.write("instance = nullptr;\n")
    source_file.write("}\n}\n")


    source_file.write("}\n")
    source_file.write("}\n")




if __name__ == "__main__":
    # json_path = sys.argv[1]
    # mem_dir = sys.argv[2]
    # class_name = sys.argv[3]

    json_path = '/home/caros/github_code/reinforcement_learnning/planning_rl/onboard/common/model_trt/config/config/scene_prediction_preprocessing_config.json'
    mem_dir = '/home/caros/github_code/reinforcement_learnning/planning_rl/onboard/common/model_trt/config/config/'
    class_name = 'preprocess'

    file_name = os.path.split(json_path)[-1]
    file_name_ext = file_name.split(".")[0]
    data = json.load(open(json_path, "r"))

    analysis_header(data, mem_dir, file_name_ext, class_name)
    analysis_source(mem_dir, file_name_ext, class_name)

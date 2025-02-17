from email.mime import base
import os
import glob

# 使用os.path.normpath确保路径中的分隔符正确
# base_path = os.getenv('SCENARIO_RUNNER_ROOT', ".").replace("\\", "/")
# scenarios_list = glob.glob(f"{base_path}/srunner/scenarios/*.py")
# print(base_path,scenarios_list)

base_path = os.getenv('PYTHONPATH', ".")
print(base_path)

import sys
print(sys.prefix)
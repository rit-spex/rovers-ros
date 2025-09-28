import os
import subprocess
import sys


print("Before:", os.getcwd())

home_dir = os.path.expanduser("~")
print(home_dir)

try:
    os.chdir(home_dir+"/SPEX/rovers-ros/src/externalScript/")
except FileNotFoundError as e:
    print(f"Error: {e}")
finally:
    print("Current Directory:", os.getcwd())




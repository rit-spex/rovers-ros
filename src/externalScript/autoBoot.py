import os
import subprocess
import sys


print("Before:", os.getcwd())

try:
    os.chdir("/home/savage22/ros/rovers-ros/")
except FileNotFoundError as e:
    print(f"Error: {e}")
finally:
    print("Current Directory:", os.getcwd())


try:
    scriptLine = ['bash', 'launchRos2']#, 'ros2', 'launch', 'main', 'main_launch.xml']
    result = subprocess.run(scriptLine, capture_output=True, text=True, check=True)
    print(scriptLine)
    print("Script output:")
    print(result.stdout)
    if result.stderr:
        print("Script errors:")
        print(result.stderr)
except subprocess.CalledProcessError as e:
    print(f"Error executing script: {e}")
    print(f"Stderr: {e.stderr}")

#try:
#    os.chdir("/home/savage22/ros/rovers-ros/")
#except FileNotFoundError as e:
#    print(f"Error: {e}")
#finally:
#    print("Current Directory:", os.getcwd())


#try:
#    result = subprocess.run(['bash', 'launchRos2'])
#    print("Script output:")
#    print(result.stdout)
#    if result.stderr:
#        print("Script errors:")
#        print(result.stderr)
#except subprocess.CalledProcessError as e:
#    print(f"Error executing script: {e}")
#    print(f"Stderr: {e.stderr}")




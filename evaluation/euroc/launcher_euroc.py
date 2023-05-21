
# ###
# #! bin/bash
# # Launch main with config file for agents (detached)
# ./cmake-build-debug/main config/euroc_config_agent_0.yaml &
# ./cmake-build-debug/main config/euroc_config_agent_1.yaml &
# ./cmake-build-debug/main config/euroc_config_agent_2.yaml &
# ./cmake-build-debug/main config/euroc_config_agent_3.yaml &
# ./cmake-build-debug/main config/euroc_config_agent_4.yaml &
# # Busy wait
# wait(2)
# # Launch all rosbags
# roslaunch evaluation/full_euroc.launch
# wait(20) # Wait some time after last rosbag stops (agents still optimizing)
# # Send SIGINT to all agents, causes output files to be written to folder named  "agent_<id>"
# pkill main
# # Shut may take a moment.
# wait(10)
# # Move result folders into a timestamped directory.
# mkdir result_<timestamp>
# mv agent_* result/
# ###
import os
import subprocess
import time

def current_time():
    return time.strftime("%H:%M:%S", time.localtime())
def timestamp_folder():
    return time.strftime("%H_%M_%S", time.localtime())


EXECUTABLE_FOLDER = "build"
EXECUTABLE_FILENAME = "main"
EXECUTABLE_PATH = os.path.join(EXECUTABLE_FOLDER,EXECUTABLE_FILENAME)

PLAYER_COMMAND = "roslaunch evaluation/euroc/full_euroc.launch"
CONFIGURATION_FOLDER = "evaluation/euroc/configs"

RESULTS_PATTERN = "agent*"

SLEEP_FINAL_OPTIMIZATION = 2 # After rosbags have been finished, how much time in seconds to wait until killing main(s)
NUM_ITER=2  # Number of iterations to be repeated for each experiment


# Parse configuration folder to extract config files  
experiment_path = os.listdir(CONFIGURATION_FOLDER)[0]

for experiment_name in os.listdir(CONFIGURATION_FOLDER):
    experiment_path = os.path.join(CONFIGURATION_FOLDER, experiment_name)
    config_files = ([file for file in os.listdir(experiment_path) if "config" in file]) # Only capture files with name "*config*"

    for iteration in range(NUM_ITER):
        # Launch main processes
        main_processes =[]
        for config_file in config_files:
            main_cmd = [EXECUTABLE_PATH, os.path.join(experiment_path, config_file)]
            print("Launching main with command:", " ".join(main_cmd))           
            process = subprocess.Popen(main_cmd, shell=False) # If it does not capture ROS env try shell=True
            main_processes.append(process) # Should not print anything

        print(f"Main processes pids: {[main_process.pid for main_process in main_processes]}")
        # Launch data playback
        print(f"Starting player at: {current_time()}")

        player_cmd = PLAYER_COMMAND.split(" ")
        player_process = subprocess.Popen(player_cmd, shell=False)

        print("Waiting player to finish")
        player_process.wait()
        print(f"Finished player at: {current_time()}")


        # Kill processes by PID
        time.sleep(SLEEP_FINAL_OPTIMIZATION)
        print("Send killing signal to main processes")
        for main_process in main_processes:
            kill_command = f"kill -SIGINT {main_process.pid}"
            print(f"Quitting process with {kill_command}")
            subprocess.run(kill_command.split(" "))

        # Wait until all the processes are dead
        print("Waiting main processes to die")
        for main_process in main_processes:
            main_process.wait()

        time.sleep(5) # Safety sleep 

        print("Copying results into folder")
        results_folder_path =  os.path.join(experiment_path, "results_"+timestamp_folder())
        print(f"Creating folder {results_folder_path}")
        os.makedirs(results_folder_path, exist_ok=True)

        results_folder_path+="/" # so that mv understand to dump all files

        move_cmd = f"mv {RESULTS_PATTERN} {results_folder_path}"
        print(f"Moving result files with command: {move_cmd}")
        print(move_cmd.split(" "))
        subprocess.run(move_cmd, shell=True) # THIS IS NOT WORKING ON MY MACHINE -->  TAKE A LOOK

        time.sleep(1) # Just in case

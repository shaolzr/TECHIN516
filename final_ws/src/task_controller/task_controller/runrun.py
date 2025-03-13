import subprocess
import time
import os
import sys

def run_script(script_name, module_name=None):
    if module_name:
        command = ["python3", "-m", module_name]
    else:
        script_path = os.path.join(os.path.dirname(__file__), script_name)
        command = ["python3", script_path]
    
    print(f"Starting {script_name}...")
    result = subprocess.run(command, capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print(f"Error in {script_name}:\n", result.stderr)
    if result.returncode != 0:
        print(f"Error occurred in {script_name}, stopping execution.")
        sys.exit(1)  # Stop execution if a script fails
    print(f"Finished {script_name}.\n")

def main():
    run_script("maze.py")  # Run maze.py first
    time.sleep(2)  # Short delay to ensure transition
    run_script("run.py", "task_controller.run")   # Run run.py as a module
    time.sleep(2)  # Short delay to ensure transition
    run_script("maze1.py") # Run maze1.py last

if __name__ == "__main__":
    main()

import subprocess
import os
import sys
import atexit
import psutil
from colorama import init, Fore, Style

# Initialize colorama
init()

# Get the directory where all user profiles are stored
users_dir = os.environ.get('PUBLIC', 'C:\\Users\\Public')

# Function to kill the subprocess and its children when the program exits
def cleanup():
    if gradle_process and gradle_process.poll() is None:
        parent = psutil.Process(gradle_process.pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()
        parent.wait()
        print("Gradle process and its children killed.")


# Register the cleanup function to be called on exit
atexit.register(cleanup)

# Start the gradlew.bat simulateJava process
gradle_command = f'gradlew.bat simulateJavaRelease -Dorg.gradle.java.home={users_dir}\\wpilib\\2025\\jdk'
print(Fore.YELLOW + Style.BRIGHT + "Starting Maple... Please wait" + Style.RESET_ALL)
gradle_process = subprocess.Popen(gradle_command, shell=True)

# Start AdvantageScope
scope_command = f'"{users_dir}\\wpilib\\2025\\advantagescope\\AdvantageScope (WPILib).exe"'

print(Fore.YELLOW + Style.BRIGHT + "Starting AdvantageScope... Please wait" + Style.RESET_ALL)
advantage_scope_process = subprocess.Popen(scope_command, shell=True)

# Wait for AdvantageScope process to complete
advantage_scope_process.wait()
print(Fore.YELLOW + Style.BRIGHT + "AdvantageScope exited" + Style.RESET_ALL)

# Ensure the script exits cleanly
sys.exit(0)
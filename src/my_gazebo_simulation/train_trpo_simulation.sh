#!/bin/bash
#SBATCH -J trpo_turtlebot_training     # Job name
#SBATCH --account=anjaneyap21          # Replace with your allocation account
#SBATCH --partition=normal_q           # Use the appropriate partition (check system docs)
#SBATCH --nodes=1                      # Number of nodes
#SBATCH --ntasks-per-node=1            # Number of tasks per node (1 task in your case)
#SBATCH --cpus-per-task=4              # Adjust based on available resources and requirements
#SBATCH --gres=gpu:1                   # Request 1 GPU if using GPU parallelization
#SBATCH --time=02:00:00                # Estimated max time (2 hours)
#SBATCH --output=trpo_training_%j.out  # Standard output log file (%j inserts job ID)
#SBATCH --error=trpo_training_%j.err   # Standard error log file

# Load necessary modules
module load python/3.10.12              # Adjust based on your environment
module load pytorch                     # PyTorch module
module load ros/2                       # Load ROS 2 environment
module load gazebo                      # Load Gazebo module

# Activate your virtual environment (if using one)
# source ~/gym_ros_envs/venv/bin/activate

# Navigate to your working directory
cd ~/gym_ros_envs/src/my_gazebo_simulation/

# Step 1: Launch headless Gazebo simulation
echo "Starting headless Gazebo simulation..."
ign gazebo my_custom_world.launch.py --headless &
SIM_PID=$!  # Store the PID to manage the process later

# Allow some time for the simulation to initialize
sleep 30  # Adjust the sleep time based on your environment setup time

# Step 2: Insert your structure into the environment
echo "Inserting structure into the Gazebo environment..."
# Command to insert structure (replace with your specific command or script)
ros2 run your_package insert_structure_node

# Step 3: Run the TRPO training script
echo "Starting TRPO training..."
python train_trpo_turtlebot.py

# Step 4: Cleanup - Stop the Gazebo simulation
echo "Stopping Gazebo simulation..."
kill $SIM_PID  # Terminate the simulation process

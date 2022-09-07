Header header

Int64 robot_number                      # Total Number of Robot
Int64[] robot_position_x                # Initial Position of Robot in x Dimension
Int64[] robot_position_y                # Initial Position of Robot in y Dimension
Int64[] robot_velocity                  # Max Velocity of the Robot
Int64[] robot_battery_level             # Robot Battery Level 
float64[] robot_battery_discharge_rate  # Robot Battery Discharge Rate
Int64[] robot_lambda                    # Robot Usage Penalty (Higher Value means higher penalty)

Int64 task_number                       # Total number of task 
Int64[] task_position_x                 # Initial Position of Task in x Dimension
Int64[] task_position_y                 # Initial Position of Task in y Dimension
Int64[] task_velocity_x                 # Velocity of Task in x dimension
Int64[] task_velocity_y                 # Velocity of Task in y dimension
Int64[] task_priority                   # Task priority level
Int64[] task_operation_time             # Time needed for the robot to finish the task

Int64 time_horizon
# Parsed Tello SDK state, SI units, ROS directions. header.stamp = arrival on the laptop (sim: sim time).
# Fields missing from a firmware are NaN. Axis/sign mapping of vel/acc/roll/pitch is provisional (spec 11.2).
std_msgs/Header header
float32 roll          # rad
float32 pitch         # rad
float32 yaw           # rad, counter-clockwise positive
float32 vel_x         # m/s
float32 vel_y         # m/s
float32 vel_z         # m/s
float32 acc_x         # m/s^2
float32 acc_y         # m/s^2
float32 acc_z         # m/s^2
float32 height        # m
float32 tof           # m
float32 baro          # SDK value, unconverted
float32 battery       # %
float32 temp_low      # degC
float32 temp_high     # degC
float32 motor_time    # s

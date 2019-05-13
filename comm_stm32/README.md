# Install needed software
1. sudo apt-get install ros-kinetic-serial

# Find your device name
1. (Do not plug in your device)
2. ls /dev/tty*
3. (Plug in your device)
4. ls /dev/tty*
5. (Find the extra name, that is your device name)

# Test communicate
1. sudo su
2. (source the setup.bash of your project.)
3. sudo chmod a+x /dev/<Your device name>
4. roslaunch comm_stm32 gripper_control.launch val:=[deiire cmd]
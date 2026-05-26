# Localization

`ros2 launch localization launch.py` will run the launch script at `launch/launch.py` that starts the entire pipeline.
Configurations for the cameras, tagslam, and the ekfs are at `config/*`. These are used when running via the launch script.

This package's main purpose is to be able to host the main launch script of the entire system.

# Debugging

If you want to debug any of the nodes started by the launch script, you can comment them out of the last line of the launch script, run the launch script, and then finally start the node with gdb.

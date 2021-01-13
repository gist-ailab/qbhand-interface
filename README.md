# QBHAND_INTERFACE

- tutorials and examples to control QB softhand with ROS

## To Do

- add roslaunch files and more examples

## Dependencies

Follow the instructions in notion pages: [ur5_setup](https://www.notion.so/UR5-5934848691d64fd584e3331e6da11bb6)


## QB-Softhand

1. GUI interface
```
cd /home/ailab-ur5/Utils/Handtool_basic_unix_v2.2.8 && ./HandTool
```
2. Run controller node
```
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_controller_gui:=true
```
3. run example
```
python src/simple_qbhand_action.py {close or open} {duration, int}
```


## Authors
* **Seunghyeok Back** [seungback](https://github.com/SeungBack)

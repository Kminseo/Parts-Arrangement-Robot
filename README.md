# Parts Arrangement Robot
<img src="https://github.com/yehengchen/video_demo/blob/master/video_demo/grasp_detection.gif" width="80%" height="80%">

## Environment

* ##### Ubuntu 18.04 
* ##### ROS Melodic
* ##### python 2.7


### 실행순서

1. `$ roslaunch gripper_ur5 gazebo_env_setting.launch`
2. `$ roslaunch gripper_ur5_moveit_config moveit_planning_execution.launch`
3. `$ roslaunch yolov3_pytorch_ros detector.launch`
4. `$ rosrun gripper_ur5 robot_sorting_yolo.py`

### Launch 파일 설명

```
$ roslaunch gripper_ur5 gazebo_env_setting.launch
```

`gripper_ur5`패키지의 `gazebo_env_setting.launch`는 Gazebo Simulation환경 구성에 필요한 로봇, 테이블, 주변환경 등 불러오기 위한 launch파일.

```
$ roslaunch gripper_ur5_moveit_config moveit_planning_execution.launch`
```

UR5 제어를 위해  Moveit을 사용하는데, UR5에 맞게 설정된 Moveit 패키지를 불러오기 위해 사용하는 Launch 파일

```
$ roslaunch yolov3_pytorch_ros detector.launch
```

Gazebo 환경에 설치된 카메라를 사용해 yolov3로  Obejct Detection을 하기위한 구성된 패키지이며, `rqt`,`rvize`를 사용해서 환경에 배치된  `screw`3종을 검출 및 분류 하는 것이 가능하다.

```
$ rosrun gripper_ur5 robot_sorting_yolo.py
```

Moveit을 사용해로 로봇을 제어, 환경상의 3가지 종류의 스크류를 생성, yolov3를 포함하며 전체적인 Pick & Place에 대한 동작 코드를 포함합니다.

 

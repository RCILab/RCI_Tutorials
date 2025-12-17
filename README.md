# [This is Example for Readme File]

Short one-line description.  
예: MPC-based Whole-Body Controller for Humanoid Robots.

## Overview
이 저장소는 [로봇 이름/시뮬레이터]에서 [제어 방법]을 구현한 ROS 2 패키지입니다.  
본 연구는 [논문명/프로젝트명]의 일부로 수행되었습니다.

## Dependencies
- ROS 2 Humble (>= 2022.05)
- Python 3.10 / C++17
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) >= 2.6.15
- [acados](https://github.com/acados/acados) >= 0.2.0

설치 예시:
```bash
sudo apt install ros-humble-ros2-control ros-humble-gazebo-ros-pkgs
pip install pinocchio==2.6.15
```

## Installation
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/<org>/<repo>.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
```bash
ros2 launch <package_name> sim.launch.py world:=lab_world
ros2 run <package_name> mpc_controller --ros-args -p horizon:=30
```

## Examples
1. **MPC Tracking** – Humanoid walking in Gazebo  
   ```bash
   ros2 launch mpc_humanoid walking.launch.py
   ```
2. **Impedance Control** – 7-DoF Arm tracking trajectory  
   ```bash
   ros2 launch impedance_control demo.launch.py
   ```

## Citation
If you use this code in your research, please cite:

```bibtex
@inproceedings{kim2025mpc,
  title     = {MPC-based Whole-Body Control for Humanoid Robots},
  author    = {Kim, Sanghyun and Others},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2025}
}
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact
Maintainer: [이름] (<email>)  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)

## colcon 설치
sudo apt install python3-colcon-co

## auto completion 사용하기

![alt text](image.png)

- 환경변수 추가

```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

## workspace 설치하기

1. ros2_ws 만들깅
```bash
cd ~
mkdir ros2_ws
cd ros2_ws
```

2. src 디렉토리 생성하기
```bash
mkdir src
```

3. build하기
```bash
colcon build
```

이 후 log(build process), install, log 디렉토리가 생성된다.

4. install directory의 local_setup.bash를 소스 해줌

```bash
vi ~./zshrc
```

## python Package 생성하기

패키지는 독립된 유닛이며, 코드를 재사용 가능한 블록으로 분리해준다.
예시로 카메라 패키지와 로봇패키지 등이 있다.

1. src 폴더로 이동하기

```bash
cd ~ros2_ws/src
```

2. 파이썬 패키지 생성하기

```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

3. 빌드 방법

```bash
colcon build
```

4. 특정 패키지만 빌드하기

```bash
colcon build --packages-select my-py-pkg
```

3. C++ 패키지 생성하기

```bash
cd src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

## ROS2 - Nodes
다른 노드와 커뮤니케이션 함

- Subprograms in your application, responsible for only one thing
- Combined into a graph
- Communicate with each other thorough topics, services, and parameters

Benfits :
- Reduce code complexity
- Fault tolerance
- Can be written in Python, C++, ...

### 카메라 패키지라면
- 카메라 드라이버
- 이미지 처리

### Motion Planning pkg
- Path correction
- Motion planning

### image Processing node와 Path Correction Node의 통신

### Hardware control
- Drivers
- Main Control loop (Motion Planing 과 통신)
- State Publisher


## 간단한 Python 노드 생성

1. 파일 생성

```bash
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg
touch my_first_node.py
```

2. 파이썬 노드 작성

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
   def __init__(self):
      super().__init__("py_test")  # 노드 이름 설정
      self.counter_ = 0
      self.get_logger().info("Hello ROS2")  # 초기 메시지 출력
      self.create_timer(0.5, self.timer_callback)  # 0.5초마다 callback 호출

   def timer_callback(self):
      self.counter_ += 1
      self.get_logger().info("Hello " + str(self.counter_))  # 카운터를 증가시키며 출력

def main(args=None):
   rclpy.init(args=args)
   node = MyNode()
   rclpy.spin(node)  # 프로그램을 지속적으로 실행, 다른 작업이 없을 경우 유지
   rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
   main()

```
- MyNode 클래스
- Node를 상속하며, ROS 2의 노드를 생성하는 기본 구조입니다.
- self.get_logger().info("string"): 메시지를 콘솔에 출력합니다.
- self.create_timer(0.5, self.timer_callback): 0.5초마다 timer_callback 함수를 호출하는 타이머를 생성합니다. 이는 주파수 2Hz에 해당하며, 초당 2번 호출됩니다.
- rclpy.spin(node): 노드가 지속해서 동작하게 하며, 다른 작업이 없다면 프로그램이 종료되지 않도록 합니다.
- node의 이름에는 “node”라는 단어가 포함되지 않아야 합니다.
- 메인 함수
- rclpy.init(): ROS 2의 초기화 함수로, 노드 실행 준비를 합니다.
- rclpy.shutdown(): ROS 2를 종료합니다.

## 노드 설치 방법

직접 실행 권한을 부여하여 노드를 실행할 수도 있지만, setup.cfg와 setup.py 파일을 사용하여 설치하는 방법이 권장됩니다.

1. setup.cfg

setup.cfg 파일은 설치 디렉토리를 정의합니다. 이를 통해 설치 시 파일의 위치를 지정할 수 있습니다.

[develop]
script-dir=$base/lib/my_py_pkg
[install]
install-scripts=$base/lib/my_py_pkg

- $base/lib/my_py_pkg: 설치 후에 실행 가능한 스크립트가 위치할 디렉토리입니다.

2. setup.py

setup.py는 패키지 메타데이터와 종속성을 설정하며, ROS 2 패키지인 package.xml과 같은 역할을 합니다.

```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='ROS 2 Python Node Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           "py_node = my_py_pkg.my_first_node:main"
        ],
    },
)
```

- entry_points: 패키지 설치 후, py_node 명령을 실행하여 my_first_node.py 파일의 main() 함수를 호출할 수 있도록 합니다.
- py_node = my_py_pkg.my_first_node:main에서 my_py_pkg.my_first_node는 my_py_pkg 패키지 내부의 my_first_node 모듈을 의미하며, main 함수가 실행됩니다.

## 노드 실행

1. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
```

colcon build를 통해 패키지를 빌드합니다. --packages-select 옵션을 사용하여 특정 패키지만 빌드할 수 있습니다.

2. 설치 디렉토리에서 노드 실행

```bash
cd install/my_py_pkg/lib/my_py_pkg
./py_node
```

install/my_py_pkg/lib/my_py_pkg 경로에서 py_node를 직접 실행할 수 있습니다. 이 파일은 entry_points 설정에 의해 자동 생성되었으며, my_first_node.py의 main() 함수를 호출합니다.

3. 다른 터미널에서 노드 실행

```bash
source ~/.bashrc
ros2 run my_py_pkg py_node
```

새로운 터미널에서 ros2 run my_py_pkg py_node 명령을 통해 노드를 실행할 수 있습니다. source ~/.bashrc를 실행하여 ROS 2 설정이 반영된 환경을 사용하도록 합니다.
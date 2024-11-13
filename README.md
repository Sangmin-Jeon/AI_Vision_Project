# 군부대 침입자 추적 로봇 
* 군부대에서 침입자 발생 시 칩입자를 추적하는 로봇

<br>

## 협업 링크
[다이어그램](https://app.diagrams.net/#G1dSkJzltzABwAcWuaZGdPVXZ0AbKhy5dB#%7B%22pageId%22%3A%22P8uuppDY_grhplGPpQES%22%7D)

<br>

## Discription
* Doosan Rokey AI Vision 로봇 팀 프로젝트
* 기술 스택 / Framework
  * Language: Python
  * Framwork: Ros2, Flask
* 구현 내용

<br>

## 프로젝트 구조
~~~bash

├── build
│   ├── COLCON_IGNORE
│   └── b4_package
│       ├── b4_package.egg-info
│       │   ├── PKG-INFO
│       │   ├── SOURCES.txt
│       │   ├── dependency_links.txt
│       │   ├── entry_points.txt
│       │   ├── requires.txt
│       │   ├── top_level.txt
│       │   └── zip-safe
│       ├── build
│       │   └── lib
│       │       └── b4_package
│       │           ├── __init__.py
│       │           ├── argument.py
│       │           ├── security_alert.py
│       │           ├── subscriber_test.py
│       │           └── talker.py
│       ├── colcon_build.rc
│       ├── colcon_command_prefix_setup_py.sh
│       ├── colcon_command_prefix_setup_py.sh.env
│       ├── install.log
│       └── prefix_override
│           ├── __pycache__
│           │   └── sitecustomize.cpython-310.pyc
│           └── sitecustomize.py
├── install
│   ├── COLCON_IGNORE
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── b4_package
│   │   ├── lib
│   │   │   ├── b4_package
│   │   │   │   ├── argument
│   │   │   │   ├── listener
│   │   │   │   ├── security_alert
│   │   │   │   └── talker
│   │   │   └── python3.10
│   │   │       └── site-packages
│   │   │           ├── b4_package
│   │   │           │   ├── __init__.py
│   │   │           │   ├── __pycache__
│   │   │           │   │   ├── __init__.cpython-310.pyc
│   │   │           │   │   ├── argument.cpython-310.pyc
│   │   │           │   │   ├── security_alert.cpython-310.pyc
│   │   │           │   │   ├── subscriber_test.cpython-310.pyc
│   │   │           │   │   └── talker.cpython-310.pyc
│   │   │           │   ├── argument.py
│   │   │           │   ├── security_alert.py
│   │   │           │   ├── subscriber_test.py
│   │   │           │   └── talker.py
│   │   │           └── b4_package-0.0.0-py3.10.egg-info
│   │   │               ├── PKG-INFO
│   │   │               ├── SOURCES.txt
│   │   │               ├── dependency_links.txt
│   │   │               ├── entry_points.txt
│   │   │               ├── requires.txt
│   │   │               ├── top_level.txt
│   │   │               └── zip-safe

...

~~~

<br>

## 참고 자료
* [Ros2 Humble Doc](https://docs.ros.org/en/humble/index.html)
* [Flask Doc](https://flask.palletsprojects.com/en/stable/)

<p align="center">
  <img src="https://github.com/jinhyuk2me/Minecraft_with_ROS2/blob/main/assets/images/banner.png?raw=true" width="100%"/>
</p>


# 🎮 Minecraft with ROS2 & SLAM

Minecraft를 ROS2 & microROS 환경에서 활용한 로봇 제어 및 SLAM 실습 및 실험 프로젝트입니다.

- 2025년 8월 29일 ~ 2025년 9월 1일(총 4일)

## 1. 프로젝트 개요

이 프로젝트에서 다음과 같은 실습을 진행했습니다:

<p align="center">
  <img src="https://github.com/jinhyuk2me/Minecraft_with_ROS2/blob/main/assets/images/point_cloud.gif?raw=true" width="100%"/>
</p>

- **조이스틱 제어**: 물리적 조이스틱을 통한 Minecraft 캐릭터 원격 조종
- **micro-ROS 통신**: ESP32와 micro-ROS를 활용한 임베디드 시스템 연동
- **SLAM 맵핑**: Cartographer를 사용한 Minecraft 환경에서의 실시간 맵 생성

## 2. 하드웨어 
- **조이스틱**: 캐릭터 움직임 제어용
- **ESP32**: micro-ROS 노드 실행 및 센서 데이터 처리
- **PC**: Minecraft 클라이언트 및 ROS 2 노드 실행

## 3. 기술 스택

| 분류 | 사용 기술 |
|------|-----------|
| **Robotics & SLAM** | [![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html) [![Cartographer](https://img.shields.io/badge/Cartographer-FF6B35?style=for-the-badge&logo=googleearthengine&logoColor=white)](https://google-cartographer-ros.readthedocs.io/) ![micro-ROS](https://img.shields.io/badge/micro--ROS-1E88E5?style=for-the-badge) |
| **Game & Development** | [![Minecraft](https://img.shields.io/badge/Minecraft-62B47A?style=for-the-badge&logo=minecraft&logoColor=white)](https://www.minecraft.net/) ![Forge](https://img.shields.io/badge/Forge-1.20.1-orange?style=for-the-badge) [![Gradle](https://img.shields.io/badge/Gradle-02303A?style=for-the-badge&logo=gradle&logoColor=white)](https://gradle.org/) |
| **Embedded & Hardware** | [![ESP32](https://img.shields.io/badge/ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-007ACC?style=for-the-badge) |
| **Languages & Runtime** | [![Java](https://img.shields.io/badge/Java-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)](https://openjdk.java.net/) [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://isocpp.org/) |
| **Environment & Tools** | [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/) [![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)](https://www.kernel.org/) [![Docker](https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/) [![CMake](https://img.shields.io/badge/CMake-064F8C?style=for-the-badge&logo=cmake&logoColor=white)](https://cmake.org/) |
| **Visualization & GUI** | [![RViz2](https://img.shields.io/badge/RViz2-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) |

## 4. 주요 기능

### 📡 임베디드 연동
<p align="center">
  <img src="https://github.com/jinhyuk2me/Minecraft_with_ROS2/blob/main/assets/images/joystick.jpg?raw=true" width="100%"/>
</p>

- ESP32 기반 micro-ROS 노드와의 실시간 통신
- IMU 및 3D LiDAR 센서 데이터 수집

### 🎮 조이스틱 제어
<p align="center">
  <img src="https://github.com/jinhyuk2me/Minecraft_with_ROS2/blob/main/assets/images/driving.gif?raw=true" width="100%"/>
</p>

- ROS2 `/cmd_vel` 메시지를 통한 실시간 캐릭터 제어

### 🗺️ SLAM 맵핑
<p align="center">
  <img src="https://github.com/jinhyuk2me/Minecraft_with_ROS2/blob/main/assets/images/slam_cartographer.gif?raw=true" width="100%"/>
</p>

- Cartographer를 사용한 Minecraft 맵의 실시간 2D 맵 생성
- 3D LiDAR 센서 시뮬레이션을 통한 환경 인식

>[!Note]
> 3D pointcloud 데이터를 Minecraft로부터 받아오는 데 걸리는 시간지연으로 인해 빠른 각속도로 회전하는 경우 맵 생성이 정상적으로 되지 않음을 확인하였습니다.<br>
> 본 프로젝트에서는 오픈소스로 구현된 툴을 사용하였기 때문에 전진/후진만 수행하여 맵을 생성하였습니다.  


## 5. 파일 구조

```
├── src/                    # Minecraft MOD 소스코드
│   ├── main/               # 메인 소스코드
│   └── minecraft_odometry/ # 추가된 ROS 2 패키지
├── scripts/                # 실행 스크립트
├── config/                 # 설정 파일
│   └── minecraft.rviz      # RViz2 설정 파일
├── docs/                   # 프로젝트 문서
└── run/                    # Minecraft 실행 환경
```

## 6. 참고사항

이 프로젝트는 실험 및 실습 목적으로 제작되었습니다.

## 7. 라이센스

이 프로젝트는 [Apache License 2.0](LICENSE.txt) 하에 배포됩니다.

---

*본 프로젝트는 [minecraft-ros2](https://github.com/minecraft-ros2/minecraft_ros2) 프로젝트의 코드를 기반으로 실험을 진행했습니다.*

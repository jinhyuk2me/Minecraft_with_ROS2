# ROS2_Minecraft

Minecraft를 ROS 2 환경에서 활용한 로봇 제어 및 SLAM 실험 프로젝트입니다.

## 프로젝트 개요

이 프로젝트는 다음과 같은 실험을 진행했습니다:

- **🎮 조이스틱 제어**: 물리적 조이스틱을 통한 Minecraft 캐릭터 원격 조종
- **📡 micro-ROS 통신**: ESP32와 micro-ROS를 활용한 임베디드 시스템 연동
- **🗺️ SLAM 맵핑**: Cartographer를 사용한 Minecraft 환경에서의 실시간 맵 생성

## 하드웨어 구성

- **조이스틱**: 캐릭터 움직임 제어용
- **ESP32**: micro-ROS 노드 실행 및 센서 데이터 처리
- **PC**: Minecraft 클라이언트 및 ROS 2 노드 실행

## 시스템 요구사항

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Java**: JDK 17+
- **Minecraft**: Forge 1.20.1


## 주요 실험 결과

### 🎮 조이스틱 제어
- ROS 2 Joy 메시지를 통한 실시간 캐릭터 제어
- 다양한 이동 모드 및 카메라 각도 조정 가능

### 🗺️ SLAM 맵핑  
- Cartographer를 사용한 Minecraft 맵의 실시간 2D/3D 맵 생성
- LiDAR 센서 시뮬레이션을 통한 환경 인식

### 📡 임베디드 연동
- ESP32 기반 micro-ROS 노드와의 실시간 통신
- 센서 데이터 수집 및 액추에이터 제어

## 파일 구조

```
├── src/                    # Minecraft MOD 소스코드
├── microros_apps/         # micro-ROS ESP32 앱
├── minecraft_odometry/    # ROS 2 패키지 (SLAM, 센서 동기화)
├── docs/                  # 문서
├── run/                   # Minecraft 실행 환경
└── minecraft.rviz         # RViz2 설정 파일
```

## 기술 스택

- **ROS 2 Humble**: 로봇 미들웨어
- **Cartographer**: SLAM 라이브러리  
- **micro-ROS**: 임베디드 ROS 2
- **Minecraft Forge**: MOD 개발 프레임워크
- **ESP32**: 마이크로컨트롤러

## 참고사항

이 프로젝트는 실험 및 실습 목적으로 제작되었습니다.

## 라이센스

이 프로젝트는 [Apache License 2.0](LICENSE.txt) 하에 배포됩니다.

---

*본 프로젝트는 [minecraft-ros2](https://github.com/minecraft-ros2/minecraft_ros2) 프로젝트의 코드를 기반으로 실험을 진행했습니다.*
# 조영진 | Robotics Software Developer

ROS 2와 두산 협동로봇, NVIDIA Isaac Sim 5.1을 활용한 팀 프로젝트를 수행하며 실제 로봇 제어, 물체 조작, 다중 로봇 관제와 다중 노드 시스템 통합을 경험했습니다.

고정된 작업 환경에서 LEGO를 조립·해체하는 로봇 제어부터 비전으로 움직이는 공구를 추적하고 작업자에게 전달하는 시스템의 상태 관리, 디지털 트윈 기반 자율 발렛파킹의 관제 알고리즘까지 수행했습니다. 각 프로젝트에서는 제가 주로 담당한 부분과 팀원 구현을 연결하며 배운 부분을 구분해 정리했습니다.

## Core Experience

- ROS 2 기반 다중 노드 로봇 시스템 통합
- Doosan M0609 및 OnRobot RG2 실기 제어
- 힘 제어와 Spiral 동작을 이용한 LEGO 조립·해체
- ROS 2 Action 기반 Task Manager와 상태 전이 관리
- 작업 상태와 안전 상태 분리, 실패·복구·재개 처리
- Docker 기반 MediaPipe 손 추적 노드와 ROS 2 연결
- YOLO Keypoint, Kalman Filter, `speedl` 모듈 통합 테스트 참여
- NVIDIA Isaac Sim 5.1 기반 디지털 트윈 프로젝트
- 다중 로봇 관제 및 ROS 2 Service·Action 기반 작업 라우팅
- MySQL 기반 운영 상태 관리
- 중앙 안전 상태와 Fail-safe 기반 요청 차단·복구 처리

---

## Project 01 — LEGO 블록 자동 조립·해체 시스템

사용자가 입력한 LEGO 도안을 분석하고, Doosan M0609와 RG2 그리퍼로 같은 형태를 조립한 뒤 블록을 순차적으로 해체하는 ROS 2 기반 시스템입니다.

### 전체 흐름

```text
도안 입력 → 색상·배치 분석 → 조립 순서 계산
→ 키팅 트레이 Pick → 목표 좌표 Place
→ 힘 제어·Spiral 결합 → 결과 확인
→ 역순 해체 → 키팅 트레이 회수
```

### 담당 역할

- `Robot Control Node` 담당
- Doosan `movel`, `amovel` 기반 Pick & Place
- 고정 키팅 트레이의 Pick 좌표 설정
- 상위 알고리즘이 계산한 Place 좌표 적용
- 힘 제어와 Spiral 동작을 이용한 LEGO 결합
- 역순 해체, 해체 전 재압착 및 동작 파라미터 조정

### 대표 문제 해결 — 해체 전 재압착

블록 하나를 분리하면 남은 블록의 결합도 느슨해져 다음 해체에서 여러 블록이 함께 빠지는 문제가 있었습니다. 적층 블록은 Spiral 해체 전에 힘 제어로 다시 눌러 결합 상태를 안정화했습니다.

```python
def force_press(self, pose: CartesianPose) -> None:
    self._task_compliance_ctrl(stx=[
        PLACE_XY_STIFFNESS, PLACE_XY_STIFFNESS, PLACE_Z_STIFFNESS,
        PLACE_ROT_STIFFNESS, PLACE_ROT_STIFFNESS, PLACE_ROT_STIFFNESS,
    ])
    self._set_desired_force(
        fd=[0.0, 0.0, -PLACE_FORCE_Z_N, 0.0, 0.0, 0.0],
        dir=[0, 0, 1, 0, 0, 0],
        time=0,
        mod=self._DR_FC_MOD_ABS,
    )
    self.move_linear(pose, "PREPRESS_DESCEND",
                     vel=PLACE_LINEAR_VELOCITY,
                     acc=PLACE_LINEAR_ACCELERATION)
    self._release_force()
    self._release_compliance_ctrl()
```

반복 시험에서 해체 성공률은 약 80%대였습니다. 완전한 해결에는 도달하지 못했지만, 단순 수직 인장 방식보다 여러 블록이 함께 분리되는 현상을 줄였습니다.

[상세 내용](projects/rokey_proj_01.md) · [코드 저장소](https://github.com/joyj0131-dev/rokey_proj_01) · [시연 영상](https://youtu.be/rr9P0iLsZmI)

---

## Project 02 — 이동 공구 전달 로봇

사용자가 음성 또는 GUI로 요청한 공구를 컨베이어에서 검출하고, 움직임을 추적해 파지한 뒤 작업자의 손으로 전달하는 ROS 2 기반 협동로봇 시스템입니다.

### 전체 흐름

```text
음성·GUI 공구 요청 → YOLO Keypoint 파지점 검출
→ RealSense 3D 위치 계산 → Kalman Filter 이동 추정
→ speedl 연속 추종·파지 → 작업자 손 추적
→ 당김 감지·그리퍼 개방 → 원점 복귀
```

### 담당 역할

- `Task Manager`와 전체 작업 상태 전이 관리
- Vision 모드 전환 및 Robot Action 요청·결과 처리
- 추적 유실, 파지 실패, 취소, Fault 처리
- STOP, RESET, RESUME 흐름 관리
- 작업 상태와 안전 상태를 GUI로 전달
- Docker 기반 MediaPipe 손 추적 노드의 초기 ROS 2 연결
- 비전·추적·서보 제어 모듈 통합 테스트 참여

### 대표 문제 해결 — 중단 상태에 따른 재개 정책

Fault 이후 모든 작업을 같은 방식으로 재시작하지 않았습니다. 공구 파지가 확인된 이후에는 전달 작업을 이어서 수행하고, 파지 상태가 불확실한 단계에서는 공구 검출부터 다시 시작하도록 구분했습니다.

```python
def _capture_resume_snapshot(self):
    if self.state in _RESUME_CONTINUE_STATES:
        # 파지가 검증된 상태: 중단된 전달 작업을 계속 수행
        self._resume_kind = 'continue'
        self._resume_state = self.state
        self._resume_tool = self.current_tool
        self._resume_grasp_spec = self._active_grasp_spec
    elif self.state in _RESUME_RETRY_PICK_STATES:
        # 파지 상태가 불확실함: 그리퍼를 열고 검출부터 재시도
        self._resume_kind = 'retry_pick'
        self._resume_state = self.state
        self._resume_tool = self.current_tool
        self._resume_grasp_spec = None
```

YOLO Keypoint, Kalman Filter와 `speedl` 제어는 다른 팀원이 주로 구현했습니다. 저는 해당 결과가 전체 작업 흐름에 연결되도록 상태와 Action 결과를 관리하고, 통합 과정에서 각 모듈의 역할과 데이터 흐름을 배웠습니다.

[상세 내용](projects/rokey_proj_02.md) · [코드 저장소](https://github.com/joyj0131-dev/rokey_proj_02) · [시연 영상](https://youtu.be/YrdxbWTCtsk)

---

## Project 03 — 디지털 트윈 기반 자율 발렛파킹 로봇 시스템

| 항목 | 내용 |
|---|---|
| 기간 | 2026.07.15 ~ 2026.07.29 |
| 환경 | Ubuntu 22.04, ROS 2 Humble, NVIDIA Isaac Sim 5.1 |
| 구분 | 4인 팀 프로젝트 |
| 담당 | 팀장 및 관제 알고리즘 |

입차·출차 전용 메카넘 로봇 4대가 차량 하부로 진입해 차량을 들어 올리고, 지정된 주차 슬롯까지 운반하는 디지털 트윈 기반 자율 발렛파킹 시스템입니다.

팀 전체 시스템은 ArUco, 휠 오도메트리, Depth Camera, LiDAR, ROS 2 Service·Action과 MySQL 기반 관제 시스템을 사용했습니다.

### 담당 역할

- 팀장으로서 프로젝트 일정과 기능 인터페이스 조율
- 관제 알고리즘 설계 및 구현
- `request_type`에 따른 ENTRY·EXIT 작업 라우팅
- 입차·출차 전용 로봇쌍의 상태 확인과 배정
- 빈 주차 슬롯 탐색 및 예약
- ROS 2 Service·Action 기반 비동기 작업 실행 연결
- 중앙 안전 상태가 `NORMAL`이 아닐 때 신규 작업 차단
- 차량·로봇·슬롯·작업·안전 상태의 MySQL 연동
- 로봇 4대의 Odometry를 관제 좌표로 변환해 DB에 기록
- 관제·로봇 제어·비전·UI 모듈 통합 테스트

### 대표 문제 해결 — 동시 요청의 중복 작업과 자원 충돌 방지

요청이 동시에 접수되면 같은 차량의 작업이 중복 생성되거나 여러 로봇이 같은 구역과 슬롯을 점유할 수 있었습니다. 이를 막기 위해 작업 접수 구간을 Lock으로 직렬화하고, 차량별 활성 작업과 중앙 안전 상태를 먼저 검사했습니다. 안전 상태가 `NORMAL`이 아니면 신규 요청을 거부하도록 구성했습니다.

구역 자원은 정해진 순서로 획득하고, 필요한 자원 중 일부만 확보되면 모두 반납하는 DB Zone Lock 정책을 적용했습니다. 작업 완료 또는 실패 결과에 따라 로봇과 슬롯 상태를 복구해 다음 요청이 잘못된 점유 상태를 이어받지 않도록 했습니다.

[코드 저장소](https://github.com/joyj0131-dev/Rokey_proj_03-Isaac-Sim-/)

---

## Tech Stack

`ROS 2 Humble` · `Python` · `Doosan M0609` · `OnRobot RG2` · `RealSense` · `NVIDIA Isaac Sim 5.1` · `MySQL` · `PyQt/PySide6` · `OpenCV` · `Docker` · `Git/GitHub`

## What I Learned

Proj 1에서는 좌표만 정확하게 지정해도 실제 물체의 결합력과 접촉 오차 때문에 로봇 동작이 실패할 수 있다는 점을 경험했습니다. Proj 2에서는 움직이는 물체를 다루기 위해 검출, 위치·속도 추정, 연속 제어와 상태 관리가 함께 연결되어야 한다는 점을 배웠습니다.

또한 팀 프로젝트에서 담당 기능만 완성하는 것에 그치지 않고, 다른 모듈의 입출력과 실패 상황을 이해하며 전체 시스템을 통합하는 경험을 쌓았습니다.

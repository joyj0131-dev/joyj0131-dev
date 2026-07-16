# 조영진 | Robotics Software Developer

ROS 2와 두산 협동로봇을 활용한 팀 프로젝트를 수행하며 실제 로봇 제어, 물체 조작, 다중 노드 시스템 통합을 경험했습니다.

고정된 작업 환경에서 LEGO를 조립·해체하는 로봇 제어부터 비전으로 움직이는 공구를 추적하고 작업자에게 전달하는 시스템의 상태 관리까지 수행했습니다. 각 프로젝트에서는 제가 주로 담당한 부분과 팀원 구현을 연결하며 배운 부분을 구분해 정리했습니다.

## Core Experience

- ROS 2 기반 다중 노드 로봇 시스템 통합
- Doosan M0609 및 OnRobot RG2 실기 제어
- 힘 제어와 Spiral 동작을 이용한 LEGO 조립·해체
- ROS 2 Action 기반 Task Manager와 상태 전이 관리
- 작업 상태와 안전 상태 분리, 실패·복구·재개 처리
- Docker 기반 MediaPipe 손 추적 노드와 ROS 2 연결
- YOLO Keypoint, Kalman Filter, `speedl` 모듈 통합 테스트 참여

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

## Tech Stack

`ROS 2 Humble` · `Python` · `Doosan M0609` · `OnRobot RG2` · `RealSense` · `PyQt/PySide6` · `OpenCV` · `Docker` · `Git/GitHub`

## What I Learned

Proj 1에서는 좌표만 정확하게 지정해도 실제 물체의 결합력과 접촉 오차 때문에 로봇 동작이 실패할 수 있다는 점을 경험했습니다. Proj 2에서는 움직이는 물체를 다루기 위해 검출, 위치·속도 추정, 연속 제어와 상태 관리가 함께 연결되어야 한다는 점을 배웠습니다.

또한 팀 프로젝트에서 담당 기능만 완성하는 것에 그치지 않고, 다른 모듈의 입출력과 실패 상황을 이해하며 전체 시스템을 통합하는 경험을 쌓았습니다.

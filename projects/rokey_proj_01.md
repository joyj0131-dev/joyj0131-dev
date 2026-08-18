# Project 01 — LEGO 블록 자동 조립·해체 시스템

## 프로젝트 개요

사용자가 입력한 LEGO 도안을 분석하고, Doosan M0609와 OnRobot RG2 그리퍼를 이용하여 같은 형태로 조립한 뒤 블록을 순차적으로 해체하는 ROS 2 기반 팀 프로젝트입니다.

| 항목 | 내용 |
|---|---|
| 기간 | 2026.06.17 ~ 2026.06.30 |
| 구성 | 3인 팀 프로젝트 |
| 환경 | ROS 2 Humble · Python · Doosan M0609 · OnRobot RG2 |
| 담당 | Robot Control Node |

- 코드 저장소: [rokey_proj_01](https://github.com/joyj0131-dev/rokey_proj_01)
- 시연 영상: [YouTube](https://youtu.be/4dupj63IoR0)

## 프로젝트 목표

- 도안 분석 결과에 따른 LEGO 자동 조립
- 미세한 위치 오차를 고려한 블록 결합
- 여러 블록이 함께 빠지지 않도록 순차 해체
- 작업 상태를 UI에 전달하고 조립 과정을 확인

## 전체 작동 흐름

```text
도안 이미지 입력
→ 블록 색상과 배치 분석
→ 조립 순서 계산
→ Robot Control Node로 BlockTask 전달
→ 키팅 트레이에서 Pick
→ 계산된 Place 위치로 이동
→ 힘 제어와 Spiral로 결합
→ 조립 결과 확인
→ 조립 역순으로 블록 해체
→ 회수 위치로 이동
```

## 시스템 구성

| 구성 | 역할 | 담당 |
|---|---|---|
| UI | 도안 입력과 작업 상태 표시 | 팀원·공동 |
| Image Processing | 블록 색상과 배치 분석 | 팀원 |
| Sequencer | 조립 순서와 목표 좌표 계산 | 팀원 |
| Robot Control | Pick, Place, 조립 및 해체 실행 | 본인 주 담당 |
| Verification | 조립 결과 확인 | 팀원 |

## 프로젝트 핵심 기술

- 도안 이미지 기반 블록 색상·배치 분석
- 조립 순서 및 Place 좌표 계산
- Doosan `movel`, `amovel` 기반 Pick & Place
- 힘 제어와 Spiral을 이용한 블록 결합
- Spiral 분리와 재압착을 이용한 순차 해체

## My Contribution

- `Robot Control Node` 구현 및 실제 로봇 연동
- 고정 키팅 트레이의 블록별 Pick 좌표 설정
- 상위 알고리즘이 계산한 Place 좌표 적용
- 적층 높이에 따른 Place 및 해체 Z 좌표 계산
- 조립용 힘·강성·Spiral 파라미터 조정
- 역순 해체 시퀀스와 해체 전 재압착 적용
- 일부 이동 구간의 특이점 문제를 중간 경유점으로 우회

## 주요 문제 해결

### 1. 미세 위치 오차로 인한 결합 실패

Place 좌표로 하강한 뒤 수직으로 누르는 방식만 사용하면 LEGO 돌기가 정확하게 맞물리지 않는 경우가 발생했습니다. Z축 힘을 유지하면서 작은 반경의 Spiral 동작을 수행해 결합 가능한 위치를 탐색했습니다.

```python
# Z축 힘을 유지한 채 XY를 미세 정렬하며 추가 삽입
ret = self._move_spiral(
    rev=PLACE_SPIRAL_REV,
    rmax=PLACE_SPIRAL_RMAX_MM,
    lmax=PLACE_SPIRAL_LMAX_MM,
    vel=PLACE_SPIRAL_VEL,
    acc=PLACE_SPIRAL_ACC,
    time=0.0,
    axis=self._DR_AXIS_Z,
    ref=self._DR_BASE,
)
if ret == -1:
    raise RuntimeError("결합 보조 Spiral 실행 실패")
```

실제 적용값은 회전 수 `0.5`, 최대 반경 `0.5mm`, Z축 추가 삽입 `0.3mm`, 목표 하향 힘 `9N`이었습니다. 값은 일반화된 정답이 아니라 당시 LEGO와 작업 환경에서 반복 시험을 통해 정한 파라미터입니다.

### 2. 해체 시 여러 블록이 함께 빠지는 문제

블록 하나를 바로 위로 당기면 아래쪽 또는 주변 블록까지 함께 빠졌습니다. 기존 Spiral 분리 방식에 조립 역순 해체와 적층 블록 재압착 과정을 연결했습니다.

```python
# 마지막에 조립한 블록부터 역순으로 해체
for color, y, stack_index in reversed(placed_blocks):
    actual_place_z = PLACE_BASE_Z_MM + stack_index * STACK_PITCH_MM

    detach_task = DetachDiscardTask(
        task_id=f"{color}_y{y:.0f}_si{stack_index}",
        color=color,
        detach_pose=detach_pose,
        basket_pose=basket_pose,
        pre_press=(stack_index > 0),
    )
    execute_detach_discard(self._motion_controller, detach_task)
```

`spiral_detach_only()`의 초기 구현은 다른 팀원이 담당했고, 저는 역순 해체, 적층 높이 계산, 재압착 조건과 전체 해체 시퀀스를 개선했습니다.

### 3. 해체 후 남은 블록의 결합력 약화

하나의 블록을 분리하는 과정에서 남은 블록의 결합도 느슨해졌습니다. 다음 블록을 분리하기 전에 힘 제어로 다시 눌러 결합 상태를 복원했습니다.

```python
if task.pre_press:
    controller.rg2_grip()
    controller.force_press(task.detach_pose)
    controller.move_linear(detach_approach_pose, "PREPRESS_LIFT")
    controller.rg2_release()
```

힘 제어 함수에서는 XY 방향은 위치를 유지하고 Z축은 상대적으로 유연하게 설정한 뒤 일정한 하향 힘을 적용했습니다.

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

## 결과

- 도안과 조립 순서에 따른 LEGO 자동 Pick & Place
- 힘 제어와 Spiral을 이용한 블록 결합
- 역순 해체 및 적층 블록 재압착 적용
- 당시 반복 시험에서 조립 95/100회, 해체 16/20회 성공

> 해당 결과는 당시 LEGO와 고정 작업 환경에서 수행한 프로젝트 내부 반복 시험 기준이며, 일반화된 성능 지표나 공식 벤치마크는 아닙니다.

## 한계와 개선 방향

- Pick 좌표가 고정값이므로 트레이 위치 변경 시 재설정 필요
- 블록 종류와 결합 상태에 따라 힘과 Spiral 파라미터가 달라짐
- 여러 블록이 함께 빠지는 현상을 완전히 제거하지 못함
- 시험 횟수는 기록했지만 조명·블록 상태 등 세부 시험 조건을 체계적으로 통제하지 못함
- 향후 비전 기반 Pick 좌표 보정과 힘 데이터 기반 결합 판정 필요

## 회고

실제 로봇에서는 목표 좌표를 지정하는 것만으로 안정적인 작업을 보장할 수 없었습니다. LEGO처럼 결합력이 존재하는 물체는 접촉 힘과 주변 블록에 미치는 영향까지 고려해야 했습니다.

해체 성공률을 완벽하게 만들지는 못했지만, 문제를 관찰하고 Spiral, 역순 해체와 재압착 동작을 반복적으로 조정하며 실제 물체 조작의 어려움을 경험했습니다.

[프로필로 돌아가기](../README.md)

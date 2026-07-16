# Project 02 — 이동 공구 전달 로봇

## 프로젝트 개요

사용자가 음성 또는 GUI로 요청한 공구를 컨베이어에서 검출하고, 이동을 추적해 파지한 뒤 작업자의 손으로 전달하는 ROS 2 기반 협동로봇 팀 프로젝트입니다.

- 원본 저장소: [rokey_proj_02](https://github.com/joyj0131-dev/rokey_proj_02)
- 시연 영상: [YouTube](https://youtu.be/YrdxbWTCtsk)

## 프로젝트 목표

- 일반 객체 검출을 넘어 실제 로봇이 사용할 공구 파지점 검출
- 컨베이어 위 이동 공구의 위치와 속도 추정
- 연속 속도 명령을 이용한 이동 공구 추종 및 파지
- 작업자 손 추적과 당김 감지를 이용한 공구 전달
- 여러 ROS 2 노드의 상태, 실패와 복구 흐름 관리

## 전체 작동 흐름

```text
음성·GUI로 공구 요청
→ YOLO Keypoint로 공구와 파지점 검출
→ RealSense로 3차원 위치 계산
→ Kalman Filter로 위치·속도 추정
→ speedl로 이동 공구를 추적하며 하강
→ 공구 파지와 파지 상태 확인
→ 안전 전달 위치로 이동
→ Docker 기반 MediaPipe로 작업자 손 추적
→ 주먹 확인 후 접근 정지
→ 당김 감지와 그리퍼 개방
→ 원점 복귀
```

## 시스템 구성

| 구성 | 역할 | 담당 |
|---|---|---|
| Operator GUI | 명령 입력과 작업 상태 표시 | 본인·공동 |
| STT Node | 웨이크워드와 음성 인식 | 팀원 |
| Task Manager | 전체 상태 전이와 실패·복구 관리 | 본인 주 담당 |
| Object Detection | YOLO Keypoint 공구 검출 | 팀원 |
| Vision Node | RealSense 3D 복원과 추적 | 팀원 |
| Robot Control | Kalman, speedl, 그리퍼와 로봇 실행 | 팀원 2 주 담당·공동 통합 |
| Hand Tracking | Docker 기반 MediaPipe 손·주먹 검출 | 본인 초기 연결·팀 공동 개선 |

## 프로젝트 핵심 기술

- YOLO Keypoint 기반 공구 파지점 검출
- RealSense eye-in-hand 카메라와 좌표 변환
- Kalman Filter 기반 위치·속도 추정
- `speedl` 기반 연속 속도 추종
- 안전한 Z축 하강을 위한 감속과 정지 조건
- Docker Container 기반 MediaPipe 손 추적
- ROS 2 Topic, Service, Action 기반 상태 관리

## My Contribution

- `Task Manager`와 전체 작업 상태 정의
- 사용자의 공구 요청부터 전달 완료까지 상태 전이 관리
- Vision 모드 전환과 Robot Action 요청·결과 처리
- 복구 가능한 추적 실패와 Fault 분기
- 작업 상태와 안전 상태의 독립 관리
- STOP, RESET, RESUME 처리
- Docker 기반 손 추적 노드의 초기 ROS 2 연결
- GUI, 비전, 제어와 음성 모듈의 통합 테스트 지원

YOLO Keypoint는 팀원 3, Kalman Filter와 `speedl` 서보 제어는 팀원 2가 주로 구현했습니다. 해당 기술을 제 구현으로 표현하지 않고, Task Manager와 통합 과정에서 데이터 흐름과 동작 원리를 배운 경험으로 구분합니다.

## 주요 문제 해결

### 1. 일반 YOLO Bounding Box의 파지점 한계

초기 일반 객체 검출은 공구의 Bounding Box만 제공해 손잡이 위치와 파지 방향을 정하기 어려웠습니다. 팀에서 두 개의 Keypoint를 검출하는 Pose 모델을 적용해 공구 방향과 파지점을 표현했습니다.

```python
# 팀원 구현: Pose 모델의 두 Keypoint를 ROS 2 Detection2D로 전달
keypoints = result.keypoints
for i, box in enumerate(result.boxes):
    detection = Detection2D()
    detection.class_name = result.names[int(box.cls[0])]
    detection.x1, detection.y1, detection.x2, detection.y2 = map(int, box.xyxy[0])

    if keypoints is not None and i < len(keypoints.xy):
        xy = keypoints.xy[i]
        detection.kpt0_x, detection.kpt0_y = float(xy[0][0]), float(xy[0][1])
        detection.kpt1_x, detection.kpt1_y = float(xy[1][0]), float(xy[1][1])
```

저는 검출 모델을 직접 구현하지 않았으며, 검출 결과가 추적과 Task Manager의 파지 흐름으로 이어지는 통합 과정에 참여했습니다.

### 2. 이동 물체의 흔들림과 추적 지연

카메라의 위치 측정값에는 노이즈와 처리 지연이 있어 측정 좌표만 따라가면 로봇이 공구보다 늦게 움직일 수 있었습니다. 팀원 2가 상태 `[x, y, z, vx, vy]`의 등속 모델 Kalman Filter를 적용해 위치와 수평 속도를 추정했습니다.

```python
# 팀원 구현: 현재 추정 상태로 lead_time 이후 위치 예측
def predict_position(self, lead_time):
    px, py, pz, vx, vy = self.x
    return np.array([
        px + vx * lead_time,
        py + vy * lead_time,
        pz,
    ])
```

### 3. 이동 공구를 따라가는 연속 제어와 Z축 충돌

한 번의 목표 좌표로 이동하는 `movel`은 계속 움직이는 공구를 따라가기 어려워 `speedl` 기반 연속 속도 제어를 사용했습니다.

실기 테스트에서는 Z축 속도를 0으로 명령해도 가속도 제한 때문에 관성 하강이 남아 공구 또는 바닥과 충돌할 위험이 있었습니다. 팀원 2가 수평 정렬 조건, 남은 거리와 가속도 제한을 이용해 안전한 하강 속도를 계산하도록 개선했습니다.

```python
# 팀원 구현: 수평 정렬 후, 남은 거리 안에서 정지 가능한 Z 속도 계산
if self._grasp_locked or self._z_locked:
    vz = 0.0
elif e_xy_norm < self.eps_descend:
    brake_distance = max(
        self._last_z_gap - self.descend_stop_margin_m,
        0.0,
    )
    safe_speed = math.sqrt(
        2.0 * self.descend_accel_m_s2 * brake_distance
    )
    vz = -min(self.descend_speed, safe_speed)
else:
    vz = 0.0
```

저는 이 제어를 직접 구현하지 않았으며, 통합 시험에서 파지 Action의 시작·성공·실패 결과와 다음 상태 전이를 확인했습니다.

### 4. 복구 가능한 실패와 Fault 구분

추적 유실과 타임아웃은 공구를 다시 검출해 복구할 수 있지만, 정의되지 않은 오류까지 자동으로 재시도하면 위험할 수 있습니다. Task Manager에서 실패 사유에 따라 재검출과 Fault를 구분했습니다.

```python
def _handle_servo_pick_result(self, result):
    if not result.success:
        recoverable = (
            'lost', 'tracking_lost', 'timeout', 'diverged', 'diverging',
        )
        if result.message in recoverable:
            self._set_state(State.DETECT_TRACK, detail=result.message)
            self._start_detect_track_timer()
        else:
            self._enter_fault(result.message)
        return

    self._set_state(State.VERIFY_GRASP)
```

### 5. 중단 위치에 따른 작업 재개

Fault 이후 모든 작업을 처음부터 재시작하지 않고 공구 파지 상태에 따라 정책을 분리했습니다.

```python
def _capture_resume_snapshot(self):
    if self.state in _RESUME_CONTINUE_STATES:
        # 파지 검증 이후: 전달 작업을 이어서 수행
        self._resume_kind = 'continue'
        self._resume_state = self.state
        self._resume_tool = self.current_tool
        self._resume_grasp_spec = self._active_grasp_spec
    elif self.state in _RESUME_RETRY_PICK_STATES:
        # 파지 상태 불확실: 그리퍼를 열고 검출부터 재시도
        self._resume_kind = 'retry_pick'
        self._resume_state = self.state
        self._resume_tool = self.current_tool
        self._resume_grasp_spec = None
```

### 6. Docker 기반 손 추적 연결

MediaPipe 실행 환경을 Docker Container로 분리하고, 손바닥 위치와 주먹 상태를 JSON 형태의 ROS 2 토픽으로 발행하는 초기 노드를 구현했습니다.

```python
def _on_image(self, msg):
    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    result = detect_hand(self._hands, bgr)

    if result is None:
        payload = {'detected': False}
    else:
        palm_px, landmarks, confidence = result
        payload = {
            'detected': True,
            'palm_px': list(palm_px),
            'confidence': confidence,
            'is_fist': is_fist(landmarks),
        }

    out = String()
    out.data = json.dumps(payload)
    self._pub.publish(out)
```

이후 카메라 트래픽을 줄이기 위한 구독 게이트와 QoS 개선은 다른 팀원이 추가했습니다.

## 상태 머신

```text
IDLE → MOVE_TO_WATCH → DETECT_TRACK → SERVO_PICK
→ VERIFY_GRASP → MOVE_SAFE → APPROACH_HAND
→ WAIT_PULL → HOME → IDLE
```

작업 상태와 안전 상태를 분리했습니다.

```python
class Safety:
    NORMAL = 'NORMAL'
    PROTECTIVE_STOP = 'PROTECTIVE_STOP'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    FAULT = 'FAULT'
    RECOVERY_REQUIRED = 'RECOVERY_REQUIRED'
```

## 결과

- YOLO Keypoint를 이용한 공구 파지점 표현
- Kalman Filter와 `speedl` 기반 이동 공구 추적 및 파지
- Z축 감속·정지 조건을 통한 충돌 위험 완화
- Docker 기반 작업자 손·주먹 인식
- 음성·GUI 요청부터 공구 전달까지 전체 흐름 통합
- Task Manager 기반 실패·복구·재개 관리

## 한계와 개선 방향

- 조명, 가림과 공구 방향에 따라 Keypoint 검출 정확도가 달라짐
- RealSense 깊이 노이즈와 처리 지연이 파지 정확도에 영향
- 컨베이어 속도와 방향이 급변하면 예측 오차가 커질 수 있음
- 하강과 파지 조건이 실험적 파라미터여서 환경 변경 시 재튜닝 필요
- 공구별 파지 폭과 힘을 별도로 설정해야 함

## 협업 및 회고

주 담당은 Task Manager였지만 담당 모듈에만 머무르지 않고 YOLO Keypoint, Kalman Filter, `speedl`, Docker 손 추적 모듈의 통합 시험을 지원했습니다.

각 알고리즘을 모두 직접 구현한 것은 아니지만, 다른 팀원의 구현이 어떤 데이터를 만들고 그 결과가 다음 노드와 로봇 동작에 어떻게 사용되는지 배웠습니다. 여러 ROS 2 노드가 연결된 시스템에서는 개별 기능뿐 아니라 실행 순서, 결과 전달과 실패 정책이 중요하다는 점을 경험했습니다.

[프로필로 돌아가기](../README.md)

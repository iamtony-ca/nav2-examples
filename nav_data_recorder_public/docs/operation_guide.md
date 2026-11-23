# nav_data_recorder 운영 가이드

ROS 2 Jazzy + Nav2 로봇에서 navigation 데이터를 상시 기록하고(기본 7일 보관), 문제가 생긴 시점을
나중에 Lichtblick이나 rviz2로 다시 보는 방법을 정리한 문서입니다. 로봇 셋업 담당자는 2~4장, 현장
사용자는 5장, 분석 담당자는 6장을 보면 됩니다.

## 목차

1. [구성 한눈에 보기](#1-구성-한눈에-보기)
2. [로봇 초기 셋업](#2-로봇-초기-셋업)
3. [우리 로봇에 맞추기 (프로필)](#3-우리-로봇에-맞추기-프로필)
4. [코드를 업데이트할 때](#4-코드를-업데이트할-때)
5. [현장에서 사용하기](#5-현장에서-사용하기)
6. [분석 PC에서 보기](#6-분석-pc에서-보기)
7. [설정 파일과 옵션](#7-설정-파일과-옵션)
8. [문제 해결](#8-문제-해결)
9. [끄기와 제거](#9-끄기와-제거)

## 1. 구성 한눈에 보기

| 구성 | 실행 방식 | 하는 일 |
|---|---|---|
| `nav_data_recorder` (C++ 노드) | 로봇 시작 스크립트에서 launch, 죽으면 5초 뒤 재시작 | 토픽을 압축해 `~/nav_data/rec/`에 10분 단위 mcap 파일로 기록, 이벤트 로그 작성 |
| `nav_data_session` | 위 launch가 부팅마다 한 번 실행 | 그 부팅의 Nav2 파라미터, BT xml, 지정한 설정 파일, git rev를 `~/nav_data/sessions/`에 저장 |
| `nav_data_maintenance` | 위 launch가 함께 띄움, CPU·디스크 우선순위 최저 | 15분마다 오래된 기록 삭제(7일 지난 것, 60 GB 초과분, 디스크 여유 15% 미만일 때 오래된 것부터). 매일 02:30이 지나면 전날 기록에서 특이 구간(멈춤, 실패, 안전 등 R1~R10)을 찾아 리포트 작성, 수동 표시 앞뒤 5분 보존. 로봇이 꺼져 있어 못 한 날은 다음 기동 때 처리 |
| `nav_data_export` | 필요할 때 사람이 실행 | 원하는 시각 구간을 표준 메시지 mcap으로 복원해 내보내기 |

측정 예: x86 산업용 로봇 정지 상태에서 디스크 39 KB/s(하루 약 3.4 GB), CPU 코어 1개 기준 6.5%, 메모리
52 MB. Nav2 TurtleBot3 시뮬레이션 주행에서 약 17 KB/s, 3.5%. 실제 사용량은 기록하는 토픽과 센서에
따라 다르므로 셋업 후 2.6절로 확인하세요.

데이터 폴더 구조:

```
~/nav_data/
  rec/20261120/083015_ab12/        로봇 기동 한 번 = 폴더 하나 (시각_부팅ID)
      083015_ab12_0.mcap ...         10분마다 파일 하나, 파일마다 따로 열어도 완결
  events/20261120.jsonl            온라인 이벤트 (상태 변화, 오류, 안전, 수동 표시, 기록기)
  index/20261120_report.md         야간 리포트 (특이 구간 순위)
  sessions/20261120_083015_ab12/   그 기동의 파라미터, BT xml, 추가 파일, git rev
  pinned/<시각>_<사유>/            수동 표시 앞뒤 5분 (90일 보관)
```

## 2. 로봇 초기 셋업

로봇마다 한 번 합니다. OS 설정(systemd, sudo)은 건드리지 않고, 작업공간 빌드와 시작 스크립트 한 줄이 전부입니다. 인터넷은 필요 없습니다. 아래에서
`~/ros2_ws`는 로봇의 ROS 2 작업공간, `<profile>`은 3장의 프로필 이름(예: `myrobot`)을 뜻합니다.
표준 Nav2 로봇이면 프로필 없이(`default`) 시작해도 됩니다.

### 2.1 코드 받기

```bash
cd ~/ros2_ws/src
git clone <이 저장소> nav_data_recorder      # 오프라인 로봇은 git bundle 등 평소 방식으로
```

### 2.2 의존성 점검

```bash
bash ~/ros2_ws/src/nav_data_recorder/nav_data_recorder/scripts/check_deps.sh
# Nav2를 작업공간에서 소스로 빌드했다면 그 setup.bash를 인자로: ... check_deps.sh ~/ros2_ws/install/setup.bash
```

마지막 줄이 `nothing to install`이면 다음 단계로 갑니다. ROS Jazzy 기본 설치(ros-base 이상)에 모두
들어 있고, nav2_msgs는 Nav2 설치(apt 또는 작업공간 빌드)에 들어 있어 보통은 그대로 통과합니다. `MISSING`이 나오면 스크립트가 출력한 .deb 주소를 인터넷 되는 PC에서
받아 옮긴 뒤 `sudo apt install ./*.deb`로 설치합니다. 버전 맞추는 방법은 README의 Offline install을
보세요.

### 2.3 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_data_recorder_msgs nav_data_profiles nav_data_recorder nav_data_tools
```

- 로봇의 메시지 패키지를 빌드하는 같은 작업공간에서 빌드합니다(기록 파일에 들어가는 타입 정의가
  로봇과 같아야 합니다).
- 평소 쓰는 colcon 옵션이 있으면 그대로 붙입니다. 시험을 빼려면 `--cmake-args -DBUILD_TESTING=OFF`.

### 2.4 시작 스크립트에 추가

로봇의 ROS 2 스택을 띄우는 스크립트에서 Nav2를 띄운 다음에 아래를 추가합니다. 스택과 같은 환경
(작업공간 setup, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, DDS 설정 파일)에서 실행되어야 합니다.
환경이 다르면 토픽이 하나도 보이지 않습니다.

```bash
nice -n 10 ionice -c2 -n7 ros2 launch nav_data_recorder nav_data_recorder.launch.py \
    robot_id:=robot01 \
    profile:=<profile> \
    ws_src:=$HOME/ros2_ws/src \
    map_name:=<사용 중인 지도 이름>
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_id` | `robot` | 로봇마다 다른 이름. 기록 메타데이터에 남음 |
| `profile` | `$NAV_DATA_PROFILE`, 없으면 `default` (표준 Nav2) | 로봇 프로필 이름 또는 폴더 경로 (3장) |
| `topics_file` | 프로필의 `topics.yaml` | 이 파일 하나만 바꿔 쓸 때 |
| `params_file` | 프로필의 `params.yaml` | 이 파일 하나만 바꿔 쓸 때 |
| `root_dir` | `params_file`의 `root_dir`, 없으면 `~/nav_data` | 저장 위치 (정리·리포트도 같은 위치를 씀) |
| `ws_src` | 없음 | 이 폴더의 git rev를 기록에 남김 |
| `map_name` | `$NAV_DATA_MAP_NAME` | 지도 이름을 기록에 남김 |
| `session_dump` | `true` | 부팅마다 Nav2 파라미터 등을 저장 |
| `maintenance` | `true` | 정리·야간 리포트(`nav_data_maintenance`)를 함께 실행 (2.5) |

- `nice`/`ionice`는 기록기가 navigation보다 CPU와 디스크를 양보하게 합니다.
- 여러 터미널을 띄우는 스크립트라면 다른 노드와 같은 방식(예: `gnome-terminal -- bash -c "<환경>; <위 명령>; exec bash"`)으로 넣습니다.

### 2.5 정리·리포트 (따로 설치할 것 없음)

2.4의 launch가 기록기와 함께 `nav_data_maintenance`를 띄웁니다. 시작 1분 뒤부터 15분마다 오래된 기록을
지우고, 매일 02:30이 지나면 전날 리포트를 만듭니다. 리포트는 로봇이 주행 중이면 멈출 때까지(최대 6시간)
기다리고, 그래도 못 하면 다음 날 02:30에 다시 시도합니다(로봇 상태를 아직 모를 때도 기다림). 로봇이 꺼져 있어
못 한 날(최근 3일)은 다음 기동 때 처리합니다. 기록기와 같은 환경(ROS
도메인, DDS 설정)에서 돌기 때문에 따로 맞출 환경 설정도 없습니다.

설정은 프로필 `params.yaml`의 `nav_data_maintenance` 부분입니다(7장).

```yaml
nav_data_maintenance:
  ros__parameters:
    keep_days: 7.0          # 이보다 오래된 기록 삭제 (10분 파일 단위)
    cap_gb: 60.0            # rec/ 합계 상한
    min_free: 0.15          # 디스크 여유 하한
    nightly_time: "02:30"   # 야간 리포트 시각, "" 이면 끔 (따옴표로 감쌀 것)
```

설정에 오류가 있어도(오타, 잘못된 시각) 정리는 계속 돕니다. 잘못된 항목은 무시되거나 야간 리포트만 꺼지고, 기록기
터미널에 `[nav_data_maintenance]`로 알립니다.

### 2.6 동작 확인

로봇을 다시 띄운 뒤 2~3분 기다려 아래를 확인합니다.

| 확인 | 명령 | 정상 |
|---|---|---|
| 기록 파일 생성 | `ls -la ~/nav_data/rec/$(date +%Y%m%d)/*/` | `.mcap` 파일이 있고 크기가 늘어남 |
| 기록기 시작 | `head -1 ~/nav_data/events/$(date +%Y%m%d).jsonl` | `"kind":"recorder","value":"start"` |
| 누락 토픽 | `grep missing_topic ~/nav_data/events/$(date +%Y%m%d).jsonl` | 없음. 있으면 8장 |
| 타입 오류 | `grep type_load_failed ~/nav_data/events/$(date +%Y%m%d).jsonl` | 없음 |
| 세션 정보 | `ls ~/nav_data/sessions/` | 오늘 날짜 폴더 |
| 정리·리포트 | `grep maintenance_start ~/nav_data/events/$(date +%Y%m%d).jsonl` (시작 1분 뒤) | 한 줄 있음. 기록기 터미널에 `[nav_data_maintenance] root ...` |
| 정리 규칙 | `ros2 run nav_data_tools nav_data_maintenance --root ~/nav_data --profile <profile> --once --dry-run` | 지울 대상과 리포트할 날짜 목록이 예상과 맞음 (처음엔 없음) |
| 디스크 사용량 | 10분 뒤 `du -sh ~/nav_data/rec/$(date +%Y%m%d)` | 예상 범위(1장)인지 |

### 2.7 안전 비트 이벤트 확인 (안전 신호를 기록하는 경우)

비상정지, 범퍼 같은 비트는 PLC마다 극성이 다릅니다. 프로필에서 지정한 대로 잡히는지 한 번 확인합니다.
비상정지 버튼을 눌렀다 풀고:

```bash
grep '"kind":"safety"' ~/nav_data/events/$(date +%Y%m%d).jsonl | tail -3
```

누른 시각에 이벤트가 있으면 정상입니다. 풀 때 생기면 극성이 반대이므로 `topics.yaml`과
`nightly_rules.yaml`에서 그 필드 앞의 `!`를 넣거나 뺍니다(3.3).

## 3. 우리 로봇에 맞추기 (프로필)

패키지에는 특정 로봇 전용 코드가 없습니다. 로봇마다 다른 것은 모두 설정 파일 몇 개("프로필")로
넘깁니다. 프로필은 저장소의 `profiles/<이름>/` 폴더 하나이고, `nav_data_profiles` 패키지가
설치합니다. 공개 저장소에는 표준 Nav2용 `profiles/default/`만 들어 있습니다.

| 프로필 파일 | 쓰는 곳 | 내용 |
|---|---|---|
| `topics.yaml` | 기록기 (launch `profile:=`) | 기록할 토픽과 이벤트 규칙 (3.2, 3.3) |
| `params.yaml` | 기록기, 정리·리포트 (launch `profile:=`) | 저장 위치, 분할 주기, 캐시, 보관 기간·용량, 리포트 시각 |
| `nightly_rules.yaml` | 야간 리포트 (launch `profile:=`) | 규칙 R1~R10과 임계값 (3.4) |
| `nav_data_analysis.json` | 분석 PC의 Lichtblick (6.1) | 패널 배치 |

프로필에 없는 파일은 `default`의 것을 씁니다. 예를 들어 토픽만 다르면 `topics.yaml` 하나만 두면 됩니다.

### 3.1 프로필 만들기

```bash
cd ~/ros2_ws/src/nav_data_recorder
cp -r profiles/default profiles/<profile>       # 필요 없는 파일은 지워도 됨
# 3.2~3.4대로 고친 뒤
cd ~/ros2_ws && colcon build --packages-select nav_data_profiles
ros2 run nav_data_tools nav_data_profile --list            # 설치된 프로필
ros2 run nav_data_tools nav_data_profile <profile> topics.yaml   # 실제로 쓰일 파일 경로
```

- 프로필은 설치 폴더에 복사되므로 고친 뒤 `nav_data_profiles`를 다시 빌드합니다(몇 초). `--symlink-install`로
  빌드했거나 `profile:=<폴더 경로>`로 넘기면 빌드 없이 반영됩니다.
- 지도 설정 파일처럼 부팅마다 같이 남길 파일은 시작 스크립트에서 `export NAV_DATA_SESSION_FILES=<파일1>:<파일2>`로 넘깁니다.
- 공개 저장소의 사본을 쓰면서 우리 프로필을 같은 저장소에 두려면: `.gitignore`가 `default` 외의
  프로필을 무시하므로 처음 한 번 `git add -f profiles/<profile>`로 등록합니다(이후 새 파일도 `-f`).
  공개 저장소로 push하기 전에는 `tools/check_publish.sh`를 실행합니다. 프로필 폴더에
  `publish_terms.txt`(사내 용어, 한 줄에 하나)를 두면 그 용어가 공개 파일에 섞였는지도 검사합니다.

### 3.2 `topics.yaml`: 기록할 토픽

먼저 로봇에서 실제 토픽을 확인합니다: `ros2 topic list -t`, `ros2 topic hz <토픽>`, `ros2 topic bw <토픽>`.

```yaml
passthrough:
  - {topic: /robot_state, latched: true}        # 상태 토픽 (마지막 값을 파일마다 다시 씀)
  - {topic: /safety_status}
  - {topic: /diagnostics, rate: 1.0}             # 초당 1개로 줄여 기록
scan:
  - {topic: /scan_front, rate: 10.0, decimation: 2}   # beam이 많으면 2~4개마다 하나
grid:
  - {topic: /local_costmap/costmap_raw, rate: 2.0, keyframe_s: 10.0}
```

- 기본 파일은 표준 Nav2 토픽을 담고 있습니다. 로봇에 없는 토픽은 지우거나 그대로 둬도 됩니다
  (시작 30초 뒤 `missing_topic` 이벤트로 한 번 알리고 넘어감).
- 큰 토픽(스캔, costmap)은 `scan:`/`grid:`로 압축합니다. 카메라 영상, 점구름은 기록하지 않는 것을 권합니다.
- global costmap이 크면 변경분만 기록하는 방식을 쓸 수 있습니다(파일 안 주석 참고). 이 방식에서
  global costmap이 costmap 레이어 아래 이름(`/global_costmap/<layer>/costmap_raw`)으로 나오면,
  `global_keyframe`에 `occupancy_output: /global_costmap/costmap`을 넣어 내보낼 때의 토픽 이름을 맞춥니다.

### 3.3 `topics.yaml`: 이벤트 규칙

메시지 타입에 대한 빌드 의존성 없이, 실행 중에 필드 값을 읽어 이벤트를 남깁니다.

```yaml
events:
  - {topic: /robot_state, kind: state, field: data}                  # 값이 바뀔 때
  - {topic: /error_code, kind: error, field: data, when: nonzero}    # 0이 아닌 값이 될 때
  - topic: /safety_status
    kind: safety
    when: rising                                                     # false -> true 일 때
    fields: ['!estop_ok', bumper_pressed]                            # '!' = 평소 true(정상)인 비트
  - topic: /my_task/_action/status                                   # 사용자 action
    kind: state
    when: action_status
    result: {type: my_msgs/action/MyTask, fields: [result.code, result.message],
             fail_field: result.code, fail_values: [3, 4]}           # 이 값이면 error로 기록
```

- `when`: `change`(기본), `nonzero`, `rising`, `any`(메시지마다), `action_status`(goal 상태 변화).
- `kind`: `state`, `error`, `safety`, `mark`. 야간 리포트와 검색에 쓰입니다.
- action 결과는 토픽에 나오지 않으므로, goal이 끝나면 기록기가 `get_result` 서비스로 받아 이벤트와
  `/nav_data/action_result` 토픽에 남깁니다. 실패가 아닌 종료(경로 재설정, 운영자 정지 등)도 ABORTED로
  끝내는 action 서버라면 `fail_field`/`fail_values`로 진짜 실패만 골라냅니다.
- 필드 이름에 `!`를 붙이면 YAML에서 따옴표로 감싸야 합니다.

### 3.4 `nightly_rules.yaml`: 야간 리포트 규칙

| 규칙 | 찾는 것 | 필요한 토픽 (`topics:`) |
|---|---|---|
| R1 멈춤 | 이동해야 하는데 속도 0.02 m/s 미만이 10초 이상 | `odom` (+ `robot_status` 또는 `nav_actions`) |
| R2 감속 | 보낸 속도가 요청 속도의 50% 미만 2초 이상 | `cmd_smoothed`, `cmd_adjusted` |
| R3 recovery | behavior action 실행, `robot_status`가 `recovery_states` 값 | `behavior_actions` (+ `robot_status`) |
| R4 실패 | goal ABORTED, 오류 코드, action 결과의 실패 값 | `nav_actions`, `bt_error_code`, `result_actions` |
| R5 재계획 과다 | 1분당 계획/재계획 횟수가 기준 이상 | `plan`, `current_planner`, `replan_request` |
| R6 경로 이탈 | 로봇과 `/plan` 거리 0.3 m 초과 3초 이상 | `tf`, `plan` |
| R7 정지 명령 | 정지 명령 값, progress checker 정체 | `stop_command`, `is_stuck` |
| R8 안전 | 안전 비트 발생 (`!` = 평소 true) | `safety` |
| R9 모니터링 | flag 필드가 true, 코드 필드가 0이 아닌 값 | `monitoring` |
| R10 수동 표시 | `/nav_data/mark` 수신 | `mark` |

- "이동해야 하는 상태"는 `robot_status` 토픽(std_msgs/String)이 있으면 그 값이 `active_states`에 있을 때,
  없으면 `nav_actions`의 goal이 실행 중(EXECUTING)일 때입니다. recovery 값은 `recovery_states`,
  야간 작업이 기다리는 값은 `driving_states`에 적습니다.
- `result_actions`에는 action 이름(`/my_task`)을 적습니다(`/_action/status`를 붙여도 됨).
- 위치 계산(R6, 이벤트의 pose)은 `/tf`와 `/tf_static`을 이어 `frames.base`까지 갑니다. `frames.base`는
  `base_link`나 `base_footprint`처럼 `map`에서 이어지는 프레임이어야 합니다.
- 토픽을 지정하지 않은 규칙은 자동으로 빠집니다.
- 임계값은 1주일 정도 운영한 뒤 리포트를 보고 조정합니다. 오탐이 많은 규칙(보통 R1, R5)부터 손봅니다.
  R5의 `plan_per_min`은 BT가 평소 재계획하는 주기보다 높게 잡아야 합니다.

## 4. 코드를 업데이트할 때

1. 코드를 갱신합니다(`git pull` 또는 평소 방식).
2. 2.3의 빌드 명령을 다시 실행합니다.
3. 기록기를 다시 시작합니다(로봇 재기동, 또는 기록기 터미널에서 Ctrl+C 후 같은 명령).

- 기록 중이던 파일은 Ctrl+C, 종료 스크립트, OS 종료, 터미널 닫힘 모두에서 정상적으로 닫힙니다. 전원이
  갑자기 꺼져도 마지막 몇 초를 빼고 남습니다(5.5).
- 프로필만 바꾼 경우: `colcon build --packages-select nav_data_profiles`(몇 초) 뒤, 기록기 설정은
  기록기(launch)를 다시 시작하면 반영됩니다. 야간 규칙은 그 뒤 다음 리포트부터 적용됩니다.

## 5. 현장에서 사용하기

### 5.1 문제가 보이면 표시 남기기

```bash
ros2 topic pub --once /nav_data/mark std_msgs/msg/String "{data: '커브에서 멈춤'}"
```

로봇 스택과 같은 ROS 환경의 터미널에서 실행합니다. 표시한 시각은 이벤트 로그에 남고, 그날 밤 앞뒤
5분이 `~/nav_data/pinned/`에 따로 저장되어 90일 보관됩니다. 기본 보관 기간(7일)이 지나도 남으니
나중에 볼 장면에는 표시를 남기는 것이 좋습니다.

### 5.2 문제 시각 찾기

| 어디서 | 내용 |
|---|---|
| `~/nav_data/index/<날짜>_report.md` | 전날의 특이 구간 순위 (R1~R10) |
| `~/nav_data/events/<날짜>.jsonl` | 실시간 이벤트. 예: `grep '"kind":"error"' ~/nav_data/events/20261120.jsonl` |
| `~/nav_data/pinned/` | 수동 표시 구간 (바로 열 수 있는 `export.mcap`과 메모 `note.json`) |

### 5.3 구간 내보내기

```bash
ros2 run nav_data_tools nav_data_export --root ~/nav_data \
    --at "2026-11-20 15:02:10" --before 60 --after 60 -o ~/nav_export/20261120_1502.mcap
```

- 시각은 로봇의 현지 시각입니다. `--from "…" --to "…"`로 구간을 직접 지정해도 됩니다.
- 결과 파일은 표준 메시지(LaserScan, OccupancyGrid 등)로 복원되어 분석 PC에 별도 패키지 없이 엽니다.
- 2분 구간이 보통 수 MB입니다. scp나 USB 등으로 분석 PC로 옮깁니다.

### 5.4 보관과 삭제

| 대상 | 규칙 (기본값) |
|---|---|
| `rec/` | 7일(`keep_days`) 지나면 10분 파일 단위로 삭제. 합계 60 GB를 넘거나 디스크 여유가 15% 미만이면 오래된 것부터 삭제(20% 회복까지) |
| `pinned/` | 90일, 합계 10 GB |
| `events/`, `index/`, `sessions/` | 90일 |

기록기가 도는 동안 15분마다 적용됩니다(2.5). 기록 중인 파일과 최근 2분 안에 바뀐 파일은 지우지 않습니다. 기록기가
꺼져 있으면 새 데이터도 없으므로 정리도 쉽니다.

저장 위치(`root_dir`)를 로봇의 다른 로그 폴더 안에 둘 수도 있습니다(예: `~/ros2_logs/nav_data`). 그 폴더를 나이
기준으로 정리하는 스크립트(`find <폴더> -mtime +60 -exec rm -rf {} +` 같은 것)가 있어도, 정리 프로세스가 15분마다
`nav_data`와 그 아래 폴더의 수정 시각을 갱신하므로 폴더째 지워지지 않습니다. 같은 파일을 양쪽이 지우려 해도 문제없이
넘어갑니다. 다만 그 스크립트의 기준보다 오래된 파일은 그쪽이 지우므로 `pinned_days`, `meta_days`를 그 기준 이하로
맞춥니다. 디스크를 비우려고 로그 폴더를 통째로 지우면(`rm -rf ~/ros2_logs/*`) 기록도 함께 사라집니다.

무엇이 지워질지 미리 보려면:

```bash
ros2 run nav_data_tools nav_data_maintenance --root ~/nav_data --profile <profile> --once --dry-run
```

### 5.5 로봇을 자주 끄고 켤 때

셋업 직후 시험 기간처럼 자주 끄고 켜도 따로 할 일은 없습니다. 켤 때마다 새 기록 폴더(`rec/<날짜>/<시각>_<부팅ID>/`)와
세션 정보가 생기고, 이전 기록은 덮어쓰지 않습니다.

| 끄는 방법 | 마지막 파일 | 잃는 구간 |
|---|---|---|
| 기록기 터미널에서 Ctrl+C | 정상 종료 | 없음 |
| 종료 스크립트(`pkill -f ros2` + `killall -9 <노드>` 형태) | 정상 종료 (기록기가 먼저 닫히고 끝남) | 없음 |
| OS 종료(shutdown), 터미널 창 닫기 | 정상 종료 | 없음 |
| 기록기만 강제 종료(`kill -9`) | 인덱스 없는 파일. 읽기·내보내기는 됨 | 1초 미만. launch가 5초 뒤 다시 띄움 |
| 전원 차단 | 인덱스 없는 파일. 읽기·내보내기는 됨 | 최대 약 `fsync_period_s`(기본 5초) + 1초 |

- 전원 차단에 대비해 기록기는 `fsync_period_s`마다 기록 파일과 이벤트 로그를 디스크에 강제로 씁니다. 이것을
  0으로 하면 OS에 맡기므로 최대 30초 정도를 잃을 수 있습니다.
- 인덱스 없이 끝난 폴더에는 `metadata.yaml`이 없습니다. 도구는 파일을 직접 읽으므로 그대로 둬도 되고,
  `ros2 bag info`로 보고 싶으면 `ros2 bag reindex <폴더> -s mcap`을 실행합니다.
- 꺼져 있던 시간은 기록에 빈 구간으로 남고, 내보내기도 그 구간을 건너 이어서 만듭니다.
- 야간 리포트 시각(02:30)에 꺼져 있었던 날은 다음에 켤 때 처리합니다(최근 3일). 리포트 도중 꺼지면 그 날은
  완료로 치지 않고 다음에 다시 만듭니다.
- 켜고 끌 때마다 생기는 세션 정보(`sessions/`)는 작고 90일 뒤 지워집니다.

## 6. 분석 PC에서 보기

### 6.1 Lichtblick 설치 (한 번)

1. [Lichtblick 릴리스](https://github.com/lichtblick-suite/lichtblick/releases)에서
   `lichtblick-<버전>-linux-amd64.deb`를 받습니다(1.31.0에서 확인).
2. `sudo apt install ./lichtblick-*-linux-amd64.deb`
3. 레이아웃을 등록합니다. 다음 실행 때 자동으로 가져옵니다.

   ```bash
   mkdir -p ~/.lichtblick-suite/layouts
   cp <저장소>/profiles/<profile>/nav_data_analysis.json ~/.lichtblick-suite/layouts/
   # 프로필에 레이아웃이 없으면 profiles/default/nav_data_analysis.json
   ```

### 6.2 Lichtblick으로 보기

```bash
lichtblick --defaultLayout=nav_data_analysis ~/nav_export/20261120_1502.mcap
```

| 패널 | 내용 |
|---|---|
| 3D (왼쪽) | 지도, global/local costmap, 스캔(거리별 색), 경로, footprint. 로봇 위치를 따라감 |
| Plot | odom 속도와 cmd_vel 단계별 값 (controller, smoother, 최종) |
| State Transitions | navigate action 상태, collision monitor 동작 |
| Log / action result | rosout, action 결과 |

조작: 스페이스바 재생/정지, 화살표 0.1초, Shift+화살표 0.01초, Alt+화살표 0.5초 이동. 타임라인을
클릭해 원하는 시각으로 갑니다. 패널 톱니바퀴로 토픽 표시를 켜고 끕니다. 로봇 전용 토픽을 보려면 패널에
추가한 뒤 레이아웃을 내보내(Layouts > Export) 프로필에 저장해 둡니다.

### 6.3 rviz2로 보기

```bash
ros2 bag play ~/nav_export/20261120_1502.mcap --clock
rviz2 --ros-args -p use_sim_time:=true      # 평소 쓰는 rviz 설정을 use_sim_time으로 열어도 됨
```

### 6.4 원본 기록을 분석 PC에서 직접 내보내기

로봇의 `~/nav_data/rec/`를 분석 PC의 `./nav_data/rec/`로 폴더째 가져와(예:
`mkdir -p nav_data && scp -r <로봇>:nav_data/rec nav_data/`) 분석 PC에서 내보낼 수도 있습니다. 이때 분석 PC에
`nav_data_recorder_msgs`, `nav_data_recorder`, `nav_data_tools`와 로봇의 메시지 패키지가 빌드·source
되어 있어야 모든 토픽이 복원되고 Lichtblick에서 해석됩니다.

```bash
ros2 run nav_data_tools nav_data_export --root ./nav_data --from "2026-11-20 14:55:00" --to "2026-11-20 15:10:00" -o out.mcap
# 또는 파일을 직접 지정: --files ./nav_data/rec/20261120/083015_ab12/*.mcap
# 출력 토픽 이름 바꾸기: --rename /global_costmap/my_layer/costmap=/global_costmap/costmap
```

## 7. 설정 파일과 옵션

| 파일 | 바꾸는 것 |
|---|---|
| `topics.yaml` (프로필) | 기록할 토픽, 주기 제한, 스캔·costmap 압축, 이벤트 규칙 |
| `params.yaml` (프로필) | `nav_data_recorder`: 저장 위치, 파일 분할 주기(기본 600초), 쓰기 캐시. `nav_data_maintenance`: 보관 기간·용량, 정리 주기, 야간 리포트 시각 |
| `mcap_writer.yaml` (패키지) | mcap 청크 크기. 작을수록 전원 차단 시 잃는 구간이 짧음 |
| `nightly_rules.yaml` (프로필) | 야간 리포트 규칙 R1~R10과 임계값 |

보관 기간과 용량 바꾸기 (예: 14일, 100 GB): 프로필 `params.yaml`에서 고치고 `nav_data_profiles`를 다시 빌드한 뒤
기록기를 다시 시작합니다.

```yaml
nav_data_maintenance:
  ros__parameters:
    keep_days: 14.0
    cap_gb: 100.0
```

| `nav_data_maintenance` 설정 | 기본값 | 뜻 |
|---|---|---|
| `keep_days` | 7.0 | 이보다 오래된 기록 파일 삭제 |
| `cap_gb` | 60.0 | `rec/` 합계 상한, 넘으면 오래된 것부터 |
| `min_free` / `target_free` | 0.15 / 0.20 | 디스크 여유가 15% 미만이면 20%가 될 때까지 오래된 것부터 |
| `pinned_days` / `pinned_cap_gb` | 90 / 10 | 수동 표시 보존 구간 |
| `meta_days` | 90 | `events/`, `index/`, `sessions/` |
| `prune_period_min` | 15 | 정리 주기 |
| `nightly_time` | `"02:30"` | 야간 리포트 시각(로봇 현지), `""`이면 끔 |
| `catch_up_days` | 3 | 꺼져 있어 못 한 날을 며칠 전까지 처리할지 |
| `nightly_max_wait_h` / `nightly_retry_min` | 6 / 60 | 주행 중이면 기다리는 최대 시간 / 못 했을 때 다시 시도 간격 |
| `start_delay_s` | 60 | 기동 후 첫 실행까지 |

수동 실행: `nav_data_maintenance --once`(위 설정을 한 번 적용. 기록기 launch가 돌고 있으면 겹치지 않도록 바로
끝남), 같은 옵션 이름으로 덮어쓰기(예: `--keep-days 14`), `--dry-run`(아무것도 지우거나 만들지 않고 할 일만 출력,
기록기가 돌고 있어도 됨). `nav_data_prune`도 단독으로 쓸 수 있습니다(`--days`, `--cap-gb`, `--dry-run` 등).

`nav_data_nightly` 옵션: `--date YYYYMMDD`(기본 어제), `--profile <이름|경로>`(기본 `$NAV_DATA_PROFILE`),
`--rules <파일>`(프로필 대신 이 파일, 기본 `$NAV_DATA_RULES`),
`--no-wait`(주행 중에도 바로 실행. 로봇이 없는 분석 PC에서 돌릴 때도 이것을 붙임: 로봇 상태를 모르면
기다리기 때문), `--max-wait-h`(기본 6, 넘기면 종료 코드 3), `--no-pin`.

## 8. 문제 해결

| 증상 | 확인할 것 / 조치 |
|---|---|
| `~/nav_data/rec/`에 오늘 폴더가 없음 | 기록기 터미널의 오류 확인. 시작 스크립트에서 로봇 스택과 같은 환경을 쓰지 않았거나 빌드가 안 된 경우가 많음 |
| `missing_topic` 이벤트 | 그 토픽을 내는 노드가 없거나 이름이 다름. 원래 없는 기능이면 `topics.yaml`에서 지우고, 이름이 다르면 고침 |
| `type_load_failed` 이벤트 | 해당 메시지 패키지가 source한 작업공간에 없음. 그 패키지를 빌드 |
| 기록기가 계속 재시작 | 기록기 터미널 로그 확인. launch가 5초마다 다시 띄움 |
| 리포트(`index/`)가 안 생김 | 기록기 터미널에서 `[nav_data_maintenance] nightly <날짜>` 줄 확인. 02:30 전이거나 로봇이 주행 중이면(최대 6시간 대기, 못 하면 다음 날 02:30) 아직 안 생기는 것이 정상. `nightly_time`이 `""`인지, launch에 `maintenance:=false`가 있는지 확인 |
| 리포트에 R1 멈춤이 너무 많음 | `active_states` 또는 `nav_actions` 확인, `R1.min_s` 상향 |
| 디스크가 계속 늘어남 | `grep pruned ~/nav_data/events/*.jsonl \| tail`, 5.4의 `--dry-run`으로 삭제 대상 확인 |
| 전원이 갑자기 꺼진 뒤 마지막 파일 | 인덱스 없이 남은 파일도 대부분 읽힘(5.5). export가 건너뛰면 mcap CLI(별도 설치)로 `mcap recover in.mcap -o out.mcap` |
| Lichtblick에서 탐색 직후 지도·costmap에 빨간 오류 | 레이아웃 `nav_data_analysis`(변환 미리 읽기 켜짐)를 쓰는지 확인. 그래도 보이면 재생을 0.1초만 하면 사라짐 |
| Lichtblick에서 일부 토픽만 해석 안 됨 | 그 메시지 패키지 없이 내보낸 파일. 6.4의 패키지를 빌드한 환경에서 다시 내보내기 |
| 기록 시작 직후 safety, error 이벤트 1건씩 | 시작 시점의 실제 상태(이미 켜진 안전 비트, 남아 있는 오류 코드)가 기록된 것 |

## 9. 끄기와 제거

| 하려는 것 | 방법 |
|---|---|
| 잠시 기록 중지 | 기록기 터미널에서 Ctrl+C. 다음 기동 때 다시 시작됨 |
| 기록 끄기 | 시작 스크립트에서 2.4의 줄을 주석 처리 |
| 정리·리포트만 끄기 | launch에 `maintenance:=false` (리포트만 끄려면 `nightly_time: ""`) |
| 데이터 삭제 | `rm -rf ~/nav_data` (되돌릴 수 없음. 필요한 구간은 먼저 내보내기) |

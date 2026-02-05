네, `fleet_decision_node_final.py`의 각 메서드에 대한 **심층 기술 명세서(Detailed Technical Specification)**를 작성해 드립니다.

이 문서는 개발자가 코드를 유지보수하거나 디버깅할 때, 각 함수가 **어떤 입력**을 받아 **어떤 논리**를 거쳐 **무엇을 반환**하는지 명확히 이해하도록 돕는 데 초점을 맞췄습니다.

---

# **Fleet Decision Node 메서드 상세 명세서**

## **1. Helper Class: `WaitManager**`

비동기(Non-blocking) 타이머 로직을 캡슐화한 클래스입니다. `rclpy.spin()` 루프를 방해하지 않고 시간을 측정합니다.

### **1.1 `start_wait(self, duration_sec, stop_type)**`

* **역할:** 대기 타이머를 시작합니다.
* **로직:**
1. 현재 대기 중이 아니거나(`not is_waiting`), 대기 사유(`stop_type`)가 변경된 경우에만 타이머를 리셋합니다.
2. 이미 같은 타입으로 대기 중이라면, `start_time`을 갱신하지 않고 유지합니다. (불필요한 타이머 초기화 방지)


* **매개변수:**
* `duration_sec` (float): 대기할 시간 (초).
* `stop_type` (MovingStopType): 대기를 유발한 원인 (로그용).



### **1.2 `check_finished(self) -> bool**`

* **역할:** 설정된 대기 시간이 지났는지 확인합니다.
* **로직:** `(현재 시간 - 시작 시간)`을 계산하여 `duration_sec`보다 크거나 같으면 `True`를 반환합니다.
* **반환값:** 타임아웃 여부 (`bool`).

---

## **2. Main Class: `FleetDecisionNode**`

### **2.1 Callbacks (이벤트 핸들러)**

#### **A. `on_agents(self, msg)**`

* **역할:** 주변 로봇들의 상태 정보를 수신하고 캐싱합니다.
* **입력:** `MultiAgentInfoArray` (토픽: `/multi_agent_infos`)
* **상세 로직:**
1. **딕셔너리 변환:** `List` 형태의 에이전트 정보를 `{machine_id: AgentInfo}` 형태의 딕셔너리(`_cached_agents`)로 변환합니다. 이는 추후 `O(1)` 검색 성능을 위함입니다.
2. **자가 상태 갱신:** 본인(`my_machine_id`)의 정보가 포함되어 있다면, `reroute` 플래그를 확인하여 자신의 상태(`_is_reroute_status`)를 `EXECUTE` 또는 `NONE`으로 갱신합니다.



#### **B. `on_collision(self, msg)**`

* **역할:** 충돌 감지 노드(`PathValidator`)로부터 트리거되는 **메인 의사결정 루프**입니다.
* **입력:** `PathAgentCollisionInfo` (토픽: `/path_agent_collision_info`)
* **상세 로직:**
1. **유효성 검사:** 메시지가 비어있으면 `RUN` 상태로 리셋합니다.
2. **TYPE_11 감지:** `msg.machine_id` 리스트가 비어있으면, "알 수 없는 장애물"로 판단하여 즉시 `TYPE_11` 대기 로직을 수행합니다.
3. **타겟 선정:** 가장 먼저 충돌이 예상되는(인덱스 0) 에이전트의 ID와 충돌 좌표 `(x, y)`를 추출합니다.
4. **판단 호출:** 추출된 정보를 `_decide_obstacle_action()`에 전달하여 행동을 결정합니다.
5. **실행:** 결정된 행동(`Command`)을 `_execute_command()`로 전달합니다.



---

### **2.2 Core Logic (의사결정 엔진)**

#### **`_decide_obstacle_action(self, target_id, collision_xy)`**

* **역할:** 상황별 조건을 평가하여 최적의 행동을 반환합니다.
* **입력:**
* `target_id` (int): 충돌 대상 로봇 ID.
* `collision_xy` (tuple): 충돌 예상 지점 (x, y).


* **반환값:** `(MovingCommand, MovingStopType)` 튜플.
* **Decision Tree (판단 순서):**
1. **Reroute 상태 전이:** 만약 `PREPARE` 상태라면 `EXECUTE`로 변경.
2. **정보 부재:** 타겟 ID가 캐시된 정보에 없으면 `TYPE_11` 반환.
3. **Manual Mode (`TYPE_1`):** 상대가 수동 운전 중이면 2초 대기.
4. **Same Path:**
* 상대가 정지 중이면 `TYPE_12` (2초 대기).
* 상대가 이동 중이면 `TYPE_2` (300초 대기 - 추종).


5. **Different Path (교차/대향):**
* **Reroute 사용 시:**
* 상대 Reroute & 경로 겹침 → **`REROUTE` (TYPE_3, 즉시 회피)**.
* 상대 Reroute & 안 겹침 → 내 상태/ID 비교 후 `TYPE_5` or `TYPE_6` or `TYPE_7`.
* 상대 Reroute 아님 → 내 상태/ID 비교 후 `TYPE_8` or `TYPE_9` or `TYPE_10`.


* **Reroute 미사용 시:**
* 단순 ID 비교로 `TYPE_5` or `TYPE_6`.







---

### **2.3 Helper Functions (상태 판별기)**

정교하게 개선된 판별 함수들입니다.

#### **A. `_check_vehicle_path(self, agent)**`

* **역할:** 나와 상대방이 **동일한 경로(같은 방향)**로 주행 중인지 판단합니다.
* **개선된 로직:**
1. **벡터 내적(Dot Product):** 나의 경로(`truncated_path`)와 상대 경로의 시작점-끝점 벡터를 계산합니다.
2. 두 벡터의 내적 값이 `0.707` (cos 45°) 이상이면 **같은 방향(SAME_PATH)**으로 봅니다. 이는 곡선 주행에서도 단순 헤딩(Yaw) 비교보다 정확합니다.
3. **Fallback:** 경로 정보가 없으면 기존의 Yaw 각도 차이(45° 이내)를 사용합니다.



#### **B. `_check_vehicle_rerouting_path(self, agent, collision_xy)**`

* **역할:** 상대방이 우회(Reroute) 중일 때, 그 우회 경로가 **나의 충돌 지점**을 덮어쓰는지 확인합니다.
* **로직:**
* 상대방의 `truncated_path`를 구성하는 모든 점(Pose)들과 `collision_xy` 사이의 거리를 계산합니다.
* 최단 거리가 `0.5m` 이내라면 **"경로가 겹친다"**고 판단하여 `True`를 반환합니다.



#### **C. `_check_vehicle_manual_mode(self, agent)**`

* **역할:** 상대방이 수동 운전 중인지 확인합니다.
* **로직:**
* `mode` 문자열이 "manual" 인지 확인.
* **OR** `status.phase`가 `MANUAL_RUNNING` 또는 `MANUAL_COMPLETE` 인지 확인 (이중 체크).



#### **D. `_check_vehicle_status(self, agent)**`

* **역할:** 상대방이 실제로 정지해 있는지 이동 중인지 판단합니다.
* **로직:**
* 단순히 상태(`phase`)가 `MOVING`이라도, 실제 속도(`twist.linear`, `twist.angular`)가 `0.01` 미만이면 **물리적으로 정지**한 것으로 간주하여 `True`(Stopped)를 반환합니다.



---

### **2.4 Execution & Pub/Sub (실행부)**

#### **`_execute_command(self, command, stop_type)`**

* **역할:** 결정된 명령을 실제 ROS 토픽으로 변환하여 발행하고, 타이머를 제어합니다.
* **로직:**
1. **상태 변경 감지:** 이전 상태(`_pre_moving_stop_type`)와 다를 경우 로그를 출력합니다.
2. **REROUTE 처리:** 즉시 `request_replan`을 발행하고 타이머를 리셋합니다.
3. **WAIT 시간 매핑:** `stop_type`과 파라미터 설정에 따라 대기 시간(`wait_time`)을 결정합니다.
* *예: TYPE_5 + Reroute ON -> 450초*


4. **타이머 제어 (비동기):**
* `wait_manager`가 대기 중이 아니면 -> 대기 시작 & `cmd_stop` 발행.
* 대기 중이고 시간이 만료됨 -> `request_replan` 발행 (재탐색 요청).
* 대기 중이고 시간 남음 -> `cmd_stop` 유지.





#### **`_reset_wait_and_run(self)`**

* **역할:** 충돌 위험이 사라졌을 때 호출됩니다.
* **동작:**
* 타이머 리셋 (`wait_manager.reset()`).
* 주행 명령 발행 (`cmd_run` -> True).
* 상태 초기화 (`TYPE_NONE`).



---

### **3. 데이터 흐름 요약 (Data Flow Summary)**

1. **Input:** `PathValidator`가 "충돌 예상!" 메시지(`PathAgentCollisionInfo`)를 보냄.
2. **Process 1:** `FleetDecisionNode`가 메시지를 받아 `target_id`와 `collision_xy`를 파싱.
3. **Process 2:** `_cached_agents`에서 해당 ID의 상세 상태(Reroute 여부, 속도, 경로 등)를 조회.
4. **Decision:** `_decide_obstacle_action`에서 If-Else 트리를 타고 내려가 `Command(WAIT/REROUTE)`와 `Type(1~12)` 결정.
5. **Execution:** `_execute_command`에서 타이머를 걸고 로봇을 멈추거나(`cmd_stop`), 즉시 경로를 다시 짬(`request_replan`).
6. **Resolution:** 상황이 해소되면 `_reset_wait_and_run`이 호출되어 다시 주행(`cmd_run`).
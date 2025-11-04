좋아! 아래는 **현재 `fleet_decision_node.py`의 모든 파라미터**에 대한 의미, 동작, 권장 범위, 튜닝 요령을 한 번에 볼 수 있게 정리한 **설명 + 튜닝 가이드**야.
(이 노드는 *이벤트 기반*이라 기본은 `RUN`, 특정 이벤트(`/path_agent_collision_info`, `/replan_flag`)가 들어올 때만 1회 명령 펄스를 내보내는 구조야. 타이머는 *복귀 판단과 타임아웃*만 담당.)

---

# 1) ID / 프레임

* `global_frame` (string, 기본 `"map"`)
  노드의 기준 좌표계. 외부 메시지의 frame 불일치가 있을 경우, 이 값을 기준으로 판단.
  • 튜닝: 일반적으로 `map` 유지.

* `my_machine_id` (int, 기본 `1`)
  본인 에이전트 식별자. `right_of_way_score()` 및 우선순위 가중치에서 사용.
  • 튜닝: 실제 차량 ID로 설정.

---

# 2) TTC(충돌 시간) 결합 파라미터

* `w1_ttc` (float, 기본 `1.0`)
  `/path_agent_collision_info.ttc_first`(직접 TTC)의 가중치.
  • ↑크게: 직접 TTC 신뢰. 센서/알고리즘이 안정적일 때.
  • ↓작게: 직접 TTC가 noisy하거나 빈번히 -1일 때.

* `w2_alt` (float, 기본 `0.8`)
  대체 TTC(거리/접근속도로 계산)의 가중치.
  • ↑크게: 상대 속도·방향 기반 근사치를 더 신뢰.
  • ↓작게: 직접 TTC만 믿고 싶을 때.

* `T_min` (float, 기본 `0.5` [s])
  0에 가까운 TTC의 폭발적 영향 방지용 최소 상한(분모 클램핑).
  • 추천: 0.3~1.0

* `d_min` (float, 기본 `0.2` [m])
  거리 항의 분모 클램핑. 근접 오차/잡음을 방지.
  • 추천: 0.1~0.5

---

# 3) 충돌 심각도(Severity) 가중치

* `a1_invT` (float, 기본 `1.2`)
  `1/T_eff` 항 가중치. **시간이 급박**할수록 심각도↑.
  • ↑: 촉박한 TTC에 더 민감.
  • 권장: 0.8~2.0

* `a2_invd` (float, 기본 `0.2`)
  `1/distance` 항 가중치. **공간적으로 가깝**다면 심각도↑.
  • ↑: 근접 충돌에 더 민감.
  • 권장: 0.1~0.6

* `a3_heading` (float, 기본 `0.5`)
  진행방향 관계(정면/교차/동차선)를 반영하는 보너스 가중치.
  • ↑: 정면·교차 충돌에 더 강하게 반응.
  • 권장: 0.3~1.0

* `a4_vclosing` (float, 기본 `0.3`)
  상대 접근속도(Closing speed) 항 가중치.
  • ↑: 빠르게 다가오는 상대에 민감.
  • 권장: 0.2~0.8

---

# 4) 양보 우선순위(Yield Priority) 가중치

* `b1_mode` (float, 기본 `0.7`)
  상대가 `manual`이면 **상대 priority**를 올림(내가 양보).
  • ↑: 수동운전/인력 개입 차량 우대 강화.
  • 권장: 0.3~1.2

* `b2_rowgap` (float, 기본 `0.9`)
  `right_of_way_score` 차이(상대-나)를 반영. 작은 machine_id/점유 구간(occupancy) 등에 가중.
  • ↑: 규칙적 우선권을 더 존중.
  • 권장: 0.5~1.5

* `b3_reroute` (float, 기본 `0.8`)
  상대가 `reroute==true`면 상대 priority↑. 이미 우회 중인 상대를 **먼저 보내주려는** 정책.
  • ↑: 흐름 복구에 유리(플로우↑).
  • 권장: 0.4~1.2

* `b4_pathsearch` (float, 기본 `0.4`)
  상대가 `PATH_SEARCHING`이면 상대 priority↑.
  • 권장: 0.2~0.8

* `b5_occupancy` (float, 기본 `0.5`)
  상대가 점유(occupancy) 구간이면 상대 priority↑.
  • 권장: 0.3~1.0

* `b6_id` (float, 기본 `0.2`)
  machine_id 작은 쪽을 우선권 높이는 규칙의 가중.
  • 권장: 0.1~0.5

* `kappa` (float, 기본 `0.6`)
  `(1 + kappa * yprio)`로 Severity에 곱해 최종 점수 반영.
  • ↑: 우선권 로직의 영향이 커짐.
  • 권장: 0.3~1.0

---

# 5) 상태 진입 임계(Enter Thresholds)

* `T_slow` (float, 기본 `6.0` [s])
  `T_eff < T_slow`면 `SLOWDOWN` 후보.
  • ↑: 더 이른 감속(보수적). ↓: 더 늦게 감속(공격적).

* `T_yield` (float, 기본 `2.5` [s])
  `T_eff < T_yield` **또는** `yprio >= Y_th`면 `YIELD`.
  • ↑: 정말 급박할 때만 정지/양보. ↓: 더 쉽게 양보.

* `yield_priority_thresh (Y_th)` (float, 기본 `0.8`)
  우선권 점수가 이 값 이상이면 `YIELD`.
  • ↑: 우선권 높아도 쉽게 정지하지 않음. ↓: 우선권에 잘 양보.

---

# 6) 복귀 히스테리시스(Exit/Resume) & K-연속 기준

* `T_resume_slow` (float, 기본 `6.5` [s])
  `SLOWDOWN` → `RUN` 복귀 시 **안전 임계**. (들어올 때보다 약간 큼: 히스테리시스)
  • 권장: `T_slow + 0.3~1.0`

* `T_resume_yield` (float, 기본 `3.5` [s])
  `YIELD` → `RUN` 복귀 시 안전 임계.
  • 권장: `T_yield + 0.7~1.5`

* `T_resume_stop` (float, 기본 `5.0` [s])
  `STOP` → `RUN` 복귀 시 안전 임계(가장 보수적으로).
  • 권장: 4.0~7.0

* `Y_exit` (float, 기본 `0.5`)
  복귀 시 우선권 점수는 이 값 **미만**이어야 함.
  • ↑: 복귀 더 보수적(더 낮은 yprio에서만 복귀). ↓: 쉽게 복귀.

* `K_slow_clean` (int, 기본 `2`) / `K_yield_clean` (기본 `3`) / `K_stop_clean` (기본 `3`)
  안전 기준이 **연속 N회** 충족되어야 `RUN` 펄스를 보냄(채터링 방지).
  • ↑: 튼튼한 복귀(느림). ↓: 빠른 복귀(가끔 깜빡일 수 있음).

---

# 7) Idle 기반 자동 복귀

* `resume_idle_sec` (float, 기본 `1.5` [s])
  `SLOWDOWN`/`YIELD` 상태에서 **최근 충돌 이벤트 자체가 없는** 시간이 이 값을 넘기면 `RUN` 펄스(타이머 블록).
  • ↑: 충돌 이벤트 간헐 시에도 쉽게 복귀. 너무 크면 불필요한 대기.

---

# 8) 상태 타임아웃(최대 지속 시간)

* `resume_timeout_slow` (float, 기본 `6.0` [s])
  `SLOWDOWN`이 이 시간 이상 지속되면 강제 `RUN`.
  • ↑: 장시간 감속 유지 허용. ↓: 흐름 우선, 빨리 복귀.

* `resume_timeout_yield` (float, 기본 `10.0` [s])
  `YIELD`가 너무 오래면 강제 `RUN`. (교차점 정체 방지)

* `resume_timeout_stop` (float, 기본 `15.0` [s])
  `STOP` 장기 정지 방지용 강제 복귀.

> **팁**: 센서/맵이 안정적이고 주행로 여유가 있으면 timeout을 좀 더 크게, 혼잡한 협소 구간이면 작게 잡아 흐름을 살릴 수 있어.

---

# 9) 디바운스 / 무시 윈도우

* `agent_event_silence_sec` (float, 기본 `1.0` [s])
  동일 `(machine_id, type_id)`에 대한 충돌 이벤트 **재처리 금지 시간**.
  • ↑: 동일 상대 이벤트 스팸 차단. 너무 크면 반응이 둔해질 수 있음.

* `replan_ignore_sec_after_agent` (float, 기본 `0.5` [s])
  **에이전트 충돌 이벤트 직후** 들어오는 외부 `/replan_flag`를 무시하는 시간.
  • ↑: 불필요한 재계획 억제. ↓: 외부 replan 요청을 빠르게 수용.

* `run_pulse_silence_sec` (float, 기본 `0.5` [s])
  `/cmd/run` 펄스를 너무 자주 내보내지 않도록 하는 **디바운스 시간**.
  • ↑: 컨트롤러 부하/채터링 감소. ↓: 더 즉각적인 RUN 명령 허용.

---

# 10) 토픽 이름

* 입력
  `topic_collision`(`/path_agent_collision_info`), `topic_agents`(`/multi_agent_infos`), `topic_replan_flag`(`/replan_flag`)

* 출력(상태/요청)
  `topic_decision_state`(`/decision_state`)
  `topic_request_replan`(`/request_replan`)
  `topic_request_reroute`(`/request_reroute`)

* 출력(제어 펄스, Bool true 1회)
  `topic_cmd_run`(`/cmd/run`), `topic_cmd_slowdown`(`/cmd/slowdown`),
  `topic_cmd_yield`(`/cmd/yield`), `topic_cmd_stop`(`/cmd/stop`)

> **주의**: RUN/감속/양보/정지 등의 실제 속도 제어는 **바깥 컨트롤러**가 이 펄스를 받아서 수행해야 함(이 노드는 “명령 신호 발생기”).

---

## 빠른 스타트 프리셋

* **보수적(안전 우선)**

  ```
  T_slow=7.0, T_yield=3.0, Y_th=0.9,
  T_resume_slow=7.8, T_resume_yield=4.2, Y_exit=0.4,
  K_slow_clean=3, K_yield_clean=4,
  resume_idle_sec=2.0,
  resume_timeout_slow=8.0, resume_timeout_yield=12.0
  ```

  → 감속/양보를 빨리 하고 복귀는 늦게.

* **공격적(흐름 우선)**

  ```
  T_slow=5.0, T_yield=2.0, Y_th=0.7,
  T_resume_slow=5.5, T_resume_yield=3.0, Y_exit=0.6,
  K_slow_clean=1, K_yield_clean=2,
  resume_idle_sec=1.0,
  resume_timeout_slow=4.0, resume_timeout_yield=7.0
  ```

  → 감속/양보는 늦게, 복귀는 빠르게.

---

## 증상별 튜닝 가이드 (현장에서 바로 쓰기)

* **양보가 너무 잦다**

  * `T_yield`↑, `Y_th`↑, `a1_invT`↓ 또는 `kappa`↓
  * 복귀가 늦으면 `T_resume_yield`↓, `Y_exit`↑, `K_yield_clean`↓

* **감속이 자주 걸려 흐름이 늘 막힌다**

  * `T_slow`↓, `a1_invT`↓, `a2_invd`↓, `a3_heading`↓
  * 복귀 빠르게: `T_resume_slow`↓, `K_slow_clean`↓

* **RUN 복귀가 들쭉날쭉(채터링)**

  * `K_*_clean`↑, `Y_exit`↑, `T_resume_*`↑, `run_pulse_silence_sec`↑

* **교차점에서 서로 양보만 하며 서서히 교착**

  * 상대 reroute 우대: `b3_reroute`↑
  * 타임아웃 단축: `resume_timeout_yield`↓, `resume_timeout_stop`↓
  * 우선권 차등 강화: `b2_rowgap`↑

* **외부 `/replan_flag`가 너무 자주 반응한다**

  * `replan_ignore_sec_after_agent`↑
  * 혹은 상위 노드에서 flag 발생 조건 보수화

* **동일 상대에 대해 과도한 반복 반응**

  * `agent_event_silence_sec`↑

---

## 내부 상호작용(중요 포인트)

* **최종 점수** = Severity × (1 + `kappa` × YieldPriority)
  → `a*` 계열(물리적 급박함)과 `b*` 계열(우선권/운영정책)이 **곱으로 결합**되므로,
  어느 한쪽을 올리면 **상대적으로 다른 쪽 변화보다 더 크게** 결과에 영향을 줄 수 있어.

* 복귀는 3가지 경로로 발생:

  1. **클린 K-연속**: `T_resume_*`, `Y_exit` 조건이 **여러 번 연속** 충족
  2. **Idle resume**: 충돌 이벤트 자체가 일정 시간 없음
  3. **Timeout**: 상태가 너무 오래 지속될 때 강제 복귀

* `REROUTE`와 `REPLAN`은 **명령 펄스만 발행** (실제 경로 재탐색/우회는 외부 노드가 수행)

---

## 권장 초기값(운영 무난 버전)

```yaml
# TTC 결합
w1_ttc: 1.0
w2_alt: 0.8
T_min: 0.5
d_min: 0.2

# Severity
a1_invT: 1.2
a2_invd: 0.2
a3_heading: 0.5
a4_vclosing: 0.3

# Yield priority
b1_mode: 0.7
b2_rowgap: 0.9
b3_reroute: 0.8
b4_pathsearch: 0.4
b5_occupancy: 0.5
b6_id: 0.2
kappa: 0.6

# Enter
T_slow: 6.0
T_yield: 2.5
yield_priority_thresh: 0.8

# Resume (Exit)
T_resume_slow: 6.5
T_resume_yield: 3.5
T_resume_stop: 5.0
Y_exit: 0.5
K_slow_clean: 2
K_yield_clean: 3
K_stop_clean: 3

# Idle/Timeout
resume_idle_sec: 1.5
resume_timeout_slow: 6.0
resume_timeout_yield: 10.0
resume_timeout_stop: 15.0

# Debounce/ignore
agent_event_silence_sec: 1.0
replan_ignore_sec_after_agent: 0.5
run_pulse_silence_sec: 0.5
```

---

필요하면 네 환경(맵 폭/교차 빈도/로봇 가감속 성능/센서 신뢰도)에 맞춰 **두 세트(안전/흐름)**를 준비하고, 시간대나 구역별로 스위칭하는 것도 좋아. 세부 상황(예: 특정 구역에서만 `b5_occupancy`나 `b3_reroute`를 높인다)을 추가하면 전체 **throughput**을 더 안정적으로 끌어올릴 수 있어.

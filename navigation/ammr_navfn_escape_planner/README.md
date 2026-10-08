# ammr_navfn_escape_planner (2026-10-05, D17)

`ammr_navfn_planner` (그대로 둠) 에 **recovery escape** 를 더한 패키지. 현장 nav2_params.yaml 의 `RecoveryGridBased2` 만 이 플러그인을 쓴다.

- 현장 증상: recovery 에서 planner 는 경로를 냈는데 MPPI(RecoveryFollowPath1)가 못 움직임. 옆 벽 3~8 cm 에서 정사각 차체(0.635x0.63)는 제자리 회전이 막히는데,
  NavFn(점 로봇)은 처음부터 크게 돌아야 하는 경로를 줘서 MPPI 가 105(진전 없음)로 끝났다.
- escape (`escape_enable: true` 일 때만): 출발 자세(= 로봇 현재 자세)에서 제자리 회전이 막혀 있으면 **지금 방향(앞/뒤) 직선 구간만** 돌려준다.
  (a) goal 이 지금 방향 직선 위면 goal 까지, (b) 아니면 처음 회전 가능한 지점 + `escape_margin` (최소 `escape_min_straight`) 까지. 다음 회복 주기가 회전 가능한 자세에서 평소대로 계획한다.
  회전 가능 = padded 외접원 안에 LETHAL·unknown 없음. 직진 검사 = unpadded footprint 의 앞장서는 변(양 끝 `escape_edge_inset`, 기본 격자 1칸 안쪽)만 격자/2 간격. costmap mutex 를 잡고 검사.
  회전 가능하거나 출발이 로봇 자세가 아니면 원래 NavFn 그대로.
- 파라미터 (모두 실행 중 변경 가능): `escape_enable` (false), `escape_max_dist` 1.5, `escape_step` 0.05, `escape_margin` 0.15, `escape_min_straight` 0.5, `escape_edge_inset` -1 (= 격자 1칸), `escape_max_len` -1 (내보내는 직선 길이 상한, 0 이하 = 없음; 10-06 사용자 '비정상 상황은 최소 이동' 으로 현장 값 0.45).
- 로그: `[<plugin 이름> escape] start cannot rotate -> straight forward|backward ...`
- sim 검증: `src/amr-gz-sim-example/mobile_robot_gz_sim/scripts/verify/MPPI_RECOVERY_REPRO_1003.md` (옆 5·8 cm 0/16 실패, 1대 현장 BT 105 51→9, 5대 회귀 없음).

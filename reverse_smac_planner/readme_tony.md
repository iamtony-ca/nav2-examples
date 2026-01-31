# for reverse planner for dubin smac planner
modified in reverse_smac_planner/src/node_hybrid.cpp





# examples for yaml
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "reverse_smac_planner::SmacPlannerHybrid"
      motion_model_for_search: "DUBIN" # (수정된 코드 사용)
      
      # [핵심] 후진이 기본 주행이므로 페널티 없음
      reverse_penalty: 1.0 
      
      # [참고] 방향 전환(직진하다가 회전)에 대한 페널티
      # 너무 자주 꼬불거리는게 싫으면 이 값을 1.2 ~ 2.0 정도로 올리세요.
      change_penalty: 50.0 # DUBIN은 전/후진 전환(Cusp)이 없으므로 사실상 의미 없음
      non_straight_penalty: 1.2 # 직진(후진직진)보다 회전(후진회전)을 약간 더 싫어하게 설정
      
      # [이전 답변에서 강조] 전진 경로 생성을 막기 위해 확장 기능 끄기
      analytic_expansion_ratio: 1.2 
      analytic_expansion_max_length: 0.7


```
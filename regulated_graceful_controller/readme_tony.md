custom goal_checker(stable_stopped_goal_checker) 에 dependency를 갖도록 소스코드 수정.

graceful_controller 자체적으로 goal_tolerance 계산을 안 하고, goal_checker 의 결과만 받아서 로직에 사용.

modified in regulated_graceful_controller/src/graceful_controller.cpp



# dynamic_avoidance 추가
collision_check_duration
lookahead_extension_dist
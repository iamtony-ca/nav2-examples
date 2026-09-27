#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""navigation_manager_node 실행 진입점 — 명령 경로 선택 (2026-09-27).

config/navigation_manager.yaml 의 ``use_command_services`` 로 둘 중 하나를 띄운다.
  - false (기본): navigation_manager_node.py     — 기존 토픽 방식 (move/pause/resume/main_stop/reset 토픽). 현장 검증 완료본 그대로.
  - true        : navigation_manager_cmd_node.py — 0923 action/service 방식 (navigate_command action,
                  navigation_control service). 기존 토픽 구독도 함께 가진다.

winros_bridge.yaml 의 ``use_command_services`` 와 **같은 값**이어야 한다.
bridge 만 true 이면 bridge 가 부를 navigate_command action 서버가 없어서 주행 명령이 전달되지 않는다.

값은 노드를 만들기 전에 정해야 하므로 ROS 인자(--params-file, -p)를 직접 읽는다.
"""
import sys

import yaml

_NODE_KEYS = ('/**', 'navigation_manager_node', '/navigation_manager_node')
_PARAM = 'use_command_services'


def _to_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ('true', '1', 'yes', 'on')


def use_command_services(argv) -> bool:
    """ROS 인자에서 use_command_services 를 찾는다. 없으면 False (기존 토픽 방식)."""
    value = None
    in_ros = False
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--ros-args':
            in_ros = True
        elif a == '--':
            in_ros = False
        elif in_ros and a == '--params-file' and i + 1 < len(argv):
            try:
                with open(argv[i + 1]) as f:
                    data = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError):
                data = {}
            for key in _NODE_KEYS:
                params = (data.get(key) or {}).get('ros__parameters') or {}
                if _PARAM in params:
                    value = _to_bool(params[_PARAM])
            i += 1
        elif in_ros and a in ('-p', '--param') and i + 1 < len(argv):
            name, _, v = argv[i + 1].partition(':=')
            if name.split(':')[-1] == _PARAM:
                value = _to_bool(v)
            i += 1
        i += 1
    return bool(value) if value is not None else False


def main(args=None) -> None:
    argv = sys.argv if args is None else args
    if use_command_services(argv):
        print('[navigation_manager] use_command_services=true -> action/service 방식 (navigation_manager_cmd_node)',
              flush=True)
        from navigation_manager import navigation_manager_cmd_node as impl
    else:
        print('[navigation_manager] use_command_services=false -> 기존 토픽 방식 (navigation_manager_node)',
              flush=True)
        from navigation_manager import navigation_manager_node as impl
    impl.main(args)


if __name__ == '__main__':
    main()

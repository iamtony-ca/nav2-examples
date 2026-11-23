"""Every rule R1..R10 fires on a synthetic message sequence (no bag needed).

Most tests use test/rules_all_topics.yaml (the default rules with every optional topic set);
the default_* tests use profiles/default/nightly_rules.yaml (plain Nav2)."""
import os
from types import SimpleNamespace as NS

import yaml

from nav_data_tools.nightly import Indexer, merge

CFG = os.path.join(os.path.dirname(__file__), 'rules_all_topics.yaml')
PROFILES = os.path.join(os.path.dirname(__file__), '..', '..', 'profiles')
DEFAULT_CFG = os.path.join(PROFILES, 'default', 'nightly_rules.yaml')
S = 1_000_000_000


def cfg(path=CFG):
    with open(path) as f:
        return yaml.safe_load(f)


def vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def twist(v):
    return NS(linear=vec(v), angular=vec())


def odom(v):
    return NS(twist=NS(twist=twist(v)))


def tf(parent, child, x, y):
    return NS(transforms=[NS(header=NS(frame_id=parent), child_frame_id=child,
                             transform=NS(translation=vec(x, y), rotation=NS(x=0, y=0, z=0, w=1)))])


def path(points):
    return NS(header=NS(frame_id='map'),
              poses=[NS(pose=NS(position=vec(x, y))) for x, y in points])


def status(*codes):
    return NS(status_list=[NS(goal_info=NS(goal_id=NS(uuid=bytes([i] * 16))), status=c)
                           for i, c in enumerate(codes)])


def safety(**bits):
    # estop_ok is active-low (true while OK), the others are true when triggered
    fields = {'estop_ok': True, 'bumper_pressed': False, 'protective_stop': False}
    return NS(**{f: bits.get(f, d) for f, d in fields.items()})


def monitoring(lost=False, ec=0):
    return NS(localization_lost=lost, error_code=ec)


def run(events, path=CFG):
    c = cfg(path)
    ix = Indexer(c, log=lambda *_: None)
    T = c['topics']
    for t, key, msg in sorted(events, key=lambda e: e[0]):
        topic = T[key] if key in T else key
        ix.on(topic, msg, int(t * S))
    hits = ix.finish(int(max(e[0] for e in events) * S))
    return {h.rule for h in hits}, hits


def test_all_rules_fire():
    ev = []
    T = cfg()['topics']
    ev.append((0.0, 'robot_status', NS(data='DRIVING')))
    ev.append((0.0, 'tf', tf('map', 'odom', 0, 0)))
    ev.append((0.0, 'plan', path([(0, 0), (100, 0)])))
    # R1: speed 0 for 15 s while DRIVING
    for i in range(0, 150):
        ev.append((1 + i * 0.1, 'odom', odom(0.0)))
    # R2: adjusted 20% of smoothed for 3 s
    for i in range(60):
        t = 20 + i * 0.05
        ev.append((t, 'cmd_smoothed', twist(0.5)))
        ev.append((t + 0.001, 'cmd_adjusted', twist(0.1)))
    # R3: recovery
    ev.append((30.0, 'robot_status', NS(data='RECOVERY')))
    ev.append((33.0, 'robot_status', NS(data='DRIVING')))
    ev.append((34.0, T['behavior_actions'][0], status(2)))
    ev.append((36.0, T['behavior_actions'][0], status(4)))
    # R4: bt error + aborted
    ev.append((40.0, 'bt_error_code', NS(data=1)))
    ev.append((41.0, T['nav_actions'][0], status(2)))
    ev.append((42.0, T['nav_actions'][0], status(6)))
    # R5: 40 plans in one minute
    for i in range(40):
        ev.append((50 + i * 1.0, 'plan', path([(0, 0), (100, 0)])))
    # R6: robot 1 m off the path for 5 s
    for i in range(50):
        ev.append((100 + i * 0.1, 'tf', tf('odom', 'base_link', 10.0, 1.0)))
    ev.append((106.0, 'tf', tf('odom', 'base_link', 11.0, 0.0)))
    # R7
    ev.append((110.0, 'stop_command', NS(data=2)))
    ev.append((112.0, 'stop_command', NS(data=0)))
    # R8
    ev.append((115.0, 'safety', safety()))
    ev.append((116.0, 'safety', safety(bumper_pressed=True)))
    # R9
    ev.append((120.0, 'monitoring', monitoring()))
    ev.append((121.0, 'monitoring', monitoring(lost=True)))
    # R10
    ev.append((125.0, 'mark', NS(data='커브에서 멈춤')))
    rules, hits = run(ev)
    assert rules == {f'R{i}' for i in range(1, 11)}, rules
    r1 = [h for h in hits if h.rule == 'R1'][0]
    assert 13.0 < r1.duration() < 16.0
    r6 = [h for h in hits if h.rule == 'R6'][0]
    assert r6.detail['dist_m'] > 0.9


def test_no_false_positives_on_normal_driving():
    ev = [(0.0, 'robot_status', NS(data='DRIVING')), (0.0, 'tf', tf('map', 'odom', 0, 0)),
          (0.0, 'plan', path([(0, 0), (100, 0)]))]
    for i in range(600):
        t = 0.1 + i * 0.1
        ev.append((t, 'odom', odom(0.5)))
        ev.append((t, 'cmd_smoothed', twist(0.5)))
        ev.append((t + 0.001, 'cmd_adjusted', twist(0.5)))
        ev.append((t, 'tf', tf('odom', 'base_link', 0.05 * i, 0.05)))
        ev.append((t, 'safety', safety()))
    rules, _ = run(ev)
    assert rules == set(), rules


def test_stop_while_idle_is_not_r1():
    ev = [(0.0, 'robot_status', NS(data='IDLE'))]
    ev += [(1 + i * 0.1, 'odom', odom(0.0)) for i in range(300)]
    rules, _ = run(ev)
    assert 'R1' not in rules


def test_merge_joins_close_points():
    from nav_data_tools.nightly import Hit
    hs = [Hit('R8', 'safety', t * S, t * S, {'field': 'bumper'}, 'f') for t in (1, 2, 3, 20)]
    m = merge(hs, 5 * S)
    assert len(m) == 2 and m[0].count == 3


def _result(action, fail, **fields):
    import json
    return NS(data=json.dumps({'action': action, 'goal': 'ab', 'status': 'ABORTED', 'fail': fail,
                               **fields}))


def test_failure_from_result_code():
    T = cfg()['topics']
    cmd_status = '/my_task/_action/status'
    ev = [(0.0, 'robot_status', NS(data='IDLE')),
          # reroute: wire status ABORTED, result_code 8 -> not a failure
          (1.0, cmd_status, status(2)), (2.0, cmd_status, status(6)),
          (2.1, 'action_result', _result('/my_task', False, result_code=8)),
          (3.0, 'robot_status', NS(data='IDLE'))]
    rules, _ = run(ev)
    assert 'R4' not in rules
    ev.append((20.0, 'action_result', _result('/my_task', True, result_code=5)))
    rules, hits = run(ev)
    r4 = [h for h in hits if h.rule == 'R4']
    assert len(r4) == 1 and r4[0].detail['result_code'] == 5
    assert T['result_actions'] == ['/my_task']


def test_replan_burst_from_current_planner():
    ev = [(0.0, 'robot_status', NS(data='DRIVING'))]
    ev += [(1 + i * 0.5, 'current_planner', NS(data='GridBased')) for i in range(30)]
    rules, hits = run(ev)
    r5 = [h for h in hits if h.rule == 'R5']
    assert r5 and 'compute_path_per_min' in r5[0].detail
    # normal: one computation every 10 s
    ev = [(0.0, 'robot_status', NS(data='DRIVING'))]
    ev += [(1 + i * 10.0, 'current_planner', NS(data='GridBased')) for i in range(30)]
    rules, _ = run(ev)
    assert 'R5' not in rules


def test_stuck_on_each_progress_checker():
    stuck = cfg()['topics']['is_stuck']
    ev = [(0.0, 'robot_status', NS(data='DRIVING')),
          (1.0, stuck[1], NS(data=True)), (4.0, stuck[1], NS(data=False)),
          (5.0, stuck[0], NS(data=False))]
    rules, hits = run(ev)
    r7 = [h for h in hits if h.rule == 'R7']
    assert len(r7) == 1 and r7[0].detail['is_stuck'] == '/progress_checker_alt'
    assert 2.9 < r7[0].duration() < 3.1


def test_active_low_field():
    ev = [(0.0, 'robot_status', NS(data='DRIVING')),
          (1.0, 'safety', safety()),
          (2.0, 'safety', safety(estop_ok=False)),           # e-stop pressed
          (3.0, 'safety', safety(estop_ok=False)),
          (4.0, 'safety', safety())]                         # cleared: no second hit
    rules, hits = run(ev)
    r8 = [h for h in hits if h.rule == 'R8']
    assert len(r8) == 1 and r8[0].detail == {'field': 'estop_ok', 'value': False}
    assert abs(r8[0].start / S - 2.0) < 1e-6


def test_default_rules_moving_means_nav_goal_executing():
    """Plain Nav2 (no robot status topic): R1 only while a navigate_* goal is EXECUTING."""
    T = cfg(DEFAULT_CFG)['topics']
    assert 'robot_status' not in T
    nav = T['nav_actions'][0]
    ev = [(0.0, nav, status(2))]                                   # goal executing
    ev += [(1 + i * 0.1, 'odom', odom(0.0)) for i in range(150)]   # stopped 15 s
    ev.append((17.0, nav, status(4)))                              # goal succeeded
    ev += [(18 + i * 0.1, 'odom', odom(0.0)) for i in range(300)]  # idle stop: not R1
    rules, hits = run(ev, DEFAULT_CFG)
    r1 = [h for h in hits if h.rule == 'R1']
    assert len(r1) == 1 and 13.0 < r1[0].duration() < 17.0, [(h.rule, h.duration()) for h in hits]


def test_default_rules_aborted_goal_is_r4():
    T = cfg(DEFAULT_CFG)['topics']
    nav = T['nav_actions'][1]
    rules, hits = run([(0.0, nav, status(2)), (5.0, nav, status(6))], DEFAULT_CFG)
    assert rules == {'R4'}, rules


def test_pose_uses_tf_static_for_base_frame():
    """map->odom and odom->base_footprint on /tf, base_footprint->base_link on /tf_static:
    R6 must still find the robot (frames.base = base_link)."""
    ev = [(0.0, 'robot_status', NS(data='DRIVING')), (0.0, 'tf', tf('map', 'odom', 0, 0)),
          (0.0, '/tf_static', tf('base_footprint', 'base_link', 0, 0)),
          (0.0, 'plan', path([(0, 0), (100, 0)]))]
    ev += [(1 + i * 0.1, 'tf', tf('odom', 'base_footprint', 10.0, 1.0)) for i in range(50)]
    ev.append((7.0, 'tf', tf('odom', 'base_footprint', 11.0, 0.0)))
    rules, hits = run(ev)
    assert 'R6' in rules, rules


def test_safety_and_monitoring_on_one_topic():
    """One status message carrying safety bits and an error code feeds both R8 and R9."""
    c = cfg()
    c['topics']['monitoring'] = c['topics']['safety']
    from nav_data_tools.nightly import Indexer
    ix = Indexer(c, log=lambda *_: None)
    topic = c['topics']['safety']
    ok = NS(estop_ok=True, bumper_pressed=False, protective_stop=False, localization_lost=False,
            error_code=0)
    bad = NS(estop_ok=False, bumper_pressed=False, protective_stop=False, localization_lost=False,
             error_code=7)
    ix.on(topic, ok, 1 * S)
    ix.on(topic, bad, 2 * S)
    rules = {h.rule for h in ix.finish(3 * S)}
    assert {'R8', 'R9'} <= rules, rules


def test_result_actions_accept_status_topic_names():
    c = cfg()
    c['topics']['result_actions'] = ['/my_task/_action/status']
    from nav_data_tools.nightly import Indexer
    ix = Indexer(c, log=lambda *_: None)
    ix.on(c['topics']['action_result'], _result('/my_task', True, result_code=5), 1 * S)
    assert {h.rule for h in ix.finish(2 * S)} == {'R4'}


def test_every_profile_rules_load():
    """Each profile's nightly_rules.yaml builds an Indexer (catches typos in a robot profile)."""
    n = 0
    for name in sorted(os.listdir(PROFILES)):
        path = os.path.join(PROFILES, name, 'nightly_rules.yaml')
        if os.path.isfile(path):
            Indexer(cfg(path), log=lambda *_: None)
            n += 1
    assert n >= 1

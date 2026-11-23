# nav_data_profiles

A profile is everything robot-specific: one directory with up to four files.

| File | Used by | Contents |
|---|---|---|
| `topics.yaml` | recorder (launch) | topics, rates, scan/costmap compaction, event rules, safety bit polarity |
| `params.yaml` | recorder, `nav_data_maintenance` (launch) | root dir, split period, writer cache; retention, nightly report time |
| `nightly_rules.yaml` | `nav_data_nightly` (run by `nav_data_maintenance`) | rules R1..R10, thresholds, robot status values, safety bits |
| `nav_data_analysis.json` | Lichtblick | panel layout |

A file a profile does not have is taken from `default/` (plain Nav2), so a profile can be as small
as one `topics.yaml`.

## Use a profile

```bash
ros2 launch nav_data_recorder nav_data_recorder.launch.py robot_id:=robot01 profile:=myrobot
ros2 run nav_data_tools nav_data_profile --list          # installed profiles
ros2 run nav_data_tools nav_data_profile myrobot topics.yaml   # path of the file that will be used
```

`profile:=` takes a name (a directory here, installed to `share/nav_data_profiles/profiles/<name>/`) or a
path to a directory anywhere (`profile:=$HOME/robot_cfg/nav_data`). `topics_file:=` and
`params_file:=` still override single files.

## Make a profile

```bash
cp -r profiles/default profiles/myrobot     # then edit; see docs/operation_guide.md chapter 3
colcon build --packages-select nav_data_profiles
```

Installed files are copies: rebuild `nav_data_profiles` after editing (a few seconds), or build with
`--symlink-install`, or pass the profile as a path.

## Private profiles

Only `default/` belongs to this repository; `.gitignore` ignores every other profile directory.
To keep a private profile in a private copy of this repository, add it once with
`git add -f profiles/<name>` (tracked files stay tracked; new files in it need `-f` again).

Before pushing to a public remote run `tools/check_publish.sh`. It fails if a non-default profile
would be published, or if a published file contains a term listed in any
`profiles/<name>/publish_terms.txt` (one regex per line; keep that file in the private profile),
an absolute home path or an IP address.

# multi-robot-coordination

Launch scripts for the 4-drone Jetson exploration swarm (Isaac ROS + PX4 + frontier exploration).

## Canonical launch scripts (use these)

One tmux-hardened script per drone — run from the **laptop**, SSHes into the drone's Jetson
and runs each component in a named tmux session (survives SSH drops; re-run a tab to re-attach):

| Drone | ID / ROS_DOMAIN_ID | Script | TP-Link IP | iPhone IP |
|---|---|---|---|---|
| delta | 1 | `launch_delta_jetson_eduroam_python_frontierexplore_tmux.sh` | 192.168.0.40 (TODO verify) | 172.20.10.12 |
| buckshee | 2 | `launch_buckshee_jetson_eduroam_python_frontierexplore_tmux.sh` | 192.168.0.60 | 172.20.10.10 |
| ghost | 3 | `launch_ghost_jetson_eduroam_python_frontierexplore_tmux.sh` | 192.168.0.50 | 172.20.10.11 |
| thunderstrike | 4 | `launch_thunderstrike_jetson_eduroam_python_frontierexplore_tmux.sh` | 192.168.0.70 | 172.20.10.13 |

Usage: `./launch_<drone>_..._tmux.sh [explore|vio|kill] [--rviz] [--debug]` — see `--help`.

**Critical PX4 setting:** each drone's `UXRCE_DDS_DOM_ID` PX4 param must EQUAL its
ROS_DOMAIN_ID above (set via QGC + FMU reboot), or /fmu/* topics land on the wrong DDS
domain and EKF2 silently gets no VIO. Root cause analysis: `ghost_debug/POSTMORTEM.md`.
Ghost was set 2026-06-09; verify the others before flying.

## Directory layout

- `archive/` — superseded launch scripts (old per-drone eduroam/TPLink/cpp/non-tmux variants, bee scripts, swarm/local wip)
- `ghost_debug/` — VIO domain-ID bug investigation: findings, postmortem, captured evidence
- `px4_params/` — PX4 parameter file snapshots
- `logs/` — flight logs (.ulg)
- `Python-Vicon-UDP-Parsing/` — Vicon UDP parsing utility

#!/usr/bin/env bash
# RM26 兑矿仿真一键停止：杀掉 start_rm26.sh 拉起的整栈 + 清理 DDS 残留
#
# 用法:
#   bash stop_rm26.sh             # 正常停止
#   bash stop_rm26.sh --dry-run   # 只列出会杀哪些进程，不动手
#
# 为什么不是简单一句 pkill:
#   1) 直接 kill -9 整个 launch 会留下孤儿子进程（launch 已经没了，
#      没人替它把子进程带走），于是出现"节点还在、但 /clock 没了"的半死状态；
#   2) 关掉 launch 之后若不再清 /dev/shm，下一次启动会看到节点能发现、
#      消息却不投递，表现就是 ros2 service call 卡在 waiting for service。
# 所以顺序是：SIGINT launch（逆序正常析构）→ 10 s 后兜底 SIGKILL 孤儿 →
#              收 ros2 daemon → 删 fastrtps 共享内存段。
set -eo pipefail
# 不用 -u：理由同 start_rm26.sh（ROS 的 setup.bash 会引用未设置的 AMENT_* 变量）

DRY=0
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY=1
elif [[ -n "${1:-}" ]]; then
  echo "用法: $0 [--dry-run]"; exit 1
fi

# launch 进程：唯一需要发 SIGINT 的那个，其余都是它的子孙
# 用 [r]xx 形式是为了让模式串自己不匹配到 pgrep / 调用它的 shell
LAUNCH_PAT='[r]os2 launch /tmp/rm26_sim\.launch\.py'

# 子进程兜底匹配表（launch 正常退出时轮不到它们，专治 kill -9 launch 后的孤儿）
NODE_PATS=(
  '[m]ujoco_ros2_control/ros2_control_node'        # MuJoCo 物理 + controller_manager
  '[m]oveit_ros_move_group/move_group'             # MoveIt（同进程内含 simple_controller_manager）
  '[m]tc/mtc_place_node'                           # MTC place
  '[m]tc/mtc_pick_node'                            # MTC pick（一次只起一个）
  '[r]obot_state_publisher/robot_state_publisher'  # /joint_states -> /tf
  '[s]tatic_transform_publisher.*base_link'        # base -> base_link
  '[s]tatic_transform_publisher.*exchange_cam_optical'  # base -> exchange_cam_optical
  '[a]rm_exchange_host.perception_node'            # 视觉（YOLO + PnP）
  '[r]viz2'                                        # 可视化
)

# 收集当前所有相关 pid（打印前先确认进程还活着，避免 pid 复用误杀）
collect() {
  {
    pgrep -f "$LAUNCH_PAT" || true
    local p
    for p in "${NODE_PATS[@]}"; do pgrep -f "$p" || true; done
  } | sort -un | while read -r pid; do
        [[ "$pid" == "$$" ]] && continue
        ps -o args= -p "$pid" 2>/dev/null | grep -q . && echo "$pid"
      done
}

pids=$(collect)
if [[ -z "$pids" ]]; then
  echo "没有在跑的 RM26 仿真进程"
  exit 0
fi

echo "== 发现进程 =="
for pid in $pids; do
  printf '  %-7s %s\n' "$pid" "$(ps -o args= -p "$pid" | cut -c1-110)"
done

if [[ "$DRY" == 1 ]]; then
  echo "[dry-run] 未做任何动作"
  exit 0
fi

# 1) 正常关闭：SIGINT 只发给 launch，由它按逆序关子进程（正常析构，
#    避免 mujoco_ros2_control / rviz2 在析构顺序错乱时 segfault 留下的脏状态）
launch_pids=$(pgrep -f "$LAUNCH_PAT" || true)
if [[ -n "$launch_pids" ]]; then
  echo "== SIGINT ros2 launch: $(echo $launch_pids) =="
  kill -INT $launch_pids || true
  for _ in $(seq 1 20); do          # 最多等 10 s
    [[ -z "$(collect)" ]] && break
    sleep 0.5
  done
fi

# 2) 兜底：还活着的（含孤儿、以及 SIGINT 不理会的）直接 SIGKILL
rest=$(collect)
if [[ -n "$rest" ]]; then
  echo "== 仍有进程存活，SIGKILL: $(echo $rest) =="
  echo "$rest" | xargs -r kill -9
  sleep 1
fi

# 3) ros2 daemon：必须 -9，不能只发 SIGTERM。
#    它的 SIGTERM 处理只把 rclpy context 关掉、xmlrpc 服务还在监听，
#    之后每条 ros2 命令都会报
#      xmlrpc.client.Fault: <class 'RuntimeError'>:!rclpy.ok()
#    只有 -9 能让它连 socket 一起干净消失；CLI 下次命令会自动起一个新的。
pkill -9 -f '[r]os2-daemon' || true

# 4) DDS 残留：kill -9 之后必须清，否则重启后"能发现、收不到"
shm_cnt=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
if [[ "$shm_cnt" -gt 0 ]]; then
  echo "== 清理 $shm_cnt 个 fastrtps 共享内存段 =="
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
else
  echo "/dev/shm 已是干净的"
fi

if [[ -n "$(collect)" ]]; then
  echo "!! 仍有残留进程，请手动检查："
  for pid in $(collect); do ps -o pid=,args= -p "$pid" | cut -c1-140; done
  exit 1
fi

echo "已停止。下次启动: bash start_rm26.sh [pick|place]"

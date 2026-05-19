#!/usr/bin/env bash
# ============================================================
# ANYmal C + Nav2 통합 검증 스크립트
# 사용법: source install/setup.bash && bash verify_anymal_nav2.sh
# ============================================================
set -e

echo "========================================"
echo " ANYmal C + Nav2 토픽/TF 상태 확인"
echo "========================================"

echo ""
echo "[ 1/5 ] /anymal_0/cmd_vel 수신 확인 (Isaac 실행 후 teleop 눌러야 발행됨)"
timeout 3 ros2 topic echo /anymal_0/cmd_vel --once 2>/dev/null \
    && echo "✅ /anymal_0/cmd_vel 수신 OK" \
    || echo "⚠️  /anymal_0/cmd_vel: 아직 메시지 없음 (Isaac Sim에서 Setup ROS2 눌렀는지 확인)"

echo ""
echo "[ 2/5 ] /anymal_0/scan 확인"
timeout 3 ros2 topic echo /anymal_0/scan --once 2>/dev/null \
    && echo "✅ /anymal_0/scan 수신 OK" \
    || echo "⚠️  /anymal_0/scan: 메시지 없음 (Isaac Sim에서 Setup LiDAR/SLAM 눌렀는지 확인)"

echo ""
echo "[ 3/5 ] /anymal_0/odom 확인"
timeout 3 ros2 topic echo /anymal_0/odom --once 2>/dev/null \
    && echo "✅ /anymal_0/odom 수신 OK" \
    || echo "⚠️  /anymal_0/odom: 메시지 없음"

echo ""
echo "[ 4/5 ] TF 트리 확인 (odom → anymal_0/base_link)"
timeout 5 ros2 run tf2_tools view_frames 2>/dev/null \
    && echo "✅ frames.pdf 생성됨 — evince frames.pdf로 확인" \
    || echo "⚠️  TF tree 생성 실패 (tf2_tools 설치 확인)"

echo ""
echo "[ 5/5 ] Nav2 /cmd_vel 출력 확인 (Nav2 goal 입력 후 확인)"
timeout 5 ros2 topic echo /cmd_vel --once 2>/dev/null \
    && echo "✅ /cmd_vel 출력 확인 (Nav2 → anymal_cmd_vel_relay → /anymal_0/cmd_vel)" \
    || echo "⚠️  /cmd_vel 없음 (Nav2 실행 후 RViz에서 goal 설정 필요)"

echo ""
echo "========================================"
echo " 전체 토픽 목록"
echo "========================================"
ros2 topic list | grep -E "anymal|cmd_vel|scan|odom|map|tf" || true

echo ""
echo "[ 수동 확인 명령어 ]"
echo "  ros2 topic hz /anymal_0/scan        # LiDAR 주기 확인"
echo "  ros2 topic hz /anymal_0/odom        # odom 주기 확인"
echo "  ros2 topic echo /cmd_vel            # Nav2 출력 확인"
echo "  ros2 run tf2_tools view_frames      # TF 트리 시각화"

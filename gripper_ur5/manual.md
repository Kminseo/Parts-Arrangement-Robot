## Python 조작 Tip
1. gripper의 좌표는 world기준 절대 좌표를 기준으로 move group 제어가 된다.
2. gripper의 각도는 R(그리퍼를 아래로 향하게 할때는 3.14), P ,Y(tool0 방향 회전) 설정이 필요하다.
3. movegroup 사용시 Joint value 설정할 때 Open : 0.00 Close : 0.02
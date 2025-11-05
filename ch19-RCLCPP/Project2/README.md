## 실습과제2
### 작성자: 2301510 한윤지
### 주제: 키보드로부터 실수값 3개를 입력받아 publish 하는 package를 작성하는 프로그램
<img width="891" height="284" alt="image" src="https://github.com/user-attachments/assets/704e52c5-ff48-4fbb-bf82-1a3f2d13133e" />

### <rqt_graph>
<img width="581" height="538" alt="image" src="https://github.com/user-attachments/assets/699bb256-5a41-4241-bfbd-b08d007602ab" />

### <명령어>
<img width="1087" height="648" alt="image" src="https://github.com/user-attachments/assets/c99247a0-1215-4186-80e4-4a78423a8964" />
<img width="1090" height="643" alt="image" src="https://github.com/user-attachments/assets/6bbf700b-e0b5-4852-a11b-e922d53f383e" />

#### ros2 topic hz /turtle1/cmd_vel
--> 메시지가 얼마나 자주 들어오는지
평균주기:1.000hz -> 초당 1개의 메시지 수신
최소/최대 주기: 약 1초 간격
표준편차(std_dev): -> 꾸준히 토픽 메시지가 들어온다는 뜻

#### ros2 topic bw /turtle1/cmd_vel
--> 초당 수신되는 데이터의 양
메시지 크기(평균): 28bytes
대역폭(B/s): 약 30–50 Bytes/s 정도 → 초당 약 1개의 메시지를 28 B 크기로 수신

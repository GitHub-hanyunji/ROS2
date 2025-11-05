## 실습과제3
### 작성자: 2301510 한윤지
### 주제: Turtlesim 패키지의 teleop_turtle 노드를 대신하는 package 작성. (거북이 무한회전 ==> 각속도,선속도 적당한값으로 전송)
<img width="1092" height="337" alt="image" src="https://github.com/user-attachments/assets/31008339-5013-4e36-ac6d-5023698d044b" />

### <거북이 실행결과>
<img width="500" height="528" alt="image" src="https://github.com/user-attachments/assets/bec18257-eb2c-4f9f-9baf-5ad9dce64c63" />

### <rqt_graph>
<img width="585" height="533" alt="image" src="https://github.com/user-attachments/assets/ca436945-ef6b-43d8-88c0-dccb35e3cde3" />

### <명령어>
<img width="1096" height="657" alt="image" src="https://github.com/user-attachments/assets/c67e9604-15e9-4081-8357-28ebe84b7280" />
<img width="1092" height="600" alt="image" src="https://github.com/user-attachments/assets/f088a7fe-f837-4d47-ab63-8b262ee2b312" />

#### ros2 topic hz /turtle1/cmd_vel
--> 메시지가 얼마나 자주 들어오는지
평균주기:1.000hz -> 초당 1개의 메시지 수신
최소/최대 주기: 약 1초 간격
표준편차(std_dev): 0.00007s -> 꾸준히 토픽 메시지가 들어온다는 뜻

#### ros2 topic bw /turtle1/cmd_vel
--> 초당 수신되는 데이터의 양
메시지 크기(평균): 52bytes
대역폭(B/s): 약 60–80 Bytes/s 정도 → 초당 약 1개의 메시지를 52 B 크기로 수신

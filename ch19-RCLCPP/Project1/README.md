## 실습과제1
### 작성자: 2301510 한윤지
### 주제: 정수값을 0으로 초기화하고 1씩 증가시키면서 publish 하는 package를 작성하는 프로그램
<img width="890" height="264" alt="image" src="https://github.com/user-attachments/assets/c3b62d47-0f43-4c75-a540-6d2749b09dd7" />

### <rqt_graph>
<img width="577" height="530" alt="image" src="https://github.com/user-attachments/assets/76328d5e-c872-4e20-9b71-f3cedcb707a5" />

### <명령어>
<img width="1085" height="660" alt="image" src="https://github.com/user-attachments/assets/e7cbe2d8-00f7-4a50-bc6b-ac3596d6a30c" />
<img width="1098" height="597" alt="image" src="https://github.com/user-attachments/assets/23e3ba47-7149-4331-b2e1-4c6577f0a2a9" />

#### ros2 topic hz /topic_pub1
--> 메시지가 얼마나 자주 들어오는지
평균주기:1.000hz -> 초당 1개의 메시지 수신
최소/최대 주기: 약 1초 간격
표준편차(std_dev): -> 꾸준히 토픽 메시지가 들어온다는 뜻

#### ros2 topic bw /topic_pub1
--> 초당 수신되는 데이터의 양
메시지 크기(평균): 8bytes
대역폭(B/s): 약 7~10 Bytes/s 정도 → 초당 약 1개의 메시지를 8 B 크기로 수신


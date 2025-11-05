## 실습과제1
### 작성자: 2301510 한윤지
### 주제: 타이머를 이용하여 100msec 마다 토픽을 발행하고 구독하는 패키지 psub1-1을 작성하라

### <pub>
<img width="1096" height="593" alt="image" src="https://github.com/user-attachments/assets/417dc845-a2dd-4f16-b274-202b745a3f08" />
### <sub>
<img width="1090" height="408" alt="image" src="https://github.com/user-attachments/assets/432c6201-d089-4e5f-8efb-4572050df901" />

### <rqt_graph>
<img width="578" height="532" alt="image" src="https://github.com/user-attachments/assets/df32766b-04a1-4485-8716-9e183a3d469f" />

### <명령어>
<img width="1093" height="681" alt="image" src="https://github.com/user-attachments/assets/97d032a2-bb4b-44f0-a90e-dd88d1a6f0b5" />
<img width="1102" height="580" alt="image" src="https://github.com/user-attachments/assets/2e809e61-b880-4ea9-a887-2bdeec52dddc" />

#### ros2 topic hz /topic_pub1
--> 초당 메시지 발행 횟수
평균주기:20.000hz -> 초당 10개의 메시지 수신
최소/최대 주기: 약 100msec 간격
표준편차(std_dev): -> 꾸준히 토픽 메시지가 들어온다는 뜻

#### ros2 topic bw /topic_pub1
--> 초당 수신되는 데이터의 양
메시지 크기(평균): 24bytes



## 실습과제2
### 작성자: 2301510 한윤지
### 주제: 타이머를 이용하여 50msec마다 정수 값을 0부터 1씩 증가시키면서 발행하고 구독하는 패키지 psub1-2를 작성하라

<img width="1090" height="526" alt="image" src="https://github.com/user-attachments/assets/b7b4ac70-63bb-4f2a-b9df-3b113951da2c" />
<img width="1094" height="483" alt="image" src="https://github.com/user-attachments/assets/10aa3229-4fd2-4fc8-bd02-fe8ae18220fa" />


### <rqt_graph>
<img width="580" height="532" alt="image" src="https://github.com/user-attachments/assets/327510d0-928c-4dbd-92b8-f0eb85d7fd09" />

### <명령어>
<img width="1094" height="717" alt="image" src="https://github.com/user-attachments/assets/3fc0e867-e141-40bc-be72-f7ef69317e4d" />
<img width="1090" height="762" alt="image" src="https://github.com/user-attachments/assets/f0d2ed8a-b485-46c1-9c37-5b3240ac2d9b" />


#### ros2 topic hz /topic_pub2
--> 1초당 메시지가 발행되는 횟수
평균주기:20.000hz -> 초당 20개의 메시지 수신
최소/최대 주기: 약 50mesc 간격
표준편차(std_dev): -> 꾸준히 토픽 메시지가 들어온다는 뜻

#### ros2 topic bw /topic_pub2
--> 초당 수신되는 데이터의 양
메시지 크기(평균): 8bytes

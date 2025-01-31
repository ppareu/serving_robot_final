# ROS2 기반 음식점 서빙 시스템

### 1. 프로젝트 개요
- **프로젝트명:** 음식점 서빙 시스템
- **목적:** 음식점 내 주문, 주방, 서빙 로봇 간의 통합적인 데이터 처리 및 효율적인 서비스 제공
- **운영 환경:** Linux Ubuntu
- **사용 기술:** ROS2, PyQt5, MySQL

---

### 2. 주요 기능
#### 1) 키오스크 시스템
- **주문 인터페이스:**
  - PyQt5 기반의 사용자 친화적인 UI 설계
  - 메뉴 선택, 장바구니 확인, 결제 기능 포함
- **ROS2와 통신:**
  - ROS2 서비스를 통해 주문 데이터를 주방으로 비동기 전송
  - GUI와 ROS2 이벤트 루프의 충돌 방지를 위한 멀티스레드 설계

#### 2) 주방 관리 시스템
- **주문 처리:**
  - ROS2의 `order_food` 서비스를 통해 주문 접수 및 처리
  - 주문 상태에 따른 실시간 데이터베이스 업데이트
- **로봇 명령:**
  - 테이블 번호에 따라 서빙 로봇에 명령 전송
  - 로봇 초기 위치 설정 및 복귀 관리

#### 3) 서빙 로봇
- **동작 흐름:**
  1. 테이블 번호에 따라 지정된 위치로 이동
  2. 음식 서빙 후 초기 위치로 복귀
- **ROS2 액션:**
  - `navigate_to_pose` 액션 클라이언트를 통해 로봇 이동 경로 설정
  - 초기 위치 복귀를 위한 재송신 기능 제공

#### 4) 시스템 로깅 및 관리
- **로그 관리:**
  - 모든 주문, 로봇 상태, 이벤트에 대한 로그 기록
  - 문제 발생 시 경고 로그(`Warn`) 출력 및 문제 해결 지원
- **데이터베이스 통합:**
  - MySQL 기반의 주문 내역 및 매출 관리
  - ROS2의 토픽 퍼블리셔와 서브스크라이버로 데이터 전송

---

### 3. 기술적 특징
#### 1) ROS2 통합
- **이벤트 루프 분리:**
  - `rclpy.spin`을 별도의 스레드에서 실행하여 PyQt5와 충돌 방지
- **멀티스레드 처리:**
  - ROS2 노드 생성 및 서비스 대기 비동기 처리
  - GUI 응답성을 보장하며 데이터 처리 병렬화

#### 2) 데이터베이스 관리
- **MySQL 연동:**
  - 주문 데이터의 실시간 저장 및 조회
  - 오늘의 총 매출 등 통계 데이터 관리

#### 3) 사용자 인터페이스
- **PyQt5 기반 UI:**
  - 메뉴 구성, 주문 내역 확인, 테이블 번호 선택 기능
  - 1초 단위의 ROS2 서비스 연결 상태 확인

---

### 4. 문제 해결 및 성과
#### 1) 문제 해결
- **GUI 멈춤 문제:**
  - ROS2 이벤트 루프와 PyQt5 이벤트 루프의 충돌 방지
  - 멀티스레드 설계를 통해 시스템 안정성 확보
- **로봇 명령 실패:**
  - 로깅 시스템을 통해 원인 분석 및 개선

#### 2) 성과
- 효율적인 음식 서빙 워크플로우 구현
- 사용자 친화적이고 안정적인 인터페이스 제공
- 로봇과의 통합적인 데이터 관리 성공

---

### 5. 결론 및 향후 개선 사항
- **결론:**
  - 본 프로젝트는 ROS2와 PyQt5의 통합적인 사용을 통해 음식점 서빙 자동화를 성공적으로 구현했습니다.
- **향후 개선 사항:**
  1. 로봇의 장애물 회피 기능 향상
  2. 데이터 시각화를 위한 대시보드 추가
  3. 더 많은 센서와의 통합으로 시스템 확장

---

**첨부 자료** 
- 프로젝트 코드 및 시연 영상 링크 : https://www.youtube.com/watch?v=L6RjKTYL3p0
- 개발 블로그 : https://ppareu.tistory.com/2

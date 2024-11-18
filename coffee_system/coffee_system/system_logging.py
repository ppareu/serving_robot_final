# system_logging.py
import rclpy
from rclpy.node import Node
from coffee_system_interface.msg import LogMsg
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import sys

# 시스템 로그 창 클래스
class LogWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('System Logs')
        self.setGeometry(100, 100, 600, 400)
        
        # 중앙 위젯 생성
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 레이아웃 설정
        layout = QVBoxLayout(central_widget)
        
        # 로그를 표시할 텍스트 에디터 생성
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True) # 읽기 전용 설정
        # 배경색을 검은색으로 설정
        self.log_display.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(self.log_display)

    def append_log(self, text):
        self.log_display.append(text)

# 시스템 로그 노드 클래스
class SystemLogging(Node):
    def __init__(self):
        super().__init__('system_logging')
        self.system_log_subscriber = self.create_subscription(
            LogMsg,
            'system_logging',
            self.system_log_callback,
            10)
        
        
        # GUI 초기화
        self.app = QApplication(sys.argv)
        self.log_window = LogWindow()
        self.log_window.show()
        
        # Qt 타이머를 사용하여 ROS 스핀
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_callback) # 타이머 실행
        self.timer.start(10)  # 10ms 간격으로 타이머 실행

    # 타이머 콜백 함수
    def timer_callback(self):
        if self.context.ok():
            rclpy.spin_once(self, timeout_sec=0)

    # 시스템 로그 콜백 함수
    def system_log_callback(self, msg):
        # 로그 메시지를 받아서 GUI에 표시
        self.log_window.append_log(msg.log)
        self.get_logger().info(f'Received log: {msg.log}')

def main(args=None):
    rclpy.init(args=args)
    node = SystemLogging()
    
    try:
        node.app.exec_() # Qt 애플리케이션 실행
    finally:
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# main_kiosk.py

import sys
import os
from collections import Counter
from functools import partial
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
import rclpy
from rclpy.node import Node
from coffee_system_interface.srv import MySrv
from coffee_system_interface.msg import CallStaff
from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtCore

MENU_ITEMS = {
    "커피": [
        {"name": "아메리카노", "price": 4000, "image": "Americano.jpg"},
        {"name": "아포카토", "price": 5000, "image": "affogato.jpg"},
        {"name": "카페라떼", "price": 5000, "image": "cafe_latte.jpg"},
        {"name": "연유라떼", "price": 6000, "image": "condensed_milk_coffee.jpg"},
        {"name": "에스프레소", "price": 3000, "image": "espresso.jpg"},
        {"name": "피콜로", "price": 6000, "image": "piccolo_latte.jpg"},
        {"name": "콜드브루", "price": 4500, "image": "cold_brew_coffee.jpg"},
        {"name": "카라멜 마끼아토", "price": 5500, "image": "caramel_macchiato.jpg"},
        {"name": "바닐라 라떼", "price": 5200, "image": "vanilla_latte.jpg"},
    ],
    "티": [
        {"name": "녹차", "price": 3000, "image": "green_tea.jpg"},
        {"name": "홍차", "price": 3500, "image": "black_tea.jpg"},
        {"name": "레몬티", "price": 3800, "image": "lemon_tea.jpg"},
        {"name": "얼그레이", "price": 4000, "image": "Earl_Grey_tea.jpg"},
        {"name": "페퍼민트", "price": 4200, "image": "peppermint_tea.jpg"},
        {"name": "캐모마일", "price": 4500, "image": "chamomile_tea.jpg"},
        {"name": "자스민", "price": 3700, "image": "jasmine_tea.jpg"},
        {"name": "로즈힙", "price": 4300, "image": "rosehip_tea.jpg"},
        {"name": "복숭아 아이스티", "price": 4500, "image": "peach_iced_tea.jpg"},
    ],
    "디저트": [
        {"name": "애플 파이", "price": 7000, "image": "apple_pie.jpg"},
        {"name": "치즈 케이크", "price": 8000, "image": "cheese_cake.jpg"},
        {"name": "초코 케이크", "price": 7500, "image": "chocolate_cake.jpg"},
        {"name": "크로아상", "price": 8500, "image": "croissant.jpg"},
        {"name": "아이스크림", "price": 6000, "image": "ice_cream.jpg"},
        {"name": "마카롱", "price": 5000, "image": "macarons.jpg"},
        {"name": "마들렌", "price": 3500, "image": "madeleines.jpg"},
        {"name": "브라우니", "price": 6500, "image": "brownie.jpg"},
        {"name": "와플", "price": 4000, "image": "waffle.jpg"},
    ]
}

UI_DIR = os.path.join(os.path.dirname(__file__), 'images')

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        self.quit()
        self.wait()

class KioskClientNode(Node):
    def __init__(self):
        super().__init__('kiosk_client_node')
        self.client = self.create_client(MySrv, 'order_food')
        # staff_call
        self.staff_publisher = self.create_publisher(CallStaff, 'staff_call', 10)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_order(self, table_num, items, callback):
        try:
            table_num = int(table_num)
        except ValueError:
            raise ValueError("Table number must be an integer")

        request = MySrv.Request(table_num=table_num, items=items)
        future = self.client.call_async(request)
        future.add_done_callback(callback)

    def call_staff(self, table_num):
        
        msg = CallStaff()
        msg.table_num = table_num
        msg.message = "직원 호출 요청"
        self.staff_publisher.publish(msg)
        self.get_logger().info(f'테이블 {table_num}: 직원 호출 요청 전송 완료')


class CoffeeKiosk(QtWidgets.QMainWindow):
    order_successful = pyqtSignal(str)
    order_rejected = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROKEY커피 테이블 키오스크")
        self.setGeometry(100, 100, 1000, 700)  # 창 크기를 조정

        rclpy.init()
        self.node = KioskClientNode()
        self.ros2_thread = ROS2Thread(self.node)
        self.ros2_thread.start()

        self.cart = []
        self.selected_table = None
        self.selected_category = "커피"
        self.init_ui()

        self.order_successful.connect(self.show_order_success)
        self.order_rejected.connect(self.show_order_rejection)

    def init_ui(self):
        self.setStyleSheet("background-color: #2C2C2C;")

        main_layout = QtWidgets.QVBoxLayout()
        container = QtWidgets.QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 카페 배너
        banner_layout = QtWidgets.QHBoxLayout()
        banner_label = QtWidgets.QLabel()
        banner_pixmap = QPixmap(os.path.join(UI_DIR, 'cafe_banner2.jpg')).scaled(1000, 120)
        banner_label.setPixmap(banner_pixmap)
        banner_layout.addWidget(banner_label)
        main_layout.addLayout(banner_layout)

        # 메뉴 카테고리 버튼
        category_layout = QtWidgets.QHBoxLayout()
        self.category_buttons = {}
        for category in MENU_ITEMS:
            button = QtWidgets.QPushButton(category)
            button.setStyleSheet("background-color: #000000; color: #D2B48C; font-weight: bold;")
            button.clicked.connect(partial(self.show_category_menu, category))
            self.category_buttons[category] = button
            category_layout.addWidget(button)
        main_layout.addLayout(category_layout)

        # 메뉴판
        self.menu_layout = QtWidgets.QGridLayout()
        main_layout.addLayout(self.menu_layout)

        # 장바구니 리스트
        self.cart_list = QtWidgets.QListWidget()
        self.cart_list.setStyleSheet("background-color: #3C3C3C; color: #F5DEB3; font-size: 14px;")
        main_layout.addWidget(self.cart_list)
        
        self.total_label = QtWidgets.QLabel("총 합계: 0원")
        self.total_label.setStyleSheet("color: #D2B48C; font-size: 16px; font-weight: bold;")
        main_layout.addWidget(self.total_label)

        self.init_table_buttons(main_layout)
        self.init_action_buttons(main_layout)

        # 카테고리별 메뉴 표시
        self.show_category_menu(self.selected_category)

    def create_menu_item(self, item):
        item_widget = QtWidgets.QWidget()
        item_layout = QtWidgets.QVBoxLayout(item_widget)
        item_layout.setAlignment(QtCore.Qt.AlignCenter)

        image_label = QtWidgets.QLabel()
        image_path = os.path.join(UI_DIR, item['image'])
        pixmap = QPixmap(image_path).scaled(100, 100) if os.path.exists(image_path) else None
        image_label.setPixmap(pixmap or QtGui.QPixmap())
        item_layout.addWidget(image_label)

        name_label = QtWidgets.QLabel(item['name'])
        name_label.setStyleSheet("color: #F5DEB3; font-size: 16px; font-weight: bold;")
        item_layout.addWidget(name_label)

        price_label = QtWidgets.QLabel(f"{item['price']} 원")
        price_label.setStyleSheet("color: #D2B48C; font-size: 14px;")
        item_layout.addWidget(price_label)

        add_button = QtWidgets.QPushButton("장바구니에 추가")
        add_button.setStyleSheet("background-color: #4B2E2E; color: #F5DEB3; font-weight: bold; padding: 5px;")
        add_button.clicked.connect(partial(self.add_to_cart, item))
        item_layout.addWidget(add_button)

        return item_widget

    def clear_layout(self, layout):
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()

    def show_category_menu(self, category):
        self.selected_category = category
        self.clear_layout(self.menu_layout)

        items = MENU_ITEMS[category]
        for i, item in enumerate(items):
            item_widget = self.create_menu_item(item)
            row, col = i // 3, i % 3
            self.menu_layout.addWidget(item_widget, row, col)

    def init_table_buttons(self, layout):
        table_layout = QtWidgets.QHBoxLayout()
        self.table_buttons = []
        self.table_button_group = QtWidgets.QButtonGroup(self)  # 버튼 그룹 생성
        self.table_button_group.setExclusive(True)  # 한 번에 하나의 버튼만 선택 가능

        for i in range(1, 10):
            button = QtWidgets.QPushButton(f"테이블 {i}")
            button.setCheckable(True)  # 버튼을 체크 가능 상태로 설정
            button.setStyleSheet("""
                QPushButton {
                    background-color: #000000; 
                    color: #D2B48C; 
                    font-weight: bold; 
                    border: 2px solid #D2B48C;
                }
                QPushButton:checked {
                    background-color: #4B2E2E; 
                    border: 2px solid #FFD700;
                    color: #FFD700;
                }
            """)
            button.clicked.connect(partial(self.select_table, button))  # 버튼 클릭 이벤트 연결
            self.table_buttons.append(button)
            self.table_button_group.addButton(button)  # 버튼 그룹에 추가
            table_layout.addWidget(button)

        layout.addLayout(table_layout)


    def init_action_buttons(self, layout):
        button_layout = QtWidgets.QHBoxLayout()

        self.staff_call_button = QtWidgets.QPushButton("직원 호출")
        self.staff_call_button.setStyleSheet("background-color: #000000; color: #D2B48C; font-weight: bold; padding: 10px; font-size: 16px;")
        self.staff_call_button.clicked.connect(self.request_staff_call)
        button_layout.addWidget(self.staff_call_button)

        self.clear_cart_button = QtWidgets.QPushButton("장바구니 비우기")
        self.clear_cart_button.setStyleSheet("background-color: #000000; color: #D2B48C; font-weight: bold; padding: 10px; font-size: 16px;")
        self.clear_cart_button.clicked.connect(self.clear_cart)
        button_layout.addWidget(self.clear_cart_button)

        self.order_button = QtWidgets.QPushButton("주문하기")
        self.order_button.setStyleSheet("background-color: #4B2E2E; color: #F5DEB3; font-weight: bold; padding: 10px; font-size: 16px;")
        self.order_button.clicked.connect(self.place_order)
        button_layout.addWidget(self.order_button)

        layout.addLayout(button_layout)

    def request_staff_call(self):
        if self.selected_table is None:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")  # 텍스트 색상을 빨간색으로 설정
            msg_box.setIcon(QtWidgets.QMessageBox.Warning)
            msg_box.setText("테이블을 선택해주세요.")
            msg_box.setWindowTitle("알림")
            msg_box.exec_()
            return
        try:
            table_num = int(self.selected_table)
            self.node.call_staff(table_num)  # 직원 호출 요청 전송
        except ValueError:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")  # 텍스트 색상을 파란색으로 설정
            msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            msg_box.setText("테이블 번호가 유효하지 않습니다.")
            msg_box.setWindowTitle("오류")
            msg_box.exec_()

    def select_table(self, button):
        if button.isChecked():
            self.selected_table = int(button.text().split()[1])
        else:
            self.selected_table = None

    def add_to_cart(self, item):
        self.cart.append(item)
        self.update_cart_display()

    # def update_cart_display(self):
    #     self.cart_list.clear()
    #     total = 0
    #     item_counter = Counter(item['name'] for item in self.cart)
    #     for item_name, count in item_counter.items():
    #         price = next(item['price'] for item in self.cart if item['name'] == item_name)
    #         self.cart_list.addItem(f"{item_name} - {count}개 ({price * count} 원)")
    #         total += price * count
    #     self.total_label.setText(f"총 합계: {total} 원")

    def update_cart_display(self):
        self.cart_list.clear()
        total = 0
        item_counter = Counter(item['name'] for item in self.cart)
        
        for item_name, count in item_counter.items():
            price = next(item['price'] for item in self.cart if item['name'] == item_name)
            self.cart_list.addItem(f"{item_name} - {count}개 ({price * count} 원)")
            total += price * count

        # 총 금액을 클래스 변수로 저장
        self.total_price = total

        # 총 합계 라벨 업데이트
        self.total_label.setText(f"총 합계: {total} 원")

    #####################################
    # def place_order(self):
    #     if not self.cart:
    #         QtWidgets.QMessageBox.warning(self, "빈 장바구니", "먼저 장바구니에 항목을 추가하세요!")
    #         return
    #     if self.selected_table is None:
    #         QtWidgets.QMessageBox.warning(self, "테이블 미선택", "테이블을 선택하세요!")
    #         return

    #     # 항목 구성
    #     items = [
    #         f"{name}: {count}" if count > 1 else name
    #         for name, count in Counter(item['name'] for item in self.cart).items()
    #     ]

    #     # 총 금액 정보를 추가
    #     items.append(f"총 합계: {self.total_price}원")

    #     # 주문 전송
    #     self.node.send_order(self.selected_table, items, self.order_response_callback)

    ###################################33

    def remove_from_cart(self, row):
        self.cart.pop(row)
        self.cart_list.takeItem(row)
        self.update_total()

    def update_total(self):
        total = sum(item['price'] for item in self.cart)
        self.total_label.setText(f"총 합계: {total} 원")

    def clear_cart(self):
        self.cart.clear()
        self.cart_list.clear()
        self.update_total()

    def place_order(self):
        if self.selected_table is None:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")  # 텍스트 색상을 빨간색으로 설정
            msg_box.setIcon(QtWidgets.QMessageBox.Warning)
            msg_box.setText("테이블을 선택해주세요.")
            msg_box.setWindowTitle("알림")
            msg_box.exec_()
            return
        if not self.cart:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")  # 텍스트 색상을 빨간색으로 설정
            msg_box.setIcon(QtWidgets.QMessageBox.Warning)
            msg_box.setText("주문할 메뉴를 추가해주세요.")
            msg_box.setWindowTitle("알림")
            msg_box.exec_()
            return

        # items = [item['name'] for item in self.cart]
        # self.node.send_order(self.selected_table, items, self.order_callback)

        # 항목 구성
        items = [
            f"{name}: {count}" if count > 1 else name
            for name, count in Counter(item['name'] for item in self.cart).items()
        ]

        # 총 금액 정보를 추가
        items.append(f"총 합계: {self.total_price}원")

        # 주문 전송
        self.node.send_order(self.selected_table, items, self.order_callback)

    def order_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.order_successful.emit("주문이 성공적으로 처리되었습니다.")
            else:
                self.order_rejected.emit("주문 처리에 실패했습니다.")
        except Exception as e:
            self.order_rejected.emit(f"오류가 발생했습니다: {str(e)}")

    def show_order_success(self, message):
        msg_box = QtWidgets.QMessageBox(self)
        msg_box.setIcon(QtWidgets.QMessageBox.Information)
        msg_box.setText(message)
        msg_box.setWindowTitle("성공")
        msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")
        msg_box.exec_()
        self.clear_cart()

    def show_order_rejection(self, message):
        msg_box = QtWidgets.QMessageBox(self)
        msg_box.setIcon(QtWidgets.QMessageBox.Warning)
        msg_box.setText(message)
        msg_box.setWindowTitle("실패")
        msg_box.setStyleSheet("QLabel { color: #F5DEB3; }")
        msg_box.exec_()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = CoffeeKiosk()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

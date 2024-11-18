import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import mysql.connector
from mysql.connector import Error
from datetime import datetime

# 주문 데이터베이스 서버 노드 클래스    
class OrderDatabaseServer(Node):
    def __init__(self):
        super().__init__('order_database_server')
        
        # 로그 메시지 구독 설정
        self.log_subscription = self.create_subscription(
            String,
            'accepted_orders',
            self.log_callback,
            10
        )
        
        # 데이터베이스 연결 설정  -> 변경사항
        try:
            self.conn = mysql.connector.connect(
                host='127.0.0.1',
                user='root',
                password='000120',
                database='coffee_system_db'
            )
            self.cursor = self.conn.cursor(buffered=True)
            self.get_logger().info('데이터베이스 연결 성공')
            
        except Error as e:
            self.get_logger().error(f'데이터베이스 연결 실패: {str(e)}')
            raise

    def handle_accepted_order(self, msg):
        """주문 수락 메시지 처리 및 데이터베이스 저장"""
        try:
            # 메시지 파싱
            data = msg.data.split(', ')
            table_num = int(data[0].split(' ')[1])
            order_info = data[1].split(': ')[1]
            items = order_info.split(', ')
            total_price = float(data[2].split(': ')[1].replace('원', ''))

            # 데이터베이스에 저장
            self.save_order_from_log(table_num, order_info, total_price)
            self.get_logger().info(f"테이블 {table_num} 주문 데이터베이스 저장 완료.")
        except Exception as e:
            self.get_logger().error(f"주문 메시지 처리 중 오류 발생: {str(e)}")

    # 로그 메시지 콜백 함수
    def log_callback(self, msg):
        try:
            # 로그 메시지 파싱
            log_data = msg.data
            if '주문 내역:' in log_data:
                # 테이블 번호 추출
                table_num_str = log_data.split('테이블 ')[1].split(' ')[0]
                table_num = int(table_num_str.replace(',', '').strip())

                # 주문 내역과 총 합계 추출
                order_info = log_data.split('주문 내역: ')[1].split(', 총 합계:')[0]
                total_price_str = log_data.split('총 합계: ')[1].split('원')[0]
                total_price = float(total_price_str.replace(',', '').strip())

                # 주문 승낙 여부 확인
                is_accepted = '승낙' in log_data.split('주문 승낙 여부: ')[1]

                if is_accepted:
                    # 주문 저장
                    self.save_order_from_log(table_num, order_info, total_price)
                    self.get_logger().info('주문이 데이터베이스에 저장되었습니다.')
                else:
                    self.get_logger().info('거절된 주문은 저장하지 않습니다.')

        except Exception as e:
            self.get_logger().error(f'로그 처리 중 오류 발생: {str(e)}')

    # 로그 메시지에서 주문 정보 추출 및 저장
    def save_order_from_log(self, table_number, order_info, total_price):
        try:
            # 주문 정보 저장
            insert_order = """
            INSERT INTO orders (table_number, order_time, total_price)
            VALUES (%s, %s, %s)
            """
            order_time = datetime.now()
            self.cursor.execute(insert_order, (table_number, order_time, total_price))
            order_id = self.cursor.lastrowid

            # 주문 항목 저장
            items = order_info.split(', ')
            for item in items:
                menu_item = item.strip()  # 항목 이름만 있음
                quantity = 1  # 기본 수량은 1로 설정

                insert_item = """
                INSERT INTO order_items (order_id, food_item, quantity)
                VALUES (%s, %s, %s)
                """
                self.cursor.execute(insert_item, (order_id, menu_item, quantity))

            # 주방 주문 저장
            for item in items:
                menu_item = item.strip()

                insert_kitchen_order = """
                INSERT INTO kitchen_orders (order_item_id, status)
                VALUES (%s, %s)
                """
                self.cursor.execute(
                    "SELECT order_item_id FROM order_items WHERE food_item = %s AND order_id = %s",
                    (menu_item, order_id),
                )
                order_item_id = self.cursor.fetchone()[0]
                self.cursor.execute(insert_kitchen_order, (order_item_id, '준비 중'))

            self.conn.commit()

        except Error as e:
            self.get_logger().error(f'주문 저장 중 오류 발생: {str(e)}')
            self.conn.rollback()


    # 노드 종료 시 데이터베이스 연결 종료
    def __del__(self):
        if hasattr(self, 'cursor') and self.cursor:
            self.cursor.close()
        if hasattr(self, 'conn') and self.conn: 
            self.conn.close()
            self.get_logger().info('데이터베이스 연결이 종료되었습니다.')

def main(args=None):
    rclpy.init(args=args)
    database_server = OrderDatabaseServer()
    
    try:
        rclpy.spin(database_server)
    except KeyboardInterrupt:
        pass
    finally:
        database_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 토픽 대역폭 모니터링 도구
실시간으로 각 토픽의 대역폭과 FPS를 모니터링합니다.
"""

import rclpy
from rclpy.node import Node
from collections import defaultdict
import time
import sys
import os

# Windows에서 색상 출력 지원
if sys.platform == 'win32':
    os.system('color')

class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor')
        
        # 모니터링할 토픽 목록
        self.topics_to_monitor = [
            '/back_camera/compressed',
            '/front_camera/compressed',
            '/spectrogram_image/compressed',
            '/velocity',
            '/sound_direction',
            '/firetruck_side',
            '/decision',
        ]
        
        # 데이터 저장
        self.topic_data = defaultdict(lambda: {
            'message_count': 0,
            'total_bytes': 0,
            'last_reset': time.time(),
            'last_msg_time': 0,
            'msg_times': []
        })
        
        # 구독자 생성 (일반 메시지로 받아서 크기 측정)
        self.subscribers = {}
        self.create_subscribers()
        
        # 모니터링 타이머 (1초마다 출력)
        self.timer = self.create_timer(1.0, self.print_stats)
        
        self.get_logger().info('🔍 대역폭 모니터 시작됨')
        self.get_logger().info(f'모니터링 토픽: {len(self.topics_to_monitor)}개')
        
    def create_subscribers(self):
        """각 토픽에 대한 구독자 생성"""
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import Int32, String
        
        # CompressedImage 토픽
        for topic in ['/back_camera/compressed', '/front_camera/compressed', '/spectrogram_image/compressed']:
            try:
                self.subscribers[topic] = self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, t=topic: self.compressed_image_callback(msg, t),
                    10
                )
            except Exception as e:
                self.get_logger().warn(f'{topic} 구독 실패: {e}')
        
        # Int32 토픽
        try:
            self.subscribers['/velocity'] = self.create_subscription(
                Int32,
                '/velocity',
                lambda msg: self.int32_callback(msg, '/velocity'),
                10
            )
        except Exception:
            pass
        
        # String 토픽들
        for topic in ['/sound_direction', '/firetruck_side', '/decision']:
            try:
                self.subscribers[topic] = self.create_subscription(
                    String,
                    topic,
                    lambda msg, t=topic: self.string_callback(msg, t),
                    10
                )
            except Exception:
                pass
    
    def compressed_image_callback(self, msg, topic_name):
        """CompressedImage 콜백"""
        data_size = len(msg.data)
        self.update_stats(topic_name, data_size)
    
    def int32_callback(self, msg, topic_name):
        """Int32 콜백"""
        data_size = 4  # int32는 4바이트
        self.update_stats(topic_name, data_size)
    
    def string_callback(self, msg, topic_name):
        """String 콜백"""
        data_size = len(msg.data.encode('utf-8'))
        self.update_stats(topic_name, data_size)
    
    def update_stats(self, topic_name, data_size):
        """통계 업데이트"""
        data = self.topic_data[topic_name]
        current_time = time.time()
        
        data['message_count'] += 1
        data['total_bytes'] += data_size
        data['last_msg_time'] = current_time
        
        # FPS 계산을 위한 메시지 시간 저장 (최근 30개)
        data['msg_times'].append(current_time)
        if len(data['msg_times']) > 30:
            data['msg_times'].pop(0)
    
    def calculate_fps(self, topic_name):
        """FPS 계산"""
        data = self.topic_data[topic_name]
        msg_times = data['msg_times']
        
        if len(msg_times) < 2:
            return 0.0
        
        time_span = msg_times[-1] - msg_times[0]
        if time_span > 0:
            return (len(msg_times) - 1) / time_span
        return 0.0
    
    def print_stats(self):
        """통계 출력"""
        current_time = time.time()
        
        # 화면 지우기 (Windows/Linux 호환)
        os.system('cls' if sys.platform == 'win32' else 'clear')
        
        print("=" * 100)
        print("🔍 ROS2 토픽 대역폭 모니터 (실시간)")
        print("=" * 100)
        print(f"{'토픽':<40} {'FPS':>8} {'대역폭 (KB/s)':>15} {'메시지 수':>12} {'상태':>10}")
        print("-" * 100)
        
        total_bandwidth = 0.0
        active_topics = 0
        
        for topic in sorted(self.topic_data.keys()):
            data = self.topic_data[topic]
            elapsed = current_time - data['last_reset']
            
            if elapsed > 0:
                bandwidth_kbps = (data['total_bytes'] / elapsed) / 1024
                fps = self.calculate_fps(topic)
                
                # 상태 표시
                time_since_last = current_time - data['last_msg_time']
                if time_since_last > 3.0:
                    status = "⚠️ 정지"
                    color = '\033[93m'  # 노란색
                elif bandwidth_kbps > 500:
                    status = "🔴 과부하"
                    color = '\033[91m'  # 빨간색
                elif bandwidth_kbps > 200:
                    status = "🟡 높음"
                    color = '\033[93m'  # 노란색
                else:
                    status = "✅ 정상"
                    color = '\033[92m'  # 녹색
                    active_topics += 1
                
                reset_color = '\033[0m'
                
                print(f"{color}{topic:<40} {fps:>8.1f} {bandwidth_kbps:>15.2f} {data['message_count']:>12} {status:>10}{reset_color}")
                
                total_bandwidth += bandwidth_kbps
            
            # 통계 리셋 (1초마다)
            data['total_bytes'] = 0
            data['message_count'] = 0
            data['last_reset'] = current_time
        
        print("-" * 100)
        
        # 총 대역폭 계산 및 경고
        total_mbps = total_bandwidth * 8 / 1024
        
        if total_mbps > 10:
            color = '\033[91m'  # 빨간색
            status = "🔴 위험: WiFi 과부하 가능성 높음!"
        elif total_mbps > 6:
            color = '\033[93m'  # 노란색
            status = "🟡 경고: 대역폭 높음"
        else:
            color = '\033[92m'  # 녹색
            status = "✅ 안정적"
        
        print(f"\n{color}📊 총 대역폭: {total_bandwidth:.2f} KB/s ({total_mbps:.2f} Mbps) - {status}\033[0m")
        print(f"📡 활성 토픽: {active_topics}/{len(self.topic_data)}")
        
        # 권장 사항
        print("\n" + "=" * 100)
        print("💡 권장 사항:")
        if total_mbps > 8:
            print("  • 🔴 대역폭이 매우 높습니다! 추가 최적화 필요")
            print("     → FPS를 10fps로 낮추기: camera_node.py에서 fps=10.0")
            print("     → YOLO 프레임 스킵 증가: process_every_n_frames=3")
            print("     → JPEG 품질 추가 감소: jpeg_quality=60")
        elif total_mbps > 5:
            print("  • 🟡 대역폭이 다소 높습니다")
            print("     → WiFi 5GHz 대역 사용 권장")
            print("     → 라우터와 거리 최소화 (3m 이내)")
        else:
            print("  • ✅ 대역폭 정상 - 최적화 잘 적용됨!")
        
        print("\n⌨️  종료: Ctrl+C")
        print("=" * 100)


def main(args=None):
    rclpy.init(args=args)
    monitor = BandwidthMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n🛑 모니터링 종료")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


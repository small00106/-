import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from Uart import process_data



class DetectionParser(Node):
    def __init__(self):
        super().__init__('detection_parser')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_dnn_detection',
            self.callback,
            10)
        self.coordinates = [0, 0, 0, 0]
    def callback(self, msg):
        for target in msg.targets:  # 遍历所有检测目标
            target_type = target.type  # 获取目标类别
            for roi in target.rois:  # 遍历每个目标的ROI信息
                rect = roi.rect  # 获取位置框
                x = rect.x_offset
                y = rect.y_offset
                w = rect.width
                h = rect.height
                confidence = roi.confidence  # 置信度
                # if target_type=="bottle":
                self.get_logger().info(
                    f"[{target_type}] x:{x}, y:{y}, w:{w}, h:{h}, confidence:{confidence:.2f}"
                    )
                self.coordinates[0]=x
                self.coordinates[1]=y 
                self.coordinates[2]=w 
                self.coordinates[3]=h 
                # if target_type=="bottle":
                #     process_data(self.coordinates)
def main():
    
    rclpy.init()
    node = DetectionParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




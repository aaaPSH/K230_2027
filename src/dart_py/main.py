from detector import Detector
from guidance import ProportionalGuidance
import cv2

def main():
    # 创建 Detector 实例
    detector = Detector()

    # 创建 Guidance 实例
    guidance = ProportionalGuidance()

    # 打开摄像头
    cap = cv2.VideoCapture(0)

    while True:
        # 读取摄像头帧
        ret, frame = cap.read()
        if not ret:
            break

        # 检测基地目标靶绿灯
        target_detected, target_info = detector.detect(frame)

        if target_detected:
            # 如果检测到目标，进行引导计算
            guidance_result = guidance.calculate_guidance(target_info)
            print("Guidance Result:", guidance_result)
        else:
            print("No target detected.")

        # 显示图像
        cv2.imshow('Frame', frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头和关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
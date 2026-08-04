import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

# 定义梯形的四个顶点坐标（需要根据实际情况调整）
src_points = np.array([[180, 240], [450, 240], [640, 480], [0, 480]], dtype=np.float32)
dst_points = np.array([[0, 0], [500, 0], [500, 500], [0, 500]], dtype=np.float32)

def perspective_transform(frame):
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    transformed_frame = cv2.warpPerspective(frame, M, (500, 500))
    return transformed_frame

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    transformed_frame = perspective_transform(frame)

    decoded_objects = pyzbar.decode(transformed_frame)
    for obj in decoded_objects:
        print("QR Code data:", obj.data.decode('utf-8'))

    cv2.imshow('Original Frame', frame)
    cv2.imshow('Transformed Frame', transformed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

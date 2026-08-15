import cv2

cap = cv2.VideoCapture(0)
image_number = 1

while True:
    ret, frame = cap.read()
    cv2.imshow('Camera Feed', frame)

    key = cv2.waitKey(1)
    if key == ord('s'):
        image_name = str(image_number) + '.jpg'
        cv2.imwrite(image_name, frame)
        print(f'Saved image as {image_name}')
        image_number += 1
    elif key == 27:
        break

cap.release()
cv2.destroyAllWindows()

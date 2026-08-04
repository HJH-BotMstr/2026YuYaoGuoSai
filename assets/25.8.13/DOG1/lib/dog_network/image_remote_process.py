import cv2
import zmq, cv2, time, json, uuid
import os.path as osp

COMPUTER_IP = "192.168.227.130"  # 服务器 IP 地址
LABELS_CN = ["火灾", "塌方", "水灾", "冒顶", "爆炸"]


def send_image(img, identify="xgo2", timeout=1.0, computer_ip=COMPUTER_IP):
    """发送图像到服务器并等待响应

    Args:
        img: 要发送的图像
        identify (str, optional): 用于标识请求的字符串. Defaults to "xgo2".
        timeout (float, optional): 等待响应的超时时间. Defaults to 1.0.
        computer_ip (str, optional): 服务器的 IP 地址. Defaults to COMPUTER_IP.

    Returns:
        dict/None: 服务器返回的结果或 None
    """
    context = zmq.Context()
    dealer = context.socket(zmq.DEALER)
    dealer.setsockopt(zmq.IDENTITY, identify.encode())
    # 连接阶段设置超时，防止长时间阻塞
    dealer.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
    dealer.setsockopt(zmq.SNDTIMEO, int(timeout * 1000))
    dealer.connect(f"tcp://{computer_ip}:5555")

    _, img_encoded = cv2.imencode(".jpg", img)
    img_bytes = img_encoded.tobytes()
    uid = str(uuid.uuid4())

    try:
        dealer.send_multipart([img_bytes, uid.encode(), str(time.time()).encode()])
    except zmq.Again:
        # 连接或发送超时
        dealer.close(linger=0)
        context.term()
        return None

    start_time = time.time()
    while True:
        try:
            msg = dealer.recv_multipart(zmq.DONTWAIT)
            ret_uuid, json_str = msg[0], msg[1]
            result = json.loads(json_str.decode())
            if result["uuid"] == uid:
                obj = result["objects"][0] if result["objects"] else None
                return obj
        except zmq.Again:
            if time.time() - start_time > timeout:
                break
            time.sleep(0.001)

    # 超时或异常，强制清理
    dealer.close(linger=0)
    context.term()
    return None


def send_image_with_retry(
    img, identify="xgo2", timeout=1.0, computer_ip=COMPUTER_IP, max_retries=3
):
    """尝试发送图像到服务器，直到成功或达到最大重试次数

    Args:
        img (_type_): 要发送的图像
        identify (str, optional): 用于标识请求的字符串. Defaults to "xgo2".
        timeout (float, optional): 等待响应的超时时间. Defaults to 1.0.
        computer_ip (_type_, optional): 服务器的 IP 地址. Defaults to COMPUTER_IP.
        max_retries (int, optional): 最大重试次数. Defaults to 3.

    Returns:
        dict/None: 服务器返回的结果或 None
    """

    for attempt in range(max_retries):
        result = send_image(img, identify, timeout, computer_ip)
        if result is not None:
            return result
        time.sleep(0.1)  # 等待一段时间后重试
    return None


def _test_send_image():
    """
    测试发送图像到服务器的功能。
    """
    pwd = osp.dirname(osp.abspath(__file__))
    img_path = osp.join(pwd, "test.jpg")
    if not osp.exists(img_path):
        print(f"Error: Test image not found at {img_path}")
        return
    img = cv2.imread(img_path)
    if img is None:
        print("Error: Could not load test image")
        return

    identify = "xgo1"
    result = send_image_with_retry(img, identify, timeout=2)

    if result is None:
        print("No response received or an error occurred.")
        return
    print(f"Received response: {result}")
    xyxy = result["xyxy"]
    conf = result["conf"]
    cls_id = result["cls"]
    print(f"检测到{LABELS_CN[cls_id]}，置信度：{conf:.2f}，坐标：{xyxy}")


if __name__ == "__main__":
    _test_send_image()

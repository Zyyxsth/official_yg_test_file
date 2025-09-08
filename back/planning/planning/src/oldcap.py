import cv2
rtsp_url = "rtsp://192.168.1.25:8554/main.264"

# 打开 RTSP 流
cap = cv2.VideoCapture(rtsp_url)

# 检查是否成功打开 RTSP 流
if not cap.isOpened():
	print("Error: Unable to open the RTSP stream")
	exit(-1)


# 读取视频帧
ret, frame = cap.read()

# 检查是否成功读取帧
if not ret:
	print("Error: Unable to read frame from RTSP stream")
	cap.release()
	exit(0)

# 获取帧的高度和宽度
height, width = frame.shape[:2]

# 计算中间方形区域的边界
if width < height:
	size = width
	y_start = (height - size) // 2
	y_end = y_start + size
	x_start = 0
	x_end = width
else:
	size = height
	x_start = (width - size) // 2
	x_end = x_start + size
	y_start = 0
	y_end = height

# 裁剪中间的方形区域
cropped_frame = frame[y_start:y_end, x_start:x_end]

# 保存裁剪后的方形区域为图像文件
cv2.imwrite("/home/hialb/Elastic-Tracker-main/src/planning/planning/src/2222.jpg", cropped_frame)
print("Square frame saved as cap_frame.jpg")

# 释放资源
cap.release()
cv2.destroyAllWindows()

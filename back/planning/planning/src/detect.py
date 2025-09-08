import sys
import cv2
import numpy as np
import json

# 检查是否提供了足够的参数
if len(sys.argv) < 3:
    print("请提供两个图像文件路径作为参数")
    sys.exit(1)

# 从命令行参数获取图像文件路径
img_1_path = sys.argv[1]
img_2_path = sys.argv[2]

# 读取图像
img_1 = cv2.imread(img_1_path, cv2.IMREAD_GRAYSCALE)
img_2 = cv2.imread(img_2_path, cv2.IMREAD_GRAYSCALE)

# 确保图像被正确加载
if img_1 is None or img_2 is None:
    print("Error: Unable to load one or both images.")
    exit()

rotated_image_1 = cv2.rotate(img_1, cv2.ROTATE_90_COUNTERCLOCKWISE)
rotated_image_2 = cv2.rotate(img_2, cv2.ROTATE_90_COUNTERCLOCKWISE)

img1 = cv2.resize(rotated_image_1, (0, 0), fx=0.4, fy=0.4)
img2 = cv2.resize(rotated_image_2, (0, 0), fx=0.4, fy=0.4)

if img1 is None or img2 is None:
    print("Error: Unable to load one or both images.")
    exit()

# 初始化ORB检测器
orb = cv2.ORB_create()

kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# 创建BFMatcher对象
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# 匹配描述子
try:
    matches = bf.match(des1, des2)
except cv2.error as e:
    print(f"Error: {e}")
    exit()

# 根据匹配对提取坐标
img1_matches = [kp1[m.queryIdx].pt for m in matches]
img2_matches = [kp2[m.trainIdx].pt for m in matches]


img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
img_add = np.copy(img2)

# 创建一个与当前帧大小相同的全零掩模，用于绘制轨迹
mask = np.zeros_like(img2)

# 扭头多少-----------------------------------------------------------------------------------------------
# 假设 img1_matches 和 img2_matches 已经按照匹配对排序好了
vectors = [(img1_matches[i][0] - img2_matches[i][0], img1_matches[i][1] - img2_matches[i][1]) for i in range(len(img1_matches))] #绿色是终点

# 归一化每个向量
unit_vectors = [(vector[0] / np.sqrt(vector[0]**2 + vector[1]**2), 
                 vector[1] / np.sqrt(vector[0]**2 + vector[1]**2)) 
                for vector in vectors]

# 求和单位向量
sum_x = sum(unit_vector[0] for unit_vector in unit_vectors)
sum_y = sum(unit_vector[1] for unit_vector in unit_vectors)

# 计算单位向量的和的长度（欧几里得范数）
magnitude = np.sqrt(sum_x**2 + sum_y**2)

# 归一化单位向量的和，得到主方向的单位向量
if magnitude > 0:
    main_direction_x = sum_x / magnitude
    main_direction_y = sum_y / magnitude
else:
    main_direction_x = 0
    main_direction_y = 0
# print(f"主方向：({main_direction_x}, {main_direction_y})")

# 计算每个向量与主方向的角度
def vector_angle(unit_vector, main_direction):
    dot_product = unit_vector[0] * main_direction_x + unit_vector[1] * main_direction_y
    magnitude_unit_vector = np.sqrt(unit_vector[0]**2 + unit_vector[1]**2)
    if magnitude_unit_vector == 0:
        return 0
    angle = np.arccos(dot_product / magnitude_unit_vector)
    return np.degrees(angle)

# 计算所有单位向量与主方向的角度
angles = [vector_angle(unit_vector, (main_direction_x, main_direction_y)) for unit_vector in unit_vectors]

# 排除差异最大的向量（例如，排除角度最大的25%）
threshold_angle = np.percentile(angles, 75)  # 选择75百分位数作为阈值
filtered_unit_vectors = [unit_vector for unit_vector, angle in zip(unit_vectors, angles) if angle < threshold_angle]

# 重新计算主方向
sum_x = sum(unit_vector[0] for unit_vector in filtered_unit_vectors)
sum_y = sum(unit_vector[1] for unit_vector in filtered_unit_vectors)
magnitude = np.sqrt(sum_x**2 + sum_y**2)
if magnitude > 0:
    main_direction_x = sum_x / magnitude
    main_direction_y = sum_y / magnitude


# 计算每个向量在主方向上的投影长度
projection_lengths = [np.dot(vector, (main_direction_x, main_direction_y)) for vector in vectors]

# 求这些投影长度的平均值
average_projection_length = np.mean(projection_lengths)

# 打印新的主方向
# print(f"新的主方向：({main_direction_x}, {main_direction_y})")
print(f"{main_direction_x} {main_direction_y} {average_projection_length}")
# 将结果写入文件
with open('/home/hialb/detect_result_x.txt', 'w') as f:
    f.write(str(main_direction_x))
with open('/home/hialb/detect_result_y.txt', 'w') as f:
    f.write(str(main_direction_y))
with open('/home/hialb/detect_result_length.txt', 'w') as f:
    f.write(str(average_projection_length))


# 扭头多少-----------------------------------------------------------------------------------------------

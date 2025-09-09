import subprocess

import sys
import os
from time import sleep
import json
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)


#from siyi_sdk import SIYISDK

# rtsp_url = "rtsp://192.168.1.25:8554/main.264"
def cap(outputname):
    rtsp_url = "rtsp://10.1.1.128:8554/stream"
    # cam = SIYISDK(server_ip="192.168.1.25", port=37260)
    # yaw, pitch, roll = cam.getAttitude()
    # print(f"Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")

    # cam = SIYISDK(server_ip="192.168.1.25", port=37260)
    #cam = SIYISDK(server_ip="192.168.1.25", port=37260)
    # file_name = 156890

    #if not cam.connect():
        #   print("No connection ")
        #  exit(1)

    # file_name = float(sys.argv[1])

    # i =0
    # sleep(2)
    # yaw, pitch, roll = cam.getAttitude()
    # print(f"Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")

    file_name = int(sys.argv[1])
    # file_name = 2222

    #sleep(2)
    #yaw, pitch, roll = cam.getAttitude()

    # # 封装为 JSON 格式
    # attitude_data = {
    #     "yaw": yaw,
    #     "pitch": pitch,
    #     "roll": roll
    # }
    # print(json.dumps(attitude_data))
    # while i<10:
    #     print(f"Attidue (yaw, pitch, roll): {cam.getAttitude()}")
    #     sleep(0.5)
    #     i += 1

    #with open('/home/hialb/attitude.txt', 'w') as f:
        #   f.write(f"{yaw} {pitch} {roll}")

    print('DONE')
    #cam.disconnect()



    # cam.disconnect()
    # 使用 ffprobe 获取 RTSP 流的宽高信息
    ffprobe_command = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=s=x:p=0",
        rtsp_url
    ]

    try:
        output = subprocess.check_output(ffprobe_command).decode('utf-8').strip()
        width, height = map(int, output.split('x'))
    except subprocess.CalledProcessError as e:
        print("Error occurred while getting stream dimensions:", e)
        exit(-1)

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
        
    file_name = outputname

    # 构建 FFmpeg 命令来截取中间方形区域
    ffmpeg_command = [
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", rtsp_url,
        "-frames:v", "1",
        "-vf", f"crop={size}:{size}:{x_start}:{y_start}",  # 使用 crop 滤镜来截取中间方形区域
        "-y",
        f"/home/hialb/Elastic-Tracker-main/src/planning/planning/src/{file_name}.jpg"
    ]

    # 执行 FFmpeg 命令
    try:
        subprocess.run(ffmpeg_command, check=True)
        print("Square frame saved as square_frame.jpg")
    except subprocess.CalledProcessError as e:
        print("Error occurred while taking screenshot:", e)
	    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <output_filename>")
        sys.exit(1)

    outputname = sys.argv[1]
    cap(outputname)



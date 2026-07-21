"""
CanMV/K230 终极视觉追踪封板代码 (融合透视畸变修正)
- 高帧率 find_blobs + 最小外接多边形
- 几何学角度过滤 (防拖鞋/背景杂乱矩形)
- 对角线交点计算 (解决圆弧段斜视误差)
- 完善的内存释放机制 (脱机自启动绝对不死机)
"""

import struct
import time
import sys
import os
import gc
import math

SENSOR_BACKEND = None
MEDIA_IMPORT_ERROR = None

try:
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import media.media as media
    SENSOR_BACKEND = "media"
except Exception as exc:
    MEDIA_IMPORT_ERROR = exc
    Sensor = None
    Display = None
    media = None

try:
    from machine import UART
except Exception:
    from Maix import UART

# --- 通讯协议定义 ---
SOF1 = 0xAA
SOF2 = 0x55
TYPE_OBS = 0x01
TYPE_HEARTBEAT = 0x02

UART_ID = 3  # 根据实际硬件修改
TX_PIN = 32
RX_PIN = 33
BAUD = 115200

# --- 摄像头与画面参数 ---
FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# ========================================================
# 🎯 阈值设置区 (请根据现场光线微调)
# ========================================================
# 红色阈值 (找靶心用)
RED_THRESHOLDS =[(19, 60, 15, 127, 0, 127)] 
# 黑色阈值 (找A4纸外框用，L下限必须是0！)
BLACK_THRESHOLDS =[(0, 45, -15, 15, -15, 15)]

# --- 从丁诺那里“偷”来的几何过滤与透视解算模块 ---
def angle_between(p0, p1, p2):
    """计算夹角，过滤畸形多边形"""
    v1x = p0[0] - p1[0]
    v1y = p0[1] - p1[1]
    v2x = p2[0] - p1[0]
    v2y = p2[1] - p1[1]
    dot = v1x*v2x + v1y*v2y
    mod1 = math.sqrt(v1x**2 + v1y**2)
    mod2 = math.sqrt(v2x**2 + v2y**2)
    if mod1 == 0 or mod2 == 0:
        return 0
    cos_angle = max(min(dot / (mod1*mod2), 1), -1)
    return math.degrees(math.acos(cos_angle))

def is_good_quad(corners):
    """如果四个角不是正常的矩形透视（内角在60~120度之间），就扔掉"""
    if len(corners) != 4:
        return False
    for i in range(4):
        p0 = corners[(i-1)%4]
        p1 = corners[i]
        p2 = corners[(i+1)%4]
        a = angle_between(p0, p1, p2)
        if not (60 <= a <= 120):
            return False
    return True

def sort_corners(pts):
    """对四个角点进行左上,右上,右下,左下排序，方便划对角线"""
    s = [p[0] + p[1] for p in pts]
    d = [p[0] - p[1] for p in pts]
    tl = pts[s.index(min(s))]
    br = pts[s.index(max(s))]
    tr = pts[d.index(max(d))]
    bl = pts[d.index(min(d))]
    return [tl, tr, br, bl]

def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    """求对角线交点，哪怕严重斜视，这也是绝对的物理靶心！"""
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None
    px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denom
    py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denom
    return int(px), int(py)

# ========================================================
# 核心识别函数
# ========================================================
def detect_black_rect_advanced(img):
    """融合了高帧率与极高几何精度的黑框识别"""
    if not hasattr(img, "find_blobs"):
        return None

    # margin=10 让反光断裂的黑胶带被强行粘合在一起！
    try:
        blobs = img.find_blobs(BLACK_THRESHOLDS, pixels_threshold=150, area_threshold=150, merge=True, margin=10)
    except:
        return None
        
    if not blobs:
        return None

    # 优先看最大的黑块
    blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

    for b in blobs:
        try:
            corners = b.min_corners()
        except:
            continue
            
        if len(corners) != 4:
            continue
            
        sorted_pts = sort_corners(corners)
        
        # 几何校验：剔除背景里的拖鞋、黑椅子等杂物
        if not is_good_quad(sorted_pts):
            continue
            
        # 求对角线交点
        pt = line_intersection(sorted_pts[0][0], sorted_pts[0][1], sorted_pts[2][0], sorted_pts[2][1],
                               sorted_pts[1][0], sorted_pts[1][1], sorted_pts[3][0], sorted_pts[3][1])
        if pt:
            cx, cy = pt
            rect_roi = b.rect() # 顺便获取外接框用于限制红点的 ROI
            return (sorted_pts, cx, cy, rect_roi)
    return None

def detect_red_target(img, roi_override=None):
    """在黑框区域内找真正的红色靶心"""
    if not hasattr(img, "find_blobs"):
        return None
    try:
        if roi_override is None:
            blobs = img.find_blobs(RED_THRESHOLDS, pixels_threshold=5, merge=True)
        else:
            blobs = img.find_blobs(RED_THRESHOLDS, roi=roi_override, pixels_threshold=5, merge=True)
    except:
        return None

    if not blobs:
        return None

    # 返回最大红块
    return max(blobs, key=lambda b: b.pixels())

# ========================================================
# 串口打包与发送
# ========================================================
def checksum(data):
    return sum(data) & 0xFF

def pack_obs(cx, cy, lost):
    err_u = 0 if lost else (cx - CENTER_X)
    err_v = 0 if lost else (cy - CENTER_Y)
    conf = 0 if lost else 100
    
    payload = struct.pack(
        "<IhhhhBBBB",
        time.ticks_ms() & 0xFFFFFFFF,
        int(cx), int(cy),
        int(err_u), int(err_v),
        int(conf), int(lost), 0, 0
    )
    
    out = bytearray([SOF1, SOF2, TYPE_OBS, 0, len(payload)])
    out.extend(payload)
    out.append(checksum(out[2:]))
    return bytes(out)

# ========================================================
# K230 硬件初始化 (防止脱机黑屏的关键)
# ========================================================
def setup_k230_camera():
    try:
        media.init() # 第一步必须初始化多媒体内存
        sensor = Sensor(id=2) # 你的硬件通道
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)
        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H)
        sensor.run()
        
        # 锁定曝光和白平衡防干扰
        time.sleep(1)
        try:
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
        except:
            pass
            
        print("Camera Init OK")
        return sensor
    except Exception as e:
        print("Camera Init Failed:", e)
        return None

# ========================================================
# 主循环
# ========================================================
try:
    uart = UART(UART_ID, BAUD, tx=TX_PIN, rx=RX_PIN)
except:
    uart = UART(UART_ID, BAUD)

sensor = setup_k230_camera()

# 动量保持变量
last_black_roi = None
last_black_ms = 0

while True:
    now = time.ticks_ms()
    
    if hasattr(os, "exitpoint"):
        os.exitpoint() # 响应 IDE 停止按键

    if sensor is None:
        time.sleep(1)
        continue

    img = sensor.snapshot()
    
    # 状态变量
    cx, cy = CENTER_X, CENTER_Y
    lost = 1
    red_target = None
    black_result = detect_black_rect_advanced(img)
    roi_for_red = None

    # 1. 解析黑框
    if black_result is not None:
        corners, bcx, bcy, rect_roi = black_result
        last_black_roi = rect_roi
        last_black_ms = now
        
        # 画出那个装逼的黄色斜框
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], 
                          corners[(i+1)%4][0], corners[(i+1)%4][1], 
                          color=(255, 255, 0), thickness=2)
            
        # 保底靶心设为黑框对角线交点
        cx, cy = bcx, bcy
        lost = 0
        roi_for_red = rect_roi
    
    # ROI动量保持 (防止反光闪烁)
    elif last_black_roi is not None and time.ticks_diff(now, last_black_ms) < 500:
        roi_for_red = last_black_roi

    # 2. 在黑框里找红点 (优先级最高！)
    red_target = detect_red_target(img, roi_for_red)
    
    if red_target:
        # 如果找到真正的靶心，覆盖黑框中心！
        cx = red_target.cx()
        cy = red_target.cy()
        lost = 0
        img.draw_rectangle(red_target.rect(), color=(255, 0, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0), size=15, thickness=2)
    elif not lost:
        # 没红点，但有黑框，画黄色十字作为保底瞄准
        img.draw_cross(cx, cy, color=(255, 255, 0), size=15, thickness=2)
    else:
        # 全丢了，画蓝色十字
        img.draw_cross(CENTER_X, CENTER_Y, color=(0, 0, 255), size=15)

    # 3. 发送串口协议
    pkt = pack_obs(cx, cy, lost)
    uart.write(pkt)

    # 4. 显示与释放内存 (防止脱机一秒黑屏的终极秘诀)
    Display.show_image(img)
    gc.collect()
    time.sleep_ms(5)

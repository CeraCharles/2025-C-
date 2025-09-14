import time, os, sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ----------------- 模式定义区 -----------------
class Mode:
    RECTANGLE     = "RECTANGLE"
    CIRCLE        = "CIRCLE"
    TRIANGLE      = "TRIANGLE"
    LINE_SEGMENT  = "LINE_SEGMENT"
    MIN_RECTANGLE = "MIN_RECTANGLE"

# ----------------- 全局状态变量 -----------------
current_mode = Mode.MIN_RECTANGLE       # 默认启动模式
line_mode_trigger = None                # 记录进入线段模式的指令 (2, 3, or 4)
line_mode_active = False                # 线段模式数据采集中标志
measurements = []                       # 存储线段模式的测量数据
start_time = 0                          # 线段模式的开始时间
last_known_distance_cm = 0              # 线段模式中最后一次有效测量的距离

# ----------------- 新增：数据缓冲系统 -----------------
class DataBuffer:
    def __init__(self, buffer_time_ms=1000):
        self.buffer_time_ms = buffer_time_ms
        self.distance_buffer = []
        self.measurement_buffer = []
        self.last_output_time = 0

    def add_data(self, distance_cm, measurement_cm):
        current_time = time.ticks_ms()
        # 清理超过1秒的旧数据
        self.distance_buffer = [(t, d) for t, d in self.distance_buffer if time.ticks_diff(current_time, t) <= self.buffer_time_ms]
        self.measurement_buffer = [(t, m) for t, m in self.measurement_buffer if time.ticks_diff(current_time, t) <= self.buffer_time_ms]

        # 添加新数据
        self.distance_buffer.append((current_time, distance_cm))
        self.measurement_buffer.append((current_time, measurement_cm))

    def should_output(self):
        current_time = time.ticks_ms()
        return time.ticks_diff(current_time, self.last_output_time) >= self.buffer_time_ms

    def get_median_and_clear(self):
        if not self.distance_buffer or not self.measurement_buffer:
            return None, None

        # 计算中位数
        distances = [d for _, d in self.distance_buffer]
        measurements = [m for _, m in self.measurement_buffer]

        distances.sort()
        measurements.sort()

        distance_median = distances[len(distances)//2]
        measurement_median = measurements[len(measurements)//2]

        # 更新输出时间并清空缓冲区
        self.last_output_time = time.ticks_ms()
        self.distance_buffer.clear()
        self.measurement_buffer.clear()

        return distance_median, measurement_median

# 创建数据缓冲实例
data_buffer = DataBuffer()

# ----------------- 资源清理函数 -----------------
def cleanup_resources():
    """清理所有可能占用的资源"""
    try:
        print("[清理] 开始清理资源...")
        # 尝试停止可能存在的传感器
        try:
            if 'sensor' in globals():
                sensor.stop()
        except:
            pass

        # 尝试清理显示器
        try:
            Display.deinit()
        except:
            pass

        # 尝试清理媒体管理器
        try:
            MediaManager.deinit()
        except:
            pass

        # 短暂延迟确保资源释放
        time.sleep_ms(200)
        print("[清理] 资源清理完成")
    except Exception as e:
        print(f"[清理] 清理过程中出错: {e}")

# ----------------- 串口命令解析 -----------------
def parse_uart_command(uart_instance):
    global current_mode, line_mode_trigger, line_mode_active, measurements, start_time
    if uart_instance.any():
        try:
            full_message = uart_instance.read().decode().strip()
            if len(full_message) != 10:
               if full_message:
                   print(f"[串口警告] 收到长度不为10的消息，已忽略: '{full_message}'")
               return
            cmd_char = full_message[8]

            if not cmd_char.isdigit():
                uart_instance.write(f"INVALID_8TH_CHAR:{cmd_char}\n")
                return

            cmd_int = int(cmd_char)
            mode_map = {
                0: Mode.RECTANGLE, 1: Mode.CIRCLE, 2: Mode.LINE_SEGMENT,
                3: Mode.LINE_SEGMENT, 4: Mode.LINE_SEGMENT, 5: Mode.MIN_RECTANGLE,
                6: Mode.TRIANGLE,
            }

            if cmd_int in mode_map:
                new_mode = mode_map[cmd_int]
                if new_mode != current_mode or new_mode == Mode.LINE_SEGMENT:
                    current_mode = new_mode
                    print(f"[模式切换] 从'{full_message}'中提取指令 '{cmd_int}', 切换到: {current_mode}")
                    uart_instance.write(f"MODE_SWITCHED_TO:{current_mode}\n")

                    if current_mode == Mode.LINE_SEGMENT:
                        line_mode_trigger = cmd_int
                        line_mode_active = True
                        measurements.clear()
                        start_time = time.ticks_ms()
                        print(f"[线段模式] 已激活 (触发指令: {line_mode_trigger})。开始采集数据...")
                    else:
                        line_mode_active = False
            else:
                uart_instance.write(f"UNKNOWN_COMMAND:{cmd_int}\n")
        except Exception as e:
            print(f"[串口解析错误] {e}")

# ----------------- 辅助函数区 -----------------
def hypot(x, y): return math.sqrt(x**2 + y**2)

def validate_roi(roi, img_width=800, img_height=480):
    """验证ROI是否有效"""
    if not roi or len(roi) != 4:
        return False
    x, y, w, h = roi
    if x < 0 or y < 0 or w <= 0 or h <= 0:
        return False
    if x + w > img_width or y + h > img_height:
        return False
    return True

def safe_create_inner_roi(corners, margin=5, img_width=800, img_height=480):
    """安全地创建内部ROI，确保不超出边界"""
    try:
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # 确保ROI不超出图像边界
        roi_x = max(0, min_x + margin)
        roi_y = max(0, min_y + margin)
        roi_w = min(max_x - margin, img_width) - roi_x
        roi_h = min(max_y - margin, img_height) - roi_y

        # 确保宽高为正
        if roi_w <= 0 or roi_h <= 0:
            print(f"[ROI警告] 计算的ROI尺寸无效: w={roi_w}, h={roi_h}")
            return None

        roi = (int(roi_x), int(roi_y), int(roi_w), int(roi_h))

        if validate_roi(roi, img_width, img_height):
            return roi
        else:
            print(f"[ROI警告] ROI验证失败: {roi}")
            return None
    except Exception as e:
        print(f"[ROI错误] 创建ROI时出错: {e}")
        return None

def calculate_angle(p1, p2, p3):
    vec1_x, vec1_y = p1[0] - p2[0], p1[1] - p2[1]
    vec2_x, vec2_y = p3[0] - p2[0], p3[1] - p2[1]
    len_vec1 = hypot(vec1_x, vec1_y)
    len_vec2 = hypot(vec2_x, vec2_y)
    if len_vec1 == 0 or len_vec2 == 0: return 0
    dot_product = vec1_x * vec2_x + vec1_y * vec2_y
    cos_angle = max(-1.0, min(1.0, dot_product / (len_vec1 * len_vec2)))
    return math.degrees(math.acos(cos_angle))

def point_in_polygon(point, polygon):
    x, y = point; n = len(polygon); inside = False
    for i in range(n):
        j = (i + 1) % n; xi, yi = polygon[i]; xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi): inside = not inside
    return inside

def point_to_line_distance(point, line_start, line_end):
    x, y = point; x1, y1 = line_start; x2, y2 = line_end
    line_len_sq = (x2 - x1)**2 + (y2 - y1)**2
    if line_len_sq == 0: return hypot(x - x1, y - y1)
    t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / line_len_sq
    t = max(0, min(1, t))
    proj_x, proj_y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
    return hypot(x - proj_x, y - proj_y)

def is_nested_with_min_distance(inner_bbox_corners, outer_rect_corners, min_distance=8):
    if not all(point_in_polygon(p, outer_rect_corners) for p in inner_bbox_corners): return False
    min_dist = float('inf')
    for i in range(4):
        inner_point = inner_bbox_corners[i]
        for j in range(4):
            outer_start, outer_end = outer_rect_corners[j], outer_rect_corners[(j+1)%4]
            dist = point_to_line_distance(inner_point, outer_start, outer_end)
            if dist < min_dist: min_dist = dist
    return min_dist >= min_distance

def calculate_pixel_area(corners):
    area = 0; n = len(corners)
    for i in range(n):
        x_i, y_i = corners[i]; x_j, y_j = corners[(i + 1) % n]
        area += (x_i * y_j - x_j * y_i)
    return abs(area) / 2

def calculate_bbox_area(bbox_corners):
    min_x = min(p[0] for p in bbox_corners)
    max_x = max(p[0] for p in bbox_corners)
    min_y = min(p[1] for p in bbox_corners)
    max_y = max(p[1] for p in bbox_corners)
    return (max_x - min_x) * (max_y - min_y)

def analyze_by_inner_roi_lines(img, roi, area_factor, debug_mode=True):
    try:
        if not validate_roi(roi):
            print(f"[线段分析] ROI无效: {roi}")
            return None

        segments = img.find_line_segments(roi=roi, merge_distance=5, max_gap=5, threshold=SEGMENT_THRESHOLD)
        if not segments or len(segments) < 2: return None
        length_scale = math.sqrt(area_factor)
        for i in range(len(segments)):
            for j in range(i+1, len(segments)):
                s1, s2 = segments[i], segments[j]; l1, l2 = s1.length(), s2.length()
                if max(l1, l2) == 0: continue
                if (min(l1, l2) / max(l1, l2)) >= SIDE_LENGTH_RATIO_THRESHOLD:
                    avg_px = (l1 + l2) / 2; real_mm = avg_px * length_scale
                    if TARGET_LENGTH_MIN_MM <= real_mm <= TARGET_LENGTH_MAX_MM:
                        return {'length_mm': real_mm, 'segments': [s1, s2], 'avg_pixel_length': avg_px}
        return None
    except OSError as e:
        print(f"[线段分析] ROI错误: {e}")
        return None
    except Exception as e:
        print(f"[线段分析] 处理错误: {e}")
        return None

# ----------------- 参数配置区 -----------------
sensor_id = 2; sensor = None
picture_width, picture_height = 800, 480
ANGLE_THRESHOLD = 50
A4_WIDTH, A4_HEIGHT = 210.0, 297.0
A4_ACTUAL_AREA = A4_WIDTH * A4_HEIGHT
FOCAL_PIX = 702.7
roi=(300, 50, 200, 380)
DISPLAY_MODE = "LCD"
MIN_RECT_DISTANCE, MIN_INNER_OBJECT_SIZE = 8, 100
TRIANGLE_MIN_DENSITY, TRIANGLE_MAX_DENSITY = 0.30, 0.70
TRIANGLE_MIN_SHAPE_FACTOR, TRIANGLE_MAX_SHAPE_FACTOR = 0.1, 0.9
SEGMENT_THRESHOLD = 50
SIDE_LENGTH_RATIO_THRESHOLD = 0.8
MARGIN = 5
TARGET_LENGTH_MIN_MM, TARGET_LENGTH_MAX_MM = 55.0, 125.0
TARGET_COUNT, TIME_LIMIT_MS = 20, 3000

# A4纸面积限制参数
MIN_A4_AREA = 5000   # 最小A4纸面积（像素）
MAX_A4_AREA = 80000  # 最大A4纸面积（像素）

if DISPLAY_MODE == "LCD": DISPLAY_WIDTH, DISPLAY_HEIGHT = 800, 480

# ----------------- 主程序区 -----------------
try:
    # 启动前先清理资源
    cleanup_resources()

    print("[系统] 初始化传感器...")
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=picture_width, height=picture_height)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    if DISPLAY_MODE == "LCD":
        print("[系统] 初始化显示器...")
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

    print("[系统] 初始化串口...")
    fpioa = FPIOA()
    fpioa.set_function(11, fpioa.UART2_TXD)
    fpioa.set_function(12, fpioa.UART2_RXD)
    uart = UART(UART.UART2, 115200, 8, UART.PARITY_NONE, UART.STOPBITS_ONE)

    print("[系统] 串口已初始化，等待10位字符串命令...")
    print('串口通信协议: 距离 t6.txt="数值";\\r\\n, 边长/长度/直径 t7.txt="数值";\\r\\n')
    print(f"[系统] A4纸面积限制: {MIN_A4_AREA} - {MAX_A4_AREA} 像素")
    print("[系统] 数据输出: 一秒内中位数")

    print("[系统] 初始化媒体管理器...")
    MediaManager.init()
    sensor.run()
    fps = time.clock()

    print("[系统] 系统启动完成!")

    while True:
        try:
            fps.tick(); os.exitpoint(); parse_uart_command(uart)
            src_img = sensor.snapshot()
            src_img.binary([(80, 255)], invert=False)
            src_img.draw_rectangle(0, 0, picture_width, roi[1], color=0, fill=True)
            src_img.draw_rectangle(0, roi[1]+roi[3], picture_width, picture_height - (roi[1]+roi[3]), color=0, fill=True)
            src_img.draw_rectangle(0, roi[1], roi[0], roi[3], color=0, fill=True)
            src_img.draw_rectangle(roi[0]+roi[2], roi[1], picture_width - (roi[0]+roi[2]), roi[3], color=0, fill=True)

            all_candidate_outer_rects = []
            for rect_obj in src_img.find_rects(threshold=3500):
                corners = rect_obj.corners()
                if len(corners) != 4: continue

                area = calculate_pixel_area(corners)

                # 过滤掉过大或过小的矩形
                if area < MIN_A4_AREA or area > MAX_A4_AREA:
                    continue

                # 检查角度是否接近90度
                is_rect = all(abs(calculate_angle(corners[i], corners[(i+1)%4], corners[(i+2)%4]) - 90) <= ANGLE_THRESHOLD for i in range(4))
                if is_rect:
                    all_candidate_outer_rects.append(corners)

            outer_rect_corners = max(all_candidate_outer_rects, key=calculate_pixel_area) if all_candidate_outer_rects else None

            if outer_rect_corners:
                outer_rect_pixel_area = calculate_pixel_area(outer_rect_corners)
                if outer_rect_pixel_area == 0: continue

                # --------- 模式判断 ---------
                if current_mode == Mode.LINE_SEGMENT:
                    distance_D_cm_area_based = (FOCAL_PIX * math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)) / 10
                    area_conversion_factor = A4_ACTUAL_AREA / outer_rect_pixel_area
                    if line_mode_active:
                        try:
                            edge_proc = src_img.copy(); edge = edge_proc.copy(); edge.erode(1); edge_proc.difference(edge)
                            inner_line_roi = safe_create_inner_roi(outer_rect_corners, MARGIN, picture_width, picture_height)

                            if inner_line_roi is not None:
                                result = analyze_by_inner_roi_lines(edge_proc, inner_line_roi, area_conversion_factor)
                                if result:
                                    measurements.append(result['length_mm'])
                                    last_known_distance_cm = distance_D_cm_area_based
                                    print(f"✅ 线段测量: {result['length_mm']:.1f} mm")
                        except OSError as e:
                            print(f"[线段模式] ROI错误: {e}，跳过本次测量")
                        except Exception as e:
                            print(f"[线段模式] 处理错误: {e}，跳过本次测量")
                        time_elapsed = time.ticks_diff(time.ticks_ms(), start_time)
                        if len(measurements) >= TARGET_COUNT or time_elapsed >= TIME_LIMIT_MS:
                            line_mode_active = False
                            print("[线段模式] 数据采集完成。")
                            if not measurements:
                                uart.write(f't7.txt="-1";\r\n')
                                print('[线段模式] 未采集到有效数据，发送: t7.txt="-1"')
                            else:
                                measurements.sort(); final_result = None
                                if line_mode_trigger == 2: final_result = min(measurements)
                                elif line_mode_trigger == 3: final_result = measurements[len(measurements)//2]
                                elif line_mode_trigger == 4: final_result = max(measurements)
                                if final_result is not None:
                                    uart.write(f't7.txt.str="{(final_result/10)}";\r\n')
                                    uart.write(f't6.txt.str="{(last_known_distance_cm)}";\r\n')
                                    print(f'发送线段结果: 长度 t7.txt="{(final_result)}", 距离 t6.txt="{(last_known_distance_cm)}"')
                            print("[线段模式] 发送完毕，等待新指令...")
                    else:
                        src_img.draw_string(10, 10, "LINE MODE FINISHED. Waiting...", color=(255,255,0), scale=2)

                elif current_mode == Mode.RECTANGLE:
                    distance_D_cm_area_based = (FOCAL_PIX * math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)) / 10
                    linear_conversion_factor = math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)
                    for r_obj in src_img.find_rects(threshold=3500):
                        i_corners = r_obj.corners()
                        if len(i_corners) != 4 or calculate_pixel_area(i_corners) >= outer_rect_pixel_area * 0.95: continue
                        if all(abs(calculate_angle(i_corners[i], i_corners[(i+1)%4], i_corners[(i+2)%4]) - 90) <= ANGLE_THRESHOLD for i in range(4)):
                            bbox = [(min(p[0] for p in i_corners), min(p[1] for p in i_corners)), (max(p[0] for p in i_corners), min(p[1] for p in i_corners)), (max(p[0] for p in i_corners), max(p[1] for p in i_corners)), (min(p[0] for p in i_corners), max(p[1] for p in i_corners))]
                            if is_nested_with_min_distance(bbox, outer_rect_corners, MIN_RECT_DISTANCE) and calculate_bbox_area(bbox) >= MIN_INNER_OBJECT_SIZE:
                                side_lengths_pixel = [hypot(i_corners[i][0] - i_corners[(i+1)%4][0], i_corners[i][1] - i_corners[(i+1)%4][1]) for i in range(2)]
                                shortest_side_mm = min(side_lengths_pixel) * linear_conversion_factor
                                shortest_side_cm = shortest_side_mm / 10

                                # 添加到缓冲区
                                data_buffer.add_data(distance_D_cm_area_based, shortest_side_cm)

                                # 检查是否应该输出
                                if data_buffer.should_output():
                                    dist_median, meas_median = data_buffer.get_median_and_clear()
                                    if dist_median is not None and meas_median is not None:
                                        uart.write(f't7.txt.str="{meas_median:.2f}";\r\n')
                                        uart.write(f't6.txt.str="{dist_median:.2f}";\r\n')
                                        print(f'发送矩形中位数: 边长 t7.txt="{meas_median:.2f}", 距离 t6.txt="{dist_median:.2f}"')

                elif current_mode == Mode.CIRCLE:
                    distance_D_cm_area_based = (FOCAL_PIX * math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)) / 10
                    linear_conversion_factor = math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)
                    try:
                        inner_roi = safe_create_inner_roi(outer_rect_corners, MARGIN, picture_width, picture_height)

                        if inner_roi is not None:
                            for c_obj in src_img.find_circles(roi=inner_roi, threshold=8000, r_min=10, r_max=500):
                                cx, cy, r = c_obj[0], c_obj[1], c_obj[2]
                                bbox = [(cx-r, cy-r), (cx+r, cy-r), (cx+r, cy+r), (cx-r, cy+r)]
                                if r*r >= MIN_INNER_OBJECT_SIZE and is_nested_with_min_distance(bbox, outer_rect_corners, MIN_RECT_DISTANCE):
                                    actual_diameter_mm = (r * 2) * linear_conversion_factor
                                    actual_diameter_cm = actual_diameter_mm / 10

                                    # 添加到缓冲区
                                    data_buffer.add_data(distance_D_cm_area_based, actual_diameter_cm)

                                    # 检查是否应该输出
                                    if data_buffer.should_output():
                                        dist_median, meas_median = data_buffer.get_median_and_clear()
                                        if dist_median is not None and meas_median is not None:
                                            uart.write(f't7.txt.str="{meas_median:.2f}";\r\n')
                                            uart.write(f't6.txt.str="{dist_median:.2f}";\r\n')
                                            print(f'发送圆形中位数: 直径 t7.txt="{meas_median:.2f}", 距离 t6.txt="{dist_median:.2f}"')
                    except OSError as e:
                        print(f"[圆形模式] ROI错误: {e}，跳过本次检测")
                    except Exception as e:
                        print(f"[圆形模式] 处理错误: {e}，跳过本次检测")

                elif current_mode == Mode.MIN_RECTANGLE:
                    outer_edges_pix = [hypot(outer_rect_corners[i][0] - outer_rect_corners[(i + 1) % 4][0], outer_rect_corners[i][1] - outer_rect_corners[(i + 1) % 4][1]) for i in range(4)]
                    if not outer_edges_pix: continue
                    pixel_width_ref = min(outer_edges_pix)
                    distance_D_cm = (FOCAL_PIX * A4_WIDTH) / pixel_width_ref / 10
                    all_nested_rects_info = []
                    for r_obj in src_img.find_rects(threshold=3500):
                        i_corners = r_obj.corners()
                        if len(i_corners) == 4 and calculate_pixel_area(i_corners) < outer_rect_pixel_area * 0.95:
                             if all(abs(calculate_angle(i_corners[i], i_corners[(i+1)%4], i_corners[(i+2)%4]) - 90) <= ANGLE_THRESHOLD for i in range(4)):
                                bbox = [(min(p[0] for p in i_corners), min(p[1] for p in i_corners)), (max(p[0] for p in i_corners), min(p[1] for p in i_corners)), (max(p[0] for p in i_corners), max(p[1] for p in i_corners)), (min(p[0] for p in i_corners), max(p[1] for p in i_corners))]
                                if is_nested_with_min_distance(bbox, outer_rect_corners, MIN_RECT_DISTANCE) and calculate_bbox_area(bbox) >= MIN_INNER_OBJECT_SIZE:
                                    side1_pix = hypot(i_corners[0][0] - i_corners[1][0], i_corners[0][1] - i_corners[1][1])
                                    side2_pix = hypot(i_corners[1][0] - i_corners[2][0], i_corners[1][1] - i_corners[2][1])
                                    side1_mm = side1_pix * (A4_WIDTH / pixel_width_ref)
                                    side2_mm = side2_pix * (A4_WIDTH / pixel_width_ref)
                                    shortest_side_mm = min(side1_mm, side2_mm)
                                    all_nested_rects_info.append({'corners': i_corners, 'shortest_side_mm': shortest_side_mm})
                    if all_nested_rects_info:
                        min_rect_info = min(all_nested_rects_info, key=lambda r: r['shortest_side_mm'])
                        final_shortest_side_mm = min_rect_info['shortest_side_mm']
                        final_shortest_side_cm = final_shortest_side_mm / 10

                        # 添加到缓冲区
                        data_buffer.add_data(distance_D_cm, final_shortest_side_cm)

                        # 检查是否应该输出
                        if data_buffer.should_output():
                            dist_median, meas_median = data_buffer.get_median_and_clear()
                            if dist_median is not None and meas_median is not None:
                                uart.write(f't7.txt.str="{meas_median:.2f}";\r\n')
                                uart.write(f't6.txt.str="{dist_median:.2f}";\r\n')
                                print(f'发送最小矩形中位数: 边长 t7.txt="{meas_median:.2f}", 距离 t6.txt="{dist_median:.2f}"')

                elif current_mode == Mode.TRIANGLE:
                    distance_D_cm_area_based = (FOCAL_PIX * math.sqrt(A4_ACTUAL_AREA / outer_rect_pixel_area)) / 10
                    area_conversion_factor = A4_ACTUAL_AREA / outer_rect_pixel_area
                    for blob in src_img.find_blobs([(0, 0)], pixels_threshold=100, area_threshold=100, merge=False):
                        if blob.w() < 15 or blob.h() < 15: continue
                        px_area = blob.pixels(); aspect = max(blob.w(), blob.h()) / min(blob.w(), blob.h()) if min(blob.w(), blob.h()) > 0 else float('inf')
                        shape = (px_area * 4 * math.pi) / (blob.perimeter()**2) if blob.perimeter() > 0 else 0
                        if (TRIANGLE_MIN_SHAPE_FACTOR <= shape <= TRIANGLE_MAX_SHAPE_FACTOR and aspect < 3.0 and TRIANGLE_MIN_DENSITY <= blob.density() <= TRIANGLE_MAX_DENSITY):
                            bbox = [(blob.x(), blob.y()), (blob.x()+blob.w(), blob.y()), (blob.x()+blob.w(), blob.y()+blob.h()), (blob.x(), blob.y()+blob.h())]
                            if is_nested_with_min_distance(bbox, outer_rect_corners, MIN_RECT_DISTANCE):
                                triangle_actual_area_mm2 = px_area * area_conversion_factor
                                if triangle_actual_area_mm2 > 0:
                                    equilateral_side_mm = math.sqrt((4 * triangle_actual_area_mm2) / math.sqrt(3))
                                    equilateral_side_cm = equilateral_side_mm / 10

                                    # 添加到缓冲区
                                    data_buffer.add_data(distance_D_cm_area_based, equilateral_side_cm)

                                    # 检查是否应该输出
                                    if data_buffer.should_output():
                                        dist_median, meas_median = data_buffer.get_median_and_clear()
                                        if dist_median is not None and meas_median is not None:
                                            uart.write(f't7.txt.str="{meas_median:.2f}";\r\n')
                                            uart.write(f't6.txt.str="{dist_median:.2f}";\r\n')
                                            print(f'发送三角形中位数: 边长 t7.txt="{meas_median:.2f}", 距离 t6.txt="{dist_median:.2f}"')

            if DISPLAY_MODE == "LCD":
                Display.show_image(src_img, x=0, y=0, layer=Display.LAYER_OSD1)

        except OSError as e:
            print(f"[主循环] ROI或图像处理错误: {e}，继续运行...")
            time.sleep_ms(50)
        except Exception as e:
            print(f"[主循环] 其他错误: {e}，继续运行...")
            time.sleep_ms(50)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {type(e).__name__}: {e}")
finally:
    print("[清理] 程序结束，清理资源...")
    cleanup_resources()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(200)  # 更长的延迟确保资源完全释放
    print("[清理] 程序完全退出")

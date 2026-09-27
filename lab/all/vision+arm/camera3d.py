import cv2
import numpy as np

# 修正 TensorRT 與新版 Numpy 的 np.bool 相容性問題
np.bool = bool 

import pyrealsense2 as rs
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ==========================================
# 1. TensorRT 引擎初始化設定
# ==========================================
ENGINE_PATH = "bottle.engine"
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4
INPUT_WIDTH = 640
INPUT_HEIGHT = 640

logger = trt.Logger(trt.Logger.WARNING)

def load_engine(engine_path):
    with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

engine = load_engine(ENGINE_PATH)
context = engine.create_execution_context()

inputs, outputs, bindings, stream = [], [], [], cuda.Stream()
for binding in engine:
    size = trt.volume(engine.get_binding_shape(binding))
    dtype = trt.nptype(engine.get_binding_dtype(binding))
    host_mem = cuda.pagelocked_empty(size, dtype)
    device_mem = cuda.mem_alloc(host_mem.nbytes)
    bindings.append(int(device_mem))
    if engine.binding_is_input(binding):
        inputs.append({'host': host_mem, 'device': device_mem})
    else:
        outputs.append({'host': host_mem, 'device': device_mem})

# ==========================================
# 2. RealSense D435i 初始化與對齊設定
# ==========================================
pipeline = rs.pipeline()
config = rs.config()

# 同時啟用彩色與深度串流，並維持解析度一致以利對齊
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

print("[INFO] 啟動 RealSense D435i...")
profile = pipeline.start(config)

# 建立對齊工具 (將深度圖對齊到彩色圖)
align_to = rs.stream.color
align = rs.align(align_to)

# 獲取彩色相機的內參 (用於 2D 轉 3D 計算)
color_profile = profile.get_stream(rs.stream.color)
intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

# ==========================================
# 3. 影像前處理與後處理函式 (加入 3D 轉換)
# ==========================================
def preprocess_image(img):
    image_resized = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
    image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
    image_normalized = image_rgb.astype(np.float32) / 255.0
    image_transposed = np.transpose(image_normalized, (2, 0, 1))
    image_expanded = np.expand_dims(image_transposed, axis=0)
    return image_expanded, image_resized

def postprocess(output, orig_img, depth_frame, cam_intrinsics):
    """解析 YOLO 輸出，獲取深度，並計算 3D 空間座標"""
    img_h, img_w = orig_img.shape[:2]
    x_factor = img_w / INPUT_WIDTH
    y_factor = img_h / INPUT_HEIGHT
    
    num_attributes = len(output) // 8400
    preds = np.reshape(output, (num_attributes, 8400)).T 
    
    boxes = []
    confidences = []
    class_ids = []
    class_names = ["Class_0", "Class_1", "Class_2", "Class_3"] 
    
    for row in preds:
        classes_scores = row[4:]
        class_id = np.argmax(classes_scores)
        conf = classes_scores[class_id]
        
        if conf > CONF_THRESHOLD:
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            
            left = int((cx - w / 2) * x_factor)
            top = int((cy - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            boxes.append([left, top, width, height])
            confidences.append(float(conf))
            class_ids.append(class_id)
            
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, IOU_THRESHOLD)
    
    if len(indices) > 0:
        for i in np.array(indices).flatten():
            box = boxes[i]
            left, top, width, height = box[0], box[1], box[2], box[3]
            score = confidences[i]
            cl_id = class_ids[i]
            label_name = class_names[cl_id] if cl_id < len(class_names) else f"Obj_{cl_id}"
            
            # 1. 計算框的中心點像素座標 (對應 640x480 原始畫面)
            cx_pixel = int(left + width / 2)
            cy_pixel = int(top + height / 2)
            
            # 防止邊界溢出
            cx_pixel = max(0, min(cx_pixel, img_w - 1))
            cy_pixel = max(0, min(cy_pixel, img_h - 1))
            
            # 2. 從對齊後的深度圖取得該點的實際距離 (單位：公尺)
            distance = depth_frame.get_distance(cx_pixel, cy_pixel)
            
            # 繪製 2D 框
            cv2.rectangle(orig_img, (left, top), (left + width, top + height), (0, 255, 0), 2)
            
            # 3. 如果成功取得深度，進行 2D 到 3D 的反投影
            if distance > 0:
                # point_3d 格式為 [X, Y, Z]，單位是公尺
                point_3d = rs.rs2_deproject_pixel_to_point(cam_intrinsics, [cx_pixel, cy_pixel], distance)
                x_m, y_m, z_m = point_3d[0], point_3d[1], point_3d[2]
                
                # 在畫面上印出 3D 座標 (轉換成公分顯示比較直觀)
                text_3d = f"X:{x_m*100:.1f} Y:{y_m*100:.1f} Z:{z_m*100:.1f} cm"
                cv2.putText(orig_img, text_3d, (left, top - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # 終端機實時印出詳細數據
                print(f"[{label_name}] 偵測到！相對相機中心座標: X={x_m:.3f}m, Y={y_m:.3f}m, Z={z_m:.3f}m")
            else:
                text_3d = "Depth: Unknown"
                cv2.putText(orig_img, text_3d, (left, top - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.putText(orig_img, f"{label_name}: {score:.2f}", (left, top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
    return orig_img

# ==========================================
# 4. 主迴圈
# ==========================================
print("[INFO] 開始影像推論與 3D 座標轉換，按 'q' 鍵退出...")
try:
    while True:
        # 擷取 RealSense 原始畫面
        frames = pipeline.wait_for_frames()
        
        # 關鍵：將深度畫面與彩色畫面進行空間對齊
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue
            
        color_image = np.asanyarray(color_frame.get_data())
        
        # 前處理
        input_data, _ = preprocess_image(color_image)
        np.copyto(inputs[0]['host'], input_data.ravel())
        
        # TensorRT 推論
        cuda.memcpy_htod_async(inputs[0]['device'], inputs[0]['host'], stream)
        context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
        cuda.memcpy_dtoh_async(outputs[0]['host'], outputs[0]['device'], stream)
        stream.synchronize()
        
        # 後處理（傳入 depth_frame 與相機內參 intrinsics）
        trt_output = outputs[0]['host']
        result_image = postprocess(trt_output, color_image, depth_frame, intrinsics)
        
        cv2.imshow("D435i + YOLO11n 3D Coordinate", result_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[INFO] 系統已關閉")

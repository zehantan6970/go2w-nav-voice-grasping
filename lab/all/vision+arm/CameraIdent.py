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

# 配置記憶體緩衝區
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
# 2. RealSense D435i 初始化設定
# ==========================================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# 若未來需要測距，可取消下方註解啟用深度流
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

print("[INFO] 啟動 RealSense D435i...")
pipeline.start(config)

# ==========================================
# 3. 影像前處理與後處理函式
# ==========================================
def preprocess_image(img):
    """將 BGR 影像轉換為 YOLO11 所需的 Tensor 格式"""
    image_resized = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
    image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
    image_normalized = image_rgb.astype(np.float32) / 255.0
    image_transposed = np.transpose(image_normalized, (2, 0, 1)) # HWC to CHW
    image_expanded = np.expand_dims(image_transposed, axis=0)    # 增加 Batch 維度
    return image_expanded, image_resized

def postprocess(output, orig_img):
    """動態解析 YOLO11 多類別輸出並繪製 Bounding Box"""
    img_h, img_w = orig_img.shape[:2]
    x_factor = img_w / INPUT_WIDTH
    y_factor = img_h / INPUT_HEIGHT
    
    # 動態計算每個錨點的屬性數量 (例如：67200 / 8400 = 8)
    num_attributes = len(output) // 8400
    
    # 將輸出重塑並轉置為 (8400, num_attributes)
    preds = np.reshape(output, (num_attributes, 8400)).T 
    
    boxes = []
    confidences = []
    class_ids = []
    
    # 💡 提示：請依據你當初訓練模型時 dataset.yaml 的順序修改下方的類別名稱
    class_names = ["water", "cola", "tea", "wanglaoji"] 
    
    for row in preds:
        # row[0:4] 是 cx, cy, w, h；row[4:] 是所有類別的信心分數
        classes_scores = row[4:]
        class_id = np.argmax(classes_scores)
        conf = classes_scores[class_id]
        
        if conf > CONF_THRESHOLD:
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            
            # 轉換為左上角座標並映射回原始影像尺寸
            left = int((cx - w / 2) * x_factor)
            top = int((cy - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            boxes.append([left, top, width, height])
            confidences.append(float(conf))
            class_ids.append(class_id)
            
    # 執行 NMS (非極大值抑制) 消除重複的框
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, IOU_THRESHOLD)
    
    # 確保有偵測到物件才進行繪製
    if len(indices) > 0:
        # 使用 flatten 確保不同 OpenCV 版本下的索引格式都能正確被讀取
        for i in np.array(indices).flatten():
            box = boxes[i]
            left, top, width, height = box[0], box[1], box[2], box[3]
            score = confidences[i]
            cl_id = class_ids[i]
            
            label_name = class_names[cl_id] if cl_id < len(class_names) else f"Obj_{cl_id}"
            
            # 繪製綠色邊界框與標籤
            cv2.rectangle(orig_img, (left, top), (left + width, top + height), (0, 255, 0), 2)
            cv2.putText(orig_img, f"{label_name}: {score:.2f}", (left, top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
    return orig_img

# ==========================================
# 4. 主迴圈：擷取、推論、顯示
# ==========================================
print("[INFO] 開始影像推論，按 'q' 鍵退出...")
try:
    while True:
        # 擷取 D435i 影像
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
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
        
        # 後處理與顯示
        trt_output = outputs[0]['host']
        result_image = postprocess(trt_output, color_image)
        
        cv2.imshow("D435i + YOLO11n TensorRT", result_image)
        
        # 按 'q' 鍵退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[INFO] 系統已關閉")

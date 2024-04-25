import cv2
import numpy as np
import onnxruntime
import time
import random
import serial
import yaml
from multiprocessing import Process, Queue
import pickle


class SerialNode:
    def __init__(self, real=True, PORT='/dev/ttyUSB0', BPS=115200):
        # self.data_buffer = []
        self.available = True
        self.current_time = time.time()
        print("Connecting...")
        if real:
            try:
                self.ser = serial.Serial(PORT, BPS, timeout=1)
                print("Connected. ")
            except Exception as e:
                print(e)
                self.available = False
                print("Fail to connect. ")
        else:
            self.available = False
    
    def _check(self):
        if self.available and self.ser.isOpen():
            print(f"Serial connected. a:{self.available} | o:{self.ser.isOpen()}")
            return True
        else:
            print(f"Port not available. a:{self.available}")
            return False

    def write_data(self, data="Hi I am Pi"):
        if self._check():
            if time.time() - self.current_time > 1: 
                self.ser.write(str(data).encode());    #writ a string to port
                print("Sent data:", data)
                self.current_time = time.time()
        else:
            print("Port not available, cannot write")
        
    def read_data(self):
        if self._check():
            print("Read data:", self.ser.readall())
        else:
            print("Port not available, cannot read")

class Camera():
    def __init__(self, video=0):
        self.cap = cv2.VideoCapture(video)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 200)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(5, 15)
        
        self.current_time = time.time()
        self.start = False
        # print('宽:', self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) )
        # print('高:', self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) )
        # print('帧率:', self.cap.get(cv2.CAP_PROP_FPS) )
        # print('亮度:', self.cap.get(cv2.CAP_PROP_BRIGHTNESS) )
        # print('对比度:', self.cap.get(cv2.CAP_PROP_CONTRAST) )
        # print('饱和度:', self.cap.get(cv2.CAP_PROP_SATURATION) )
        # print('色调:', self.cap.get(cv2.CAP_PROP_HUE) )
        # print('曝光度:', self.cap.get(cv2.CAP_PROP_EXPOSURE) )
        # exit()
 
    def get_frame(self):
        success, img = self.cap.read()
        img = cv2.flip(img, -1)
        return img
    
    def multi_get_frame(self, queue):
        while True:
            if queue.empty():
                if self.start:
                    _, serialized_data, bruh = queue.get()
                    if _ == 2:
                        # while time.time() - self.current_time < 1:
                        #     pass
                        # else:
                        #     print()
                        #     print("""
                        # #         afblkjedgvw;oidefihswe
                        # #         asflajhesbfcakefvae
                        # #         afciashkfbckaesjfbca
                        # #         fahfcajsbfe""")
                        #     print()
                        #     self.current_time = time.time()
                        # time.sleep(1)
                        serialized_frame = pickle.dumps(self.get_frame())
                        queue.put([0, serialized_frame, time.time() - self.current_time])
                        print("Camera put frame. ")
                    else:
                        queue.put([_, serialized_data, bruh])
                else:
                    serialized_frame = pickle.dumps(self.get_frame())
                    queue.put([0, serialized_frame, time.time() - self.current_time])
                    self.start = True
                    print("Camera put frame. ")
                    
                print("Camera time:", time.time() - self.current_time)
                self.current_time = time.time()

class Predictor():
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as file:
            params = yaml.safe_load(file)
        
        self.net = onnxruntime.InferenceSession(params['model_path'])
        self.model_h = params['model_h']
        self.model_w = params['model_w']
        self.stride = params['stride']
        self.dic_labels = list(params['dic_labels'].values())
        self.target_ids = params['target_ids']
        self.thred_nms = params['thred_nms']
        self.thred_cond = params['thred_cond']
        
        self.current_time = time.time()
        
        # # 读取图片
        # img = cv2.imread(r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\yolo_train\tennis_dataset2\val\file4.jpg")
        # bboxes = self.detection(img)
    
    # 数据预处理
    def _preprocess(self, src_img):
        size = [self.model_w, self.model_h]
        output = cv2.resize(src_img,(size[0], size[1]),interpolation=cv2.INTER_AREA)
        output = output.transpose(2,0,1)
        output = output.reshape((1, 3, size[1], size[0])) / 255

        return output.astype('float32')
    
    def multi_predict(self, queue):
        while True:
            if not queue.empty():
                _, serialized_frame, cumul_time = queue.get()
                if _ == 0:
                    shared_data = pickle.loads(serialized_frame)
                    detected, boxes, confs, ids, predict_time = self.infer_img(shared_data)
                    serialized_img = pickle.dumps([shared_data, detected, boxes, confs, ids, self.dic_labels, predict_time])
                    queue.put([1, serialized_img, time.time() - self.current_time + cumul_time])
                    print("Predictor put img. ")
                else:
                    queue.put([_, serialized_frame, cumul_time])
                    
                print("Predictor time:", time.time() - self.current_time)
                self.current_time = time.time()
        
    def infer_img(self, img):
        t1 = time.time()
        pred = []
        # 输入图像的原始宽高
        H, W, _ = img.shape
        # 数据预处理: resize, 1/255
        data = self._preprocess(img)
        # 模型推理
        input_name = self.net.get_inputs()[0].name
        feature_map = self.net.run([], {input_name: data})[0][0]
        
        # 输出特征图转置: CHW, HWC
        feature_map = feature_map.transpose(1, 2, 0)
        # 输出特征图的宽高
        feature_map_height = feature_map.shape[0]
        feature_map_width = feature_map.shape[1]
        # 特征图后处理
        for h in range(feature_map_height):
            for w in range(feature_map_width):
                data = feature_map[h][w]

                # 解析检测框置信度
                obj_score, cls_score = data[0], data[5:].max()
                score = (obj_score ** 0.6) * (cls_score ** 0.4)

                # 阈值筛选
                if score > self.thred_cond:
                    # 检测框类别
                    cls_index = np.argmax(data[5:])
                    if cls_index not in self.target_ids:
                        continue
                    # 检测框中心点偏移
                    x_offset, y_offset = tanh(data[1]), tanh(data[2])
                    # 检测框归一化后的宽高
                    box_width, box_height = sigmoid(data[3]), sigmoid(data[4])
                    # 检测框归一化后中心点
                    box_cx = (w + x_offset) / feature_map_width
                    box_cy = (h + y_offset) / feature_map_height
                    
                    # cx,cy,w,h => x1, y1, x2, y2
                    x1, y1 = box_cx - 0.5 * box_width, box_cy - 0.5 * box_height
                    x2, y2 = box_cx + 0.5 * box_width, box_cy + 0.5 * box_height
                    x1, y1, x2, y2 = int(x1 * W), int(y1 * H), int(x2 * W), int(y2 * H)

                    pred.append([x1, y1, x2, y2, score, cls_index])
                    
        if len(pred) == 0:
            return [0, 0, 0, 0, time.time()-t1]
        else:
            print(len([1] + self._nms(np.array(pred)) + [time.time()-t1]))
            return [1] + self._nms(np.array(pred)) + [time.time()-t1]
    
    # nms算法
    def _nms(self, dets):
        # dets:N*M,N是bbox的个数，M的前4位是对应的（x1,y1,x2,y2），第5位是对应的分数
        # #thresh:0.3,0.5....
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        scores = dets[:, 4]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # 求每个bbox的面积
        order = scores.argsort()[::-1]  # 对分数进行倒排序
        keep = []  # 用来保存最后留下来的bboxx下标

        while order.size > 0:
            i = order[0]  # 无条件保留每次迭代中置信度最高的bbox
            keep.append(i)

            # 计算置信度最高的bbox和其他剩下bbox之间的交叉区域
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h

            # 求交叉区域的面积占两者（置信度高的bbox和其他bbox）面积和的必烈
            ovr = inter / (areas[i] + areas[order[1:]] - inter)

            # 保留ovr小于thresh的bbox，进入下一次迭代。
            inds = np.where(ovr <= self.thred_nms)[0]

            # 因为ovr中的索引不包括order[0]所以要向后移动一位
            order = order[inds + 1]

        dets = np.array(dets)
        boxes = dets[keep, 0:4]
        confs = dets[keep, 4]
        clses = dets[keep, 5].astype(int)

        return [boxes, confs, clses]

# sigmoid函数
def sigmoid(x):
    return 1. / (1 + np.exp(-x))

# tanh函数
def tanh(x):
    return 2. / (1 + np.exp(-2 * x)) - 1

class PostProcessor():
    def __init__(self, display=True):
        self.display = display
        self.usb_serial = SerialNode(True)
        self.center_thres = 80
        self.area_range = 1000
        self.desired_area = 34000
        
        self.current_time = time.time()
        
        self.data_buffer = []
    
    def _can_catch(self, area):
        if self.desired_area - self.area_range < area < self.desired_area + self.area_range:
            return 1
        else:
            return 0
    
    def process(self, frame, detected, boxes, confs, ids, dic_labels, cumul_time, temp_data):
        t1 = time.time()
        if detected:
            for box, score, id in zip(boxes, confs, ids):
                
                print("Target detected:", dic_labels[id])
                c_coors = np.array([box[0]+box[2], box[1]+box[3]]) / 2
                print("Center coor:", c_coors)
                print("Box:", box)
                area = int(abs((box[0] - box[2]) * (box[1] - box[3])))
                print("Area:", area)
                
                data_to_be_write = ""
                data_to_be_write += '1'
                data_to_be_write += str(self._is_centered(c_coors[0], frame.shape[1]))
                print("Center?:", data_to_be_write[-1])
                data_to_be_write += str(self._can_catch(area))
                print("Can catch?:", data_to_be_write[-1])

                # while time.time() - t1 < 2:
                #     pass
                
                label = '%s:%.2f'%(dic_labels[id], score)
                if self.display:
                    self._plot_one_box(box.astype(np.int16), frame, color=(255,0,0), label=label, line_thickness=None)
                
                if id != 0:
                    self.data_buffer = []
                else:
                    self.data_buffer.append(id)
        else:
            print("No target. ")
            data_to_be_write = '000'
            self.data_buffer = []
            
        process_time = (time.time()-t1)
        print("Postprocess time:", process_time, "\n")
        
        str_FPS = "FPS: %.2f"%(1./(cumul_time + process_time))
        cv2.putText(frame, str_FPS, (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 3)
        if self.display:
            cv2.imshow("real-time vision", frame)
            cv2.resizeWindow('real-time vision', 640, 480)
            cv2.waitKey(1)
        
        if len(self.data_buffer) == 5:
            self.usb_serial.write_data(data_to_be_write)

    def multi_postprocess(self, queue):
        while True:
            if not queue.empty():
                _, serialized_data, cumul_time = queue.get()
                if _ == 1:
                    print("Postprocessor get data. ")
                    # shared_frame = pickle.loads(serialized_data)
                    frame, detected, boxes, confs, ids, dict_labels, predict_time = pickle.loads(serialized_data)
                    temp_data = '(A)'
                    self.process(frame, detected, boxes, confs, ids, dict_labels, cumul_time, temp_data)
                    queue.put([2, 'bruh', 0])
                    print("Predictor processed. ")
                else:
                    queue.put([_, serialized_data, cumul_time])
                print("PostProcessor time:", time.time() - self.current_time)
                self.current_time = time.time()
                    
    def _is_centered(self, x, w):
        # only determine x centering
        temp = self.center_thres / 2
        left_thres = w/2 - temp
        right_thres = w/2 + temp
        if 0 < x < left_thres:
            return 1
        elif left_thres < x < right_thres:
            return 2
        elif right_thres < x < w:
            return 3
        
    
    def _plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        """
        description: Plots one bounding box on image img,
                    this function comes from YoLov5 project.
        param: 
            x:      a box likes [x1,y1,x2,y2]
            img:    a opencv image object
            color:  color to draw rectangle, such as (0,255,0)
            label:  str
            line_thickness: int
        return:
            no return
        """
        tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        )  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(
                img,
                label,
                (c1[0], c1[1] - 2),
                0,
                tl / 3,
                [225, 255, 255],
                thickness=tf,
                lineType=cv2.LINE_AA,
            )



if __name__ == "__main__":
  
    queue = Queue()
    cap = Camera()
    predictor = Predictor(r"/home/3848c4/Desktop/3848/tennis_c3_fast.yaml")
    post_processor = PostProcessor(True)
    
    cam_process = Process(target=cap.multi_get_frame, args=(queue,))
    predict_process = Process(target=predictor.multi_predict, args=(queue,))
    post_processor_process = Process(target=post_processor.multi_postprocess, args=(queue,))
    
    cam_process.start()
    predict_process.start()
    post_processor_process.start()

    # temp_data = '(A)'
    while True:
        key=cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            post_processor.usb_serial.ser.close()
            break

    cap.cap.release()
    cv2.destroyAllWindows()
    

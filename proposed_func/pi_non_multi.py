import cv2
import numpy as np
import onnxruntime as ort
import time
import random
import serial
import yaml
import socket


# class WifiNode():
#     def __init__(self, real=True, IP="192.168.4.1", PORT=80, BPS=460800):
#         self.available = True
#         self.byte_len = len('000'.encode('utf-8'))
#         if real:
#             try:
#                 self.p = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 self.p.settimeout(10)
#                 self.p.connect((IP, int(PORT)))
#             except Exception as e:
#                 print(e)
#                 self.available = False
#         else:
#             self.available = False
    
#     def _check(self):
#         if self.available: # and self.ser.isOpen():
#             print(f"Serial successful. a:{self.available}|o:to-do")
#             return True
#         else:
#             print(f"Port not available. a:{self.available}|o:to-do")
#             return False

#     def write_data(self, data="Hello I am Ras PI", encoding='utf-8'):
#         if self._check():
#             self.p.send(data.encode(encoding))    #writ a string to port
#             print("Sent data:", data, "| len:", len(data))
#         else:
#             print("Port not available, cannot write")
        
#     def read_data(self, decoding='utf-8'):
#         if self._check():
#             response = self.p.recv(self.byte_len)
#             print("Received data:", response, "|len:", len(response))
#             response = response.decode(decoding)
#             print("Received data(decoded):", response, "|len:", len(response))
#             return response
#         else:
#             return "Port not available, cannot read"

class SerialNode:
    def __init__(self, real=True, PORT='/dev/ttyUSB0', BPS=115200):
        # self.ser = serial.Serial(PORT, BPS, timeout=3)
        self.available = True
        print("Connecting...")
        if real:
            try:
                self.ser = serial.Serial(PORT, BPS, timeout=1)
                time.sleep(0.5)
                print("Connected. ")
            except Exception as e:
                print(e)
                self.available = False
                print("Fail to connect. ")
        else:
            self.available = False
        
        # self.write_data()
    
    def _check(self):
        if self.available and self.ser.isOpen():
            print(f"Serial connected. a:{self.available} | o:{self.ser.isOpen()}")
            return True
        else:
            print(f"Port not available. a:{self.available} | o:{self.ser.isOpen()}")
            return False

    def write_data(self, data="Hi I am Pi"):
        if self._check():
            self.ser.write(str(data).encode());    #writ a string to port
            print("Sent data:", data)
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
    

class Predictor():
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as file:
            params = yaml.safe_load(file)
        
        self.net = ort.InferenceSession(params['model_path'], ort.SessionOptions())
        self.model_h = params['model_h']
        self.model_w = params['model_w']
        self.nl = params['nl']
        self.na = params['na']
        self.stride = params['stride']
        self.anchor_grid = np.asarray(params['anchors'], dtype=np.float32).reshape(self.nl, -1, 2)
        self.dic_labels = params['dic_labels']
        self.target_ids = params['target_ids']
        self.thred_nms = params['thred_nms']
        self.thred_cond = params['thred_cond']
    
    def _make_grid(self, nx, ny):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    
    def cal_outputs(self, outs):
        row_ind = 0
        grid = [np.zeros(1)] * self.nl
        for i in range(self.nl):
            h, w = int(self.model_w/ self.stride[i]), int(self.model_h / self.stride[i])
            length = int(self.na * h * w)
            if grid[i].shape[2:4] != (h, w):
                grid[i] = self._make_grid(w, h)
    
            outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
                grid[i], (self.na, 1))) * int(self.stride[i])
            outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
                self.anchor_grid[i], h * w, axis=0)
            row_ind += length
        return outs

    def post_process_opencv(self, outputs, img_h, img_w):
        conf = outputs[:,4].tolist()
        c_x = outputs[:,0] / self.model_w * img_w
        c_y = outputs[:,1] / self.model_h * img_h
        w  = outputs[:,2] / self.model_w * img_w
        h  = outputs[:,3] / self.model_h * img_h
        p_cls = outputs[:,5:]
        if len(p_cls.shape)==1:
            p_cls = np.expand_dims(p_cls,1)
        cls_id = np.argmax(p_cls,axis=1)
    
        p_x1 = np.expand_dims(c_x-w/2,-1)
        p_y1 = np.expand_dims(c_y-h/2,-1)
        p_x2 = np.expand_dims(c_x+w/2,-1)
        p_y2 = np.expand_dims(c_y+h/2,-1)
        areas = np.concatenate((p_x1,p_y1,p_x2,p_y2),axis=-1)
        
        areas = areas.tolist()
        ids = cv2.dnn.NMSBoxes(areas, conf, self.thred_cond, self.thred_nms)
        if len(ids)>0:
            if len(ids.shape) == 1: ids = ids.reshape((-1, ids.shape[0]))
            valid_ids = ids[np.isin(cls_id[ids], self.target_ids)]
            return  np.array(areas)[valid_ids], np.array(conf)[valid_ids], cls_id[valid_ids]
        else:
            return [], [], []

    def infer_img(self, frame):
        t1 = time.time()
        
        # 图像预处理
        img = cv2.resize(frame, [self.model_w, self.model_h], interpolation=cv2.INTER_AREA)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)
    
        # 模型推理
        outs = self.net.run(None, {self.net.get_inputs()[0].name: blob})[0].squeeze(axis=0)
        outs = self.cal_outputs(outs)
        img_h, img_w, _ = np.shape(frame)
        # print(img_h, img_w)
        boxes, confs, ids = self.post_process_opencv(outs, img_h, img_w)
        
        detected = len(ids)
        
        predict_time = time.time() - t1
        print("Predict time:", predict_time)
        
        return detected, boxes, confs, ids, predict_time

class PostProcessor():
    def __init__(self, display=True):
        self.display = display
        self.usb_serial = SerialNode()
        # self.wifi_server = WifiNode("192.168.50.21")
        self.center_thres = 80
        self.area_range = 1000
        self.desired_area = 34000
    
    def _can_catch(self, area):
        if self.desired_area - self.area_range < area < self.desired_area + self.area_range:
            return 1
        else:
            return 0
    
    def process(self, frame, detected, boxes, confs, ids, dic_labels, predict_time):
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
        else:
            print("No target. ")
            data_to_be_write = '000'
        
        
        process_time = (time.time()-t1)
        print("Postprocess time:", process_time, "\n")
        
        str_FPS = "FPS: %.2f"%(1./(predict_time + process_time))
        cv2.putText(frame, str_FPS, (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 3)
        if self.display:
            cv2.imshow("real-time vision", frame)
            cv2.resizeWindow('real-time vision', 640, 480)
        
        # ta = time.time()
        # self.usb_serial.ser.close()
        # self.usb_serial = SerialNode()
        self.usb_serial.write_data(data_to_be_write)
        # self.usb_serial.read_data()
        # tb = time.time()
        
        # while time.time() - ta < 0.5:
        #     pass
        
        # print("BRUHHHH:", time.time() - ta)
        # self.usb_serial.read_data()

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
        # return 
    
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
    cap = Camera()
    predictor = Predictor(r"/home/3848c4/Desktop/3848/tennis.yaml")
    post_processor = PostProcessor(False)

    # temp_data = '(A)'
    while True:
        frame = cap.get_frame()
        detected, boxes, confs, ids, predict_time = predictor.infer_img(frame)
        dic_labels = predictor.dic_labels
        post_processor.process(frame, detected, boxes, confs, ids, dic_labels, predict_time) #, temp_data)
        
        key=cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            post_processor.usb_serial.ser.close()
            break
        # elif key == ord('t'):
        #     if temp_data == '(A)':
        #         temp_data = '(Z)'
        #     else:
        #         temp_data = '(A)'
        #     time.sleep(2)
        # elif key & 0xFF == ord('s'):
        #     flag_det = not flag_det
        #     print(flag_det)
        # elif key == ord("c"):
        #     i += 1
        #     file_name = a[i]

    cap.cap.release()
    cv2.destroyAllWindows()
    

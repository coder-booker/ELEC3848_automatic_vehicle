import cv2
import numpy as np
import onnxruntime as ort
import time
import serial


class SerialNode:
    def __init__(self, PORT='/dev/ttyUSB0', BPS=115200):
        self.ser = serial.Serial(PORT, BPS, timeout=3)
        self.available = False
        
        if not self.ser.isOpen():
            print("Port not available")
        else:
            self.available = True
            print("successful")


    def write_data(self, data="Hello I am Ras PI", encoding='utf-8'):
        if self.available:
            self.ser.write(str(data).encode(encoding));    #writ a string to port
            print("write")
        else:
            print("Port not available, cannot write")
        
    def read_data(self):
        if self.available:
            return self.ser.readall()
        else:
            print("Port not available, cannot read")
    
    
        
 
def plot_one_box(x, img, color=None, label=None, line_thickness=None):
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
 
def _make_grid( nx, ny):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)
 
def cal_outputs(outs,nl,na,model_w,model_h,anchor_grid,stride):
    
    row_ind = 0
    grid = [np.zeros(1)] * nl
    for i in range(nl):
        h, w = int(model_w/ stride[i]), int(model_h / stride[i])
        length = int(na * h * w)
        if grid[i].shape[2:4] != (h, w):
            grid[i] = _make_grid(w, h)
 
        outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
            grid[i], (na, 1))) * int(stride[i])
        outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
            anchor_grid[i], h * w, axis=0)
        row_ind += length
    return outs
 
def post_process_opencv(outputs,model_h,model_w,img_h,img_w,thred_nms,thred_cond):
    
    p_cls = outputs[:,5:]
    if len(p_cls.shape)==1:
        p_cls = np.expand_dims(p_cls,1)
    cls_id = np.argmax(p_cls,axis=1)
    
    conf = outputs[:,4].tolist()
    c_x = outputs[:,0]/model_w*img_w
    c_y = outputs[:,1]/model_h*img_h
    w = outputs[:,2]/model_w*img_w
    h = outputs[:,3]/model_h*img_h
 
    p_x1 = np.expand_dims(c_x-w/2,-1)
    p_y1 = np.expand_dims(c_y-h/2,-1)
    p_x2 = np.expand_dims(c_x+w/2,-1)
    p_y2 = np.expand_dims(c_y+h/2,-1)
    areas = np.concatenate((p_x1,p_y1,p_x2,p_y2),axis=-1)
    
    areas = areas.tolist()
    ids = cv2.dnn.NMSBoxes(areas,conf,thred_cond,thred_nms)
    
    
    if len(ids)>0:
        if len(ids.shape) == 1: ids = ids.reshape((-1, ids.shape[0]))
        tennis_idx = ids[cls_id[ids] == 1]
        return  np.array(areas)[tennis_idx], np.array(conf)[tennis_idx], cls_id[tennis_idx]
    else:
        return [], [], []
    
def infer_img(img0,net,model_h,model_w,nl,na,stride,anchor_grid,thred_nms=0.4,thred_cond=0.5):
    # 图像预处理
    img = cv2.resize(img0, [model_w,model_h], interpolation=cv2.INTER_AREA)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)
 
    # 模型推理
    outs = net.run(None, {net.get_inputs()[0].name: blob})[0].squeeze(axis=0)
    outs = cal_outputs(outs,nl,na,model_w,model_h,anchor_grid,stride)
    img_h,img_w,_ = np.shape(img0)
    print(img_h, img_w)
    boxes, confs, ids = post_process_opencv(outs,model_h,model_w,img_h,img_w,thred_nms,thred_cond)
    
    detected = len(ids)
    
    return detected, boxes, confs, ids

def get_quadrant(target_coors, w, h):
    x, y = target_coors
    boundary = [w/2, h/2]
    if 0 < x < boundary[0] and 0 < y < boundary[1]:
        return 1
    elif boundary[0] < x < w and 0 < y < boundary[1]:
        return 2
    elif 0 < x < boundary[0] and boundary[1] < y < h:
        return 3
    elif boundary[0] < x < w and boundary[1] < y < h:
        return 4
    else:
        return -1

def is_centered(x, w, h, thres):
    # only determine x centering
    
    temp = thres / 2
    return w/2 - temp < x < w/2 + temp
    
    # boundary = [
    #     [w/2 - 20, h/2 - 20], 
    #     [w/2 + 20, h/2 + 20]
    # ] # top-left and bottom-right

# def print_data(data):
#     global usb_serial
#     print(data)
#     usb_serial.write(data)
    


if __name__ == "__main__":
 
    model_pb_path = r"./tennis_c3.onnx"
    so = ort.SessionOptions()
    net = ort.InferenceSession(model_pb_path, so)
    
    # 标签字典
    dic_labels= {0:'Racket', 1:'Tennis ball', 2:'Person'}
    
    # 模型参数
    model_h = 320
    model_w = 320
    nl = 3
    na = 3
    stride=[8.,16.,32.]
    anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
    anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)
    
    # video = 0
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    flag_det = True
    
    usb_serial = SerialNode()
    print("bruh")
    
    while True:
        success, img0 = cap.read()        
        img0 = cv2.flip(img0, -1)
        # print(img0.shape)
        # img0 = cv2.imread(r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\test.jpg")
        if flag_det:
            t1 = time.time()
            detected, det_boxes, scores, ids = infer_img(img0,net,model_h,model_w,nl,na,stride,anchor_grid,thred_nms=0.4,thred_cond=0.5)
            t2 = time.time()
            
            # print(len(det_boxes))
            if detected:
                for box,score,id in zip(det_boxes,scores,ids):
                    c_coors = np.array([box[0]+box[2], box[1]+box[3]]) / 2
                    
                    print("Target detected:", dic_labels[id])
                    print("Center coor:", c_coors)
                    print("Quadrant:", get_quadrant(c_coors, 640, 480))
                    
                    data_to_be_write = []
                    data_to_be_write.append(str(dic_labels[id]))
                    # data_to_be_write.append(f"{c_coors[0]},{c_coors[1]}")
                    data_to_be_write.append(str(get_quadrant(c_coors, 640, 480)))
                    if is_centered(c_coors[0], 640, 480, 80):
                        print("Centered!")
                        data_to_be_write.append("1")
                    else:
                        data_to_be_write.append("0")
                    
                    while time.time() - t1 < 0.4:
                        pass
                    usb_serial.write_data('-'.join(data_to_be_write))
                    
                    label = '%s:%.2f'%(dic_labels[id],score)
            
                    plot_one_box(box.astype(np.int16), img0, color=(255,0,0), label=label, line_thickness=None)
            else:
                print("No target. ")
                
            str_FPS = "FPS: %.2f"%(1./(t2-t1))
            cv2.putText(img0, str_FPS, (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 3)
            
            print("Predict time:", t2 - t1)
            print("Postprocess time:", time.time() - t2, "\n")
            
            cv2.imshow("video", img0)
            # cv2.resizeWindow("video", 800, 800)
        
        data_being_read = usb_serial.read_data()
        
        key=cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("Quit. ")
            break

    cap.release() 
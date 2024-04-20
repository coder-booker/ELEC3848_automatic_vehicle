import cv2
import numpy as np
import onnxruntime as ort
import time
import random
 
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
 
def _make_grid(nx, ny):
    xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
    return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)
 
 
def post_process_opencv(outputs,model_h,model_w,img_h,img_w,thred_nms,thred_cond):
    conf = outputs[:,4].tolist()
    c_x = outputs[:,0]/model_w*img_w
    c_y = outputs[:,1]/model_h*img_h
    w  = outputs[:,2]/model_w*img_w
    h  = outputs[:,3]/model_h*img_h
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
    ids = cv2.dnn.NMSBoxes(areas,conf,thred_cond,thred_nms)
    if len(ids)>0:
        return  np.array(areas)[ids],np.array(conf)[ids],cls_id[ids]
    else:
        return [],[],[]


def cal_outputs(outs,nl,na,model_w,model_h,anchor_grid,stride):
    
    row_ind = 0
    grid = [np.zeros(1)] * nl
    for i in range(nl):
        h, w = int(model_w/ stride[i]), int(model_h / stride[i])
        length = int(na * h * w)
        if grid[i].shape[2:4] != (h, w):
            grid[i] = _make_grid(w, h)

        # print(outs[row_ind:row_ind + length, 0:2].shape)
        # print(((np.tile(grid[i], (na, 1))) * int(stride[i])).shape)
        outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
            grid[i], (na, 1))) * int(stride[i])
        # grid_i = np.reshape(grid[i], (1, h, w, 2))
        # grid_i = np.tile(grid_i, (na, 1, 1, 1))
        # print(outs[row_ind:row_ind + length, 0:2].shape)
        # print(grid_i.shape)
        # outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + grid_i) * int(stride[i])
        
        outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
            anchor_grid[i], h * w, axis=0)
        row_ind += length
    return outs



# # def process_output(outputs, img_w, img_h, strides, confThreshold, nmsThreshold):
# #     # Convert coordinates
# #     outputs = convert_coordinates(outputs, strides, img_w=img_w, img_h=img_h)

# #     # Flatten the outputs
# #     outputs = np.concatenate([np.reshape(output, (-1, 6)) for output in outputs], axis=0)

# #     # Get the boxes, scores, and classes
# #     boxes = outputs[:, :4]
# #     scores = outputs[:, 4]
# #     classes = np.argmax(outputs[:, 5:], axis=-1)

# #     # Perform Non-Maximum Suppression
# #     indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), confThreshold, nmsThreshold)

# #     # Get the final boxes, scores, and classes
# #     final_boxes = [boxes[i[0]] for i in indices]
# #     final_scores = [scores[i[0]] for i in indices]
# #     final_classes = [classes[i[0]] for i in indices]

# #     return final_boxes, final_scores, final_classes

# def process_output(outputs, img_w, img_h, strides, confThreshold, nmsThreshold):
#     # Check if there are any detections
#     # if np.all(outputs[..., 4] < confThreshold):
#     #     return [], [], []
    
#     # Convert coordinates
#     outputs = convert_coordinates(outputs, strides, img_w=img_w, img_h=img_h)

#     # Flatten the outputs
#     outputs = np.concatenate([np.reshape(output, (-1, 6)) for output in outputs], axis=0)
#     print(outputs.shape)
    
#     # Get the boxes, scores, and classes
#     boxes = outputs[:, :4]
#     classes = np.argmax(outputs[:, 5:], axis=-1)
#     scores = outputs[:, 4] * classes[:]

#     # Perform Non-Maximum Suppression
#     indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), confThreshold, nmsThreshold)

#     # Get the final boxes, scores, and classes
#     indices = np.array(indices).flatten()
#     final_boxes = boxes[indices]
#     final_scores = scores[indices]
#     final_classes = classes[indices]

#     return final_boxes, final_scores, final_classes

# def convert_coordinates(outs, strides, img_w, img_h):
#     # for i in range(len(outs)):
#     #     stride = strides[i]
#     #     grid_y, grid_x = np.mgrid[:outs[i].shape[0], :outs[i].shape[1]]
#     #     outs[i][..., 0] = (grid_x + outs[i][..., 0]) * stride
#     #     outs[i][..., 1] = (grid_y + outs[i][..., 1]) * stride
#     #     outs[i][..., 2] *= img_w
#     #     outs[i][..., 3] *= img_h
#     i = 0
#     stride = strides[i]
#     out = outs[i]
#     grid_y, grid_x = np.mgrid[:outs[i].shape[0], :outs[i].shape[1]]
#     print(outs[i][0:2, 0:2, 0], "\n->\n", (grid_x[0:2, 0:2] + outs[i][0:2, 0:2, 0]) * stride)
#     print(outs[i][0:2, 0:2, 1], "\n->\n", (grid_y[0:2, 0:2] + outs[i][0:2, 0:2, 1]) * stride)
#     print(outs[i][0:2, 0:2, 2], "\n->\n", outs[i][0:2, 0:2, 2] * int(img_w/stride))
#     print(outs[i][0:2, 0:2, 3], "\n->\n", outs[i][0:2, 0:2, 3] * int(img_h/stride))
    
#     print(outs[i][0:2, 0:2, :])
#     out = outs[i]
#     img = np.zeros((480, 640), dtype=np.uint8)
#     for i in range(out.shape[0]):
#         for j in range(out.shape[1]):
#             # img[i, j] = int(out[i, j, 0] * 255)
#             cv2.rectangle(img, (i*8, j*8), (i*8+8, j*8+8), (int((11+out[i, j, 4]) * out[i, j, 5] * 25.5)), -1)
#             # print((11+out[i, j, 4]))
#     cv2.imshow('Image', img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     # outs[i][..., 0] = (grid_x + outs[i][..., 0]) * stride
#     # outs[i][..., 1] = (grid_y + outs[i][..., 1]) * stride
#     # outs[i][..., 2] *= img_w
#     # outs[i][..., 3] *= img_h
#     exit()
#     return outs

def infer_img(img0,net,model_h,model_w,nl,na,stride,anchor_grid,thred_nms=0.4,thred_cond=0.5):
    # 图像预处理
    img = cv2.resize(img0, [model_w, model_h], interpolation=cv2.INTER_AREA)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)
 
    # 模型推理
    outs = net.run(None, {net.get_inputs()[0].name: blob})[0].squeeze(axis=0)
    print(outs.shape)
    # 输出坐标矫正
    outs = cal_outputs(outs,nl,na,model_w,model_h,anchor_grid,stride)
    # outs = convert_coordinates(outs, stride, model_w, model_h)
    print(outs.shape)
    # 检测框计算
    # img_h,img_w,_ = np.shape(img0)
    boxes,confs,ids = post_process_opencv(outs,model_h,model_w,img_h,img_w,thred_nms,thred_cond)
    # boxes, confs, ids = process_output(outs, model_h, model_w, stride, thred_nms, thred_cond)
 
    return  boxes,confs,ids
 
 
 
 
if __name__ == "__main__":
 
    # 模型加载
    model_pb_path = r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\tennis_c3.onnx"
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
    
    video = 0
    # cap = cv2.VideoCapture(video)
    flag_det = True
    
    # while cap.isOpened():
    while 1:
        # success, img0 = cap.read()
        img0 = cv2.imread(r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\test.jpg")
        # print(len(img0))
        # print(len(img0[0]))
        cv2.imshow("bruh", img0)
        # time.sleep(3)
        cv2.waitKey(0)
        # exit()
        
        t1 = time.time()
        if flag_det:
            det_boxes,scores,ids = infer_img(img0,net,model_h,model_w,nl,na,stride,anchor_grid,thred_nms=0.4,thred_cond=0.5)
            
            if len(det_boxes):
                
                print("detected")
                for box,score,id in zip(det_boxes,scores,ids):
                    label = '%s:%.2f'%(dic_labels[id],score)
            
                    plot_one_box(box.astype(np.int16), img0, color=(255,0,0), label=label, line_thickness=None)
            else:
                print("no target")
            
            time.sleep(1)
            
        
        t2 = time.time()
        str_FPS = "FPS: %.2f"%(1./(t2-t1))
        cv2.putText(img0,str_FPS,(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),3)
        
        cv2.imshow("video",img0)
        cv2.imwrite(r"D:\HKU\Year3\sem2\ELEC3848\project\ino_github\ELEC3848_automatic_vehicle\proposed_func\output.jpg", img0)
        
        key=cv2.waitKey(1) & 0xFF    
        if key == ord('q'):
        
            break
        elif key & 0xFF == ord('s'):
            flag_det = not flag_det
            print(flag_det)
            
    # cap.release() 
    cv2.destroyAllWindows()
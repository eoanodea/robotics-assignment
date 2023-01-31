import cv2
import torch
from tracker import *
import numpy as np
path = r"E:\yolov5\yolov5"
model = torch.hub.load(path, 'yolov5n', pretrained=True, source='local')
model.classes = 0;

cap=cv2.VideoCapture(0)


def POINTS(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE :  
        colorsBGR = [x, y]
        print(colorsBGR)
        

cv2.namedWindow('FRAME')
cv2.setMouseCallback('FRAME', POINTS)

tracker = Tracker()

area_1a=[(0,0),(200,0),(200,640),(0,640)]
area_1b=[(200,0),(400,0),(400,640),(200,640)]
area_2a=[(760,0),(960,0),(960,640),(760,640)]
area_2b=[(560,0),(760,0),(760,640),(560,640)]
while True:
    ret,frame=cap.read()
    frame=cv2.resize(frame,(960,640))
    #cv2.polylines(frame,[np.array(area_1a,np.int32)],True,(0,255,0),3)
    #cv2.polylines(frame,[np.array(area_2a, np.int32)], True, (0, 255, 0), 3)
    #cv2.polylines(frame,[np.array(area_1b,np.int32)],True,(0,0,255),3)
    #cv2.polylines(frame,[np.array(area_2b, np.int32)], True, (0, 0, 255), 3)
    results=model(frame)
    #frame=np.squeeze(results.render())
    list=[]
    for index,row in results.pandas().xyxy[0].iterrows():
        x1=int(row['xmin'])
        y1=int(row['ymin'])
        x2=int(row['xmax'])
        y2=int(row['ymax'])
        b=str(row['name'])
        if 'person' in b:
            list.append([x1, y1, x2, y2])
        xcen=int((x1+x2)/2)
        ycen=int((y1+y2)/2)
        cv2.putText(frame,'+',(xcen,ycen),cv2.FONT_HERSHEY_DUPLEX,0.5,(186,226,65),1)

    boxes_ids=tracker.update(list)
    ids=[]
    for box_id in boxes_ids:
        x, y, w, h, id = box_id
        ids.append(id)
    if ids:
        minId=min(ids)
    for box_id in boxes_ids:
        x,y,w,h,id=box_id
        cv2.rectangle(frame, (x, y), (w, h), (51, 231, 247), 3)
        cv2.putText(frame, str(id), (x, y + 30), cv2.FONT_HERSHEY_DUPLEX, 1.2, (249, 247, 247), 2)
        if id==minId:
            print(id)
            xTrack = int((x + w) / 2)
            yTrack = int((y + h) / 2)
            result2a=cv2.pointPolygonTest(np.array(area_2a, np.int32), (int(xTrack), int(yTrack)), False)
            result1a=cv2.pointPolygonTest(np.array(area_1a,np.int32),(int(xTrack),int(yTrack)),False)
            result2b=cv2.pointPolygonTest(np.array(area_2b, np.int32), (int(xTrack), int(yTrack)), False)
            result1b=cv2.pointPolygonTest(np.array(area_1b,np.int32),(int(xTrack),int(yTrack)),False)
            cv2.putText(frame, 'o', (xTrack, yTrack), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)
            if result1a==1:
                result='BIG LEFT'
            elif result2a==1:
                result='BIG RIGHT'
            elif result1b==1:
                result='left'
            elif result2b==1:
                result='right'
            else:
                result='center'
            print(result)
    cv2.imshow('FRAME',frame)
    if cv2.waitKey(1)&0xFF==27:
        break
cap.release()
cv2.destroyAllWindows()
    
    

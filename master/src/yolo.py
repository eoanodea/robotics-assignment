import cv2
import base64
import torch
from tracker import Tracker
import numpy as np

path = r"/usr/src/app/yolov5"

class ImageTracker:

    def __init__(self):
        self._model = torch.hub.load(path, 'yolov5n', pretrained=True, source='local')
        self._model.classes = 0

        self._area_1a = [(0, 0), (90, 0), (90, 480), (0, 480)]
        self._area_1b = [(90, 0), (230, 0), (230, 480), (90, 480)]
        self._area_2a = [(550, 0), (640, 0), (640, 480), (550, 480)]
        self._area_2b = [(420, 0), (550, 0), (550, 480), (420, 480)]

        self._tracker = Tracker()

    def on_image(self, base64_image):
        # Remove the data URI scheme from the base64 data
        #img_data = base64_image.split(",")[1]
        # Decode the base64 image data and convert to a numpy array
        img_bytes = base64.b64decode(base64_image)
        img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
        #img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        img = img_arr.reshape(480, 640, 3)

        frame = img

        # Process the frame here...
        results = self._model(frame)

        person_list = []
        for index, row in results.pandas().xyxy[0].iterrows():
            x1 = int(row['xmin'])
            y1 = int(row['ymin'])
            x2 = int(row['xmax'])
            y2 = int(row['ymax'])
            b = str(row['name'])
            if 'person' in b:
                person_list.append([x1, y1, x2, y2])
            xcen = int((x1 + x2) / 2)
            ycen = int((y1 + y2) / 2)
            cv2.putText(frame, '+', (xcen, ycen), cv2.FONT_HERSHEY_DUPLEX, 0.5, (186, 226, 65), 1)

        boxes_ids = self._tracker.update(person_list)
        ids = []
        for box_id in boxes_ids:
            x, y, w, h, new_id = box_id
            ids.append(new_id)
        if ids:
            sort_id = sorted(ids)
        for box_id in boxes_ids:
            x, y, w, h, new_id = box_id
            cv2.rectangle(frame, (x, y), (w, h), (51, 231, 247), 3)
            if new_id == sort_id[0]:
                x_track = int((x + w) / 2)
                y_track = int((y + h) / 2)
                cv2.putText(frame, str('FOLLOWING'), (x, y + 30), cv2.FONT_HERSHEY_DUPLEX, 1.2, (64, 214, 119), 2)
                result2a = cv2.pointPolygonTest(np.array(self._area_2a, np.int32), (int(x_track), int(y_track)), False)
                result1a = cv2.pointPolygonTest(np.array(self._area_1a, np.int32), (int(x_track), int(y_track)), False)
                result2b = cv2.pointPolygonTest(np.array(self._area_2b, np.int32), (int(x_track), int(y_track)), False)
                result1b = cv2.pointPolygonTest(np.array(self._area_1b, np.int32), (int(x_track), int(y_track)), False)

                # result: 0 - center,
                # 1 - HARD LEFT,
                # 2 - HARD RIGHT,
                # 3 - turn left,
                # 4 - turn right
                if result1a == 1:
                    result = 'hard left'
                elif result2a == 1:
                    result = 'hard right'
                elif result1b == 1:
                    result = 'left'
                elif result2b == 1:
                    result = 'right'
                else:
                    result = 'center'

                return result
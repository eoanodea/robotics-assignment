import numpy as np
from PIL import Image
import cv2

pil_image = Image.open('realhuman.jpg').convert('RGB')
open_cv_image = np.array(pil_image)
cap = cv2.VideoCapture('realhuman.jpg')
ret, frame = cap.read()
ret, jpeg = cv2.imencode('.jpg', frame)
img = cv2.imdecode(jpeg, cv2.IMREAD_UNCHANGED)
# Convert RGB to BGR
open_cv_image = open_cv_image[:, :, ::-1].copy()
print(open_cv_image)
cv2.namedWindow('image')
cv2.imshow('image',open_cv_image)
if cv2.waitKey(0) & 0xFF == 27:
    cv2.destroyAllWindows()
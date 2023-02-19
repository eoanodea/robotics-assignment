import numpy as np
from PIL import Image
import cv2

# Define a callback function to process depth images
    # Convert the depth image message to a NumPy array
f = open('depth_image_raw.txt', 'r')
image_string = f.read()
image_string=image_string.replace(',',' ')


# Convert the string to a list of ints
image_list = [int(x) for x in image_string.split()]

# Convert the list of floats to a NumPy array
image_array = np.array(image_list)
#img = cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED)

image_array = image_array[0:480*640]
image_array = image_array.reshape(480, 640)
# Reshape the NumPy array to the desired shape (in this example, a 2x3 array)

# Print the resulting array
cv2.namedWindow('FRAME')
cv2.imshow('FRAME', image_array)

if cv2.waitKey(0) & 0xFF == 27:
    cv2.destroyAllWindows()

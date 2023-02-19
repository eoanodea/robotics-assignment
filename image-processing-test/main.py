import base64
import yaml


with open('../data-samples/depth_image_raw.txt', 'r') as file:
    print('fetching file as yaml')
    for doc in yaml.safe_load_all(file):
        if doc is not None and 'data' in doc:
            print("Converting file to base64")
            data_bytes = bytes(doc['data'])
            base64Image = base64.b64encode(data_bytes)
            print(base64Image)
    

import yaml

with open('../data-samples/depth_image_raw.txt', 'r') as file:
    for doc in yaml.safe_load_all(file):
        print(doc['width'])
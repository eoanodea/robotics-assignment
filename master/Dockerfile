FROM python:3

WORKDIR /usr/src/app

RUN python -m pip install --upgrade pip

RUN apt-get update \ 
  && apt-get install -y iputils-ping ffmpeg libsm6 libxext6 git

RUN git clone https://github.com/ultralytics/yolov5 /usr/src/app/yolov5

RUN pip install -r yolov5/requirements.txt 

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN mkdir -p /etc/redis/
RUN cp /usr/src/app/redis.conf /etc/redis/redis.conf

CMD [ "python", "-u", "src/main.py" ]

# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]
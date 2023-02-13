FROM python:3

WORKDIR /usr/src/app

RUN python -m pip install --upgrade pip

# RUN apt install -y redis-server

RUN apt-get update \ 
  && apt-get install -y iputils-ping

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN mkdir -p /etc/redis/
RUN cp /usr/src/app/redis.conf /etc/redis/redis.conf

CMD [ "python", "-u", "src/main.py" ]

# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]
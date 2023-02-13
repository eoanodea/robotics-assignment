FROM python:3

WORKDIR /usr/src/app

RUN python -m pip install --upgrade pip

# RUN apt update
# RUN apt install -y redis-server

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN mkdir -p /etc/redis/
RUN cp /usr/src/app/redis.conf /etc/redis/redis.conf

CMD [ "python", "-u", "src/main.py" ]

# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]
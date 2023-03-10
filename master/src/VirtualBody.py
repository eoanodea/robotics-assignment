import redis
import time
import logging
import json

from yolo import ImageTracker


class VirtualBody:

    def __init__(self, perception_channel, commands_channel, redis_host, redis_port):
        print("Initializing")
        self._my_perc_ch = perception_channel
        self._my_comm_ch = commands_channel
        # self._my_msg_broker = redis.from_url(redis_host, decode_responses=True)
        self._my_msg_broker = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        self._my_perceptions = None
        self._image_tracker = ImageTracker()
        try:
            # print(self._my_msg_broker.info())
            self._pub_sub = self._my_msg_broker.pubsub()
            self._pub_sub.subscribe(self._my_perc_ch)
            msg = None
            while not msg:
                msg = self._pub_sub.get_message()
                time.sleep(0.1)
                print("Still waiting..")
            print("subscribed: ", msg, "to", self._my_perc_ch)
        except Exception as e:
            logging.exception(e)
            print(e)
            raise e

    def get_perceptions(self):
        while True:
            msg = self._pub_sub.get_message()
            if msg:
                self._my_perceptions = json.loads(msg['data'])
                if self._my_perceptions['type'] == 'image':                    
                    image = self._my_msg_broker.hget(self._my_perceptions['hashset'], self._my_perceptions['ID'])
                    if image is not None:
                        parsed_image = json.loads(image)
                        self._my_msg_broker.hdel(self._my_perceptions['hashset'], self._my_perceptions['ID'])
                        return parsed_image
            time.sleep(0.1)

    def plan(self, perceptions):
        # Some logic based on the perception
        start_time = time.time()
        cmd = self._image_tracker.on_image(perceptions['data'])
        
        end_time = time.time()
        elapsed_time = round(end_time - start_time, 3)

        print(cmd, "Elapsed time:", elapsed_time, "seconds")
        if not cmd:
            return 'center'
        return cmd

    def send_command(self, command):
        # send the command to redis
        self._my_msg_broker.publish(self._my_comm_ch, command)

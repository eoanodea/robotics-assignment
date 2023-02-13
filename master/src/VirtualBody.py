import redis
import time
import logging
import json

class VirtualBody():

    def __init__(self, perception_channel, commands_channel):
        print("Initilizing")
        self._my_perc_ch = perception_channel
        self._my_comm_ch = commands_channel
        self._my_msg_broker = redis.Redis(host='192.168.0.107', port=6379, decode_responses=True)
        self._my_perceptions = None
        try:
            print("Message Broker info")
            print(self._my_msg_broker.info())
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
                return self._my_perceptions
            time.sleep(0.1)

    def plan(self, perceptions):

        
        return 'forward'
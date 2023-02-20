import time
import os


from VirtualBody import VirtualBody

# MY_PERC_CH = "/mobile_base/events/bumper"  # from the physical body to the virtual body and controller
MY_PERC_CH = "/debug/percept"  # from the physical body to the virtual body and controller
MY_COMM_CH = "/mobile_base/commands/velocity"     # from virtual body to physical


REDIS_HOST = os.environ['REDIS_URL']
REDIS_PORT = os.environ['REDIS_PORT']

if __name__ == "__main__":
    print("Starting main")
    my_virt_body = VirtualBody(MY_PERC_CH, MY_COMM_CH, REDIS_HOST, int(REDIS_PORT))
    while True:
        percepts = my_virt_body.get_perceptions()
        print("perceptions: ", percepts)
        command = my_virt_body.plan(percepts)
        print("command: ", command)
        my_virt_body.send_command(command)
        time.sleep(0.1)
import time

from VirtualBody import VirtualBody

MY_PERC_CH = "/mobile_base/events/bumper"  # from the physical body to the virtual body and controller
MY_COMM_CH = "COMMANDS"     # from virtual body to physical

if __name__ == "__main__":
    print("Starting main")
    my_virt_body = VirtualBody(MY_PERC_CH, MY_COMM_CH)
    while True:
        percepts = my_virt_body.get_perceptions()
        print("perceptions: ", percepts)
        command = my_virt_body.plan(percepts)
        # print(command)
        my_virt_body.send_command(command)
        time.sleep(0.1)
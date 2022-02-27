import numpy as np
import time
import json
import paho.mqtt.client as mqtt
import pandas as pd
import sys

TOPIC = 'topic/recv_data'


# initialize for the topic on which the msg is received
recv_topic = None
# initialize a recv msg to store the payload received from MQTT
recv_msg = None


def main_cb(client, userdata, msg):
    # use `global` to change a variable outside of the callback function
    global recv_topic, recv_msg

    recv_topic = msg.topic
    print('In callback, topic:', recv_topic,"\n")

    # NOTE: the payload must be in JSON format
    recv_msg = json.loads(msg.payload)
    print('In callback, msg:', recv_msg,"\n")


host = "localhost"
client = mqtt.Client("DobotCR5")
client.connect(host, port=1883)
client.loop_start()

client.subscribe(TOPIC)

client.on_message = main_cb


while True:
    time.sleep(1)
    if recv_msg:
        # access the Dictionary from the parsed JSON          
        coord1 = recv_msg[0]['coord1']
        coord2 = recv_msg[0]['coord2']
        coord3 = recv_msg[0]['coord3']
        coord4 = recv_msg[0]['coord4']
        coord5 = recv_msg[0]['coord5']
        coord6 = recv_msg[0]['coord6']
        labels = recv_msg[0]['labels']
        view = recv_msg[0]['view']
        # print(f"{coord1 = }")
        # print(f"{labels = }")
        # print(f"{view = }")
        # required_labels = [x.strip() for x in labels.split(',')]
        # print(f"{required_labels = }")

        # save to CSV file
        #df = pd.DataFrame([recv_msg]).T
        #df.to_csv('OneDrive\Desktop\Internship\test.csv', index=False)

        # testing load it back
        df = pd.read_csv('OneDrive/Desktop/Internship/test.csv')
        data_dict = df.to_dict(orient='records')[0]
        print(f"{data_dict}")

        # reset to None
        recv_msg = None

        # or use this to exit the Python script
        sys.exit()

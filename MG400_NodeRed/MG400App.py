from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
from multiprocessing import Process
from threading import Thread
import numpy as np
import time
import paho.mqtt.client as mqtt
import json
import pandas as pd

# initialize for the topic on which the msg is received
recv_topic = ""
# initialize a recv msg to store the payload received from MQTT
recv_msg = ""

# define all topics
TOPICS = [
    # end the program
    "topic/Terminate",
    # toggle jog
    "topic/ToggleJog",
    # New Jog
    "topic/Jog",
    #run to
    "topic/runTo",
    #receive change topic signal
    "topic/Break",
    #Emergency Stop
    "topic/CoreFunc",
    #Hold button status for teach point
    "topic/TPMove",
    #Export teach points data
    "topic/recv_data",
    #filename of csv file
    "topic/filename",
]

def main_cb(client, userdata, msg):
    # use `global` to change a variable outside of the callback function
    global recv_topic, recv_msg
    recv_topic = msg.topic
    if recv_topic == "topic/recv_data":
        recv_msg = json.loads(msg.payload)
        print('In callback, msg:', recv_msg)
    else:
        recv_msg = msg.payload.decode('utf-8')
        print('In callback, msg:', recv_msg)
    
# localhost / 127.0.0.1
host = "localhost"

client = mqtt.Client("MG400")
client.connect(host, port=1883)
client.loop_start()

# Subscribe to all topics
for topic in TOPICS:
    client.subscribe(topic)

client.on_message = main_cb


# def debug_run():
#     def debug_publish():
#         while True:
#             for topic in TOPICS:
#                 client.publish(topic, 'dummy message', qos=1)
#                 time.sleep(0.5)

#                 # the recv_topic & recv_msg can be accessed in main function
#                 # then you can do your stuff with them here
#                 print(f"In main function, topic: {recv_topic}")
#                 print(f"In main function, msg: {recv_msg}\n")

#                 if recv_topic == 'topic/Terminate':
#                     print("TERMINATING")
#                     client.loop_stop()
#                     client.disconnect()
#                     exit()

#     p1 = Thread(target=debug_publish)
#     p1.start()

# Main function where all robot code logic exist
size = 6
Angle = [0] * size
Pose = [0] *size



def main(client_dashboard, client_feedback, Angle, Pose):
    # Remove alarm
    client_dashboard.ClearError()
    time.sleep(0.5)
    # Description The upper function was enabled successfully
    client_dashboard.EnableRobot()
    time.sleep(0.5)
    
    while True:  
        print("Loop 1")
        #Other functions put herse
        Angle = client_dashboard.GetAngle()
        client.publish("topic/Angle", Angle)
        Pose = client_dashboard.GetPose()
        client.publish("topic/Pose",Pose)
        time.sleep(0.25)
        
        #to unlock robot
        if recv_topic == "topic/CoreFunc" and recv_msg == "Enable":
            client_dashboard.EnableRobot()

        if recv_topic == "topic/CoreFunc" and recv_msg == "Disable":
            client_dashboard.DisableRobot()

        if recv_topic == "topic/CoreFunc" and recv_msg == "ClearErr":
            client_dashboard.ClearError()

        #Teach point
        if recv_topic == "topic/runTo":
            time.sleep(0.25)
            point = recv_msg
           
           # add additional 2 joint angles as the Dobot API reads 6 joint angles eventhough 4 are used
           # augment the string and change it to an array of joint angles
            pointCompare = "{"+point+",0.000000,0.000000}"
            pointString = point.split(",")
            
            #topic/TPMove for hold button
            while recv_topic == "topic/runTo" or recv_topic == "topic/TPMove":
                print("Loop 2")
                #Stop robot
                client_feedback.MoveJog("")
                time.sleep(0.20)
                #Re-enable robot
                client_dashboard.EnableRobot()
                time.sleep(0.20)
                Angle = client_dashboard.GetAngle()
                client.publish("topic/Angle", Angle)
                #If button is held down 
                while recv_topic == "topic/TPMove" and recv_msg == "GO":
                    print("Loop 3")
                    #trigger the mg400 to move to the designated point 
                    client_feedback.JointMovJ2((float(pointString[0])),float((pointString[1])),float((pointString[2])),float((pointString[3]))) 
                    time.sleep(0.25)
                    Angle = client_dashboard.GetAngle()
                    print(Angle)
                    #if button is released
                    if recv_msg == "NOGO" and recv_topic == "topic/TPMove":
                        break

                    #if receive terminate app 
                    if recv_msg == "Terminate":
                        break

                #if reached teach point position
                if Angle == pointCompare:
                    client.publish("topic/break", payload = "Done")
                    time.sleep(0.25)

                #if receive terminate app 
                if recv_msg == "Terminate":
                    break
            while recv_topic == "topic/Jog":
                client_feedback.MoveJog(recv_msg)
                time.sleep(0.2)
                Angle = client_dashboard.GetAngle()
                client.publish("topic/Angle", Angle)
                Pose = client_dashboard.GetPose()
                client.publish("topic/Pose",Pose)
                # use sleep to avoid keep sending the MoveJog command to the dobot
                time.sleep(0.2)

                if recv_msg == "Terminate":
                    break
            #Terminate will shut down app 
            if recv_msg == "Terminate":
                client_dashboard.DisableRobot()
                client.loop_stop()
                client.disconnect()

                client_dashboard.close()
                client_feedback.close()
                break

            #save filename
            if recv_topic == "topic/filename":
                filename = recv_msg
                filepath = "CSV_SaveLoc"
                fullpath = filepath + filename
            #export data to file
            if recv_topic == "topic/recv_data":
                # access the Dictionary from the parsed JSON          
                coord1 = recv_msg[0]['J1']
                coord2 = recv_msg[0]['J2']
                coord3 = recv_msg[0]['J3']
                coord4 = recv_msg[0]['J4']
                labels = recv_msg[0]['labels']
                view = recv_msg[0]['view']

                # save to CSV file
                df = pd.DataFrame([recv_msg]).T
                df.to_csv(fullpath, index=False)

                if recv_topic == "topic/CoreFunc" and recv_msg == "GetData":
                    df = pd.read_csv(fullpath)
                    lines= len(list(df))
                    #df = pd.read_csv('OneDrive/Desktop/Internship/test2.csv')
                    row = 0
                    data_dict = "Random String"
                    while row <= lines:
                        time.sleep(1)
                        data_dict = df.to_dict(orient='records')[row]
                        print(f"{data_dict}")
                        send = str(data_dict)

                        #publish the data in the called csv file to node red 
                        client.publish("topic/recv_joint",send)
                        row = row + 1
                    client.publish("topic/break", payload = "Done")

                # #print(pointString)
                # time.sleep(0.25)
                # Angle = client_dashboard.GetAngle()
                # time.sleep(0.25)
                # #client.publish("topic/break", payload = "Break")
                # #print(pointCompare)
                # #print(Angle)
                # time.sleep(0.5)

                # if Angle == pointCompare or (recv_msg == "Stop" and recv_topic == "topic/runTo"):
                #     #client_feedback.MoveJog()
                #     time.sleep(0.25)
                #     client_dashboard.EnableRobot()
                #     client.publish("topic/break", payload = "Done")
                #     time.sleep(0.25)

                

        # if recv_topic == "topic/ToggleJog":
        #     if recv_msg == "Start":
        #         while True:
        #             if recv_topic == "topic/Jog":    
        #                 client_feedback.MoveJog(recv_msg)
                    
        #             time.sleep(0.2)
        #             Angle = client_dashboard.GetAngle()
        #             client.publish("topic/Angle", Angle)
        #             Pose = client_dashboard.GetPose()
        #             client.publish("topic/Pose",Pose)
        #             # use sleep to avoid keep sending the MoveJog command to the dobot
        #             time.sleep(0.2)

        #             if recv_msg == "Terminate":
        #                 break
                        
        #             if recv_topic == "topic/ToggleJog" and recv_msg == "Stop":
        #                 break

     
        



                    
# Enable threads on ports 29999 and 30003
if __name__ == '__main__':
    client_dashboard = dobot_api_dashboard('192.168.1.6', 29999)
    client_feedback = dobot_api_feedback('192.168.1.6', 30003)
    p1 = Thread(target=main, args=(client_dashboard, client_feedback, Angle, Pose))
    p1.start()
    

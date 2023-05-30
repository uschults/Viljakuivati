import paho.mqtt.client as mqtt

broker = '127.0.0.1'
port = 1883
# generate client ID with pub prefix randomly
username = ''
password = ''

def on_connect(client, userdata, flags, rc):
     if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("mootor")
        client.subscribe("mootor/mootor1")


def on_message(client, userdata, msg):
    print(msg)
    print("Message received-> "+ msg.topic + " " + str(msg.payload))  # Print a received msg


client = mqtt.Client("testing name")  # Create instance of client with client ID “digi_mqtt_test”
client.username_pw_set(username, password)
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker, port)
client.loop_forever()  # Start networking daemon
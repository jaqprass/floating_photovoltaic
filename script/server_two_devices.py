import struct
import paho.mqtt.client as mqtt
import json
import base64
import csv
from datetime import datetime


# gives connection message
def on_connect(mqttc, mosq, obj, rc):
    print("Connected with result code:" + str(rc))
    # subscribe for device
    mqttc.subscribe('application/8/device/5f2048face309136/rx')
    mqttc.subscribe('application/8/device/88e2415fdf5eb2f2/rx')


# gives message from device
def on_message(mqttc, obj, msg):
    try:
        x = json.loads(msg.payload.decode('utf-8'))
        device = x["devEUI"]
        payload_raw = x["data"]
        payload_bytes = base64.b64decode(payload_raw)

        Tamb = struct.unpack('<f', payload_bytes[0:4])
        H = struct.unpack('<f', payload_bytes[4:8])
        T1 = struct.unpack('<f', payload_bytes[8:12])
        T2 = struct.unpack('<f', payload_bytes[12:16])
        T3 = struct.unpack('<f', payload_bytes[16:20])
        Isc = struct.unpack('<f', payload_bytes[20:24])
        V = struct.unpack('<f', payload_bytes[24:28])
        I = struct.unpack('<f', payload_bytes[28:32])

        print("Data received:", Tamb[0], H[0], T1[0], T2[0], T3[0], Isc[0], V[0], I[0])

        day = datetime.now().strftime("%d/%m/%Y")
        hour = datetime.now().strftime("%H:%M")

        # grownd device
        if device == "88e2415fdf5eb2f2":
            name_file_now = "data_grownd_" + datetime.now().strftime("%d_%m_%Y") + ".csv"
            if file_name_ground != name_file_now:
                create_csv()

        # floting device
        if device == "5f2048face309136":
            name_file_now = "data_floating_" + datetime.now().strftime("%d_%m_%Y") + ".csv"
            if file_name_floating != name_file_now:
                create_csv()

        # Open csv file again and write the data
        with open(name_file_now, "a", newline="") as csv_file:
            writer = csv.writer(csv_file, delimiter=';')
            writer.writerow([day, hour, Tamb[0], H[0], T1[0], T2[0], T3[0], Isc[0], V[0], I[0]])

    except Exception as e:
        print(e)
        pass


def create_csv():
    # creates csv for the grown data
    global file_name_ground
    file_name_ground = "data_grownd_" + datetime.now().strftime("%d_%m_%Y") + ".csv"
    # Open csv file
    with open(file_name_ground, "w", newline="") as csv_file:
        writer = csv.writer(csv_file, delimiter=';')

        writer.writerow(["Dia", "Hora", "Temp Amb [ºC]", "Umidade [%]", "T1 [ºC]", "T2 [ºC]", "T3 [ºC]",
                        "Isc [mA]", "Tensão [V]", "Corrente [mA]"])

    # creates csv for the floating data
    global file_name_floating
    file_name_floating = "data_floating_" + datetime.now().strftime("%d_%m_%Y") + ".csv"
    # Open csv file
    with open(file_name_floating, "w", newline="") as csv_file:
        writer = csv.writer(csv_file, delimiter=';')

        writer.writerow(["Dia", "Hora", "Temp Amb [ºC]", "Umidade [%]", "T1 [ºC]", "T2 [ºC]", "T3 [ºC]",
                         "Isc [mA]", "Tensão [V]", "Corrente [mA]"])


mqttc = mqtt.Client()
# Assign event callbacks
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect("lorawan-srv01.asav.brm", 1883, 60)

create_csv()

# and listen to server
run = True
while run:
    mqttc.loop()

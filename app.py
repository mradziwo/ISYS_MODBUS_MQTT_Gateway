import paho.mqtt.client as mqtt
import time
import json
import socket
import yaml
from datetime import datetime  
import shutil
import sys
import threading
import ISA010

_version="0.0.1"

class communication:
    
    def start(self,config):
        raise NotImplementedError('users must define start(configuration) to use this base class')
    def stop(self):
        raise NotImplementedError('users must define stop() to use this base class')
    def write(self,message):
        raise NotImplementedError('users must define write() to use this base class')
    def read(self,n):
        raise NotImplementedError('users must define read() to use this base class')
        
class communicationConfig:
    def __init__(self, ip, port):
        self.ip=ip
        self.port=port
    
class dummy_communication(communication):
    def start(self, config):
        print ("configured")
    def stop(self):
        print ("stopped")
    def write(self,message):
        print ("Message to write:", message)
    def read(self,n):
        t="A +042.81 +022.70 +000000 +000000     CH4\r"
        req=t[:n]
        print ("read request returns:", req)
        return(req)
    
class RSOverMoxa(communication):
    def start(self,config):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self._socket.connect(("192.168.20.119", 4001))
        try:
            self._socket.connect((config.ip, config.port))
            print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tConnected to RS485 gateway: "+config.ip+":",config.port)
        except socket.error as e:
            print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tError while connecting :: %s" % e)
        sys.stdout.flush()
        

        
    def stop(self):
        self._socket.close()
        
    def write(self, message):
        self._socket.sendall(message)
    
    def read(self,n):
        message=""
        while len(message) <n:
            message = message+self._socket.recv(1024).decode()
        return (message)





def startSubscriptions(names):
    for name in names:
        client.subscribe(Topics[name])
        line=datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tSubscribed to: "+Topics[name]
        print (line)

def NotifyBroadcasts(names):
    for name in names:
        client.subscribe(Topics[name])
        line=datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tBroadcasting at: "+Topics[name]
        print (line)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    line=datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tDevice "+DEVICEName+" connected to MQTT Broker at "+MQTTIP+":"+str(MQTTPort)+" with result code "+str(rc)
    print (line)
    client.publish(Topics["Status"], "Online" ,retain=True)
    client.publish(Topics["Version"], str(_version),retain=True)
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    startSubscriptions(["Read","Write","MaskWrite","Disconnect","PWM","KasAwar","DKG"])
    NotifyBroadcasts(["Version","Status"])
    sys.stdout.flush()
    
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

    
def Read_callback(client, userdata, message):
    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tClient received Read Registers request; PAYLOAD: "+str(message.payload))
    sys.stdout.flush()
    try:
        request=json.loads((message.payload).decode())
    except:
        print(message.payload.decode()+" not recognized as valid json string" )
        return()
    try:
        slaveID=request["slaveID"]
        startingAddress=request["startingAddress"]
        noOfPoints=request["noOfPoints"]
    except:
        print("Message:\n"+str(message.payload)+"\ndid not contain slaveID, startingAddress, or noOfPoints fields" )
        print("""Example:\n{\n"  slaveID": 1,\n"  startingAddress": 33,\n  "noOfPoints": 1\n}""") 
        return
    try:
        reply=controller.readRegisters(slaveID, startingAddress, noOfPoints)
        return reply
    except:
        raise

def Write_callback(client, userdata, message):
    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tClient received Preset Register request; PAYLOAD: "+str(message.payload))
    sys.stdout.flush()    
    try:
        request=json.loads((message.payload).decode())
    except:
        print(message.payload.decode()+" not recognized as valid json string" )
        return()
    try:
        slaveID=request["slaveID"]
        registerAddress=request["registerAddress"]
        presetData=request["presetData"]
    except:
        print("Message:\n"+str(message.payload)+"\ndid not contain slaveID, registerAddress, or presetData fields" )
        print("""Example:\n{\n"  slaveID": 1,\n"  startingAddress": 33,\n  "presetData": 1\n}""")     

def MaskWrite_callback(client, userdata, message):
    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tClient received Mask Preset Register request; PAYLOAD: "+str(message.payload))
    sys.stdout.flush()
    try:
        request=json.loads((message.payload).decode())
    except:
        print(message.payload.decode()+" not recognized as valid json string" )
        return()
    try:
        slaveID=request["slaveID"]
        registerAddress=request["registerAddress"]
        andMask=request["andMask"]
        orMask=request["orMask"]
    except:
        print("Message:\n"+str(message.payload)+"\ndid not contain slaveID, registerAddress, andMask or orMask fields" )
        print("""Example:\n{\n"  slaveID": 1,\n"  startingAddress": 33,\n  "andMask": 1,\n  "orMask":1024\n}""") 

def Disconnect_callback(client, userdata, message):
    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tClient received Disconnect request")
    sys.stdout.flush()
    client.publish(Topics["Status"], "Offline" , retain=True)
    client.disconnect()

def PWM_callback(client, userdata, message):
    controller.setPWM(int(message.payload))
    status=controller.readStatus()
    client.publish(Topics["DataStatus"], json.dumps(status))

def DKG_callback(client, userdata, message):
    controller.setDKG(int(message.payload))
    status=controller.readStatus()
    client.publish(Topics["DataStatus"], json.dumps(status))

def KasAwar_callback(client, userdata, message):
    controller.setKasAwar(int(message.payload))
    status=controller.readStatus()
    client.publish(Topics["DataStatus"], json.dumps(status))

def buildTopics(root, name):
    Topics={}
    base=root+'/'+name+'/'
    Topics.update({"Status":base+'Info/Status'})
    Topics.update({"Version":base+'Info/Version'})
    Topics.update({"Disconnect":base+'Disconnect'})
    Topics.update({"Read":base+"ReadRegisters"})
    Topics.update({"Write":base+"PresetRegister"})
    Topics.update({"MaskWrite":base+"MaskPresetRegister"})
    Topics.update({"Reply":base+'Reply'})
    Topics.update({"DataStatus":base+'Data/Status'})
    Topics.update({"PWM":base+'Control/PWM'})
    Topics.update({"KasAwar":base+'Control/KasAwar'})
    Topics.update({"DKG":base+'Control/DKG'})
    return (Topics)
    
def closeAll():
    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tProcess stopped")
    sys.stdout.flush()
    controller.serviceMode(0)
    try:
        controller.close()
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
    client.disconnect()
    client.loop_stop(force=True)



controller=[]

if __name__ == "__main__":
    try:
        with open("./config/config.yaml", 'r') as stream:
            conf=yaml.load(stream)
    except yaml.YAMLError as exc:
        print ("Error processing configuration file")
        print(exc)
        sys.stdout.flush()
    except IOError as exc:
        if exc.errno == 2:
            print ("Configuration file \\config\\config.yaml not found. Substituting with default configuration.")
            sys.stdout.flush()
            shutil.copy("config.yaml", "./config/config.yaml")
            try:
                with open("./config/config.yaml", 'r') as stream:
                    conf=yaml.load(stream)
            except yaml.YAMLError as exc:
                print ("Error processing configuration file")
                print(exc)
                sys.stdout.flush()
        else:
            raise
            
    MQTTIP=conf["MQTT"]["IP"]
    MQTTPort=conf["MQTT"]["Port"]
    MQTTUser=conf["MQTT"]["User"]
    MQTTPassword=conf["MQTT"]["Password"]
    MQTTKeepAlive=conf["MQTT"]["KeepAlive"]
    MQTTRootPath=conf["MQTT"]["rootPath"]

    DEVICEIP=conf["Device"]["IP"]
    DEVICEPort=conf["Device"]["Port"]
    DEVICESlaveAdderess=conf["Device"]["SlaveAddress"]
    DEVICEName=conf["Device"]["Name"]
    ScanRate=conf["Settings"]["ScanRate"]


    print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tMODBUS MQTT Gateway V 0.0.1")

    device_root=MQTTRootPath
    config=communicationConfig(DEVICEIP,DEVICEPort)
    #FlowMeterDevice=FMA1600(dummy_communication,config)
    username=MQTTUser
    password=MQTTPassword
    Topics = buildTopics(device_root, DEVICEName)
    client = mqtt.Client(client_id=DEVICEName+"_MODBUS_MQTT_GATEWAY")
    client.username_pw_set(username, password=None)
    client.will_set(Topics["Status"], payload="Offline", qos=0, retain=True)
    client.on_connect = on_connect
    client.on_message = on_message

    

    client.message_callback_add(Topics["Read"], Read_callback)
    client.message_callback_add(Topics["Write"], Write_callback)
    client.message_callback_add(Topics["MaskWrite"], MaskWrite_callback)
    client.message_callback_add(Topics["Disconnect"], Disconnect_callback)
    client.message_callback_add(Topics["PWM"], PWM_callback)
    client.message_callback_add(Topics["DKG"], DKG_callback)
    client.message_callback_add(Topics["KasAwar"], KasAwar_callback)

    try:
        client.connect(MQTTIP, MQTTPort, MQTTKeepAlive)
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tConnected to MQTT Broker: "+MQTTIP+":",MQTTPort)
    except socket.error as e:
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tError while connecting to MQTT broker : "+MQTTIP+":",MQTTPort,":: %s" % e)
        closeAll()
        quit()

    try:
        controller=ISA010.ISA010(DEVICEIP,DEVICEPort,DEVICESlaveAdderess)
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tOpened Connection to ISA010 via ethernet RS484 gateway: "+DEVICEIP+":",DEVICEPort)
    except:
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tError while connecting to ISA010 : "+DEVICEIP+":",DEVICEPort)
        closeAll()
        raise
        quit()

    try:
        controller.serviceMode(1)
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tController enterd Service Mode")
    except:
        print (datetime.utcnow().strftime('[%Y-%m-%d %H:%M:%S.%f')[:-3]+"]\tError while entering ServiceMode")
        closeAll()
        raise
        quit()
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_start()

    while True:
        status=controller.readStatus()
        (rc, mid)=client.publish(Topics["DataStatus"], json.dumps(status),qos=2)
        print("RC:"+str(rc))
        print("mid:"+str(mid))
        if rc == 4:
            break
        time.sleep(ScanRate)
 
    closeAll()
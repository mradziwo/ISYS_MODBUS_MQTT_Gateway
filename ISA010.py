import modbus
import threading

class ISA010( modbus.GatewayInstrument ):
    mutex = threading.Lock()

    def __init__(self, gatewayAddress, gatewayPort, slaveAddress,):
        modbus.GatewayInstrument.__init__(self, gatewayAddress, gatewayPort, slaveAddress)

    def readFirmware(self):
        with self.mutex:
            reply=self.read_register(1)
        return reply

    def flag(self,number, pos):
        return 1 if number&(1<<pos)!=0 else 0
        
    def readStatus(self):
        with self.mutex:
            reply=self.read_registers(2,15)
        status={}
        status.update({"Uin":reply[2]/10.})
        status.update({"PWM":reply[3]})
        status.update({"Imp":reply[4]})
        status.update({"OgraniczenieMocy":reply[5]})
        status.update({"Awaria":reply[7]})
        status.update({"Faza":reply[8]})
        status.update({"MocI2C":reply[9]})
        status.update({"TemperaturaSpalin":reply[10]/10.})
        status.update({"Blad":reply[11]})
        status.update({"NapiecieZasilana":reply[12]})
        status.update({"MocUART":reply[13]})
        status.update({"Cisnienie":reply[14]/10.})
        status.update({"DIP":{"1":self.flag(reply[6],0),"2":self.flag(reply[6],1),"3":self.flag(reply[6],2)}})
        status.update({"Wyjscia":{"DKG":self.flag(reply[0],0),"KasowanieAwarii":self.flag(reply[0],1),"Awaria":self.flag(reply[0],2),\
                            "PotwierdzeniePracy":self.flag(reply[0],3),"Bypass":self.flag(reply[0],4),"Chlodzenie":self.flag(reply[0],5)}})
        status.update({"Wejscia":{"StartStop":self.flag(reply[1],0),"TermostatKanalowy":self.flag(reply[1],1),\
                            "PresostatPrzeplywuPowietrza":self.flag(reply[1],2), "PresostatGazu":self.flag(reply[1],3),\
                            "PowierdzeniePracy":self.flag(reply[1],4), "AlarmPalnika":self.flag(reply[1],5), \
                            "ZasilanieDKG":self.flag(reply[1],6), "ZaworV1":self.flag(reply[1],7)}})
        return status
    
    def serviceMode(self, val):
        if val == 1:
            try:
                with self.mutex:
                    self.write_register(1, 7493, functioncode=6)
                print ("Tryb Serwisowy ON")
            except:
                raise
        else:
            with self.mutex:
                self.write_register(1,1, functioncode=6)
            print ("Tryb Serwisowy OFF")

    def setPWM(self, PWM):
        with self.mutex:
            return self.write_register(5, PWM, functioncode=6)
    
    def setKasAwar(self, value):
        if value == 1:
            orMask=0x02
        else:
            orMask=0x00
        with self.mutex:
            reply=self.mask_write_register(2,or_mask=orMask, and_mask=0xFFFD)
        return  reply

    def setDKG(self, value):
        if value == 1:
            orMask=0x01
        else:
            orMask=0x00
        with self.mutex:
            reply=self.mask_write_register(2,or_mask=orMask, and_mask=0xFFFE)
        return reply
    
    def readOutputs(self):
        with self.mutex:
            reply=self.read_register(2,functioncode=3)
        return reply

    def readRegisters(self, slaveID, startingAddress, noOfPoints):
        with self.mutex:
            currentSlave=self.address
            self.address=slaveID
            reply= self.read_registers(startingAddress,noOfPoints)
            self.address=currentSlave
        return reply

    def presetRegister(self, slaveID, registerAddress, presetData):
        with self.mutex:
            currentSlave=self.address
            self.address=slaveID
            reply= self.write_register(registerAddress,presetData,functioncode=6)
            self.address=currentSlave
        return reply     
   
    def MaskPresetRegister(self, slaveID, registerAddress, andMask, orMask):
        with self.mutex:
            currentSlave=self.address
            self.address=slaveID
            reply= self.mask_write_register(registerAddress,or_mask=orMask,and_mask=andMask)
            self.address=currentSlave
        return reply

    def close(self):
        self.serviceMode(1)
        self.setDKG(0)
        self.setKasAwar(0)
        self.setPWM(0)
        self.serviceMode(0)
        self.serial.close()
#modbus.minimalmodbus._checkResponseByteCount=modbus._checkResponseByteCount
modbus.minimalmodbus._predictResponseSize=modbus._predictResponseSize
if __name__ == '__main__':

    modbus.minimalmodbus._print_out( 'TESTING ISA010 MODBUS MODULE')

    a = ISA010('192.168.20.118',4001, 1)
    #a.debug = True
  
    
    a.serviceMode(1)

    
    try:
      #  modbus.minimalmodbus._print_out( 'Firmware:                  {0}'.format( a.readFirmware() ))
      #  modbus.minimalmodbus._print_out( 'Status:                    {0}'.format( a.readStatus() ))
      #  modbus.minimalmodbus._print_out( 'SetPWM100:                 {0}'.format( a.setPWM(100) ))       
      #  modbus.minimalmodbus._print_out( 'Status:                    {0}'.format( a.readStatus() ))
      #  modbus.minimalmodbus._print_out( 'SetPWM0:                   {0}'.format( a.setPWM(0) ))       
        modbus.minimalmodbus._print_out( 'Status:                    {0}'.format( a.readStatus() ))
        modbus.minimalmodbus._print_out( 'Outputs:                    {}'.format(a.readOutputs()))
        modbus.minimalmodbus._print_out( 'SetKASAwar1:                {}'.format( a.setKasAwar(1) ))  
        modbus.minimalmodbus._print_out( 'Outputs:                    {}'.format(a.readOutputs()))      
        modbus.minimalmodbus._print_out( 'Status:                    {0}'.format( a.readStatus() ))
        modbus.minimalmodbus._print_out( 'SetKASAwar0:                {}'.format( a.setKasAwar(0) ))   
        modbus.minimalmodbus._print_out( 'Outputs:                    {}'.format(a.readOutputs()))   
        modbus.minimalmodbus._print_out( 'SetDKG1:                    {}'.format( a.setDKG(1) ))  
        modbus.minimalmodbus._print_out( 'Outputs:                    {}'.format(a.readOutputs()))      
        modbus.minimalmodbus._print_out( 'Status:                    {0}'.format( a.readStatus() ))
        modbus.minimalmodbus._print_out( 'SetDKG0:                {}'.format( a.setDKG(0) ))   
        modbus.minimalmodbus._print_out( 'Outputs:                    {}'.format(a.readOutputs()))   
        modbus.minimalmodbus._print_out( 'Status:                     {0}'.format( a.readStatus() ))

    finally:
        a.serviceMode(0)
        a.close()

    modbus.minimalmodbus._print_out( 'DONE!' )

pass 
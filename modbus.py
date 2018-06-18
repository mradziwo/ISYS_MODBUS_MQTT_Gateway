import minimalmodbus
import socket 
import sys
import time

_NUMBER_OF_BYTES_PER_REGISTER = 2
_SECONDS_TO_MILLISECONDS = 1000
_ASCII_HEADER = ':'
_ASCII_FOOTER = '\r\n'

# Several instrument instances can share the same serialport
_SERIALPORTS = {}
_LATEST_READ_TIMES = {}


class SerialOverTCP():

    version = "0.0.10"
    def __init__ (self, gatewayAddress, gatewayPort, baudrate, debug=False):
        self.gatewayAddress=gatewayAddress
        self.gatewayPort=gatewayPort
        self.port=gatewayAddress+":"+str(gatewayPort)
        self.baudrate=baudrate
        self.debug=debug
        self.open()
        self.timeout=1000

    def open(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self._socket.connect(("192.168.20.119", 4001))
        try:
            self._socket.connect((self.gatewayAddress, self.gatewayPort))
            minimalmodbus._print_out('\nConnected to RS485 gateway at  {}'. \
                    format(self.port))
        except socket.error as e:
            minimalmodbus._print_out('\nMinimalModbus debug mode. Error while connecting to RS485 gateway at  {}: {}'. \
                format(self.port, e)) 

    def close(self):
        self._socket.close()
        if self.debug:
            minimalmodbus._print_out('\nMinimalModbus debug mode. Disconnected from RS485 gateway at  {}'. \
                format(self.port))
    
    def write(self,payload):
        self._socket.sendall(payload)

    def read(self,n, request):
        if request[3:5]==b'06' or request[3:5]==b'16':
            normal=1
        else:
            normal=0
            n=n+2
        message= bytearray()
        while len(message) <n:
            if self.debug:
                minimalmodbus._print_out('\nMinimalModbus debug mode. CurrentMessage  {}, Message length: {}, Expected Length{}'. \
                    format(str(message), len(message), n))
            message +=self._socket.recv(1024)

        if normal==0:         
            extra=message[-6:-4]
            extraVal=ord(minimalmodbus._hexdecode(extra.decode()))

            del message[-6:-4]  #crappy implementation of modbus
            crc=message[-4:-2]

            readcrc=ord(minimalmodbus._hexdecode(crc.decode()))

            modifiedcrc=minimalmodbus._hexencode(minimalmodbus._numToOneByteString(readcrc+extraVal))

            message[-4:-2]=modifiedcrc.encode()

        return (message)






class GatewayInstrument(minimalmodbus.Instrument):

    def __init__(self, gatewayAddress, gatewayPort, slaveAddress, baudrate=9600):
        port=gatewayAddress+":"+str(gatewayPort)
        if port not in _SERIALPORTS or not _SERIALPORTS[port]:
            self.serial = _SERIALPORTS[port] = SerialOverTCP(gatewayAddress, gatewayPort, baudrate, debug=False)
        else:
            self.serial = _SERIALPORTS[port]
            if self.serial.port is None:
                self.serial.open()
        """The serial port object as defined by the pySerial module. Created by the constructor.
        Attributes:
            - port (str):      Serial port name.
                - Most often set by the constructor (see the class documentation).
            - baudrate (int):  Baudrate in Baud.
                - Defaults to :data:`BAUDRATE`.
            - parity (probably int): Parity. See the pySerial module for documentation.
                - Defaults to :data:`PARITY`.
            - bytesize (int):  Bytesize in bits.
                - Defaults to :data:`BYTESIZE`.
            - stopbits (int):  The number of stopbits.
                - Defaults to :data:`STOPBITS`.
            - timeout (float): Timeout value in seconds.
                - Defaults to :data:`TIMEOUT`.
        """

        self.address = slaveAddress
        """Slave address (int). Most often set by the constructor (see the class documentation). """

        self.mode = 'ascii'
        """Slave mode (str), can be MODE_RTU or MODE_ASCII.  Most often set by the constructor (see the class documentation).
        New in version 0.6.
        """

        self.debug = False
        """Set this to :const:`True` to print the communication details. Defaults to :const:`False`."""

        self.close_port_after_each_call = False
        """If this is :const:`True`, the serial port will be closed after each call. Defaults to :data:`CLOSE_PORT_AFTER_EACH_CALL`. To change it, set the value ``minimalmodbus.CLOSE_PORT_AFTER_EACH_CALL=True`` ."""

        self.precalculate_read_size = True
        """If this is :const:`False`, the serial port reads until timeout
        instead of just reading a specific number of bytes. Defaults to :const:`True`.
        New in version 0.5.
        """
        
        self.handle_local_echo = False
        """Set to to :const:`True` if your RS-485 adaptor has local echo enabled. 
        Then the transmitted message will immeadiately appear at the receive line of the RS-485 adaptor.
        MinimalModbus will then read and discard this data, before reading the data from the slave.
        Defaults to :const:`False`.
        New in version 0.7.
        """

        if  self.close_port_after_each_call:
            self.serial.close()
    
    def close(self):
        self.serial.close()

    def _genericCommand(self, functioncode, registeraddress, value=None, \
            numberOfDecimals=0, numberOfRegisters=1, signed=False, payloadformat=None):
        """Generic command for reading and writing registers and bits.
        Args:
            * functioncode (int): Modbus function code.
            * registeraddress (int): The register address  (use decimal numbers, not hex).
            * value (numerical or string or None or list of int): The value to store in the register. Depends on payloadformat.
            * numberOfDecimals (int): The number of decimals for content conversion. Only for a single register.
            * numberOfRegisters (int): The number of registers to read/write. Only certain values allowed, depends on payloadformat.
            * signed (bool): Whether the data should be interpreted as unsigned or signed. Only for a single register or for payloadformat='long'.
            * payloadformat (None or string): None, 'long', 'float', 'string', 'register', 'registers'. Not necessary for single registers or bits.
        If a value of 77.0 is stored internally in the slave register as 770,
        then use ``numberOfDecimals=1`` which will divide the received data from the slave by 10
        before returning the value. Similarly ``numberOfDecimals=2`` will divide
        the received data by 100 before returning the value. Same functionality is also used
        when writing data to the slave.
        Returns:
            The register data in numerical value (int or float), or the bit value 0 or 1 (int), or ``None``.
        Raises:
            ValueError, TypeError, IOError
        """
        NUMBER_OF_BITS = 1
        NUMBER_OF_BYTES_FOR_ONE_BIT = 1
        NUMBER_OF_BYTES_BEFORE_REGISTERDATA = 1
        ALL_ALLOWED_FUNCTIONCODES = list(range(1, 7)) + [15, 16]+ [22]  # To comply with both Python2 and Python3
        MAX_NUMBER_OF_REGISTERS = 255

        # Payload format constants, so datatypes can be told apart.
        # Note that bit datatype not is included, because it uses other functioncodes.
        PAYLOADFORMAT_LONG      = 'long'
        PAYLOADFORMAT_FLOAT     = 'float'
        PAYLOADFORMAT_STRING    = 'string'
        PAYLOADFORMAT_REGISTER  = 'register'
        PAYLOADFORMAT_REGISTERS = 'registers'

        ALL_PAYLOADFORMATS = [PAYLOADFORMAT_LONG, PAYLOADFORMAT_FLOAT, \
            PAYLOADFORMAT_STRING, PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_REGISTERS]

        ## Check input values ##
        minimalmodbus._checkFunctioncode(functioncode, ALL_ALLOWED_FUNCTIONCODES)  # Note: The calling facade functions should validate this
        minimalmodbus._checkRegisteraddress(registeraddress)
        minimalmodbus._checkInt(numberOfDecimals, minvalue=0, description='number of decimals')
        minimalmodbus._checkInt(numberOfRegisters, minvalue=1, maxvalue=MAX_NUMBER_OF_REGISTERS, description='number of registers')
        minimalmodbus._checkBool(signed, description='signed')

        if payloadformat is not None:
            if payloadformat not in ALL_PAYLOADFORMATS:
                raise ValueError('Wrong payload format variable. Given: {0!r}'.format(payloadformat))

        ## Check combinations of input parameters ##
        numberOfRegisterBytes = numberOfRegisters * _NUMBER_OF_BYTES_PER_REGISTER

                    # Payload format
        if functioncode in [3, 4, 6, 16] and payloadformat is None:
            payloadformat = PAYLOADFORMAT_REGISTER

        if functioncode in [3, 4, 6, 16]:
            if payloadformat not in ALL_PAYLOADFORMATS:
                raise ValueError('The payload format is unknown. Given format: {0!r}, functioncode: {1!r}.'.\
                    format(payloadformat, functioncode))
        else:
            if payloadformat is not None:
                raise ValueError('The payload format given is not allowed for this function code. ' + \
                    'Given format: {0!r}, functioncode: {1!r}.'.format(payloadformat, functioncode))

                    # Signed and numberOfDecimals
        if signed:
            if payloadformat not in [PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_LONG]:
                raise ValueError('The "signed" parameter can not be used for this data format. ' + \
                    'Given format: {0!r}.'.format(payloadformat))

        if numberOfDecimals > 0 and payloadformat != PAYLOADFORMAT_REGISTER:
            raise ValueError('The "numberOfDecimals" parameter can not be used for this data format. ' + \
                'Given format: {0!r}.'.format(payloadformat))

                    # Number of registers
        if functioncode not in [3, 4, 16] and numberOfRegisters != 1:
            raise ValueError('The numberOfRegisters is not valid for this function code. ' + \
                'NumberOfRegisters: {0!r}, functioncode {1}.'.format(numberOfRegisters, functioncode))

        if functioncode == 16 and payloadformat == PAYLOADFORMAT_REGISTER and numberOfRegisters != 1:
            raise ValueError('Wrong numberOfRegisters when writing to a ' + \
                'single register. Given {0!r}.'.format(numberOfRegisters))
            # Note: For function code 16 there is checking also in the content conversion functions.

                    # Value
        if functioncode in [5, 6, 15, 16, 22] and value is None:
            raise ValueError('The input value is not valid for this function code. ' + \
                'Given {0!r} and {1}.'.format(value, functioncode))

        if functioncode == 16 and payloadformat in [PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_FLOAT, PAYLOADFORMAT_LONG]:
            minimalmodbus._checkNumerical(value, description='input value')

        if functioncode == 6 and payloadformat == PAYLOADFORMAT_REGISTER:
            minimalmodbus._checkNumerical(value, description='input value')

                    # Value for string
        if functioncode == 16 and payloadformat == PAYLOADFORMAT_STRING:
            minimalmodbus._checkString(value, 'input string', minlength=1, maxlength=numberOfRegisterBytes)
            # Note: The string might be padded later, so the length might be shorter than numberOfRegisterBytes.

                    # Value for registers
        if functioncode == 16 and payloadformat == PAYLOADFORMAT_REGISTERS:
            if not isinstance(value, list):
                raise TypeError('The value parameter must be a list. Given {0!r}.'.format(value))

            if len(value) != numberOfRegisters:
                raise ValueError('The list length does not match number of registers. ' + \
                    'List: {0!r},  Number of registers: {1!r}.'.format(value, numberOfRegisters))

        ## Build payload to slave ##
        if functioncode in [1, 2]:
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._numToTwoByteString(NUMBER_OF_BITS)

        elif functioncode in [3, 4]:
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._numToTwoByteString(numberOfRegisters)

        elif functioncode == 5:
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._createBitpattern(functioncode, value)

        elif functioncode == 6:
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._numToTwoByteString(value, numberOfDecimals, signed=signed)

        elif functioncode == 15:
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._numToTwoByteString(NUMBER_OF_BITS) + \
                            minimalmodbus._numToOneByteString(NUMBER_OF_BYTES_FOR_ONE_BIT) + \
                            minimalmodbus._createBitpattern(functioncode, value)

        elif functioncode == 16:
            if payloadformat == PAYLOADFORMAT_REGISTER:
                registerdata = minimalmodbus._numToTwoByteString(value, numberOfDecimals, signed=signed)

            elif payloadformat == PAYLOADFORMAT_STRING:
                registerdata = minimalmodbus._textstringToBytestring(value, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_LONG:
                registerdata = minimalmodbus._longToBytestring(value, signed, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_FLOAT:
                registerdata = minimalmodbus._floatToBytestring(value, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTERS:
                registerdata = minimalmodbus._valuelistToBytestring(value, numberOfRegisters)

            assert len(registerdata) == numberOfRegisterBytes
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            minimalmodbus._numToTwoByteString(numberOfRegisters) + \
                            minimalmodbus._numToOneByteString(numberOfRegisterBytes) + \
                            registerdata
        elif functioncode == 22:
            registerdata = minimalmodbus._valuelistToBytestring(value, 2)
            payloadToSlave = minimalmodbus._numToTwoByteString(registeraddress) + \
                            registerdata
        ## Communicate ##
        payloadFromSlave = self._performCommand(functioncode, payloadToSlave)

        ## Check the contents in the response payload ##
        if functioncode in [1, 2, 3, 4]:
            minimalmodbus._checkResponseByteCount(payloadFromSlave)  # response byte count

        if functioncode in [5, 6, 15, 16]:
            minimalmodbus._checkResponseRegisterAddress(payloadFromSlave, registeraddress)  # response register address

        if functioncode == 5:
            minimalmodbus._checkResponseWriteData(payloadFromSlave, minimalmodbus._createBitpattern(functioncode, value))  # response write data

        if functioncode == 6:
            minimalmodbus._checkResponseWriteData(payloadFromSlave, \
                minimalmodbus._numToTwoByteString(value, numberOfDecimals, signed=signed))  # response write data

        if functioncode == 15:
            minimalmodbus._checkResponseNumberOfRegisters(payloadFromSlave, NUMBER_OF_BITS)  # response number of bits

        if functioncode == 16:
            minimalmodbus._checkResponseNumberOfRegisters(payloadFromSlave, numberOfRegisters)  # response number of registers

        ## Calculate return value ##
        if functioncode in [1, 2]:
            registerdata = payloadFromSlave[NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
            if len(registerdata) != NUMBER_OF_BYTES_FOR_ONE_BIT:
                raise ValueError('The registerdata length does not match NUMBER_OF_BYTES_FOR_ONE_BIT. ' + \
                    'Given {0}.'.format(len(registerdata)))

            return minimalmodbus._bitResponseToValue(registerdata)

        if functioncode in [3, 4]:
            registerdata = payloadFromSlave[NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
            if len(registerdata) != numberOfRegisterBytes:
                raise ValueError('The registerdata length does not match number of register bytes. ' + \
                    'Given {0!r} and {1!r}.'.format(len(registerdata), numberOfRegisterBytes))

            if payloadformat == PAYLOADFORMAT_STRING:
                return minimalmodbus._bytestringToTextstring(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_LONG:
                return minimalmodbus._bytestringToLong(registerdata, signed, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_FLOAT:
                return minimalmodbus._bytestringToFloat(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTERS:
                return minimalmodbus._bytestringToValuelist(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTER:
                return minimalmodbus._twoByteStringToNum(registerdata, numberOfDecimals, signed=signed)

            raise ValueError('Wrong payloadformat for return value generation. ' + \
                'Given {0}'.format(payloadformat))

        if functioncode == 22:
            registerdata = payloadFromSlave[NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
            return ("OK") 

    def _communicate(self, request, number_of_bytes_to_read):
        """Talk to the slave via a serial port.
        Args:
            request (str): The raw request that is to be sent to the slave.
            number_of_bytes_to_read (int): number of bytes to read
        Returns:
            The raw data (string) returned from the slave.
        Raises:
            TypeError, ValueError, IOError
        Note that the answer might have strange ASCII control signs, which
        makes it difficult to print it in the promt (messes up a bit).
        Use repr() to make the string printable (shows ASCII values for control signs.)
        Will block until reaching *number_of_bytes_to_read* or timeout.
        If the attribute :attr:`Instrument.debug` is :const:`True`, the communication details are printed.
        If the attribute :attr:`Instrument.close_port_after_each_call` is :const:`True` the
        serial port is closed after each call.
        Timing::
                                                  Request from master (Master is writing)
                                                  |
                                                  |       Response from slave (Master is reading)
                                                  |       |
            ----W----R----------------------------W-------R----------------------------------------
                     |                            |       |
                     |<----- Silent period ------>|       |
                                                  |       |
                             Roundtrip time  ---->|-------|<--
        The resolution for Python's time.time() is lower on Windows than on Linux.
        It is about 16 ms on Windows according to
        http://stackoverflow.com/questions/157359/accurate-timestamping-in-python
        For Python3, the information sent to and from pySerial should be of the type bytes.
        This is taken care of automatically by MinimalModbus.
        
        
        """

        minimalmodbus._checkString(request, minlength=1, description='request')
        minimalmodbus._checkInt(number_of_bytes_to_read)

        if self.debug:
            minimalmodbus._print_out('\nMinimalModbus debug mode. Writing to instrument (expecting {} bytes back): {!r} ({})'. \
                format(number_of_bytes_to_read, request, minimalmodbus._hexlify(request)))

        if self.close_port_after_each_call:
            self.serial.open()

        #self.serial.flushInput() TODO

        if sys.version_info[0] > 2:
            request = bytes(request, encoding='latin1')  # Convert types to make it Python3 compatible

        # Sleep to make sure 3.5 character times have passed
        minimum_silent_period   = minimalmodbus._calculate_minimum_silent_period(self.serial.baudrate)
        time_since_read         = time.time() - _LATEST_READ_TIMES.get(self.serial.port, 0)

        if time_since_read < minimum_silent_period:
            sleep_time = minimum_silent_period - time_since_read

            if self.debug:
                template = 'MinimalModbus debug mode. Sleeping for {:.1f} ms. ' + \
                        'Minimum silent period: {:.1f} ms, time since read: {:.1f} ms.'
                text = template.format(
                    sleep_time * _SECONDS_TO_MILLISECONDS,
                    minimum_silent_period * _SECONDS_TO_MILLISECONDS,
                    time_since_read * _SECONDS_TO_MILLISECONDS)
                minimalmodbus._print_out(text)

            time.sleep(sleep_time)

        elif self.debug:
            template = 'MinimalModbus debug mode. No sleep required before write. ' + \
                'Time since previous read: {:.1f} ms, minimum silent period: {:.2f} ms.'
            text = template.format(
                time_since_read * _SECONDS_TO_MILLISECONDS,
                minimum_silent_period * _SECONDS_TO_MILLISECONDS)
            minimalmodbus._print_out(text)

        # Write request
        latest_write_time = time.time()
        
        self.serial.write(request)

        # Read and discard local echo
        if self.handle_local_echo:
            localEchoToDiscard = self.serial.read(len(request))
            if self.debug:
                template = 'MinimalModbus debug mode. Discarding this local echo: {!r} ({} bytes).' 
                text = template.format(localEchoToDiscard, len(localEchoToDiscard))
                minimalmodbus._print_out(text)
            if localEchoToDiscard != request:
                template = 'Local echo handling is enabled, but the local echo does not match the sent request. ' + \
                    'Request: {!r} ({} bytes), local echo: {!r} ({} bytes).' 
                text = template.format(request, len(request), localEchoToDiscard, len(localEchoToDiscard))
                raise IOError(text)

        # Read response
        answer = self.serial.read(number_of_bytes_to_read,request)
        _LATEST_READ_TIMES[self.serial.port] = time.time()

        if self.close_port_after_each_call:
            self.serial.close()

        if sys.version_info[0] > 2:
            answer = str(answer, encoding='latin1')  # Convert types to make it Python3 compatible

        if self.debug:
            template = 'MinimalModbus debug mode. Response from instrument: {!r} ({}) ({} bytes), ' + \
                'roundtrip time: {:.1f} ms. Timeout setting: {:.1f} ms.\n'
            text = template.format(
                answer,
                minimalmodbus._hexlify(answer),
                len(answer),
                (_LATEST_READ_TIMES.get(self.serial.port, 0) - latest_write_time) * _SECONDS_TO_MILLISECONDS,
                self.serial.timeout * _SECONDS_TO_MILLISECONDS)
            minimalmodbus._print_out(text)

        if len(answer) == 0:
            raise IOError('No communication with the instrument (no answer)')

        return answer
    def mask_write_register(self, registeraddress, or_mask=0x00, and_mask=0xFF):
        """Modifies the contents of a specified 4XXXX register using a combination of an
        AND mask, an OR mask, and the register’s current contents.  The function can be
        used to set or clear individual bits in the register. 
        The function’s algorithm is:
        Result  =  ( Current Contents AND And_Mask )  OR  ( Or_Mask AND ~(And_Mask) )

            * registeraddress (int): The slave register start address (use decimal numbers, not hex).
            * or_mask(int): OR mask used to mask write the register - only the lowest byte will be used - rest will be ignored.
            * and_mask(int): AND mask used to mask write the register - only the lowest byte will be used - rest will be ignored.
        
        Returns:
            None
        Raises:
            ValueError, TypeError, IOError
        """
        if or_mask>0xFFFF:
            raise ValueError('The "or_mask" must be on range 0x0000 - 0xFFFF. Given: {0}'.format(or_mask))
        if and_mask>0xFFFF:
            raise ValueError('The "and_mask" must be on range 0x0000 - 0xFFFF. Given: {0}'.format(and_mask))
        
        # Note: The content of the list is checked at content conversion.
        values=[ and_mask&0xFFFF,or_mask&0xFFFF]
        self._genericCommand(22, registeraddress, values)

def _predictResponseSize(mode, functioncode, payloadToSlave):
    """Calculate the number of bytes that should be received from the slave.
    Args:
     * mode (str): The modbus protcol mode (MODE_RTU or MODE_ASCII)
     * functioncode (int): Modbus function code.
     * payloadToSlave (str): The raw request that is to be sent to the slave (not hex encoded string)
    Returns:
        The preducted number of bytes (int) in the response.
    Raises:
        ValueError, TypeError.
    """
    MIN_PAYLOAD_LENGTH = 4  # For implemented functioncodes here
    BYTERANGE_FOR_GIVEN_SIZE = slice(2, 4)  # Within the payload

    NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION = 4
    NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD = 1

    RTU_TO_ASCII_PAYLOAD_FACTOR = 2

    NUMBER_OF_RTU_RESPONSE_STARTBYTES   = 2
    NUMBER_OF_RTU_RESPONSE_ENDBYTES     = 2
    NUMBER_OF_ASCII_RESPONSE_STARTBYTES = 5
    NUMBER_OF_ASCII_RESPONSE_ENDBYTES   = 4

    # Argument validity testing
    minimalmodbus._checkMode(mode)
    minimalmodbus._checkFunctioncode(functioncode, None)
    minimalmodbus._checkString(payloadToSlave, description='payload', minlength=MIN_PAYLOAD_LENGTH)


    # Calculate payload size
    if functioncode in [5, 6, 15, 16]:
        response_payload_size = NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION

    elif functioncode in [1, 2, 3, 4]:
        given_size = minimalmodbus._twoByteStringToNum(payloadToSlave[BYTERANGE_FOR_GIVEN_SIZE])
        if functioncode == 1 or functioncode == 2:
            # Algorithm from MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b
            number_of_inputs = given_size
            response_payload_size = NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD + \
                                    number_of_inputs // 8 + (1 if number_of_inputs % 8 else 0)

        elif functioncode == 3 or functioncode == 4:
            number_of_registers = given_size
            response_payload_size = NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD + \
                                    number_of_registers * _NUMBER_OF_BYTES_PER_REGISTER

    elif functioncode == 22:
        response_payload_size = 6

    else:
        raise ValueError('Wrong functioncode: {}. The payload is: {!r}'.format( \
            functioncode, payloadToSlave))
    # Calculate number of bytes to read
    if mode == minimalmodbus.MODE_ASCII:
        return NUMBER_OF_ASCII_RESPONSE_STARTBYTES + \
            response_payload_size * RTU_TO_ASCII_PAYLOAD_FACTOR + \
            NUMBER_OF_ASCII_RESPONSE_ENDBYTES
    else:
        return NUMBER_OF_RTU_RESPONSE_STARTBYTES + \
            response_payload_size + \
            NUMBER_OF_RTU_RESPONSE_ENDBYTES

def _checkFunctioncode(functioncode, listOfAllowedValues=[]):
    """Check that the given functioncode is in the listOfAllowedValues.
    Also verifies that 1 <= function code <= 127.
    Args:
        * functioncode (int): The function code
        * listOfAllowedValues (list of int): Allowed values. Use *None* to bypass this part of the checking.
    Raises:
        TypeError, ValueError
    """
    FUNCTIONCODE_MIN = 1
    FUNCTIONCODE_MAX = 127

    minimalmodbus._checkInt(functioncode, FUNCTIONCODE_MIN, FUNCTIONCODE_MAX, description='functioncode')

    if listOfAllowedValues is None:
        return

    if not isinstance(listOfAllowedValues, list):
        raise TypeError('The listOfAllowedValues should be a list. Given: {0!r}'.format(listOfAllowedValues))

    for value in listOfAllowedValues:
        minimalmodbus._checkInt(value, FUNCTIONCODE_MIN, FUNCTIONCODE_MAX, description='functioncode inside listOfAllowedValues')

    if functioncode not in listOfAllowedValues:
        raise ValueError('Wrong function code: {0}, allowed values are {1!r}'.format(functioncode, listOfAllowedValues))

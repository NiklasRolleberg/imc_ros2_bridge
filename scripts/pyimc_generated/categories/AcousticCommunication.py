'''
IMC Acoustic Communication messages.
'''

from .. import _base
import enum as _enum

class LblRange(_base.base_message):
    '''Distance to the acoustic transponder.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            range : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_range']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Acoustic Communication", id = 200, stable = None, description = "When the vehicle uses Long Base Line navigation, this message notifies that a new range was received from one of the acoustics transponders. The message fields are used to identify the range value and the transponder name.", fields = ('id', 'range',), abbrev = "LblRange", name = "LBL Range", flags = "periodic")

    id = _base.mutable_attr({'name': 'Beacon Identification Number', 'type': 'uint8_t'}, "Identification number of the acoustic transponder from which the range information was received.")
    '''Identification number of the acoustic transponder from which the range information was received. Type: uint8_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "Distance to the acoustic transponder.")
    '''Distance to the acoustic transponder. Type: fp32_t'''

    def __init__(self, id = None, range = None):
        '''Class constructor
        
        Distance to the acoustic transponder.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            range : fp32_t, unit: m'''
        self._id = id
        self._range = range


class LblBeacon(_base.base_message):
    '''Transponder delay.

       This message class contains the following fields and their respective types:
    beacon : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp32_t, unit: m

            query_channel : uint8_t, unit: NOT FOUND

            reply_channel : uint8_t, unit: NOT FOUND

            transponder_delay : uint8_t, unit: ms'''

    __slots__ = ['_Attributes', '_header', '_footer', '_beacon', '_lat', '_lon', '_depth', '_query_channel', '_reply_channel', '_transponder_delay']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 202, stable = None, description = "Position and configuration of an LBL transponder (beacon).", fields = ('beacon', 'lat', 'lon', 'depth', 'query_channel', 'reply_channel', 'transponder_delay',), abbrev = "LblBeacon", name = "LBL Beacon Configuration", flags = None)

    beacon = _base.mutable_attr({'name': 'Beacon Name', 'type': 'plaintext'}, "Name/Label of the acoustic transponder.")
    '''Name/Label of the acoustic transponder. Type: plaintext'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t', 'unit': 'm'}, "The beacon's depth.")
    '''The beacon's depth. Type: fp32_t'''
    query_channel = _base.mutable_attr({'name': 'Interrogation channel', 'type': 'uint8_t'}, "Interrogation channel.")
    '''Interrogation channel. Type: uint8_t'''
    reply_channel = _base.mutable_attr({'name': 'Reply channel', 'type': 'uint8_t'}, "Reply channel.")
    '''Reply channel. Type: uint8_t'''
    transponder_delay = _base.mutable_attr({'name': 'Transponder delay', 'type': 'uint8_t', 'unit': 'ms'}, "Transponder delay.")
    '''Transponder delay. Type: uint8_t'''

    def __init__(self, beacon = None, lat = None, lon = None, depth = None, query_channel = None, reply_channel = None, transponder_delay = None):
        '''Class constructor
        
        Transponder delay.

       This message class contains the following fields and their respective types:
    beacon : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp32_t, unit: m

            query_channel : uint8_t, unit: NOT FOUND

            reply_channel : uint8_t, unit: NOT FOUND

            transponder_delay : uint8_t, unit: ms'''
        self._beacon = beacon
        self._lat = lat
        self._lon = lon
        self._depth = depth
        self._query_channel = query_channel
        self._reply_channel = reply_channel
        self._transponder_delay = transponder_delay


class LblConfig(_base.base_message):
    '''A list of LBL beacon configuration messages.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            beacons : message-list, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        SET_CFG = 0
        '''Name: Set LBL Configuration'''
    
        GET_CFG = 1
        '''Name: Retrieve LBL Configuration'''
    
        CUR_CFG = 2
        '''Name: Reply to a GET command'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_beacons']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 203, stable = None, description = "Long Base Line configuration.", fields = ('op', 'beacons',), abbrev = "LblConfig", name = "LBL Configuration", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Used to define the type of the operation this message holds. Enumerated (Local).")
    '''Used to define the type of the operation this message holds. Enumerated (Local). Type: uint8_t'''
    beacons = _base.mutable_attr({'name': 'Beacons', 'type': 'message-list', 'message-type': 'LblBeacon'}, "A list of LBL beacon configuration messages.")
    '''A list of LBL beacon configuration messages. Type: message-list'''

    def __init__(self, op = None, beacons = None):
        '''Class constructor
        
        A list of LBL beacon configuration messages.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            beacons : message-list, unit: NOT FOUND'''
        self._op = op
        self._beacons = beacons


class AcousticMessage(_base.base_message):
    '''Message to send.

       This message class contains the following fields and their respective types:
    message : message, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_message']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Acoustic Communication", id = 206, stable = None, description = "Send an acoustic message.", fields = ('message',), abbrev = "AcousticMessage", name = "Acoustic Message", flags = None)

    message = _base.mutable_attr({'name': 'Message to send', 'type': 'message'}, "Message to send.")
    '''Message to send. Type: message'''

    def __init__(self, message = None):
        '''Class constructor
        
        Message to send.

       This message class contains the following fields and their respective types:
    message : message, unit: NOT FOUND'''
        self._message = message


class SimAcousticMessage(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: NOT FOUND

            lon : fp64_t, unit: NOT FOUND

            depth : fp32_t, unit: NOT FOUND

            sentence : plaintext, unit: NOT FOUND

            txtime : fp64_t, unit: s

            modem_type : plaintext, unit: NOT FOUND

            sys_src : plaintext, unit: NOT FOUND

            seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: SAM'''
    
        EMPTY = 0
        '''No active flags'''
    
        ACK = 1
        '''Name: Acknowledgement'''
    
        DELAYED = 2
        '''Name: Delayed'''
    
        REPLY = 3
        '''Name: Reply'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_depth', '_sentence', '_txtime', '_modem_type', '_sys_src', '_seq', '_sys_dst', '_flags', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Acoustic Communication", id = 207, stable = None, description = "Send an acoustic message.", fields = ('lat', 'lon', 'depth', 'sentence', 'txtime', 'modem_type', 'sys_src', 'seq', 'sys_dst', 'flags', 'data',), abbrev = "SimAcousticMessage", name = "Simulated Acoustic Message", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t'}, "Absolute latitude of sending vehicle.")
    '''Absolute latitude of sending vehicle. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t'}, "Absolute longitude of sending vehicle.")
    '''Absolute longitude of sending vehicle. Type: fp64_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t'}, "Depth of sending vehicle.")
    '''Depth of sending vehicle. Type: fp32_t'''
    sentence = _base.mutable_attr({'name': 'Sentence', 'type': 'plaintext'}, "Sentence string sent/received by the modem")
    '''Sentence string sent/received by the modem Type: plaintext'''
    txtime = _base.mutable_attr({'name': 'Transmission Time', 'unit': 's', 'type': 'fp64_t'}, "Transmission time.")
    '''Transmission time. Type: fp64_t'''
    modem_type = _base.mutable_attr({'name': 'Modem Type', 'type': 'plaintext'}, "The modem being used.")
    '''The modem being used. Type: plaintext'''
    sys_src = _base.mutable_attr({'name': 'Source system', 'type': 'plaintext'}, "Name of source system.")
    '''Name of source system. Type: plaintext'''
    seq = _base.mutable_attr({'name': 'Sequence Id', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    sys_dst = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'SAM'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "No description available")
    '''No description available Type: rawdata'''

    def __init__(self, lat = None, lon = None, depth = None, sentence = None, txtime = None, modem_type = None, sys_src = None, seq = None, sys_dst = None, flags = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: NOT FOUND

            lon : fp64_t, unit: NOT FOUND

            depth : fp32_t, unit: NOT FOUND

            sentence : plaintext, unit: NOT FOUND

            txtime : fp64_t, unit: s

            modem_type : plaintext, unit: NOT FOUND

            sys_src : plaintext, unit: NOT FOUND

            seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''
        self._lat = lat
        self._lon = lon
        self._depth = depth
        self._sentence = sentence
        self._txtime = txtime
        self._modem_type = modem_type
        self._sys_src = sys_src
        self._seq = seq
        self._sys_dst = sys_dst
        self._flags = flags
        self._data = data


class AcousticOperation(_base.base_message):
    '''Argument for message send ('MSG') requests.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            system : plaintext, unit: NOT FOUND

            range : fp32_t, unit: m

            msg : message, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: AOP'''
    
        ABORT = 0
        '''Name: Abort'''
    
        ABORT_IP = 1
        '''Name: Abort in Progress'''
    
        ABORT_TIMEOUT = 2
        '''Name: Abort Timeout'''
    
        ABORT_ACKED = 3
        '''Name: Abort Acknowledged'''
    
        RANGE = 4
        '''Name: Range Request'''
    
        RANGE_IP = 5
        '''Name: Range in Progress'''
    
        RANGE_TIMEOUT = 6
        '''Name: Range Timeout'''
    
        RANGE_RECVED = 7
        '''Name: Range Received'''
    
        BUSY = 8
        '''Name: Modem is Busy'''
    
        UNSUPPORTED = 9
        '''Name: Unsupported operation'''
    
        NO_TXD = 10
        '''Name: Transducer Not Detected'''
    
        MSG = 11
        '''Name: Send Message'''
    
        MSG_QUEUED = 12
        '''Name: Message Send -- Queued'''
    
        MSG_IP = 13
        '''Name: Message Send -- In progress'''
    
        MSG_DONE = 14
        '''Name: Message Send -- Done'''
    
        MSG_FAILURE = 15
        '''Name: Message Send -- Failure'''
    
        MSG_SHORT = 16
        '''Name: Send Short Message'''
    
        REVERSE_RANGE = 17
        '''Name: Initiate Reverse Range'''
    
        FORCED_ABORT = 18
        '''Name: Forced Abort'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_system', '_range', '_msg']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 211, stable = None, description = "Acoustic operation.", fields = ('op', 'system', 'range', 'msg',), abbrev = "AcousticOperation", name = "Acoustic Operation", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'AOP'}, "Operation type. Enumerated (Local).")
    '''Operation type. Enumerated (Local). Type: uint8_t'''
    system = _base.mutable_attr({'name': 'System', 'type': 'plaintext'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: plaintext'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: fp32_t'''
    msg = _base.mutable_attr({'name': 'Message To Send', 'type': 'message'}, "Argument for message send ('MSG') requests.")
    '''Argument for message send ('MSG') requests. Type: message'''

    def __init__(self, op = None, system = None, range = None, msg = None):
        '''Class constructor
        
        Argument for message send ('MSG') requests.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            system : plaintext, unit: NOT FOUND

            range : fp32_t, unit: m

            msg : message, unit: NOT FOUND'''
        self._op = op
        self._system = system
        self._range = range
        self._msg = msg


class AcousticSystemsQuery(_base.base_message):
    '''Request a list of known underwater acoustic systems. The recipient of this message shall reply with an AcousticSystems message.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 212, stable = None, description = "Request a list of known underwater acoustic systems. The recipient of this message shall reply with an AcousticSystems message.", fields = [], abbrev = "AcousticSystemsQuery", name = "Acoustic Systems Query", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Request a list of known underwater acoustic systems. The recipient of this message shall reply with an AcousticSystems message.

       This message class contains the following fields and their respective types:
'''


class AcousticSystems(_base.base_message):
    '''Comma separated list of known acoustic system names.

       This message class contains the following fields and their respective types:
    list : plaintext, unit: List'''

    __slots__ = ['_Attributes', '_header', '_footer', '_list']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 213, stable = None, description = "This message is sent in reply to an AcousticSystemsQuery message and lists all known underwater acoustic systems (modems, narrow band transponders, etc).", fields = ('list',), abbrev = "AcousticSystems", name = "Acoustic Systems", flags = None)

    list = _base.mutable_attr({'name': 'System List', 'type': 'plaintext', 'unit': 'List'}, "Comma separated list of known acoustic system names.")
    '''Comma separated list of known acoustic system names. Type: plaintext'''

    def __init__(self, list = None):
        '''Class constructor
        
        Comma separated list of known acoustic system names.

       This message class contains the following fields and their respective types:
    list : plaintext, unit: List'''
        self._list = list


class AcousticLink(_base.base_message):
    '''Signal Integrity value illustrates distortion of the last received acoustic signal. It is calculated based on cross-correlation measurements. Higher *Signal Integrity Level* values correspond to less distorted signals. An acoustic link is considered weak if the *Signal Integrity Level* value is less than 100.

       This message class contains the following fields and their respective types:
    peer : plaintext, unit: NOT FOUND

            rssi : fp32_t, unit: dB

            integrity : uint16_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_peer', '_rssi', '_integrity']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 214, stable = None, description = "This message is used to report the perceived link quality to other acoustic peers.", fields = ('peer', 'rssi', 'integrity',), abbrev = "AcousticLink", name = "Acoustic Link Quality", flags = None)

    peer = _base.mutable_attr({'name': 'Peer Name', 'type': 'plaintext'}, "The name of the peer on the other side of this link.")
    '''The name of the peer on the other side of this link. Type: plaintext'''
    rssi = _base.mutable_attr({'name': 'Received Signal Strength Indicator', 'type': 'fp32_t', 'unit': 'dB'}, "RSSI is a signed floating point number. Higher RSSI values correspond to stronger signals. The signal strength is acceptable when measured RSSI values lie between -20 dB and -85 dB.")
    '''RSSI is a signed floating point number. Higher RSSI values correspond to stronger signals. The signal strength is acceptable when measured RSSI values lie between -20 dB and -85 dB. Type: fp32_t'''
    integrity = _base.mutable_attr({'name': 'Signal Integrity Level', 'type': 'uint16_t'}, "Signal Integrity value illustrates distortion of the last received acoustic signal. It is calculated based on cross-correlation measurements. Higher *Signal Integrity Level* values correspond to less distorted signals. An acoustic link is considered weak if the *Signal Integrity Level* value is less than 100.")
    '''Signal Integrity value illustrates distortion of the last received acoustic signal. It is calculated based on cross-correlation measurements. Higher *Signal Integrity Level* values correspond to less distorted signals. An acoustic link is considered weak if the *Signal Integrity Level* value is less than 100. Type: uint16_t'''

    def __init__(self, peer = None, rssi = None, integrity = None):
        '''Class constructor
        
        Signal Integrity value illustrates distortion of the last received acoustic signal. It is calculated based on cross-correlation measurements. Higher *Signal Integrity Level* values correspond to less distorted signals. An acoustic link is considered weak if the *Signal Integrity Level* value is less than 100.

       This message class contains the following fields and their respective types:
    peer : plaintext, unit: NOT FOUND

            rssi : fp32_t, unit: dB

            integrity : uint16_t, unit: NOT FOUND'''
        self._peer = peer
        self._rssi = rssi
        self._integrity = integrity


class AcousticRequest(_base.base_message):
    '''Argument for message send ('MSG') or ('RAW') but in this case expects DevDataBinary message requests.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            range : fp32_t, unit: m

            type : uint8_t, unit: Enumerated (Local)

            msg : message, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: TYPE'''
    
        ABORT = 0
        '''Name: Abort'''
    
        RANGE = 1
        '''Name: Range'''
    
        REVERSE_RANGE = 2
        '''Name: Reverse Range'''
    
        MSG = 3
        '''Name: Message'''
    
        RAW = 4
        '''Name: Raw'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_destination', '_timeout', '_range', '_type', '_msg']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 215, stable = None, description = "Request Acoustic sending.", fields = ('req_id', 'destination', 'timeout', 'range', 'type', 'msg',), abbrev = "AcousticRequest", name = "Acoustic Transmission Request", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    destination = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "The name of the system where to send this message.")
    '''The name of the system where to send this message. Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'fp64_t', 'unit': 's'}, "Period of time to send message (in seconds).")
    '''Period of time to send message (in seconds). Type: fp64_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: fp32_t'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    msg = _base.mutable_attr({'name': 'Message To Send', 'type': 'message'}, "Argument for message send ('MSG') or ('RAW') but in this case expects DevDataBinary message requests.")
    '''Argument for message send ('MSG') or ('RAW') but in this case expects DevDataBinary message requests. Type: message'''

    def __init__(self, req_id = None, destination = None, timeout = None, range = None, type = None, msg = None):
        '''Class constructor
        
        Argument for message send ('MSG') or ('RAW') but in this case expects DevDataBinary message requests.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            range : fp32_t, unit: m

            type : uint8_t, unit: Enumerated (Local)

            msg : message, unit: NOT FOUND'''
        self._req_id = req_id
        self._destination = destination
        self._timeout = timeout
        self._range = range
        self._type = type
        self._msg = msg


class AcousticStatus(_base.base_message):
    '''The meaning of this field depends on the operation and is explained in the operation's description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND

            range : fp32_t, unit: m'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: TYPE'''
    
        ABORT = 0
        '''Name: Abort'''
    
        RANGE = 1
        '''Name: Range'''
    
        REVERSE_RANGE = 2
        '''Name: Reverse Range'''
    
        MSG = 3
        '''Name: Message'''
    
        RAW = 4
        '''Name: Raw'''
    
    
    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: STATUS'''
    
        QUEUED = 0
        '''Name: Queued'''
    
        IN_PROGRESS = 1
        '''Name: In Progress'''
    
        SENT = 2
        '''Name: Sent'''
    
        RANGE_RECEIVED = 3
        '''Name: Range Received'''
    
        DELIVERED = 4
        '''Name: Delivered'''
    
        BUSY = 100
        '''Name: Busy'''
    
        INPUT_FAILURE = 101
        '''Name: Input Error'''
    
        ERROR = 102
        '''Name: Error trying to send acoustic text'''
    
        UNSUPPORTED = 666
        '''Name: Message Type is not defined or is unsupported'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_type', '_status', '_info', '_range']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 216, stable = None, description = "Reply sent in response to a Acoustic Text sending request.", fields = ('req_id', 'type', 'status', 'info', 'range',), abbrev = "AcousticStatus", name = "Acoustic Transmission Status", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'STATUS'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    info = _base.mutable_attr({'name': 'Information', 'type': 'plaintext'}, "Status description.")
    '''Status description. Type: plaintext'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: fp32_t'''

    def __init__(self, req_id = None, type = None, status = None, info = None, range = None):
        '''Class constructor
        
        The meaning of this field depends on the operation and is explained in the operation's description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND

            range : fp32_t, unit: m'''
        self._req_id = req_id
        self._type = type
        self._status = status
        self._info = info
        self._range = range


class AcousticRelease(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    system : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: AROP'''
    
        OPEN = 0
        '''Name: Open'''
    
        CLOSE = 1
        '''Name: Close'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_system', '_op']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 217, stable = None, description = "Request a system (local or remote) to activate its acoustic release.", fields = ('system', 'op',), abbrev = "AcousticRelease", name = "Acoustic Release Request", flags = None)

    system = _base.mutable_attr({'name': 'System', 'type': 'plaintext'}, "The name of the system that should execute an acoustic release.")
    '''The name of the system that should execute an acoustic release. Type: plaintext'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'AROP'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, system = None, op = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    system : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)'''
        self._system = system
        self._op = op


class UamTxFrame(_base.base_message):
    '''The actual data frame to transmit. The data size shall not exceed the MTU of the acoustic modem.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: UTF'''
    
        EMPTY = 0
        '''No active flags'''
    
        ACK = 1
        '''Name: Acknowledgement'''
    
        DELAYED = 2
        '''Name: Delayed'''
    
        FORCED = 4
        '''Name: Forced'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_sys_dst', '_flags', '_data']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 814, stable = None, description = "This message shall be sent to acoustic modem drivers to request transmission of a data frame via the acoustic channel.", fields = ('seq', 'sys_dst', 'flags', 'data',), abbrev = "UamTxFrame", name = "UamTxFrame", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Id', 'type': 'uint16_t'}, "A sequence identifier that should be incremented for each request. This number will then be used to issue transmission status updates via the message UamTxStatus.")
    '''A sequence identifier that should be incremented for each request. This number will then be used to issue transmission status updates via the message UamTxStatus. Type: uint16_t'''
    sys_dst = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "The canonical name of the destination system. If supported, the special destination 'broadcast' shall be used to dispatch messages to all nodes.")
    '''The canonical name of the destination system. If supported, the special destination 'broadcast' shall be used to dispatch messages to all nodes. Type: plaintext'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'UTF'}, "Transmission flags. Bitfield (Local).")
    '''Transmission flags. Bitfield (Local). Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "The actual data frame to transmit. The data size shall not exceed the MTU of the acoustic modem.")
    '''The actual data frame to transmit. The data size shall not exceed the MTU of the acoustic modem. Type: rawdata'''

    def __init__(self, seq = None, sys_dst = None, flags = None, data = None):
        '''Class constructor
        
        The actual data frame to transmit. The data size shall not exceed the MTU of the acoustic modem.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''
        self._seq = seq
        self._sys_dst = sys_dst
        self._flags = flags
        self._data = data


class UamRxFrame(_base.base_message):
    '''The actual received data frame.

       This message class contains the following fields and their respective types:
    sys_src : plaintext, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: URF'''
    
        EMPTY = 0
        '''No active flags'''
    
        PROMISCUOUS = 1
        '''Name: Promiscuous'''
    
        DELAYED = 2
        '''Name: Delayed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_sys_src', '_sys_dst', '_flags', '_data']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 815, stable = None, description = "This message shall be dispatched by acoustic modem drivers each time a data frame is received over the acoustic channel.", fields = ('sys_src', 'sys_dst', 'flags', 'data',), abbrev = "UamRxFrame", name = "UamRxFrame", flags = None)

    sys_src = _base.mutable_attr({'name': 'Source System', 'type': 'plaintext'}, "The canonical name of the node that transmitted the data frame. If this name cannot be resolved the string 'unknown' shall be used.")
    '''The canonical name of the node that transmitted the data frame. If this name cannot be resolved the string 'unknown' shall be used. Type: plaintext'''
    sys_dst = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "The canonical name of the destination node of the data frame. If this name cannot be resolved the string 'unknown' shall be used.")
    '''The canonical name of the destination node of the data frame. If this name cannot be resolved the string 'unknown' shall be used. Type: plaintext'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'URF'}, "Reception flags. Bitfield (Local).")
    '''Reception flags. Bitfield (Local). Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "The actual received data frame.")
    '''The actual received data frame. Type: rawdata'''

    def __init__(self, sys_src = None, sys_dst = None, flags = None, data = None):
        '''Class constructor
        
        The actual received data frame.

       This message class contains the following fields and their respective types:
    sys_src : plaintext, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)

            data : rawdata, unit: NOT FOUND'''
        self._sys_src = sys_src
        self._sys_dst = sys_dst
        self._flags = flags
        self._data = data


class UamTxStatus(_base.base_message):
    '''Where applicable this field shall contain a human-readable message explaining the error.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            value : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''

    class VALUE(_enum.IntEnum):
        '''Full name: Value
        Prefix: UTS'''
    
        DONE = 0
        '''Name: Transmission Completed'''
    
        FAILED = 1
        '''Name: Transmission Failed'''
    
        CANCELED = 2
        '''Name: Transmission Canceled'''
    
        BUSY = 3
        '''Name: Modem is busy'''
    
        INV_ADDR = 4
        '''Name: Invalid address'''
    
        IP = 5
        '''Name: In Progress'''
    
        UNSUPPORTED = 6
        '''Name: Unsupported operation'''
    
        INV_SIZE = 7
        '''Name: Invalid transmission size'''
    
        SENT = 8
        '''Name: Message has been sent'''
    
        DELIVERED = 9
        '''Name: Message has been acknowledged by the destination'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_value', '_error']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 816, stable = None, description = "This message shall be used by acoustic modem drivers to send updates on the transmission status of data frames.", fields = ('seq', 'value', 'error',), abbrev = "UamTxStatus", name = "UamTxStatus", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Id', 'type': 'uint16_t'}, "The sequence identifier of the frame transmission request.")
    '''The sequence identifier of the frame transmission request. Type: uint16_t'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'UTS'}, "Frame transmission status. Enumerated (Local).")
    '''Frame transmission status. Enumerated (Local). Type: uint8_t'''
    error = _base.mutable_attr({'name': 'Error Message', 'type': 'plaintext'}, "Where applicable this field shall contain a human-readable message explaining the error.")
    '''Where applicable this field shall contain a human-readable message explaining the error. Type: plaintext'''

    def __init__(self, seq = None, value = None, error = None):
        '''Class constructor
        
        Where applicable this field shall contain a human-readable message explaining the error.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            value : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''
        self._seq = seq
        self._value = value
        self._error = error


class UamRxRange(_base.base_message):
    '''The actual range. Negative values denote invalid measurements.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys : plaintext, unit: NOT FOUND

            value : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_sys', '_value']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 817, stable = None, description = "Acoustic range measurement.", fields = ('seq', 'sys', 'value',), abbrev = "UamRxRange", name = "UamRxRange", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Id', 'type': 'uint16_t'}, "The sequence identifier of the ranging request.")
    '''The sequence identifier of the ranging request. Type: uint16_t'''
    sys = _base.mutable_attr({'name': 'System', 'type': 'plaintext'}, "The canonical name of the ranged system.")
    '''The canonical name of the ranged system. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'm'}, "The actual range. Negative values denote invalid measurements.")
    '''The actual range. Negative values denote invalid measurements. Type: fp32_t'''

    def __init__(self, seq = None, sys = None, value = None):
        '''Class constructor
        
        The actual range. Negative values denote invalid measurements.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys : plaintext, unit: NOT FOUND

            value : fp32_t, unit: m'''
        self._seq = seq
        self._sys = sys
        self._value = value


class UamTxRange(_base.base_message):
    '''Maximum amount of time to wait for a reply.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            timeout : fp32_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_sys_dst', '_timeout']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 818, stable = None, description = "Request an acoustic modem driver to measure the distance to another system.", fields = ('seq', 'sys_dst', 'timeout',), abbrev = "UamTxRange", name = "UamTxRange", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Id', 'type': 'uint16_t'}, "A sequence identifier that should be incremented for each request. This number will then be used to issue transmission status updates via the message UamTxStatus.")
    '''A sequence identifier that should be incremented for each request. This number will then be used to issue transmission status updates via the message UamTxStatus. Type: uint16_t'''
    sys_dst = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "The canonical name of the target system.")
    '''The canonical name of the target system. Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'fp32_t', 'unit': 's'}, "Maximum amount of time to wait for a reply.")
    '''Maximum amount of time to wait for a reply. Type: fp32_t'''

    def __init__(self, seq = None, sys_dst = None, timeout = None):
        '''Class constructor
        
        Maximum amount of time to wait for a reply.

       This message class contains the following fields and their respective types:
    seq : uint16_t, unit: NOT FOUND

            sys_dst : plaintext, unit: NOT FOUND

            timeout : fp32_t, unit: s'''
        self._seq = seq
        self._sys_dst = sys_dst
        self._timeout = timeout


class UsblAngles(_base.base_message):
    '''Target's elevation.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            bearing : fp32_t, unit: rad

            elevation : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_bearing', '_elevation']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 890, stable = None, description = "This message contains information, collected using USBL, about the bearing and elevation of a target.", fields = ('target', 'bearing', 'elevation',), abbrev = "UsblAngles", name = "USBL Angles", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'uint16_t'}, "Target's IMC address.")
    '''Target's IMC address. Type: uint16_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp32_t', 'unit': 'rad'}, "Target's bearing.")
    '''Target's bearing. Type: fp32_t'''
    elevation = _base.mutable_attr({'name': 'Elevation', 'type': 'fp32_t', 'unit': 'rad'}, "Target's elevation.")
    '''Target's elevation. Type: fp32_t'''

    def __init__(self, target = None, bearing = None, elevation = None):
        '''Class constructor
        
        Target's elevation.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            bearing : fp32_t, unit: rad

            elevation : fp32_t, unit: rad'''
        self._target = target
        self._bearing = bearing
        self._elevation = elevation


class UsblPosition(_base.base_message):
    '''Z coordinate of the target in the local device's reference frame.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 891, stable = None, description = "This message contains information, collected using USBL, about a target's position.", fields = ('target', 'x', 'y', 'z',), abbrev = "UsblPosition", name = "USBL Position", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'uint16_t'}, "Target's IMC address.")
    '''Target's IMC address. Type: uint16_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp32_t', 'unit': 'm'}, "X coordinate of the target in the local device's reference frame.")
    '''X coordinate of the target in the local device's reference frame. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp32_t', 'unit': 'm'}, "Y coordinate of the target in the local device's reference frame.")
    '''Y coordinate of the target in the local device's reference frame. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp32_t', 'unit': 'm'}, "Z coordinate of the target in the local device's reference frame.")
    '''Z coordinate of the target in the local device's reference frame. Type: fp32_t'''

    def __init__(self, target = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z coordinate of the target in the local device's reference frame.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''
        self._target = target
        self._x = x
        self._y = y
        self._z = z


class UsblFix(_base.base_message):
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z_units : uint8_t, unit: Enumerated (Global)

            z : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_lat', '_lon', '_z_units', '_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 892, stable = None, description = "This message contains the WGS-84 position of a target computed using USBL.", fields = ('target', 'lat', 'lon', 'z_units', 'z',), abbrev = "UsblFix", name = "USBL Fix", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'uint16_t'}, "Target's IMC address.")
    '''Target's IMC address. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''

    def __init__(self, target = None, lat = None, lon = None, z_units = None, z = None):
        '''Class constructor
        
        Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.

       This message class contains the following fields and their respective types:
    target : uint16_t, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z_units : uint8_t, unit: Enumerated (Global)

            z : fp32_t, unit: m'''
        self._target = target
        self._lat = lat
        self._lon = lon
        self._z_units = z_units
        self._z = z


class UsblAnglesExtended(_base.base_message):
    '''Accuracy of the fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            lbearing : fp32_t, unit: rad

            lelevation : fp32_t, unit: rad

            bearing : fp32_t, unit: rad

            elevation : fp32_t, unit: rad

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            accuracy : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_lbearing', '_lelevation', '_bearing', '_elevation', '_phi', '_theta', '_psi', '_accuracy']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 898, stable = None, description = "This message contains information, collected using USBL, about the bearing and elevation of a target.", fields = ('target', 'lbearing', 'lelevation', 'bearing', 'elevation', 'phi', 'theta', 'psi', 'accuracy',), abbrev = "UsblAnglesExtended", name = "USBL Angles Extended", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'plaintext'}, "Target's system name.")
    '''Target's system name. Type: plaintext'''
    lbearing = _base.mutable_attr({'name': 'Local Bearing', 'type': 'fp32_t', 'unit': 'rad'}, "Target's bearing in the local device's reference frame.")
    '''Target's bearing in the local device's reference frame. Type: fp32_t'''
    lelevation = _base.mutable_attr({'name': 'Local Elevation', 'type': 'fp32_t', 'unit': 'rad'}, "Target's elevation in the local device's reference frame.")
    '''Target's elevation in the local device's reference frame. Type: fp32_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp32_t', 'unit': 'rad'}, "Target's bearing in the navigation reference frame.")
    '''Target's bearing in the navigation reference frame. Type: fp32_t'''
    elevation = _base.mutable_attr({'name': 'Elevation', 'type': 'fp32_t', 'unit': 'rad'}, "Target's elevation in the navigation reference frame.")
    '''Target's elevation in the navigation reference frame. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Roll Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the device longitudinal axis.")
    '''Rotation around the device longitudinal axis. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Pitch Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267949, 'max': 1.5707963267949}, "Rotation around the device lateral or transverse axis.")
    '''Rotation around the device lateral or transverse axis. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Yaw Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the device vertical axis.")
    '''Rotation around the device vertical axis. Type: fp32_t'''
    accuracy = _base.mutable_attr({'name': 'Accuracy', 'type': 'fp32_t', 'unit': 'rad', 'min': 0, 'max': 3.141592653589793}, "Accuracy of the fix.")
    '''Accuracy of the fix. Type: fp32_t'''

    def __init__(self, target = None, lbearing = None, lelevation = None, bearing = None, elevation = None, phi = None, theta = None, psi = None, accuracy = None):
        '''Class constructor
        
        Accuracy of the fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            lbearing : fp32_t, unit: rad

            lelevation : fp32_t, unit: rad

            bearing : fp32_t, unit: rad

            elevation : fp32_t, unit: rad

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            accuracy : fp32_t, unit: rad'''
        self._target = target
        self._lbearing = lbearing
        self._lelevation = lelevation
        self._bearing = bearing
        self._elevation = elevation
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._accuracy = accuracy


class UsblPositionExtended(_base.base_message):
    '''Accuracy of the position fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            n : fp32_t, unit: m

            e : fp32_t, unit: m

            d : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            accuracy : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_x', '_y', '_z', '_n', '_e', '_d', '_phi', '_theta', '_psi', '_accuracy']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 899, stable = None, description = "This message contains information, collected using USBL, about a target's position.", fields = ('target', 'x', 'y', 'z', 'n', 'e', 'd', 'phi', 'theta', 'psi', 'accuracy',), abbrev = "UsblPositionExtended", name = "USBL Position Extended", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'plaintext'}, "Target's system name.")
    '''Target's system name. Type: plaintext'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp32_t', 'unit': 'm'}, "X coordinate of the target in the local device's reference frame.")
    '''X coordinate of the target in the local device's reference frame. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp32_t', 'unit': 'm'}, "Y coordinate of the target in the local device's reference frame.")
    '''Y coordinate of the target in the local device's reference frame. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp32_t', 'unit': 'm'}, "Z coordinate of the target in the local device's reference frame.")
    '''Z coordinate of the target in the local device's reference frame. Type: fp32_t'''
    n = _base.mutable_attr({'name': 'N', 'type': 'fp32_t', 'unit': 'm'}, "X coordinate of the target in the navigation reference frame.")
    '''X coordinate of the target in the navigation reference frame. Type: fp32_t'''
    e = _base.mutable_attr({'name': 'E', 'type': 'fp32_t', 'unit': 'm'}, "Y coordinate of the target in the navigation reference frame.")
    '''Y coordinate of the target in the navigation reference frame. Type: fp32_t'''
    d = _base.mutable_attr({'name': 'D', 'type': 'fp32_t', 'unit': 'm'}, "Z coordinate of the target in the navigation reference frame.")
    '''Z coordinate of the target in the navigation reference frame. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Roll Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the device longitudinal axis.")
    '''Rotation around the device longitudinal axis. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Pitch Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267949, 'max': 1.5707963267949}, "Rotation around the device lateral or transverse axis.")
    '''Rotation around the device lateral or transverse axis. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Yaw Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the device vertical axis.")
    '''Rotation around the device vertical axis. Type: fp32_t'''
    accuracy = _base.mutable_attr({'name': 'Accuracy', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "Accuracy of the position fix.")
    '''Accuracy of the position fix. Type: fp32_t'''

    def __init__(self, target = None, x = None, y = None, z = None, n = None, e = None, d = None, phi = None, theta = None, psi = None, accuracy = None):
        '''Class constructor
        
        Accuracy of the position fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            n : fp32_t, unit: m

            e : fp32_t, unit: m

            d : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            accuracy : fp32_t, unit: m'''
        self._target = target
        self._x = x
        self._y = y
        self._z = z
        self._n = n
        self._e = e
        self._d = d
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._accuracy = accuracy


class UsblFixExtended(_base.base_message):
    '''Accuracy of the position fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z_units : uint8_t, unit: Enumerated (Global)

            z : fp32_t, unit: m

            accuracy : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_lat', '_lon', '_z_units', '_z', '_accuracy']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Acoustic Communication", id = 900, stable = None, description = "This message contains the WGS-84 position of a target computed using USBL.", fields = ('target', 'lat', 'lon', 'z_units', 'z', 'accuracy',), abbrev = "UsblFixExtended", name = "USBL Fix Extended", flags = None)

    target = _base.mutable_attr({'name': 'Target', 'type': 'plaintext'}, "Target's system name.")
    '''Target's system name. Type: plaintext'''
    lat = _base.mutable_attr({'name': 'Latitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    accuracy = _base.mutable_attr({'name': 'Accuracy', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "Accuracy of the position fix.")
    '''Accuracy of the position fix. Type: fp32_t'''

    def __init__(self, target = None, lat = None, lon = None, z_units = None, z = None, accuracy = None):
        '''Class constructor
        
        Accuracy of the position fix.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z_units : uint8_t, unit: Enumerated (Global)

            z : fp32_t, unit: m

            accuracy : fp32_t, unit: m'''
        self._target = target
        self._lat = lat
        self._lon = lon
        self._z_units = z_units
        self._z = z
        self._accuracy = accuracy


class UsblModem(_base.base_message):
    '''Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_lat', '_lon', '_z', '_z_units']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 901, stable = None, description = "Position and configuration of an Ultra-Short Base Line modem.", fields = ('name', 'lat', 'lon', 'z', 'z_units',), abbrev = "UsblModem", name = "USBL Modem Configuration", flags = None)

    name = _base.mutable_attr({'name': 'Modem Name', 'type': 'plaintext'}, "Name/Label of the acoustic modem.")
    '''Name/Label of the acoustic modem. Type: plaintext'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''

    def __init__(self, name = None, lat = None, lon = None, z = None, z_units = None):
        '''Class constructor
        
        Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)'''
        self._name = name
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units


class UsblConfig(_base.base_message):
    '''A list of USBL modem configuration messages.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            modems : message-list, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        SET_CFG = 0
        '''Name: Set LBL Configuration'''
    
        GET_CFG = 1
        '''Name: Retrieve LBL Configuration'''
    
        CUR_CFG = 2
        '''Name: Reply to a GET command'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_modems']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Acoustic Communication", id = 902, stable = None, description = "Ultra-Short Base Line configuration.", fields = ('op', 'modems',), abbrev = "UsblConfig", name = "USBL Configuration", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Used to define the type of the operation this message holds. Enumerated (Local).")
    '''Used to define the type of the operation this message holds. Enumerated (Local). Type: uint8_t'''
    modems = _base.mutable_attr({'name': 'Modems', 'type': 'message-list', 'message-type': 'UsblModem'}, "A list of USBL modem configuration messages.")
    '''A list of USBL modem configuration messages. Type: message-list'''

    def __init__(self, op = None, modems = None):
        '''Class constructor
        
        A list of USBL modem configuration messages.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            modems : message-list, unit: NOT FOUND'''
        self._op = op
        self._modems = modems


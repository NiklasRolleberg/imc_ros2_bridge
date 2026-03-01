'''
IMC Networking messages.
'''

from .. import _base
import enum as _enum

class Heartbeat(_base.base_message):
    '''The Heartbeat message is used to inform other modules that the sending entity's system is running normally and communications are alive.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 150, stable = None, description = "The Heartbeat message is used to inform other modules that the sending entity's system is running normally and communications are alive.", fields = [], abbrev = "Heartbeat", name = "Heartbeat", flags = "periodic")


    def __init__(self, ):
        '''Class constructor
        
        The Heartbeat message is used to inform other modules that the sending entity's system is running normally and communications are alive.

       This message class contains the following fields and their respective types:
'''


class Announce(_base.base_message):
    '''Semicolon separated list of URLs. Examples of such URLs are: - *imc+udp://192.168.106.34:6002/* - *dune://0.0.0.0/uid/1294925553839635/* - *http://192.168.106.34/dune/*.

       This message class contains the following fields and their respective types:
    sys_name : plaintext, unit: NOT FOUND

            sys_type : uint8_t, unit: Enumerated (Global)

            owner : uint16_t, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            services : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_sys_name', '_sys_type', '_owner', '_lat', '_lon', '_height', '_services']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 151, stable = None, description = "A system description that is to be broadcasted to other systems.", fields = ('sys_name', 'sys_type', 'owner', 'lat', 'lon', 'height', 'services',), abbrev = "Announce", name = "Announce", flags = None)

    sys_name = _base.mutable_attr({'name': 'System Name', 'type': 'plaintext'}, "System name.")
    '''System name. Type: plaintext'''
    sys_type = _base.mutable_attr({'name': 'System Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'SystemType'}, "System type. Enumerated (Global).")
    '''System type. Enumerated (Global). Type: uint8_t'''
    owner = _base.mutable_attr({'name': 'Control Owner', 'type': 'uint16_t'}, "The owner IMC system ID.")
    '''The owner IMC system ID. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude. If lat=0 and lon=0 means location value is unknown.")
    '''WGS-84 Latitude. If lat=0 and lon=0 means location value is unknown. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude. If lat=0 and lon=0 means location value is unknown.")
    '''WGS-84 Longitude. If lat=0 and lon=0 means location value is unknown. Type: fp64_t'''
    height = _base.mutable_attr({'name': 'Height WGS-84', 'type': 'fp32_t', 'unit': 'm'}, "Height above WGS-84 ellipsoid.")
    '''Height above WGS-84 ellipsoid. Type: fp32_t'''
    services = _base.mutable_attr({'name': 'Services', 'type': 'plaintext'}, "Semicolon separated list of URLs. Examples of such URLs are: - *imc+udp://192.168.106.34:6002/* - *dune://0.0.0.0/uid/1294925553839635/* - *http://192.168.106.34/dune/*.")
    '''Semicolon separated list of URLs. Examples of such URLs are: - *imc+udp://192.168.106.34:6002/* - *dune://0.0.0.0/uid/1294925553839635/* - *http://192.168.106.34/dune/*. Type: plaintext'''

    def __init__(self, sys_name = None, sys_type = None, owner = None, lat = None, lon = None, height = None, services = None):
        '''Class constructor
        
        Semicolon separated list of URLs. Examples of such URLs are: - *imc+udp://192.168.106.34:6002/* - *dune://0.0.0.0/uid/1294925553839635/* - *http://192.168.106.34/dune/*.

       This message class contains the following fields and their respective types:
    sys_name : plaintext, unit: NOT FOUND

            sys_type : uint8_t, unit: Enumerated (Global)

            owner : uint16_t, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            services : plaintext, unit: NOT FOUND'''
        self._sys_name = sys_name
        self._sys_type = sys_type
        self._owner = owner
        self._lat = lat
        self._lon = lon
        self._height = height
        self._services = services


class AnnounceService(_base.base_message):
    '''Informs about the availability of the service on internal and external networks. Bitfield (Local).

       This message class contains the following fields and their respective types:
    service : plaintext, unit: NOT FOUND

            service_type : uint8_t, unit: Bitfield (Local)'''

    class SERVICE_TYPE(_enum.IntFlag):
        '''Full name: ServiceType
        Prefix: SRV_TYPE'''
    
        EMPTY = 0
        '''No active flags'''
    
        EXTERNAL = 1
        '''Name: External'''
    
        LOCAL = 2
        '''Name: Local'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_service', '_service_type']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 152, stable = None, description = "Announcement about the existence of a service.", fields = ('service', 'service_type',), abbrev = "AnnounceService", name = "Announce Service", flags = None)

    service = _base.mutable_attr({'name': 'Service', 'type': 'plaintext'}, "Semicolon separated list of URLs (see :ref:`Announce`).")
    '''Semicolon separated list of URLs (see :ref:`Announce`). Type: plaintext'''
    service_type = _base.mutable_attr({'name': 'ServiceType', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'SRV_TYPE'}, "Informs about the availability of the service on internal and external networks. Bitfield (Local).")
    '''Informs about the availability of the service on internal and external networks. Bitfield (Local). Type: uint8_t'''

    def __init__(self, service = None, service_type = None):
        '''Class constructor
        
        Informs about the availability of the service on internal and external networks. Bitfield (Local).

       This message class contains the following fields and their respective types:
    service : plaintext, unit: NOT FOUND

            service_type : uint8_t, unit: Bitfield (Local)'''
        self._service = service
        self._service_type = service_type


class RSSI(_base.base_message):
    '''RSSI measurement.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 153, stable = None, description = "Measure of the RSSI by a networking device. Indicates the gain or loss in the signal strength due to the transmission and reception equipment and the transmission medium and distance.", fields = ('value',), abbrev = "RSSI", name = "Receive Signal Strength Information", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'max': 100, 'unit': '%'}, "RSSI measurement.")
    '''RSSI measurement. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        RSSI measurement.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %'''
        self._value = value


class VSWR(_base.base_message):
    '''VSWR measurement.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 154, stable = None, description = "Measure of the VSWR by a networking device.", fields = ('value',), abbrev = "VSWR", name = "Voltage Standing Wave Ratio", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t'}, "VSWR measurement.")
    '''VSWR measurement. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        VSWR measurement.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''
        self._value = value


class LinkLevel(_base.base_message):
    '''Link level value.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 155, stable = None, description = "Measurement of link level quality. For instance, this may correspond to the acknowledgment ratio of a link. But, generally, the measure is link-dependent.", fields = ('value',), abbrev = "LinkLevel", name = "Link Level", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t'}, "Link level value.")
    '''Link level value. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Link level value.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''
        self._value = value


class Sms(_base.base_message):
    '''Message contents.

       This message class contains the following fields and their respective types:
    number : plaintext, unit: NOT FOUND

            timeout : uint16_t, unit: NOT FOUND

            contents : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_number', '_timeout', '_contents']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 156, stable = None, description = "Send a SMS message.", fields = ('number', 'timeout', 'contents',), abbrev = "Sms", name = "SMS", flags = "deprecated")

    number = _base.mutable_attr({'name': 'Number', 'type': 'plaintext'}, "Target mobile device number.")
    '''Target mobile device number. Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t'}, "Timeout for sending message.")
    '''Timeout for sending message. Type: uint16_t'''
    contents = _base.mutable_attr({'name': 'Contents', 'type': 'plaintext'}, "Message contents.")
    '''Message contents. Type: plaintext'''

    def __init__(self, number = None, timeout = None, contents = None):
        '''Class constructor
        
        Message contents.

       This message class contains the following fields and their respective types:
    number : plaintext, unit: NOT FOUND

            timeout : uint16_t, unit: NOT FOUND

            contents : plaintext, unit: NOT FOUND'''
        self._number = number
        self._timeout = timeout
        self._contents = contents


class SmsTx(_base.base_message):
    '''Message data.

       This message class contains the following fields and their respective types:
    seq : uint32_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : uint16_t, unit: s

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_destination', '_timeout', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 157, stable = None, description = "Request to send SMS.", fields = ('seq', 'destination', 'timeout', 'data',), abbrev = "SmsTx", name = "SMS Transmit", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Number', 'type': 'uint32_t'}, "Sequence number.")
    '''Sequence number. Type: uint32_t'''
    destination = _base.mutable_attr({'name': 'Destination', 'type': 'plaintext'}, "Number or name of the recipient.")
    '''Number or name of the recipient. Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "Timeout for sending message.")
    '''Timeout for sending message. Type: uint16_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Message data.")
    '''Message data. Type: rawdata'''

    def __init__(self, seq = None, destination = None, timeout = None, data = None):
        '''Class constructor
        
        Message data.

       This message class contains the following fields and their respective types:
    seq : uint32_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : uint16_t, unit: s

            data : rawdata, unit: NOT FOUND'''
        self._seq = seq
        self._destination = destination
        self._timeout = timeout
        self._data = data


class SmsRx(_base.base_message):
    '''Message data.

       This message class contains the following fields and their respective types:
    source : plaintext, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_source', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 158, stable = None, description = "Received SMS data.", fields = ('source', 'data',), abbrev = "SmsRx", name = "SMS Receive", flags = None)

    source = _base.mutable_attr({'name': 'Source', 'type': 'plaintext'}, "Number of name of the sender.")
    '''Number of name of the sender. Type: plaintext'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Message data.")
    '''Message data. Type: rawdata'''

    def __init__(self, source = None, data = None):
        '''Class constructor
        
        Message data.

       This message class contains the following fields and their respective types:
    source : plaintext, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''
        self._source = source
        self._data = data


class SmsState(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    seq : uint32_t, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: SMS'''
    
        ACCEPTED = 0
        '''Name: Accepted'''
    
        REJECTED = 1
        '''Name: Rejected'''
    
        INTERRUPTED = 2
        '''Name: Interrupted'''
    
        COMPLETED = 3
        '''Name: Completed'''
    
        IDLE = 4
        '''Name: Idle'''
    
        TRANSMITTING = 5
        '''Name: Transmitting'''
    
        RECEIVING = 6
        '''Name: Receiving'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_seq', '_state', '_error']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 159, stable = None, description = None, fields = ('seq', 'state', 'error',), abbrev = "SmsState", name = "SMS State", flags = None)

    seq = _base.mutable_attr({'name': 'Sequence Number', 'type': 'uint32_t'}, "Sequence number.")
    '''Sequence number. Type: uint32_t'''
    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'SMS'}, "Current state of an SMS transaction. Enumerated (Local).")
    '''Current state of an SMS transaction. Enumerated (Local). Type: uint8_t'''
    error = _base.mutable_attr({'name': 'Error Message', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, seq = None, state = None, error = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    seq : uint32_t, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''
        self._seq = seq
        self._state = state
        self._error = error


class TextMessage(_base.base_message):
    '''Message contents.

       This message class contains the following fields and their respective types:
    origin : plaintext, unit: NOT FOUND

            text : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_origin', '_text']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 160, stable = None, description = "A text message has been received.", fields = ('origin', 'text',), abbrev = "TextMessage", name = "Text Message", flags = None)

    origin = _base.mutable_attr({'name': 'Origin', 'type': 'plaintext'}, "Message origin (if known).")
    '''Message origin (if known). Type: plaintext'''
    text = _base.mutable_attr({'name': 'Text', 'type': 'plaintext'}, "Message contents.")
    '''Message contents. Type: plaintext'''

    def __init__(self, origin = None, text = None):
        '''Class constructor
        
        Message contents.

       This message class contains the following fields and their respective types:
    origin : plaintext, unit: NOT FOUND

            text : plaintext, unit: NOT FOUND'''
        self._origin = origin
        self._text = text


class IridiumMsgRx(_base.base_message):
    '''Message data.

       This message class contains the following fields and their respective types:
    origin : plaintext, unit: NOT FOUND

            htime : fp64_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_origin', '_htime', '_lat', '_lon', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 170, stable = None, description = None, fields = ('origin', 'htime', 'lat', 'lon', 'data',), abbrev = "IridiumMsgRx", name = "Received Iridium Message", flags = None)

    origin = _base.mutable_attr({'name': 'Origin Identifier', 'type': 'plaintext'}, "The unique identifier of this message's origin device (e.g. lauv-xtreme-2, manta-0).")
    '''The unique identifier of this message's origin device (e.g. lauv-xtreme-2, manta-0). Type: plaintext'''
    htime = _base.mutable_attr({'name': 'Timestamp', 'type': 'fp64_t', 'unit': 's'}, "Timestamp (Epoch time).")
    '''Timestamp (Epoch time). Type: fp64_t'''
    lat = _base.mutable_attr({'name': 'Latitude Reference', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude Reference', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Message data.")
    '''Message data. Type: rawdata'''

    def __init__(self, origin = None, htime = None, lat = None, lon = None, data = None):
        '''Class constructor
        
        Message data.

       This message class contains the following fields and their respective types:
    origin : plaintext, unit: NOT FOUND

            htime : fp64_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            data : rawdata, unit: NOT FOUND'''
        self._origin = origin
        self._htime = htime
        self._lat = lat
        self._lon = lon
        self._data = data


class IridiumMsgTx(_base.base_message):
    '''Message data.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            ttl : uint16_t, unit: s

            destination : plaintext, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_ttl', '_destination', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 171, stable = None, description = None, fields = ('req_id', 'ttl', 'destination', 'data',), abbrev = "IridiumMsgTx", name = "Transmit Iridium Message", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "The request identifier used to receive transmission updates.")
    '''The request identifier used to receive transmission updates. Type: uint16_t'''
    ttl = _base.mutable_attr({'name': 'Time to live', 'type': 'uint16_t', 'unit': 's'}, "Time, in seconds, after which there will be no more atempts to transmit the message.")
    '''Time, in seconds, after which there will be no more atempts to transmit the message. Type: uint16_t'''
    destination = _base.mutable_attr({'name': 'Destination Identifier', 'type': 'plaintext'}, "The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0).")
    '''The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0). Type: plaintext'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Message data.")
    '''Message data. Type: rawdata'''

    def __init__(self, req_id = None, ttl = None, destination = None, data = None):
        '''Class constructor
        
        Message data.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            ttl : uint16_t, unit: s

            destination : plaintext, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''
        self._req_id = req_id
        self._ttl = ttl
        self._destination = destination
        self._data = data


class IridiumTxStatus(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''

    class STATUS(_enum.IntEnum):
        '''Full name: Status Code
        Prefix: TXSTATUS'''
    
        OK = 1
        '''Name: Successfull transmission'''
    
        ERROR = 2
        '''Name: Error while trying to transmit message'''
    
        QUEUED = 3
        '''Name: Message has been queued for transmission'''
    
        TRANSMIT = 4
        '''Name: Message is currently being transmitted'''
    
        EXPIRED = 5
        '''Name: Message's TTL has expired. Transmition cancelled.'''
    
        EMPTY = 6
        '''Name: No more messages to be transmitted or received.'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_status', '_text']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Networking", id = 172, stable = None, description = None, fields = ('req_id', 'status', 'text',), abbrev = "IridiumTxStatus", name = "Iridium Transmission Status", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "The request identifier used to receive transmission updates")
    '''The request identifier used to receive transmission updates Type: uint16_t'''
    status = _base.mutable_attr({'name': 'Status Code', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TXSTATUS'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    text = _base.mutable_attr({'name': 'Status Text', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, req_id = None, status = None, text = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''
        self._req_id = req_id
        self._status = status
        self._text = text


class GroupMembershipState(_base.base_message):
    '''Communication link assertion for each group member. One bit to assert each system communication link state.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            links : uint32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_group_name', '_links']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 180, stable = None, description = "Group communication link assertion.", fields = ('group_name', 'links',), abbrev = "GroupMembershipState", name = "Group Membership State", flags = "periodic")

    group_name = _base.mutable_attr({'name': 'Group Name', 'type': 'plaintext'}, "Name of the group of systems.")
    '''Name of the group of systems. Type: plaintext'''
    links = _base.mutable_attr({'name': 'Communication Links Assertion', 'type': 'uint32_t'}, "Communication link assertion for each group member. One bit to assert each system communication link state.")
    '''Communication link assertion for each group member. One bit to assert each system communication link state. Type: uint32_t'''

    def __init__(self, group_name = None, links = None):
        '''Class constructor
        
        Communication link assertion for each group member. One bit to assert each system communication link state.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            links : uint32_t, unit: NOT FOUND'''
        self._group_name = group_name
        self._links = links


class SystemGroup(_base.base_message):
    '''List of names of system in the group, separated by commas.

       This message class contains the following fields and their respective types:
    GroupName : plaintext, unit: NOT FOUND

            Action : uint8_t, unit: Enumerated (Local)

            GroupList : plaintext, unit: NOT FOUND'''

    class ACTION(_enum.IntEnum):
        '''Full name: Group List Action
        Prefix: OP'''
    
        Dis = 0
        '''Name: Disband'''
    
        Set = 1
        '''Name: Set'''
    
        Req = 2
        '''Name: Request'''
    
        Chg = 3
        '''Name: Change'''
    
        Rep = 4
        '''Name: Report'''
    
        Frc = 5
        '''Name: Force'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_GroupName', '_Action', '_GroupList']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 181, stable = None, description = "Group of systems configuration.", fields = ('GroupName', 'Action', 'GroupList',), abbrev = "SystemGroup", name = "System Group", flags = None)

    GroupName = _base.mutable_attr({'name': 'Group Name', 'type': 'plaintext'}, "Name of the group of systems.")
    '''Name of the group of systems. Type: plaintext'''
    Action = _base.mutable_attr({'name': 'Group List Action', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Actions on the group list. Enumerated (Local).")
    '''Actions on the group list. Enumerated (Local). Type: uint8_t'''
    GroupList = _base.mutable_attr({'name': 'Systems Name List', 'type': 'plaintext'}, "List of names of system in the group, separated by commas.")
    '''List of names of system in the group, separated by commas. Type: plaintext'''

    def __init__(self, GroupName = None, Action = None, GroupList = None):
        '''Class constructor
        
        List of names of system in the group, separated by commas.

       This message class contains the following fields and their respective types:
    GroupName : plaintext, unit: NOT FOUND

            Action : uint8_t, unit: Enumerated (Local)

            GroupList : plaintext, unit: NOT FOUND'''
        self._GroupName = GroupName
        self._Action = Action
        self._GroupList = GroupList


class LinkLatency(_base.base_message):
    '''ID of system that was the source of the communication package.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: s

            sys_src : uint16_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_sys_src']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 182, stable = None, description = "Communications latency between two systems.", fields = ('value', 'sys_src',), abbrev = "LinkLatency", name = "Link Latency", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 's', 'min': 0}, "Time taken between the communications package/message is sent from the source until it arrives to the destination.")
    '''Time taken between the communications package/message is sent from the source until it arrives to the destination. Type: fp32_t'''
    sys_src = _base.mutable_attr({'name': 'Communications Source System ID', 'type': 'uint16_t'}, "ID of system that was the source of the communication package.")
    '''ID of system that was the source of the communication package. Type: uint16_t'''

    def __init__(self, value = None, sys_src = None):
        '''Class constructor
        
        ID of system that was the source of the communication package.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: s

            sys_src : uint16_t, unit: NOT FOUND'''
        self._value = value
        self._sys_src = sys_src


class ExtendedRSSI(_base.base_message):
    '''Indicates the units used for the RSSI value. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND

            units : uint8_t, unit: Enumerated (Global)'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_units']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Networking", id = 183, stable = None, description = "Measure of the RSSI by a networking device. Indicates the gain or loss in the signal strenght due to the transmission and reception equipment and the transmission medium and distance.", fields = ('value', 'units',), abbrev = "ExtendedRSSI", name = "Extended Receive Signal Strength Information", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t'}, "RSSI measurement.")
    '''RSSI measurement. Type: fp32_t'''
    units = _base.mutable_attr({'name': 'RSSI Units', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'RSSIUnits'}, "Indicates the units used for the RSSI value. Enumerated (Global).")
    '''Indicates the units used for the RSSI value. Enumerated (Global). Type: uint8_t'''

    def __init__(self, value = None, units = None):
        '''Class constructor
        
        Indicates the units used for the RSSI value. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND

            units : uint8_t, unit: Enumerated (Global)'''
        self._value = value
        self._units = units


class HistoricData(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    base_lat : fp32_t, unit: °

            base_lon : fp32_t, unit: °

            base_time : fp32_t, unit: s

            data : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_base_lat', '_base_lon', '_base_time', '_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 184, stable = None, description = "This message holds a list of inline data samples produced by one or more vehicles in the past. It is used to transfer data over disruption tolerant networks.", fields = ('base_lat', 'base_lon', 'base_time', 'data',), abbrev = "HistoricData", name = "Historic Data Series", flags = None)

    base_lat = _base.mutable_attr({'name': 'Base Latitude', 'type': 'fp32_t', 'unit': '°'}, "All data sent inside this message will have offsets relative to this latitude.")
    '''All data sent inside this message will have offsets relative to this latitude. Type: fp32_t'''
    base_lon = _base.mutable_attr({'name': 'Base Longitude', 'type': 'fp32_t', 'unit': '°'}, "All data sent inside this message will have offsets relative to this longitude.")
    '''All data sent inside this message will have offsets relative to this longitude. Type: fp32_t'''
    base_time = _base.mutable_attr({'name': 'Base Timestamp', 'type': 'fp32_t', 'unit': 's'}, "All data sent inside this message will use this time as the origin (0).")
    '''All data sent inside this message will use this time as the origin (0). Type: fp32_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'message-list', 'message-type': 'RemoteData'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, base_lat = None, base_lon = None, base_time = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    base_lat : fp32_t, unit: °

            base_lon : fp32_t, unit: °

            base_time : fp32_t, unit: s

            data : message-list, unit: NOT FOUND'''
        self._base_lat = base_lat
        self._base_lon = base_lon
        self._base_time = base_time
        self._data = data


class CompressedHistory(_base.base_message):
    '''A message-list of HistoricSample messages compressed with GZip algorithm.

       This message class contains the following fields and their respective types:
    base_lat : fp32_t, unit: °

            base_lon : fp32_t, unit: °

            base_time : fp32_t, unit: s

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_base_lat', '_base_lon', '_base_time', '_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 185, stable = None, description = "This message holds a list of inline data samples produced by one or more vehicles in the past. It is used to transfer data over disruption tolerant networks.", fields = ('base_lat', 'base_lon', 'base_time', 'data',), abbrev = "CompressedHistory", name = "Compressed Historic Data Series", flags = None)

    base_lat = _base.mutable_attr({'name': 'Base Latitude', 'type': 'fp32_t', 'unit': '°'}, "All data sent inside this message will have offsets relative to this latitude.")
    '''All data sent inside this message will have offsets relative to this latitude. Type: fp32_t'''
    base_lon = _base.mutable_attr({'name': 'Base Longitude', 'type': 'fp32_t', 'unit': '°'}, "All data sent inside this message will have offsets relative to this longitude.")
    '''All data sent inside this message will have offsets relative to this longitude. Type: fp32_t'''
    base_time = _base.mutable_attr({'name': 'Base Timestamp', 'type': 'fp32_t', 'unit': 's'}, "All data sent inside this message will use this time as the origin (0).")
    '''All data sent inside this message will use this time as the origin (0). Type: fp32_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "A message-list of HistoricSample messages compressed with GZip algorithm.")
    '''A message-list of HistoricSample messages compressed with GZip algorithm. Type: rawdata'''

    def __init__(self, base_lat = None, base_lon = None, base_time = None, data = None):
        '''Class constructor
        
        A message-list of HistoricSample messages compressed with GZip algorithm.

       This message class contains the following fields and their respective types:
    base_lat : fp32_t, unit: °

            base_lon : fp32_t, unit: °

            base_time : fp32_t, unit: s

            data : rawdata, unit: NOT FOUND'''
        self._base_lat = base_lat
        self._base_lon = base_lon
        self._base_time = base_time
        self._data = data


class HistoricSample(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    sys_id : uint16_t, unit: NOT FOUND

            priority : int8_t, unit: NOT FOUND

            x : int16_t, unit: m

            y : int16_t, unit: m

            z : int16_t, unit: dm

            t : int16_t, unit: s

            sample : message, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_sys_id', '_priority', '_x', '_y', '_z', '_t', '_sample']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 186, stable = None, description = None, fields = ('sys_id', 'priority', 'x', 'y', 'z', 't', 'sample',), abbrev = "HistoricSample", name = "Historic Data Sample", flags = None)

    sys_id = _base.mutable_attr({'name': 'Original System Id', 'type': 'uint16_t'}, "The IMC identifier of the system that produced this sample.")
    '''The IMC identifier of the system that produced this sample. Type: uint16_t'''
    priority = _base.mutable_attr({'name': 'Priority', 'type': 'int8_t'}, "The priority for this data sample. Default priority is 0. Samples with higher priorities will *always* be transmitted before samples with lower priorities. Samples with -127 priority will not be transmitted but just logged to disk locally.")
    '''The priority for this data sample. Default priority is 0. Samples with higher priorities will *always* be transmitted before samples with lower priorities. Samples with -127 priority will not be transmitted but just logged to disk locally. Type: int8_t'''
    x = _base.mutable_attr({'name': 'X offset', 'type': 'int16_t', 'unit': 'm'}, "Northing offsets relative to base latitude / longitude expressed in the enclosing `HistoricData` message.")
    '''Northing offsets relative to base latitude / longitude expressed in the enclosing `HistoricData` message. Type: int16_t'''
    y = _base.mutable_attr({'name': 'Y offset', 'type': 'int16_t', 'unit': 'm'}, "Easting offsets relative to base latitude / longitude expressed in the enclosing `HistoricData` message.")
    '''Easting offsets relative to base latitude / longitude expressed in the enclosing `HistoricData` message. Type: int16_t'''
    z = _base.mutable_attr({'name': 'Z offset', 'type': 'int16_t', 'unit': 'dm'}, "Altitude / depth offsets relative to sea level expressed in decimeters. Negative values mean depth and positive values mean altitude.")
    '''Altitude / depth offsets relative to sea level expressed in decimeters. Negative values mean depth and positive values mean altitude. Type: int16_t'''
    t = _base.mutable_attr({'name': 'Time offset', 'type': 'int16_t', 'unit': 's'}, "Time offset in seconds relative to the base time expressed in the enclosing `HistoricData` message.")
    '''Time offset in seconds relative to the base time expressed in the enclosing `HistoricData` message. Type: int16_t'''
    sample = _base.mutable_attr({'name': 'Data Sample', 'type': 'message'}, "No description available")
    '''No description available Type: message'''

    def __init__(self, sys_id = None, priority = None, x = None, y = None, z = None, t = None, sample = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    sys_id : uint16_t, unit: NOT FOUND

            priority : int8_t, unit: NOT FOUND

            x : int16_t, unit: m

            y : int16_t, unit: m

            z : int16_t, unit: dm

            t : int16_t, unit: s

            sample : message, unit: NOT FOUND'''
        self._sys_id = sys_id
        self._priority = priority
        self._x = x
        self._y = y
        self._z = z
        self._t = t
        self._sample = sample


class HistoricDataQuery(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            max_size : uint16_t, unit: NOT FOUND

            data : message, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Request Type
        Prefix: HRTYPE'''
    
        QUERY = 1
        '''Name: Query'''
    
        REPLY = 2
        '''Name: Reply'''
    
        CLEAR = 3
        '''Name: Clear'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_type', '_max_size', '_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 187, stable = None, description = None, fields = ('req_id', 'type', 'max_size', 'data',), abbrev = "HistoricDataQuery", name = "Historic Data Query", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Id', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    type = _base.mutable_attr({'name': 'Request Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'HRTYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    max_size = _base.mutable_attr({'name': 'Maximum Size', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'message', 'message-type': 'HistoricData'}, "No description available")
    '''No description available Type: message'''

    def __init__(self, req_id = None, type = None, max_size = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            max_size : uint16_t, unit: NOT FOUND

            data : message, unit: NOT FOUND'''
        self._req_id = req_id
        self._type = type
        self._max_size = max_size
        self._data = data


class RemoteCommand(_base.base_message):
    '''Command to be unpacked by the recipient.

       This message class contains the following fields and their respective types:
    original_source : uint16_t, unit: NOT FOUND

            destination : uint16_t, unit: NOT FOUND

            timeout : fp64_t, unit: s

            cmd : message, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_original_source', '_destination', '_timeout', '_cmd']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 188, stable = None, description = "Command to remote system. If a system receives a RemoteCommand and it isn't the intended recipient, then it should resend it.", fields = ('original_source', 'destination', 'timeout', 'cmd',), abbrev = "RemoteCommand", name = "Remote Command", flags = None)

    original_source = _base.mutable_attr({'name': 'Original Source', 'type': 'uint16_t'}, "IMC id of the original sender.")
    '''IMC id of the original sender. Type: uint16_t'''
    destination = _base.mutable_attr({'name': 'Destination', 'type': 'uint16_t'}, "IMC id of the recipient.")
    '''IMC id of the recipient. Type: uint16_t'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'fp64_t', 'unit': 's'}, "Expiration time of the message (Epoch Time), in seconds. If the message doesn't reach the destination within timeout, the validity of the message expires and there will be no more attempts to transmit the message.")
    '''Expiration time of the message (Epoch Time), in seconds. If the message doesn't reach the destination within timeout, the validity of the message expires and there will be no more attempts to transmit the message. Type: fp64_t'''
    cmd = _base.mutable_attr({'name': 'Command', 'type': 'message'}, "Command to be unpacked by the recipient.")
    '''Command to be unpacked by the recipient. Type: message'''

    def __init__(self, original_source = None, destination = None, timeout = None, cmd = None):
        '''Class constructor
        
        Command to be unpacked by the recipient.

       This message class contains the following fields and their respective types:
    original_source : uint16_t, unit: NOT FOUND

            destination : uint16_t, unit: NOT FOUND

            timeout : fp64_t, unit: s

            cmd : message, unit: NOT FOUND'''
        self._original_source = original_source
        self._destination = destination
        self._timeout = timeout
        self._cmd = cmd


class CommSystemsQuery(_base.base_message):
    '''Comma separated list of known Radio system names.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Bitfield (Local)

            comm_interface : uint16_t, unit: Bitfield (Local)

            model : uint16_t, unit: Enumerated (Local)

            list : plaintext, unit: List'''

    class TYPE(_enum.IntFlag):
        '''Full name: Type
        Prefix: CIQ'''
    
        EMPTY = 0
        '''No active flags'''
    
        QUERY = 1
        '''Name: Query Systems'''
    
        REPLY = 2
        '''Name: Reply'''
    
    
    class COMM_INTERFACE(_enum.IntFlag):
        '''Full name: Communication Interface
        Prefix: CIQ'''
    
        EMPTY = 0
        '''No active flags'''
    
        ACOUSTIC = 1
        '''Name: Acoustic'''
    
        SATELLITE = 2
        '''Name: Satellite'''
    
        GSM = 4
        '''Name: GSM'''
    
        MOBILE = 8
        '''Name: Mobile'''
    
        RADIO = 16
        '''Name: Radio'''
    
    
    class MODEL(_enum.IntEnum):
        '''Full name: Model
        Prefix: CIQ'''
    
        UNKNOWN = 0
        '''Name: unknown'''
    
        M3DR = 1
        '''Name: 3DR'''
    
        RDFXXXXPTP = 2
        '''Name: RDFXXXxPtP'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_comm_interface', '_model', '_list']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 189, stable = None, description = "Presence of Communication Interfaces query.", fields = ('type', 'comm_interface', 'model', 'list',), abbrev = "CommSystemsQuery", name = "Communication Systems Query", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'CIQ'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    comm_interface = _base.mutable_attr({'name': 'Communication Interface', 'type': 'uint16_t', 'unit': 'Bitfield', 'prefix': 'CIQ'}, "Communication interface to be used for reports. Bitfield (Local).")
    '''Communication interface to be used for reports. Bitfield (Local). Type: uint16_t'''
    model = _base.mutable_attr({'name': 'Model', 'type': 'uint16_t', 'unit': 'Enumerated', 'prefix': 'CIQ'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint16_t'''
    list = _base.mutable_attr({'name': 'System List', 'type': 'plaintext', 'unit': 'List'}, "Comma separated list of known Radio system names.")
    '''Comma separated list of known Radio system names. Type: plaintext'''

    def __init__(self, type = None, comm_interface = None, model = None, list = None):
        '''Class constructor
        
        Comma separated list of known Radio system names.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Bitfield (Local)

            comm_interface : uint16_t, unit: Bitfield (Local)

            model : uint16_t, unit: Enumerated (Local)

            list : plaintext, unit: List'''
        self._type = type
        self._comm_interface = comm_interface
        self._model = model
        self._list = list


class TelemetryMsg(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            req_id : uint32_t, unit: NOT FOUND

            ttl : uint16_t, unit: s

            code : uint8_t, unit: Enumerated (Local)

            destination : plaintext, unit: NOT FOUND

            Source : plaintext, unit: NOT FOUND

            acknowledge : uint8_t, unit: Bitfield (Local)

            status : uint8_t, unit: Enumerated (Local)

            data : rawdata, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: TM'''
    
        TX = 1
        '''Name: Tx'''
    
        RX = 2
        '''Name: Rx'''
    
        TXSTATUS = 3
        '''Name: TxStatus'''
    
    
    class CODE(_enum.IntEnum):
        '''Full name: Code
        Prefix: TM'''
    
        CODE_UNK = 0
        '''Name: Code unknown'''
    
        CODE_REPORT = 1
        '''Name: Code Report'''
    
        CODE_IMC = 2
        '''Name: Code IMC'''
    
        CODE_RAW = 3
        '''Name: Code raw'''
    
    
    class ACKNOWLEDGE(_enum.IntFlag):
        '''Full name: Acknowledge
        Prefix: TM'''
    
        EMPTY = 0
        '''No active flags'''
    
        NAK = 0
        '''Name: Not acknowledge'''
    
        AK = 1
        '''Name: acknowledge'''
    
    
    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: TM'''
    
        NONE = 0
        '''Name: Does not apply'''
    
        DONE = 1
        '''Name: Successfull transmission'''
    
        FAILED = 2
        '''Name: Error while trying to transmit message'''
    
        QUEUED = 3
        '''Name: Message has been queued for transmission'''
    
        TRANSMIT = 4
        '''Name: Message is currently being transmitted'''
    
        EXPIRED = 5
        '''Name: Message's TTL has expired. Transmition cancelled'''
    
        EMPTY = 6
        '''Name: No more messages to be transmitted or received'''
    
        INV_ADDR = 7
        '''Name: Invalid address'''
    
        INV_SIZE = 8
        '''Name: Invalid transmission size'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_req_id', '_ttl', '_code', '_destination', '_Source', '_acknowledge', '_status', '_data']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 190, stable = None, description = "Message to handle telemetry transmissions.", fields = ('type', 'req_id', 'ttl', 'code', 'destination', 'Source', 'acknowledge', 'status', 'data',), abbrev = "TelemetryMsg", name = "Telemetry Message", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TM'}, "Type of telemetry transmissions. Enumerated (Local).")
    '''Type of telemetry transmissions. Enumerated (Local). Type: uint8_t'''
    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint32_t'}, "The request identifier used to receive transmission updates.")
    '''The request identifier used to receive transmission updates. Type: uint32_t'''
    ttl = _base.mutable_attr({'name': 'Time to live', 'type': 'uint16_t', 'unit': 's'}, "Time, in seconds, which will be considered a non-transmitted message.")
    '''Time, in seconds, which will be considered a non-transmitted message. Type: uint16_t'''
    code = _base.mutable_attr({'name': 'Code', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TM'}, "Type of telemetry transmissions. Enumerated (Local).")
    '''Type of telemetry transmissions. Enumerated (Local). Type: uint8_t'''
    destination = _base.mutable_attr({'name': 'Destination Identifier', 'type': 'plaintext'}, "The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0).")
    '''The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0). Type: plaintext'''
    Source = _base.mutable_attr({'name': 'Source Identifier', 'type': 'plaintext'}, "The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0).")
    '''The unique identifier of this message's destination (e.g. lauv-xtreme-2, manta-0). Type: plaintext'''
    acknowledge = _base.mutable_attr({'name': 'Acknowledge', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'TM'}, "Type of telemetry transmissions. Bitfield (Local).")
    '''Type of telemetry transmissions. Bitfield (Local). Type: uint8_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TM'}, "State of the transmitted message. Enumerated (Local).")
    '''State of the transmitted message. Enumerated (Local). Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "No description available")
    '''No description available Type: rawdata'''

    def __init__(self, type = None, req_id = None, ttl = None, code = None, destination = None, Source = None, acknowledge = None, status = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            req_id : uint32_t, unit: NOT FOUND

            ttl : uint16_t, unit: s

            code : uint8_t, unit: Enumerated (Local)

            destination : plaintext, unit: NOT FOUND

            Source : plaintext, unit: NOT FOUND

            acknowledge : uint8_t, unit: Bitfield (Local)

            status : uint8_t, unit: Enumerated (Local)

            data : rawdata, unit: NOT FOUND'''
        self._type = type
        self._req_id = req_id
        self._ttl = ttl
        self._code = code
        self._destination = destination
        self._Source = Source
        self._acknowledge = acknowledge
        self._status = status
        self._data = data


class TransmissionRequest(_base.base_message):
    '''Data to be transmitted if selected *data_mode* is *RAW*.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            comm_mean : uint8_t, unit: Enumerated (Local)

            destination : plaintext, unit: NOT FOUND

            deadline : fp64_t, unit: NOT FOUND

            range : fp32_t, unit: m

            data_mode : uint8_t, unit: Enumerated (Local)

            msg_data : message, unit: NOT FOUND

            txt_data : plaintext, unit: NOT FOUND

            raw_data : rawdata, unit: NOT FOUND'''

    class COMM_MEAN(_enum.IntEnum):
        '''Full name: Communication Mean
        Prefix: CMEAN'''
    
        WIFI = 0
        '''Name: WiFi'''
    
        ACOUSTIC = 1
        '''Name: Acoustic'''
    
        SATELLITE = 2
        '''Name: Satellite'''
    
        GSM = 3
        '''Name: GSM'''
    
        ANY = 4
        '''Name: Any'''
    
        ALL = 5
        '''Name: All'''
    
    
    class DATA_MODE(_enum.IntEnum):
        '''Full name: Data Mode
        Prefix: DMODE'''
    
        INLINEMSG = 0
        '''Name: Inline Message'''
    
        TEXT = 1
        '''Name: Text'''
    
        RAW = 2
        '''Name: Raw Data'''
    
        ABORT = 3
        '''Name: Abort'''
    
        RANGE = 4
        '''Name: Range'''
    
        REVERSE_RANGE = 5
        '''Name: Reverse Range'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_comm_mean', '_destination', '_deadline', '_range', '_data_mode', '_msg_data', '_txt_data', '_raw_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 515, stable = None, description = "Request data to be sent over a specified communication mean.", fields = ('req_id', 'comm_mean', 'destination', 'deadline', 'range', 'data_mode', 'msg_data', 'txt_data', 'raw_data',), abbrev = "TransmissionRequest", name = "Transmission Request", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "The unique identifier for this request.")
    '''The unique identifier for this request. Type: uint16_t'''
    comm_mean = _base.mutable_attr({'name': 'Communication Mean', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'CMEAN'}, "Communication mean to be used to transfer these data. Enumerated (Local).")
    '''Communication mean to be used to transfer these data. Enumerated (Local). Type: uint8_t'''
    destination = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "The name of the system where to send this message.")
    '''The name of the system where to send this message. Type: plaintext'''
    deadline = _base.mutable_attr({'name': 'Deadline', 'type': 'fp64_t'}, "Deadline for message transmission (seconds since epoch).")
    '''Deadline for message transmission (seconds since epoch). Type: fp64_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: fp32_t'''
    data_mode = _base.mutable_attr({'name': 'Data Mode', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DMODE'}, "Type of data to be transmitted. Abort and Range mode can only be used with comm_mean=ACOUSTIC Enumerated (Local).")
    '''Type of data to be transmitted. Abort and Range mode can only be used with comm_mean=ACOUSTIC Enumerated (Local). Type: uint8_t'''
    msg_data = _base.mutable_attr({'name': 'Message Data', 'type': 'message'}, "Data to be transmitted if selected *data_mode* is *INLINEMSG*.")
    '''Data to be transmitted if selected *data_mode* is *INLINEMSG*. Type: message'''
    txt_data = _base.mutable_attr({'name': 'Text Data', 'type': 'plaintext'}, "Data to be transmitted if selected *data_mode* is *TEXT*.")
    '''Data to be transmitted if selected *data_mode* is *TEXT*. Type: plaintext'''
    raw_data = _base.mutable_attr({'name': 'Raw Data', 'type': 'rawdata'}, "Data to be transmitted if selected *data_mode* is *RAW*.")
    '''Data to be transmitted if selected *data_mode* is *RAW*. Type: rawdata'''

    def __init__(self, req_id = None, comm_mean = None, destination = None, deadline = None, range = None, data_mode = None, msg_data = None, txt_data = None, raw_data = None):
        '''Class constructor
        
        Data to be transmitted if selected *data_mode* is *RAW*.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            comm_mean : uint8_t, unit: Enumerated (Local)

            destination : plaintext, unit: NOT FOUND

            deadline : fp64_t, unit: NOT FOUND

            range : fp32_t, unit: m

            data_mode : uint8_t, unit: Enumerated (Local)

            msg_data : message, unit: NOT FOUND

            txt_data : plaintext, unit: NOT FOUND

            raw_data : rawdata, unit: NOT FOUND'''
        self._req_id = req_id
        self._comm_mean = comm_mean
        self._destination = destination
        self._deadline = deadline
        self._range = range
        self._data_mode = data_mode
        self._msg_data = msg_data
        self._txt_data = txt_data
        self._raw_data = raw_data


class TransmissionStatus(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            range : fp32_t, unit: m

            info : plaintext, unit: NOT FOUND'''

    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: TSTAT'''
    
        IN_PROGRESS = 0
        '''Name: In progress'''
    
        SENT = 1
        '''Name: Sent'''
    
        DELIVERED = 51
        '''Name: Delivered'''
    
        MAYBE_DELIVERED = 52
        '''Name: Delivery is unknown'''
    
        RANGE_RECEIVED = 60
        '''Name: Range received'''
    
        INPUT_FAILURE = 101
        '''Name: Input Error'''
    
        TEMPORARY_FAILURE = 102
        '''Name: Temporary Error'''
    
        PERMANENT_FAILURE = 103
        '''Name: Permanent Failure'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_status', '_range', '_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 516, stable = None, description = "Reply sent in response to a communications request.", fields = ('req_id', 'status', 'range', 'info',), abbrev = "TransmissionStatus", name = "Transmission Status", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TSTAT'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: fp32_t'''
    info = _base.mutable_attr({'name': 'Information', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, req_id = None, status = None, range = None, info = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            range : fp32_t, unit: m

            info : plaintext, unit: NOT FOUND'''
        self._req_id = req_id
        self._status = status
        self._range = range
        self._info = info


class SmsRequest(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            sms_text : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_destination', '_timeout', '_sms_text']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 517, stable = None, description = "Request SMS Text sending.", fields = ('req_id', 'destination', 'timeout', 'sms_text',), abbrev = "SmsRequest", name = "SMS Transmission Request", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    destination = _base.mutable_attr({'name': 'Destination', 'type': 'plaintext'}, "Recipient identifier (number or name).")
    '''Recipient identifier (number or name). Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'fp64_t', 'unit': 's'}, "Period of time to send message (in seconds).")
    '''Period of time to send message (in seconds). Type: fp64_t'''
    sms_text = _base.mutable_attr({'name': 'SMS Text', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, req_id = None, destination = None, timeout = None, sms_text = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            sms_text : plaintext, unit: NOT FOUND'''
        self._req_id = req_id
        self._destination = destination
        self._timeout = timeout
        self._sms_text = sms_text


class SmsStatus(_base.base_message):
    '''Error description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND'''

    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: SMSSTAT'''
    
        QUEUED = 0
        '''Name: Queued'''
    
        SENT = 1
        '''Name: Sent'''
    
        INPUT_FAILURE = 101
        '''Name: Input Error'''
    
        ERROR = 102
        '''Name: Error trying to send sms'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_status', '_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 518, stable = None, description = "Reply sent in response to a SMS sending request.", fields = ('req_id', 'status', 'info',), abbrev = "SmsStatus", name = "SMS Transmission Status", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'SMSSTAT'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    info = _base.mutable_attr({'name': 'Information', 'type': 'plaintext'}, "Error description.")
    '''Error description. Type: plaintext'''

    def __init__(self, req_id = None, status = None, info = None):
        '''Class constructor
        
        Error description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND'''
        self._req_id = req_id
        self._status = status
        self._info = info


class TCPRequest(_base.base_message):
    '''IMC message to be transmitted .

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            msg_data : message, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_destination', '_timeout', '_msg_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 521, stable = None, description = "Request data to be sent over a TCP connection", fields = ('req_id', 'destination', 'timeout', 'msg_data',), abbrev = "TCPRequest", name = "TCP Transmission Request", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    destination = _base.mutable_attr({'name': 'Destination', 'type': 'plaintext'}, "Recipient identifier (number or name).")
    '''Recipient identifier (number or name). Type: plaintext'''
    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'fp64_t', 'unit': 's'}, "Period of time to send message (in seconds).")
    '''Period of time to send message (in seconds). Type: fp64_t'''
    msg_data = _base.mutable_attr({'name': 'Message Data', 'type': 'message'}, "IMC message to be transmitted .")
    '''IMC message to be transmitted . Type: message'''

    def __init__(self, req_id = None, destination = None, timeout = None, msg_data = None):
        '''Class constructor
        
        IMC message to be transmitted .

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            destination : plaintext, unit: NOT FOUND

            timeout : fp64_t, unit: s

            msg_data : message, unit: NOT FOUND'''
        self._req_id = req_id
        self._destination = destination
        self._timeout = timeout
        self._msg_data = msg_data


class TCPStatus(_base.base_message):
    '''Error description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND'''

    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: TCPSTAT'''
    
        QUEUED = 0
        '''Name: Queued'''
    
        SENT = 1
        '''Name: Sent'''
    
        INPUT_FAILURE = 100
        '''Name: Input Error'''
    
        HOST_UNKNOWN = 101
        '''Name: Host Unknown'''
    
        CANT_CONNECT = 102
        '''Name: Can't Connect'''
    
        ERROR = 103
        '''Name: Error trying to send sms'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_req_id', '_status', '_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 522, stable = None, description = "Reply sent in response to a TCP sending request.", fields = ('req_id', 'status', 'info',), abbrev = "TCPStatus", name = "TCP Transmission Status", flags = None)

    req_id = _base.mutable_attr({'name': 'Request Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TCPSTAT'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    info = _base.mutable_attr({'name': 'Information', 'type': 'plaintext'}, "Error description.")
    '''Error description. Type: plaintext'''

    def __init__(self, req_id = None, status = None, info = None):
        '''Class constructor
        
        Error description.

       This message class contains the following fields and their respective types:
    req_id : uint16_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)

            info : plaintext, unit: NOT FOUND'''
        self._req_id = req_id
        self._status = status
        self._info = info


class CreateSession(_base.base_message):
    '''Session timeout, in seconds. If no messages are received from the remote peer, the session will be closed after this ammount of seconds have ellapsed.

       This message class contains the following fields and their respective types:
    timeout : uint32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 806, stable = "false", description = "Request creating a new session with this remote peer. Example session sequence is shown in the following diagram. .. figure:: ../images/session_sequence.png :align: center", fields = ('timeout',), abbrev = "CreateSession", name = "Create Session", flags = None)

    timeout = _base.mutable_attr({'name': 'Session Timeout', 'type': 'uint32_t'}, "Session timeout, in seconds. If no messages are received from the remote peer, the session will be closed after this ammount of seconds have ellapsed.")
    '''Session timeout, in seconds. If no messages are received from the remote peer, the session will be closed after this ammount of seconds have ellapsed. Type: uint32_t'''

    def __init__(self, timeout = None):
        '''Class constructor
        
        Session timeout, in seconds. If no messages are received from the remote peer, the session will be closed after this ammount of seconds have ellapsed.

       This message class contains the following fields and their respective types:
    timeout : uint32_t, unit: NOT FOUND'''
        self._timeout = timeout


class CloseSession(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_sessid']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 807, stable = "false", description = "Request closing of an ongoing session", fields = ('sessid',), abbrev = "CloseSession", name = "Close Session", flags = None)

    sessid = _base.mutable_attr({'name': 'Session Identifier', 'type': 'uint32_t'}, "No description available")
    '''No description available Type: uint32_t'''

    def __init__(self, sessid = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND'''
        self._sessid = sessid


class SessionSubscription(_base.base_message):
    '''Comma-separated list of messages to subscribe. Example: \"EstimatedState,EulerAngles,Temperature\"

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND

            messages : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_sessid', '_messages']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 808, stable = "false", description = None, fields = ('sessid', 'messages',), abbrev = "SessionSubscription", name = "Session Subscription", flags = None)

    sessid = _base.mutable_attr({'name': 'Session Identifier', 'type': 'uint32_t'}, "No description available")
    '''No description available Type: uint32_t'''
    messages = _base.mutable_attr({'name': 'Messages to subscribe', 'type': 'plaintext'}, "Comma-separated list of messages to subscribe. Example: \"EstimatedState,EulerAngles,Temperature\"")
    '''Comma-separated list of messages to subscribe. Example: \"EstimatedState,EulerAngles,Temperature\" Type: plaintext'''

    def __init__(self, sessid = None, messages = None):
        '''Class constructor
        
        Comma-separated list of messages to subscribe. Example: \"EstimatedState,EulerAngles,Temperature\"

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND

            messages : plaintext, unit: NOT FOUND'''
        self._sessid = sessid
        self._messages = messages


class SessionKeepAlive(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_sessid']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 809, stable = "false", description = "Message exchanged to prevent a session from timing out", fields = ('sessid',), abbrev = "SessionKeepAlive", name = "Session Keep-Alive", flags = None)

    sessid = _base.mutable_attr({'name': 'Session Identifier', 'type': 'uint32_t'}, "No description available")
    '''No description available Type: uint32_t'''

    def __init__(self, sessid = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND'''
        self._sessid = sessid


class SessionStatus(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)'''

    class STATUS(_enum.IntEnum):
        '''Full name: Status
        Prefix: STATUS'''
    
        ESTABLISHED = 1
        '''Name: Established'''
    
        CLOSED = 2
        '''Name: Closed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_sessid', '_status']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 810, stable = "false", description = "Message transmitted periodically to inform the state of a communication session", fields = ('sessid', 'status',), abbrev = "SessionStatus", name = "Session Status", flags = "periodic")

    sessid = _base.mutable_attr({'name': 'Session Identifier', 'type': 'uint32_t'}, "No description available")
    '''No description available Type: uint32_t'''
    status = _base.mutable_attr({'name': 'Status', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'STATUS'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, sessid = None, status = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    sessid : uint32_t, unit: NOT FOUND

            status : uint8_t, unit: Enumerated (Local)'''
        self._sessid = sessid
        self._status = status


class MessagePart(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    uid : uint8_t, unit: NOT FOUND

            frag_number : uint8_t, unit: NOT FOUND

            num_frags : uint8_t, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_uid', '_frag_number', '_num_frags', '_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Networking", id = 877, stable = None, description = None, fields = ('uid', 'frag_number', 'num_frags', 'data',), abbrev = "MessagePart", name = "Message Fragment", flags = None)

    uid = _base.mutable_attr({'name': 'Transmission Unique Id', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    frag_number = _base.mutable_attr({'name': 'Fragment Number', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    num_frags = _base.mutable_attr({'name': 'Total Number of fragments', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Fragment Data', 'type': 'rawdata'}, "No description available")
    '''No description available Type: rawdata'''

    def __init__(self, uid = None, frag_number = None, num_frags = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    uid : uint8_t, unit: NOT FOUND

            frag_number : uint8_t, unit: NOT FOUND

            num_frags : uint8_t, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''
        self._uid = uid
        self._frag_number = frag_number
        self._num_frags = num_frags
        self._data = data


class CommRestriction(_base.base_message):
    '''Textual description for why this restriction is needed.

       This message class contains the following fields and their respective types:
    restriction : uint8_t, unit: Bitfield (Local)

            reason : plaintext, unit: NOT FOUND'''

    class RESTRICTION(_enum.IntFlag):
        '''Full name: Restricted Communication Means
        Prefix: MEAN'''
    
        EMPTY = 0
        '''No active flags'''
    
        SATELLITE = 1
        '''Name: Satellite'''
    
        ACOUSTIC = 2
        '''Name: Acoustic'''
    
        WIFI = 4
        '''Name: WiFi'''
    
        GSM = 8
        '''Name: GSM'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_restriction', '_reason']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Networking", id = 2010, stable = None, description = "This message is used to restrict the vehicle from using some communication means.", fields = ('restriction', 'reason',), abbrev = "CommRestriction", name = "Communication Restriction", flags = None)

    restriction = _base.mutable_attr({'name': 'Restricted Communication Means', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'MEAN'}, "The restricted communication means. Bitfield (Local).")
    '''The restricted communication means. Bitfield (Local). Type: uint8_t'''
    reason = _base.mutable_attr({'name': 'Reason', 'type': 'plaintext'}, "Textual description for why this restriction is needed.")
    '''Textual description for why this restriction is needed. Type: plaintext'''

    def __init__(self, restriction = None, reason = None):
        '''Class constructor
        
        Textual description for why this restriction is needed.

       This message class contains the following fields and their respective types:
    restriction : uint8_t, unit: Bitfield (Local)

            reason : plaintext, unit: NOT FOUND'''
        self._restriction = restriction
        self._reason = reason


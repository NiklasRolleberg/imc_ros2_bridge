'''
IMC Storage messages.
'''

from .. import _base
import enum as _enum

class StorageUsage(_base.base_message):
    '''The percentage of storage used by the reporting device.

       This message class contains the following fields and their respective types:
    available : uint32_t, unit: MiB

            value : uint8_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_available', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Storage", id = 100, stable = None, description = "Report of storage usage.", fields = ('available', 'value',), abbrev = "StorageUsage", name = "Storage Usage", flags = "periodic")

    available = _base.mutable_attr({'name': 'Available', 'type': 'uint32_t', 'unit': 'MiB'}, "The available storage of the reporting device.")
    '''The available storage of the reporting device. Type: uint32_t'''
    value = _base.mutable_attr({'name': 'Usage', 'type': 'uint8_t', 'max': 100, 'unit': '%'}, "The percentage of storage used by the reporting device.")
    '''The percentage of storage used by the reporting device. Type: uint8_t'''

    def __init__(self, available = None, value = None):
        '''Class constructor
        
        The percentage of storage used by the reporting device.

       This message class contains the following fields and their respective types:
    available : uint32_t, unit: MiB

            value : uint8_t, unit: %'''
        self._available = available
        self._value = value


class CacheControl(_base.base_message):
    '''Message to store.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            snapshot : plaintext, unit: NOT FOUND

            message : message, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Control Operation
        Prefix: COP'''
    
        STORE = 0
        '''Name: Store'''
    
        LOAD = 1
        '''Name: Load'''
    
        CLEAR = 2
        '''Name: Clear'''
    
        COPY = 3
        '''Name: Copy Snapshot'''
    
        COPY_COMPLETE = 4
        '''Name: Snapshot Copy Complete'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_snapshot', '_message']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Storage", id = 101, stable = None, description = "Control caching of messages to persistent storage.", fields = ('op', 'snapshot', 'message',), abbrev = "CacheControl", name = "Cache Control", flags = None)

    op = _base.mutable_attr({'name': 'Control Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'COP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    snapshot = _base.mutable_attr({'name': 'Snapshot destination', 'type': 'plaintext'}, "Destination for the cache snapshot file.")
    '''Destination for the cache snapshot file. Type: plaintext'''
    message = _base.mutable_attr({'name': 'Message', 'type': 'message'}, "Message to store.")
    '''Message to store. Type: message'''

    def __init__(self, op = None, snapshot = None, message = None):
        '''Class constructor
        
        Message to store.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            snapshot : plaintext, unit: NOT FOUND

            message : message, unit: NOT FOUND'''
        self._op = op
        self._snapshot = snapshot
        self._message = message


class LoggingControl(_base.base_message):
    '''The meaning of this field depends on the operation and is explained in the operation's description.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            name : plaintext, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Control Operation
        Prefix: COP'''
    
        REQUEST_START = 0
        '''Name: Request Start of Logging'''
    
        STARTED = 1
        '''Name: Logging Started'''
    
        REQUEST_STOP = 2
        '''Name: Request Logging Stop'''
    
        STOPPED = 3
        '''Name: Logging Stopped'''
    
        REQUEST_CURRENT_NAME = 4
        '''Name: Request Current Log Name'''
    
        CURRENT_NAME = 5
        '''Name: Current Log Name'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_name']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Storage", id = 102, stable = None, description = "Control logging of messages to persistent storage.", fields = ('op', 'name',), abbrev = "LoggingControl", name = "Logging Control", flags = None)

    op = _base.mutable_attr({'name': 'Control Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'COP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    name = _base.mutable_attr({'name': 'Log Label / Path', 'type': 'plaintext'}, "The meaning of this field depends on the operation and is explained in the operation's description.")
    '''The meaning of this field depends on the operation and is explained in the operation's description. Type: plaintext'''

    def __init__(self, op = None, name = None):
        '''Class constructor
        
        The meaning of this field depends on the operation and is explained in the operation's description.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            name : plaintext, unit: NOT FOUND'''
        self._op = op
        self._name = name


class LogBookEntry(_base.base_message):
    '''Message text.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            htime : fp64_t, unit: s

            context : plaintext, unit: NOT FOUND

            text : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: LBET'''
    
        INFO = 0
        '''Name: Information'''
    
        WARNING = 1
        '''Name: Warning'''
    
        ERROR = 2
        '''Name: Error'''
    
        CRITICAL = 3
        '''Name: Critical'''
    
        DEBUG = 4
        '''Name: Debug'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_htime', '_context', '_text']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Storage", id = 103, stable = None, description = "Human readable message reporting an event of interest.", fields = ('type', 'htime', 'context', 'text',), abbrev = "LogBookEntry", name = "Log Book Entry", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'prefix': 'LBET', 'unit': 'Enumerated'}, "Type of message. Enumerated (Local).")
    '''Type of message. Enumerated (Local). Type: uint8_t'''
    htime = _base.mutable_attr({'name': 'Timestamp', 'type': 'fp64_t', 'unit': 's'}, "Timestamp (Epoch time).")
    '''Timestamp (Epoch time). Type: fp64_t'''
    context = _base.mutable_attr({'name': 'Context', 'type': 'plaintext'}, "Message context.")
    '''Message context. Type: plaintext'''
    text = _base.mutable_attr({'name': 'Text', 'type': 'plaintext'}, "Message text.")
    '''Message text. Type: plaintext'''

    def __init__(self, type = None, htime = None, context = None, text = None):
        '''Class constructor
        
        Message text.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            htime : fp64_t, unit: s

            context : plaintext, unit: NOT FOUND

            text : plaintext, unit: NOT FOUND'''
        self._type = type
        self._htime = htime
        self._context = context
        self._text = text


class LogBookControl(_base.base_message):
    '''Argument, currently used only for 'REPLY'.

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            htime : fp64_t, unit: s

            msg : message-list, unit: NOT FOUND'''

    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: LBC'''
    
        GET = 0
        '''Name: Get'''
    
        CLEAR = 1
        '''Name: Clear'''
    
        GET_ERR = 2
        '''Name: Get Errors'''
    
        REPLY = 3
        '''Name: Reply'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_command', '_htime', '_msg']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Storage", id = 104, stable = None, description = "Control history log.", fields = ('command', 'htime', 'msg',), abbrev = "LogBookControl", name = "Log Book Control", flags = None)

    command = _base.mutable_attr({'name': 'Command', 'prefix': 'LBC', 'type': 'uint8_t', 'unit': 'Enumerated'}, "Command to perform. Enumerated (Local).")
    '''Command to perform. Enumerated (Local). Type: uint8_t'''
    htime = _base.mutable_attr({'name': 'Timestamp', 'type': 'fp64_t', 'unit': 's'}, "Timestamp for command (Epoch time).")
    '''Timestamp for command (Epoch time). Type: fp64_t'''
    msg = _base.mutable_attr({'name': 'Messages', 'type': 'message-list', 'message-type': 'LogBookEntry'}, "Argument, currently used only for 'REPLY'.")
    '''Argument, currently used only for 'REPLY'. Type: message-list'''

    def __init__(self, command = None, htime = None, msg = None):
        '''Class constructor
        
        Argument, currently used only for 'REPLY'.

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            htime : fp64_t, unit: s

            msg : message-list, unit: NOT FOUND'''
        self._command = command
        self._htime = htime
        self._msg = msg


class ReplayControl(_base.base_message):
    '''Pathname of file to replay.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            file : plaintext, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: ROP'''
    
        START = 0
        '''Name: Start'''
    
        STOP = 1
        '''Name: Stop'''
    
        PAUSE = 2
        '''Name: Pause'''
    
        RESUME = 3
        '''Name: Resume'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_file']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 105, stable = None, description = "Control replay of LSF logged data.", fields = ('op', 'file',), abbrev = "ReplayControl", name = "Replay Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'prefix': 'ROP', 'type': 'uint8_t', 'unit': 'Enumerated'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    file = _base.mutable_attr({'name': 'File To Replay', 'type': 'plaintext'}, "Pathname of file to replay.")
    '''Pathname of file to replay. Type: plaintext'''

    def __init__(self, op = None, file = None):
        '''Class constructor
        
        Pathname of file to replay.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            file : plaintext, unit: NOT FOUND'''
        self._op = op
        self._file = file


class ClockControl(_base.base_message):
    '''Timezone.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            clock : fp64_t, unit: s

            tz : int8_t, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: COP'''
    
        SYNC_EXEC = 0
        '''Name: Execute Sync.'''
    
        SYNC_REQUEST = 1
        '''Name: Request Sync.'''
    
        SYNC_STARTED = 2
        '''Name: Sync. Started'''
    
        SYNC_DONE = 3
        '''Name: Sync. done'''
    
        SET_TZ = 4
        '''Name: Set Timezone '''
    
        SET_TZ_DONE = 5
        '''Name: Timezone Setup'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_clock', '_tz']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 106, stable = None, description = "Clock control.", fields = ('op', 'clock', 'tz',), abbrev = "ClockControl", name = "Clock Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'prefix': 'COP', 'unit': 'Enumerated'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    clock = _base.mutable_attr({'name': 'Clock', 'type': 'fp64_t', 'unit': 's'}, "Clock value (Epoch time).")
    '''Clock value (Epoch time). Type: fp64_t'''
    tz = _base.mutable_attr({'name': 'Timezone', 'type': 'int8_t', 'min': -23.0, 'max': 23}, "Timezone.")
    '''Timezone. Type: int8_t'''

    def __init__(self, op = None, clock = None, tz = None):
        '''Class constructor
        
        Timezone.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            clock : fp64_t, unit: s

            tz : int8_t, unit: NOT FOUND'''
        self._op = op
        self._clock = clock
        self._tz = tz


class HistoricCTD(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    conductivity : fp32_t, unit: S/m

            temperature : fp32_t, unit: °C

            depth : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_conductivity', '_temperature', '_depth']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 107, stable = None, description = "This message is used to store historic (transmitted afterwards) CTD data .", fields = ('conductivity', 'temperature', 'depth',), abbrev = "HistoricCTD", name = "Historic CTD", flags = None)

    conductivity = _base.mutable_attr({'name': 'Conductivity', 'type': 'fp32_t', 'unit': 'S/m'}, "No description available")
    '''No description available Type: fp32_t'''
    temperature = _base.mutable_attr({'name': 'Temperature', 'type': 'fp32_t', 'unit': '°C'}, "No description available")
    '''No description available Type: fp32_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, conductivity = None, temperature = None, depth = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    conductivity : fp32_t, unit: S/m

            temperature : fp32_t, unit: °C

            depth : fp32_t, unit: m'''
        self._conductivity = conductivity
        self._temperature = temperature
        self._depth = depth


class HistoricTelemetry(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    altitude : fp32_t, unit: m

            roll : uint16_t, unit: NOT FOUND

            pitch : uint16_t, unit: NOT FOUND

            yaw : uint16_t, unit: NOT FOUND

            speed : int16_t, unit: dm'''

    __slots__ = ['_Attributes', '_header', '_footer', '_altitude', '_roll', '_pitch', '_yaw', '_speed']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 108, stable = None, description = "This message is used to store historic (transmitted afterwards) telemetry information.", fields = ('altitude', 'roll', 'pitch', 'yaw', 'speed',), abbrev = "HistoricTelemetry", name = "Historic Telemetry", flags = None)

    altitude = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    roll = _base.mutable_attr({'name': 'Roll', 'type': 'uint16_t'}, "Roll encoded as α.(65535/(2.π))")
    '''Roll encoded as α.(65535/(2.π)) Type: uint16_t'''
    pitch = _base.mutable_attr({'name': 'Pitch', 'type': 'uint16_t'}, "Pitch encoded as α.(65535/(2.π))")
    '''Pitch encoded as α.(65535/(2.π)) Type: uint16_t'''
    yaw = _base.mutable_attr({'name': 'Yaw', 'type': 'uint16_t'}, "Yaw encoded as α.(65535/(2.π))")
    '''Yaw encoded as α.(65535/(2.π)) Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'int16_t', 'unit': 'dm'}, "No description available")
    '''No description available Type: int16_t'''

    def __init__(self, altitude = None, roll = None, pitch = None, yaw = None, speed = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    altitude : fp32_t, unit: m

            roll : uint16_t, unit: NOT FOUND

            pitch : uint16_t, unit: NOT FOUND

            yaw : uint16_t, unit: NOT FOUND

            speed : int16_t, unit: dm'''
        self._altitude = altitude
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._speed = speed


class HistoricSonarData(_base.base_message):
    '''Sonar data encoded as in 'encoding'.

       This message class contains the following fields and their respective types:
    altitude : fp32_t, unit: m

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            bearing : fp32_t, unit: NOT FOUND

            pxl : int16_t, unit: NOT FOUND

            encoding : uint8_t, unit: Enumerated (Local)

            sonar_data : rawdata, unit: NOT FOUND'''

    class ENCODING(_enum.IntEnum):
        '''Full name: Encoding
        Prefix: ENC'''
    
        ONE_BYTE_PER_PIXEL = 0
        '''Name: One Byte Per Pixel'''
    
        PNG = 1
        '''Name: PNG compressed image'''
    
        JPEG = 2
        '''Name: JPEG compressed image'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_altitude', '_width', '_length', '_bearing', '_pxl', '_encoding', '_sonar_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 109, stable = None, description = "This message is used to store historic (transmitted afterwards) sonar data.", fields = ('altitude', 'width', 'length', 'bearing', 'pxl', 'encoding', 'sonar_data',), abbrev = "HistoricSonarData", name = "Historic Sonar Data", flags = None)

    altitude = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    width = _base.mutable_attr({'name': 'Width', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    length = _base.mutable_attr({'name': 'Length', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    pxl = _base.mutable_attr({'name': 'Pixels Per Line', 'type': 'int16_t'}, "The number of pixels per line as the data in 'sonar_data' may correspond to more than one sequential sidescan lines.")
    '''The number of pixels per line as the data in 'sonar_data' may correspond to more than one sequential sidescan lines. Type: int16_t'''
    encoding = _base.mutable_attr({'name': 'Encoding', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'ENC'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    sonar_data = _base.mutable_attr({'name': 'SonarData', 'type': 'rawdata'}, "Sonar data encoded as in 'encoding'.")
    '''Sonar data encoded as in 'encoding'. Type: rawdata'''

    def __init__(self, altitude = None, width = None, length = None, bearing = None, pxl = None, encoding = None, sonar_data = None):
        '''Class constructor
        
        Sonar data encoded as in 'encoding'.

       This message class contains the following fields and their respective types:
    altitude : fp32_t, unit: m

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            bearing : fp32_t, unit: NOT FOUND

            pxl : int16_t, unit: NOT FOUND

            encoding : uint8_t, unit: Enumerated (Local)

            sonar_data : rawdata, unit: NOT FOUND'''
        self._altitude = altitude
        self._width = width
        self._length = length
        self._bearing = bearing
        self._pxl = pxl
        self._encoding = encoding
        self._sonar_data = sonar_data


class HistoricEvent(_base.base_message):
    '''Type of event. Enumerated (Local).

       This message class contains the following fields and their respective types:
    text : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Event Type
        Prefix: EVTYPE'''
    
        INFO = 0
        '''Name: Information'''
    
        ERROR = 1
        '''Name: Error'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_text', '_type']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 110, stable = None, description = "This message is used to store historic event log entries.", fields = ('text', 'type',), abbrev = "HistoricEvent", name = "Historic Event", flags = None)

    text = _base.mutable_attr({'name': 'Event', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    type = _base.mutable_attr({'name': 'Event Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'EVTYPE'}, "Type of event. Enumerated (Local).")
    '''Type of event. Enumerated (Local). Type: uint8_t'''

    def __init__(self, text = None, type = None):
        '''Class constructor
        
        Type of event. Enumerated (Local).

       This message class contains the following fields and their respective types:
    text : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)'''
        self._text = text
        self._type = type


class VerticalProfile(_base.base_message):
    '''Longitude where the profile was calculated.

       This message class contains the following fields and their respective types:
    parameter : uint8_t, unit: Enumerated (Local)

            numSamples : uint8_t, unit: NOT FOUND

            samples : message-list, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad'''

    class PARAMETER(_enum.IntEnum):
        '''Full name: Parameter
        Prefix: PROF'''
    
        TEMPERATURE = 0
        '''Name: Temperature'''
    
        SALINITY = 1
        '''Name: Salinity'''
    
        CONDUCTIVITY = 2
        '''Name: Conductivity'''
    
        PH = 3
        '''Name: pH'''
    
        REDOX = 4
        '''Name: Redox'''
    
        CHLOROPHYLL = 5
        '''Name: Chlorophyll'''
    
        TURBIDITY = 6
        '''Name: Turbidity'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_parameter', '_numSamples', '_samples', '_lat', '_lon']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 111, stable = None, description = "This message is used to store historic profiles for water parameters: Temperature, Salinity, Chlorophyll...", fields = ('parameter', 'numSamples', 'samples', 'lat', 'lon',), abbrev = "VerticalProfile", name = "Vertical Profile", flags = None)

    parameter = _base.mutable_attr({'name': 'Parameter', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'PROF'}, "Water parameter used to calculate the vertical profile. Enumerated (Local).")
    '''Water parameter used to calculate the vertical profile. Enumerated (Local). Type: uint8_t'''
    numSamples = _base.mutable_attr({'name': 'Number of Samples', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    samples = _base.mutable_attr({'name': 'Samples', 'type': 'message-list', 'message-type': 'ProfileSample'}, "No description available")
    '''No description available Type: message-list'''
    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "Latitude where the profile was calculated.")
    '''Latitude where the profile was calculated. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "Longitude where the profile was calculated.")
    '''Longitude where the profile was calculated. Type: fp64_t'''

    def __init__(self, parameter = None, numSamples = None, samples = None, lat = None, lon = None):
        '''Class constructor
        
        Longitude where the profile was calculated.

       This message class contains the following fields and their respective types:
    parameter : uint8_t, unit: Enumerated (Local)

            numSamples : uint8_t, unit: NOT FOUND

            samples : message-list, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad'''
        self._parameter = parameter
        self._numSamples = numSamples
        self._samples = samples
        self._lat = lat
        self._lon = lon


class ProfileSample(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    depth : uint16_t, unit: dm

            avg : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_depth', '_avg']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Storage", id = 112, stable = None, description = "Samples to calculate a vertical profile.", fields = ('depth', 'avg',), abbrev = "ProfileSample", name = "Profile Sample", flags = None)

    depth = _base.mutable_attr({'name': 'Depth', 'type': 'uint16_t', 'unit': 'dm'}, "No description available")
    '''No description available Type: uint16_t'''
    avg = _base.mutable_attr({'name': 'Average', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, depth = None, avg = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    depth : uint16_t, unit: dm

            avg : fp32_t, unit: NOT FOUND'''
        self._depth = depth
        self._avg = avg


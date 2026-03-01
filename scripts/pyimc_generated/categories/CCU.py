'''
IMC CCU messages.
'''

from .. import _base
import enum as _enum

class ReportedState(_base.base_message):
    '''How the position was received/calculated Enumerated (Local).

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp64_t, unit: m

            roll : fp64_t, unit: rad

            pitch : fp64_t, unit: rad

            yaw : fp64_t, unit: rad

            rcp_time : fp64_t, unit: s

            sid : plaintext, unit: NOT FOUND

            s_type : uint8_t, unit: Enumerated (Local)'''

    class S_TYPE(_enum.IntEnum):
        '''Full name: Source Type
        Prefix: STYPE'''
    
        WI_FI = 0
        '''Name: Wi-Fi'''
    
        TRACKER = 1
        '''Name: Tracker'''
    
        SMS = 2
        '''Name: SMS'''
    
        ACOUSTIC_MODEM = 3
        '''Name: Acoustic Modem'''
    
        UNKNOWN = 254
        '''Name: Unknown source'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_depth', '_roll', '_pitch', '_yaw', '_rcp_time', '_sid', '_s_type']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "CCU", id = 600, stable = None, description = "A vehicle state that is reported to other consoles (including PDAConsole). Source can be acoustic tracker, SMS, Wi-Fi, etc...", fields = ('lat', 'lon', 'depth', 'roll', 'pitch', 'yaw', 'rcp_time', 'sid', 's_type',), abbrev = "ReportedState", name = "Reported State", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp64_t', 'unit': 'm'}, "The reported depth. In the case of not knowing the depth 0 will be reported. Airplanes usually have negative values (por positive altitude).")
    '''The reported depth. In the case of not knowing the depth 0 will be reported. Airplanes usually have negative values (por positive altitude). Type: fp64_t'''
    roll = _base.mutable_attr({'name': 'Roll', 'type': 'fp64_t', 'unit': 'rad'}, "The phi Euler angle from the vehicle's attitude.")
    '''The phi Euler angle from the vehicle's attitude. Type: fp64_t'''
    pitch = _base.mutable_attr({'name': 'Pitch', 'type': 'fp64_t', 'unit': 'rad'}, "The theta Euler angle from the vehicle's attitude.")
    '''The theta Euler angle from the vehicle's attitude. Type: fp64_t'''
    yaw = _base.mutable_attr({'name': 'Yaw', 'type': 'fp64_t', 'unit': 'rad'}, "The psi Euler angle from the vehicle's attitude.")
    '''The psi Euler angle from the vehicle's attitude. Type: fp64_t'''
    rcp_time = _base.mutable_attr({'name': 'Reception Time', 'type': 'fp64_t', 'unit': 's', 'note': 'Correct interpretation of this field may require synchronization of clocks between sender and recipient'}, "The time when the packet was sent, as seen by the packet dispatcher. The number of seconds is represented in Universal Coordinated Time (UCT) in seconds since Jan 1, 1970 using IEEE double precision floating point numbers.")
    '''The time when the packet was sent, as seen by the packet dispatcher. The number of seconds is represented in Universal Coordinated Time (UCT) in seconds since Jan 1, 1970 using IEEE double precision floating point numbers. Type: fp64_t'''
    sid = _base.mutable_attr({'name': 'System Identifier', 'type': 'plaintext'}, "The id of the system whose position is being reported (it can be a vehicle's id, a boat name, etc)")
    '''The id of the system whose position is being reported (it can be a vehicle's id, a boat name, etc) Type: plaintext'''
    s_type = _base.mutable_attr({'name': 'Source Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'STYPE'}, "How the position was received/calculated Enumerated (Local).")
    '''How the position was received/calculated Enumerated (Local). Type: uint8_t'''

    def __init__(self, lat = None, lon = None, depth = None, roll = None, pitch = None, yaw = None, rcp_time = None, sid = None, s_type = None):
        '''Class constructor
        
        How the position was received/calculated Enumerated (Local).

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp64_t, unit: m

            roll : fp64_t, unit: rad

            pitch : fp64_t, unit: rad

            yaw : fp64_t, unit: rad

            rcp_time : fp64_t, unit: s

            sid : plaintext, unit: NOT FOUND

            s_type : uint8_t, unit: Enumerated (Local)'''
        self._lat = lat
        self._lon = lon
        self._depth = depth
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._rcp_time = rcp_time
        self._sid = sid
        self._s_type = s_type


class RemoteSensorInfo(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            sensor_class : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            alt : fp32_t, unit: m

            heading : fp32_t, unit: rad

            data : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_sensor_class', '_lat', '_lon', '_alt', '_heading', '_data']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "CCU", id = 601, stable = None, description = "Whenever the CUCS receives a message from one of the existing sensors (through SMS, ZigBee, Acoustic Comms, ...) it disseminates that info recurring to this message.", fields = ('id', 'sensor_class', 'lat', 'lon', 'alt', 'heading', 'data',), abbrev = "RemoteSensorInfo", name = "Remote Sensor Info", flags = None)

    id = _base.mutable_attr({'name': 'Id', 'type': 'plaintext'}, "An unique string that identifies the sensor. Used mostly for logging/presentation.")
    '''An unique string that identifies the sensor. Used mostly for logging/presentation. Type: plaintext'''
    sensor_class = _base.mutable_attr({'name': 'Class', 'type': 'plaintext'}, "The class of a sensor tells the type of sensor originating this message. It will determine how the sensor is to be shown and (optionally) how the custom data (tuplelist) is to be interpreted.")
    '''The class of a sensor tells the type of sensor originating this message. It will determine how the sensor is to be shown and (optionally) how the custom data (tuplelist) is to be interpreted. Type: plaintext'''
    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    alt = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    heading = _base.mutable_attr({'name': 'Heading', 'type': 'fp32_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp32_t'''
    data = _base.mutable_attr({'name': 'Custom Data', 'type': 'plaintext', 'unit': 'TupleList'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, id = None, sensor_class = None, lat = None, lon = None, alt = None, heading = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            sensor_class : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            alt : fp32_t, unit: m

            heading : fp32_t, unit: rad

            data : plaintext, unit: TupleList'''
        self._id = id
        self._sensor_class = sensor_class
        self._lat = lat
        self._lon = lon
        self._alt = alt
        self._heading = heading
        self._data = data


class Map(_base.base_message):
    '''A list of map features.

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            features : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_features']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "CCU", id = 602, stable = None, description = "This message represents a simple map that is transferred between CCU consoles (from Neptus to ACCU)", fields = ('id', 'features',), abbrev = "Map", name = "Map", flags = None)

    id = _base.mutable_attr({'name': 'Identifier', 'type': 'plaintext'}, "The id of the map")
    '''The id of the map Type: plaintext'''
    features = _base.mutable_attr({'name': 'Features', 'type': 'message-list', 'message-type': 'MapFeature'}, "A list of map features.")
    '''A list of map features. Type: message-list'''

    def __init__(self, id = None, features = None):
        '''Class constructor
        
        A list of map features.

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            features : message-list, unit: NOT FOUND'''
        self._id = id
        self._features = features


class MapFeature(_base.base_message):
    '''The enclosing feature definition.

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            feature_type : uint8_t, unit: Enumerated (Local)

            rgb_red : uint8_t, unit: NOT FOUND

            rgb_green : uint8_t, unit: NOT FOUND

            rgb_blue : uint8_t, unit: NOT FOUND

            feature : message-list, unit: NOT FOUND'''

    class FEATURE_TYPE(_enum.IntEnum):
        '''Full name: FeatureType
        Prefix: FTYPE'''
    
        POI = 0
        '''Name: Point of Interest'''
    
        FILLEDPOLY = 1
        '''Name: Filled Polygon'''
    
        CONTOUREDPOLY = 2
        '''Name: Countoured Polygon'''
    
        LINE = 3
        '''Name: Line'''
    
        TRANSPONDER = 4
        '''Name: Transponder'''
    
        STARTLOC = 5
        '''Name: Start Location'''
    
        HOMEREF = 6
        '''Name: Home Reference'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_feature_type', '_rgb_red', '_rgb_green', '_rgb_blue', '_feature']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "CCU", id = 603, stable = None, description = "A feature to appear on the map", fields = ('id', 'feature_type', 'rgb_red', 'rgb_green', 'rgb_blue', 'feature',), abbrev = "MapFeature", name = "Map Feature", flags = None)

    id = _base.mutable_attr({'name': 'Identifier', 'type': 'plaintext'}, "The unique identifier for this feature (used as the name for points of interest)")
    '''The unique identifier for this feature (used as the name for points of interest) Type: plaintext'''
    feature_type = _base.mutable_attr({'name': 'FeatureType', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'FTYPE'}, "The type of feature Enumerated (Local).")
    '''The type of feature Enumerated (Local). Type: uint8_t'''
    rgb_red = _base.mutable_attr({'name': 'RedComponent', 'type': 'uint8_t'}, "The red component of the color for this point")
    '''The red component of the color for this point Type: uint8_t'''
    rgb_green = _base.mutable_attr({'name': 'GreenComponent', 'type': 'uint8_t'}, "The green component of the color for this point")
    '''The green component of the color for this point Type: uint8_t'''
    rgb_blue = _base.mutable_attr({'name': 'BlueComponent', 'type': 'uint8_t'}, "The blue component of the color for this point")
    '''The blue component of the color for this point Type: uint8_t'''
    feature = _base.mutable_attr({'name': 'Feature', 'type': 'message-list', 'message-type': 'MapPoint'}, "The enclosing feature definition.")
    '''The enclosing feature definition. Type: message-list'''

    def __init__(self, id = None, feature_type = None, rgb_red = None, rgb_green = None, rgb_blue = None, feature = None):
        '''Class constructor
        
        The enclosing feature definition.

       This message class contains the following fields and their respective types:
    id : plaintext, unit: NOT FOUND

            feature_type : uint8_t, unit: Enumerated (Local)

            rgb_red : uint8_t, unit: NOT FOUND

            rgb_green : uint8_t, unit: NOT FOUND

            rgb_blue : uint8_t, unit: NOT FOUND

            feature : message-list, unit: NOT FOUND'''
        self._id = id
        self._feature_type = feature_type
        self._rgb_red = rgb_red
        self._rgb_green = rgb_green
        self._rgb_blue = rgb_blue
        self._feature = feature


class MapPoint(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            alt : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_alt']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "CCU", id = 604, stable = None, description = "This message represents a point in the world.", fields = ('lat', 'lon', 'alt',), abbrev = "MapPoint", name = "MapPoint", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    alt = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, lat = None, lon = None, alt = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            alt : fp32_t, unit: m'''
        self._lat = lat
        self._lon = lon
        self._alt = alt


class CcuEvent(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            id : plaintext, unit: NOT FOUND

            arg : message, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Event Type
        Prefix: EVT'''
    
        LOG_ENTRY = 1
        '''Name: Log Book Entry Added'''
    
        PLAN_ADDED = 2
        '''Name: Plan Added'''
    
        PLAN_REMOVED = 3
        '''Name: Plan Removed'''
    
        PLAN_CHANGED = 4
        '''Name: Plan Changed'''
    
        MAP_FEATURE_ADDED = 5
        '''Name: Map feature added'''
    
        MAP_FEATURE_REMOVED = 6
        '''Name: Map feature removed'''
    
        MAP_FEATURE_CHANGED = 7
        '''Name: Map feature changed'''
    
        TELEOPERATION_STARTED = 8
        '''Name: The sender is now teleoperating the vehicle'''
    
        TELEOPERATION_ENDED = 9
        '''Name: The sender stopped teleoperating the vehicle'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_id', '_arg']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "CCU", id = 606, stable = None, description = "This message is used to signal events among running CCUs.", fields = ('type', 'id', 'arg',), abbrev = "CcuEvent", name = "CCU Event", flags = None)

    type = _base.mutable_attr({'name': 'Event Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'EVT'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    id = _base.mutable_attr({'name': 'Identifier', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    arg = _base.mutable_attr({'name': 'Additional Data', 'type': 'message'}, "No description available")
    '''No description available Type: message'''

    def __init__(self, type = None, id = None, arg = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            id : plaintext, unit: NOT FOUND

            arg : message, unit: NOT FOUND'''
        self._type = type
        self._id = id
        self._arg = arg


class RemoteState(_base.base_message):
    '''Heading.

       This message class contains the following fields and their respective types:
    lat : fp32_t, unit: rad

            lon : fp32_t, unit: rad

            depth : uint8_t, unit: m

            speed : fp32_t, unit: m/s

            psi : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_depth', '_speed', '_psi']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "CCU", id = 750, stable = None, description = "State summary for a remote vehicle.", fields = ('lat', 'lon', 'depth', 'speed', 'psi',), abbrev = "RemoteState", name = "Remote State", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp32_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp32_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'uint8_t', 'unit': 'm'}, "Depth.")
    '''Depth. Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t', 'unit': 'm/s'}, "Speed.")
    '''Speed. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Heading', 'type': 'fp32_t', 'unit': 'rad'}, "Heading.")
    '''Heading. Type: fp32_t'''

    def __init__(self, lat = None, lon = None, depth = None, speed = None, psi = None):
        '''Class constructor
        
        Heading.

       This message class contains the following fields and their respective types:
    lat : fp32_t, unit: rad

            lon : fp32_t, unit: rad

            depth : uint8_t, unit: m

            speed : fp32_t, unit: m/s

            psi : fp32_t, unit: rad'''
        self._lat = lat
        self._lon = lon
        self._depth = depth
        self._speed = speed
        self._psi = psi


class NeptusBlob(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    content_type : plaintext, unit: NOT FOUND

            content : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_content_type', '_content']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "CCU", id = 888, stable = None, description = None, fields = ('content_type', 'content',), abbrev = "NeptusBlob", name = "Neptus Blob", flags = None)

    content_type = _base.mutable_attr({'name': 'ContentType', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    content = _base.mutable_attr({'name': 'Content', 'type': 'rawdata'}, "No description available")
    '''No description available Type: rawdata'''

    def __init__(self, content_type = None, content = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    content_type : plaintext, unit: NOT FOUND

            content : rawdata, unit: NOT FOUND'''
        self._content_type = content_type
        self._content = content


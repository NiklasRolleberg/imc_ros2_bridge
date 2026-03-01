'''
IMC Guidance messages.
'''

from .. import _base
import enum as _enum

class DesiredHeading(_base.base_message):
    '''The value of the desired heading angle, relative to true north, in radians.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 400, stable = None, description = "Desired Heading angle reference value for the control layer.", fields = ('value',), abbrev = "DesiredHeading", name = "Desired Heading", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': 'rad'}, "The value of the desired heading angle, relative to true north, in radians.")
    '''The value of the desired heading angle, relative to true north, in radians. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired heading angle, relative to true north, in radians.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''
        self._value = value


class DesiredZ(_base.base_message):
    '''Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_z_units']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 401, stable = None, description = "Desired Z reference value for the control layer.", fields = ('value', 'z_units',), abbrev = "DesiredZ", name = "Desired Z", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'm'}, "The value of the desired z reference in meters.")
    '''The value of the desired z reference in meters. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''

    def __init__(self, value = None, z_units = None):
        '''Class constructor
        
        Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)'''
        self._value = value
        self._z_units = z_units


class DesiredSpeed(_base.base_message):
    '''Indicates the units used for the speed value. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_speed_units']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 402, stable = None, description = "Desired Speed reference value for the control layer.", fields = ('value', 'speed_units',), abbrev = "DesiredSpeed", name = "Desired Speed", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t'}, "The value of the desired speed, in the scale specified by the \"Speed Units\" field.")
    '''The value of the desired speed, in the scale specified by the \"Speed Units\" field. Type: fp64_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Indicates the units used for the speed value. Enumerated (Global).")
    '''Indicates the units used for the speed value. Enumerated (Global). Type: uint8_t'''

    def __init__(self, value = None, speed_units = None):
        '''Class constructor
        
        Indicates the units used for the speed value. Enumerated (Global).

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)'''
        self._value = value
        self._speed_units = speed_units


class DesiredRoll(_base.base_message):
    '''The value of the desired roll angle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 403, stable = None, description = "Desired Roll angle reference value for the control layer.", fields = ('value',), abbrev = "DesiredRoll", name = "Desired Roll", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': 'rad'}, "The value of the desired roll angle.")
    '''The value of the desired roll angle. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired roll angle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''
        self._value = value


class DesiredPitch(_base.base_message):
    '''The value of the desired pitch angle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 404, stable = None, description = "Desired Pitch angle reference value for the control layer.", fields = ('value',), abbrev = "DesiredPitch", name = "Desired Pitch", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': 'rad'}, "The value of the desired pitch angle.")
    '''The value of the desired pitch angle. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired pitch angle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad'''
        self._value = value


class DesiredVerticalRate(_base.base_message):
    '''The value of the desired vertical rate speed in meters per second.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 405, stable = None, description = "Desired Vertical Rate speed reference value for the control layer.", fields = ('value',), abbrev = "DesiredVerticalRate", name = "Desired Vertical Rate", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': 'm/s'}, "The value of the desired vertical rate speed in meters per second.")
    '''The value of the desired vertical rate speed in meters per second. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired vertical rate speed in meters per second.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''
        self._value = value


class DesiredPath(_base.base_message):
    '''Desired Path flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    path_ref : uint32_t, unit: NOT FOUND

            start_lat : fp64_t, unit: rad

            start_lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_lat : fp64_t, unit: rad

            end_lon : fp64_t, unit: rad

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            lradius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FL'''
    
        EMPTY = 0
        '''No active flags'''
    
        START = 1
        '''Name: Start Point'''
    
        DIRECT = 2
        '''Name: Direct'''
    
        NO_Z = 4
        '''Name: No Altitude/Depth control'''
    
        x3DTRACK = 8
        '''Name: 3D Tracking'''
    
        CCLOCKW = 16
        '''Name: Counter-Clockwise loiter'''
    
        LOITER_CURR = 32
        '''Name: Loiter from current position'''
    
        TAKEOFF = 64
        '''Name: Takeoff'''
    
        LAND = 128
        '''Name: Land'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_path_ref', '_start_lat', '_start_lon', '_start_z', '_start_z_units', '_end_lat', '_end_lon', '_end_z', '_end_z_units', '_speed', '_speed_units', '_lradius', '_flags']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 406, stable = None, description = "Perform path control. The path is specified by two WGS-84 waypoints, respective altitude / depth settings, optional loitering at the end point, and some control flags. The start and end waypoints are defined by the specified end point fields ('end_{lat/lon/z}') plus: 1. the start waypoint fields ('start_{lat|lon|z}') if the 'START' flag (bit in 'flags' field) is set; or 2. the end point of the previous path recently tracked; or 3. the current location is the 'DIRECT' flag is set or if the tracker has been idle for some time. Altitude and depth control can be performed as follows: 1. by default, the tracker will just transmit an altitude/depth reference value equal to 'end_z' to the appropriate controller; 2. if the 'NO_Z' flag is set no altitude/depth control will take place, hence they can be controlled independently; 3. if the '3DTRACK' flag is set, 3D-tracking will be done (if supported by the active controller). Loitering can be specified at the end point with a certain radius ('lradius'), duration ('lduration'), and clockwise or counter-clockwise direction ('CCLOCKW' flag).", fields = ('path_ref', 'start_lat', 'start_lon', 'start_z', 'start_z_units', 'end_lat', 'end_lon', 'end_z', 'end_z_units', 'speed', 'speed_units', 'lradius', 'flags',), abbrev = "DesiredPath", name = "Desired Path", flags = None)

    path_ref = _base.mutable_attr({'name': 'Path Reference', 'type': 'uint32_t'}, "Unsigned integer reference for the scope of the desired path message. Path reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level path controllers must inherit the same path reference sent by maneuver.")
    '''Unsigned integer reference for the scope of the desired path message. Path reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level path controllers must inherit the same path reference sent by maneuver. Type: uint32_t'''
    start_lat = _base.mutable_attr({'name': 'Start Point -- Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 latitude of start point. This will be ignored unless the 'START' flag is set.")
    '''WGS-84 latitude of start point. This will be ignored unless the 'START' flag is set. Type: fp64_t'''
    start_lon = _base.mutable_attr({'name': 'Start Point -- WGS-84 Longitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 longitude of start point. This will be ignored unless the 'START' flag is set.")
    '''WGS-84 longitude of start point. This will be ignored unless the 'START' flag is set. Type: fp64_t'''
    start_z = _base.mutable_attr({'name': 'Start Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set.")
    '''Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set. Type: fp32_t'''
    start_z_units = _base.mutable_attr({'name': 'Start Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the start point's z reference. Enumerated (Global).")
    '''Units of the start point's z reference. Enumerated (Global). Type: uint8_t'''
    end_lat = _base.mutable_attr({'name': 'End Point -- WGS84 Latitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 latitude of end point.")
    '''WGS-84 latitude of end point. Type: fp64_t'''
    end_lon = _base.mutable_attr({'name': 'End Point -- WGS-84 Longitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 longitude of end point.")
    '''WGS-84 longitude of end point. Type: fp64_t'''
    end_z = _base.mutable_attr({'name': 'End Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Depth or altitude for the end point. This parameter will be ignored if the 'NO_Z' flag is set.")
    '''Depth or altitude for the end point. This parameter will be ignored if the 'NO_Z' flag is set. Type: fp32_t'''
    end_z_units = _base.mutable_attr({'name': 'End Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the end point's z reference. Enumerated (Global).")
    '''Units of the end point's z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    lradius = _base.mutable_attr({'name': 'Loiter -- Radius', 'type': 'fp32_t', 'unit': 'm'}, "Radius for loitering at end point. Specify less or equal to 0 for no loitering.")
    '''Radius for loitering at end point. Specify less or equal to 0 for no loitering. Type: fp32_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FL'}, "Desired Path flags. Bitfield (Local).")
    '''Desired Path flags. Bitfield (Local). Type: uint8_t'''

    def __init__(self, path_ref = None, start_lat = None, start_lon = None, start_z = None, start_z_units = None, end_lat = None, end_lon = None, end_z = None, end_z_units = None, speed = None, speed_units = None, lradius = None, flags = None):
        '''Class constructor
        
        Desired Path flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    path_ref : uint32_t, unit: NOT FOUND

            start_lat : fp64_t, unit: rad

            start_lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_lat : fp64_t, unit: rad

            end_lon : fp64_t, unit: rad

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            lradius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)'''
        self._path_ref = path_ref
        self._start_lat = start_lat
        self._start_lon = start_lon
        self._start_z = start_z
        self._start_z_units = start_z_units
        self._end_lat = end_lat
        self._end_lon = end_lon
        self._end_z = end_z
        self._end_z_units = end_z_units
        self._speed = speed
        self._speed_units = speed_units
        self._lradius = lradius
        self._flags = flags


class DesiredControl(_base.base_message):
    '''Desired Control flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: N

            y : fp64_t, unit: N

            z : fp64_t, unit: N

            k : fp64_t, unit: Nm

            m : fp64_t, unit: Nm

            n : fp64_t, unit: Nm

            flags : uint8_t, unit: Bitfield (Local)'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FL'''
    
        EMPTY = 0
        '''No active flags'''
    
        X = 1
        '''Name: Value of X is meaningful'''
    
        Y = 2
        '''Name: Value of Y is meaningful'''
    
        Z = 4
        '''Name: Value of Z is meaningful'''
    
        K = 8
        '''Name: Value of K is meaningful'''
    
        M = 16
        '''Name: Value of M is meaningful'''
    
        N = 32
        '''Name: Value of N is meaningful'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z', '_k', '_m', '_n', '_flags']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 407, stable = None, description = "Set the desired virtual forces and torques to be applied to the vehicle.", fields = ('x', 'y', 'z', 'k', 'm', 'n', 'flags',), abbrev = "DesiredControl", name = "Desired Control", flags = None)

    x = _base.mutable_attr({'name': 'Force along the x axis', 'type': 'fp64_t', 'unit': 'N'}, "Force X along the vehicle's x axis.")
    '''Force X along the vehicle's x axis. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Force along the y axis', 'type': 'fp64_t', 'unit': 'N'}, "Force Y along the vehicle's y axis.")
    '''Force Y along the vehicle's y axis. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Force along the z axis', 'type': 'fp64_t', 'unit': 'N'}, "Force Z along the vehicle's z axis.")
    '''Force Z along the vehicle's z axis. Type: fp64_t'''
    k = _base.mutable_attr({'name': 'Torque about the x axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque K about the vehicle's x axis.")
    '''Torque K about the vehicle's x axis. Type: fp64_t'''
    m = _base.mutable_attr({'name': 'Torque about the y axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque M about the vehicle's y axis.")
    '''Torque M about the vehicle's y axis. Type: fp64_t'''
    n = _base.mutable_attr({'name': 'Torque about the z axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque N about the vehicle's z axis.")
    '''Torque N about the vehicle's z axis. Type: fp64_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FL'}, "Desired Control flags. Bitfield (Local).")
    '''Desired Control flags. Bitfield (Local). Type: uint8_t'''

    def __init__(self, x = None, y = None, z = None, k = None, m = None, n = None, flags = None):
        '''Class constructor
        
        Desired Control flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: N

            y : fp64_t, unit: N

            z : fp64_t, unit: N

            k : fp64_t, unit: Nm

            m : fp64_t, unit: Nm

            n : fp64_t, unit: Nm

            flags : uint8_t, unit: Bitfield (Local)'''
        self._x = x
        self._y = y
        self._z = z
        self._k = k
        self._m = m
        self._n = n
        self._flags = flags


class DesiredHeadingRate(_base.base_message):
    '''The value of the desired heading rate speed in radians per second.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 408, stable = None, description = "Desired Heading Rate speed reference value for the control layer.", fields = ('value',), abbrev = "DesiredHeadingRate", name = "Desired Heading Rate", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': 'rad/s'}, "The value of the desired heading rate speed in radians per second.")
    '''The value of the desired heading rate speed in radians per second. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired heading rate speed in radians per second.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: rad/s'''
        self._value = value


class DesiredVelocity(_base.base_message):
    '''Desired Velocity flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    u : fp64_t, unit: m/s

            v : fp64_t, unit: m/s

            w : fp64_t, unit: m/s

            p : fp64_t, unit: m/s

            q : fp64_t, unit: m/s

            r : fp64_t, unit: m/s

            flags : uint8_t, unit: Bitfield (Local)'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FL'''
    
        EMPTY = 0
        '''No active flags'''
    
        SURGE = 1
        '''Name: Value of u is meaningful'''
    
        SWAY = 2
        '''Name: Value of v is meaningful'''
    
        HEAVE = 4
        '''Name: Value of w is meaningful'''
    
        ROLL = 8
        '''Name: Value of p is meaningful'''
    
        PITCH = 16
        '''Name: Value of q is meaningful'''
    
        YAW = 32
        '''Name: Value of r is meaningful'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_u', '_v', '_w', '_p', '_q', '_r', '_flags']
    Attributes = _base.MessageAttributes(source = "Vehicle", usedby = None, category = "Guidance", id = 409, stable = None, description = "Desired value for each linear and angular speeds.", fields = ('u', 'v', 'w', 'p', 'q', 'r', 'flags',), abbrev = "DesiredVelocity", name = "Desired Velocity", flags = None)

    u = _base.mutable_attr({'name': 'Desired Linear Speed in xx', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along the vehicle's x axis.")
    '''Desired speed along the vehicle's x axis. Type: fp64_t'''
    v = _base.mutable_attr({'name': 'Desired Linear Speed in yy', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along the vehicle's y axis.")
    '''Desired speed along the vehicle's y axis. Type: fp64_t'''
    w = _base.mutable_attr({'name': 'Desired Linear Speed in zz', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along the vehicle's z axis.")
    '''Desired speed along the vehicle's z axis. Type: fp64_t'''
    p = _base.mutable_attr({'name': 'Desired Angular Speed in xx', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed about the vehicle's x axis.")
    '''Desired speed about the vehicle's x axis. Type: fp64_t'''
    q = _base.mutable_attr({'name': 'Desired Angular Speed in yy', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed about the vehicle's y axis.")
    '''Desired speed about the vehicle's y axis. Type: fp64_t'''
    r = _base.mutable_attr({'name': 'Desired Angular Speed in zz', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed about the vehicle's z axis.")
    '''Desired speed about the vehicle's z axis. Type: fp64_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FL'}, "Desired Velocity flags. Bitfield (Local).")
    '''Desired Velocity flags. Bitfield (Local). Type: uint8_t'''

    def __init__(self, u = None, v = None, w = None, p = None, q = None, r = None, flags = None):
        '''Class constructor
        
        Desired Velocity flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    u : fp64_t, unit: m/s

            v : fp64_t, unit: m/s

            w : fp64_t, unit: m/s

            p : fp64_t, unit: m/s

            q : fp64_t, unit: m/s

            r : fp64_t, unit: m/s

            flags : uint8_t, unit: Bitfield (Local)'''
        self._u = u
        self._v = v
        self._w = w
        self._p = p
        self._q = q
        self._r = r
        self._flags = flags


class PathControlState(_base.base_message):
    '''Estimated time to reach target waypoint. The value will be 65535 if the time is unknown or undefined, and 0 when loitering.

       This message class contains the following fields and their respective types:
    path_ref : uint32_t, unit: NOT FOUND

            start_lat : fp64_t, unit: rad

            start_lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_lat : fp64_t, unit: rad

            end_lon : fp64_t, unit: rad

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            lradius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            course_error : fp32_t, unit: rad

            eta : uint16_t, unit: s'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FL'''
    
        EMPTY = 0
        '''No active flags'''
    
        NEAR = 1
        '''Name: Near Endpoint'''
    
        LOITERING = 2
        '''Name: Loitering'''
    
        NO_Z = 4
        '''Name: No Altitude/Depth control'''
    
        x3DTRACK = 8
        '''Name: 3D Tracking'''
    
        CCLOCKW = 16
        '''Name: Counter-Clockwise loiter'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_path_ref', '_start_lat', '_start_lon', '_start_z', '_start_z_units', '_end_lat', '_end_lon', '_end_z', '_end_z_units', '_lradius', '_flags', '_x', '_y', '_z', '_vx', '_vy', '_vz', '_course_error', '_eta']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 410, stable = None, description = "Path control state issued by Path Controller.", fields = ('path_ref', 'start_lat', 'start_lon', 'start_z', 'start_z_units', 'end_lat', 'end_lon', 'end_z', 'end_z_units', 'lradius', 'flags', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'course_error', 'eta',), abbrev = "PathControlState", name = "Path Control State", flags = None)

    path_ref = _base.mutable_attr({'name': 'Path Reference', 'type': 'uint32_t'}, "Unsigned integer reference of the desired path message to which this PathControlState message refers to. Path reference should only be set by a maneuver, not by path controllers.")
    '''Unsigned integer reference of the desired path message to which this PathControlState message refers to. Path reference should only be set by a maneuver, not by path controllers. Type: uint32_t'''
    start_lat = _base.mutable_attr({'name': 'Start Point -- Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 latitude of start point.")
    '''WGS-84 latitude of start point. Type: fp64_t'''
    start_lon = _base.mutable_attr({'name': 'Start Point -- WGS-84 Longitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 longitude of start point.")
    '''WGS-84 longitude of start point. Type: fp64_t'''
    start_z = _base.mutable_attr({'name': 'Start Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set.")
    '''Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set. Type: fp32_t'''
    start_z_units = _base.mutable_attr({'name': 'Start Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the start point's z reference. Enumerated (Global).")
    '''Units of the start point's z reference. Enumerated (Global). Type: uint8_t'''
    end_lat = _base.mutable_attr({'name': 'End Point -- Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 latitude of end point.")
    '''WGS-84 latitude of end point. Type: fp64_t'''
    end_lon = _base.mutable_attr({'name': 'End Point -- WGS-84 Longitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 longitude of end point.")
    '''WGS-84 longitude of end point. Type: fp64_t'''
    end_z = _base.mutable_attr({'name': 'End Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Depth or altitude for the end point. This parameter should be ignored if the 'NO_Z' flag is set.")
    '''Depth or altitude for the end point. This parameter should be ignored if the 'NO_Z' flag is set. Type: fp32_t'''
    end_z_units = _base.mutable_attr({'name': 'End Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the end point's z reference. Enumerated (Global).")
    '''Units of the end point's z reference. Enumerated (Global). Type: uint8_t'''
    lradius = _base.mutable_attr({'name': 'Loiter -- Radius', 'type': 'fp32_t', 'unit': 'm'}, "Radius for loitering at end point. Will be 0 if no loitering is active.")
    '''Radius for loitering at end point. Will be 0 if no loitering is active. Type: fp32_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FL'}, "Path control state flags. Bitfield (Local).")
    '''Path control state flags. Bitfield (Local). Type: uint8_t'''
    x = _base.mutable_attr({'name': 'Along Track Position', 'unit': 'm', 'type': 'fp32_t'}, "Along-Track position value.")
    '''Along-Track position value. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Cross Track Position', 'unit': 'm', 'type': 'fp32_t'}, "Cross-Track position value.")
    '''Cross-Track position value. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Vertical Track Position', 'unit': 'm', 'type': 'fp32_t'}, "Vertical-Track position value.")
    '''Vertical-Track position value. Type: fp32_t'''
    vx = _base.mutable_attr({'name': 'Along Track Velocity', 'unit': 'm/s', 'type': 'fp32_t'}, "Along-Track velocity value.")
    '''Along-Track velocity value. Type: fp32_t'''
    vy = _base.mutable_attr({'name': 'Cross Track Velocity', 'unit': 'm/s', 'type': 'fp32_t'}, "Cross-Track velocity value.")
    '''Cross-Track velocity value. Type: fp32_t'''
    vz = _base.mutable_attr({'name': 'Vertical Track Velocity', 'unit': 'm/s', 'type': 'fp32_t'}, "Vertical-Track velocity value.")
    '''Vertical-Track velocity value. Type: fp32_t'''
    course_error = _base.mutable_attr({'name': 'Course Error', 'unit': 'rad', 'type': 'fp32_t'}, "Course error value.")
    '''Course error value. Type: fp32_t'''
    eta = _base.mutable_attr({'name': 'Estimated Time to Arrival (ETA)', 'unit': 's', 'type': 'uint16_t'}, "Estimated time to reach target waypoint. The value will be 65535 if the time is unknown or undefined, and 0 when loitering.")
    '''Estimated time to reach target waypoint. The value will be 65535 if the time is unknown or undefined, and 0 when loitering. Type: uint16_t'''

    def __init__(self, path_ref = None, start_lat = None, start_lon = None, start_z = None, start_z_units = None, end_lat = None, end_lon = None, end_z = None, end_z_units = None, lradius = None, flags = None, x = None, y = None, z = None, vx = None, vy = None, vz = None, course_error = None, eta = None):
        '''Class constructor
        
        Estimated time to reach target waypoint. The value will be 65535 if the time is unknown or undefined, and 0 when loitering.

       This message class contains the following fields and their respective types:
    path_ref : uint32_t, unit: NOT FOUND

            start_lat : fp64_t, unit: rad

            start_lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_lat : fp64_t, unit: rad

            end_lon : fp64_t, unit: rad

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            lradius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            course_error : fp32_t, unit: rad

            eta : uint16_t, unit: s'''
        self._path_ref = path_ref
        self._start_lat = start_lat
        self._start_lon = start_lon
        self._start_z = start_z
        self._start_z_units = start_z_units
        self._end_lat = end_lat
        self._end_lon = end_lon
        self._end_z = end_z
        self._end_z_units = end_z_units
        self._lradius = lradius
        self._flags = flags
        self._x = x
        self._y = y
        self._z = z
        self._vx = vx
        self._vy = vy
        self._vz = vz
        self._course_error = course_error
        self._eta = eta


class AllocatedControlTorques(_base.base_message):
    '''Torque N about the vehicle's z axis.

       This message class contains the following fields and their respective types:
    k : fp64_t, unit: Nm

            m : fp64_t, unit: Nm

            n : fp64_t, unit: Nm'''

    __slots__ = ['_Attributes', '_header', '_footer', '_k', '_m', '_n']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 411, stable = None, description = "Control torques allocated to the actuators.", fields = ('k', 'm', 'n',), abbrev = "AllocatedControlTorques", name = "Allocated Control Torques", flags = None)

    k = _base.mutable_attr({'name': 'Torque about the x axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque K about the vehicle's x axis.")
    '''Torque K about the vehicle's x axis. Type: fp64_t'''
    m = _base.mutable_attr({'name': 'Torque about the y axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque M about the vehicle's y axis.")
    '''Torque M about the vehicle's y axis. Type: fp64_t'''
    n = _base.mutable_attr({'name': 'Torque about the x axis', 'type': 'fp64_t', 'unit': 'Nm'}, "Torque N about the vehicle's z axis.")
    '''Torque N about the vehicle's z axis. Type: fp64_t'''

    def __init__(self, k = None, m = None, n = None):
        '''Class constructor
        
        Torque N about the vehicle's z axis.

       This message class contains the following fields and their respective types:
    k : fp64_t, unit: Nm

            m : fp64_t, unit: Nm

            n : fp64_t, unit: Nm'''
        self._k = k
        self._m = m
        self._n = n


class ControlParcel(_base.base_message):
    '''Anti-windup parcel value.

       This message class contains the following fields and their respective types:
    p : fp32_t, unit: NOT FOUND

            i : fp32_t, unit: NOT FOUND

            d : fp32_t, unit: NOT FOUND

            a : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_p', '_i', '_d', '_a']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Guidance", id = 412, stable = None, description = "Report of PID control parcels.", fields = ('p', 'i', 'd', 'a',), abbrev = "ControlParcel", name = "Control Parcel", flags = None)

    p = _base.mutable_attr({'name': 'Proportional Parcel', 'type': 'fp32_t'}, "Proportional parcel value.")
    '''Proportional parcel value. Type: fp32_t'''
    i = _base.mutable_attr({'name': 'Integrative Parcel', 'type': 'fp32_t'}, "Integral parcel value.")
    '''Integral parcel value. Type: fp32_t'''
    d = _base.mutable_attr({'name': 'Derivative Parcel', 'type': 'fp32_t'}, "Derivative parcel value.")
    '''Derivative parcel value. Type: fp32_t'''
    a = _base.mutable_attr({'name': 'Anti-Windup Parcel', 'type': 'fp32_t'}, "Anti-windup parcel value.")
    '''Anti-windup parcel value. Type: fp32_t'''

    def __init__(self, p = None, i = None, d = None, a = None):
        '''Class constructor
        
        Anti-windup parcel value.

       This message class contains the following fields and their respective types:
    p : fp32_t, unit: NOT FOUND

            i : fp32_t, unit: NOT FOUND

            d : fp32_t, unit: NOT FOUND

            a : fp32_t, unit: NOT FOUND'''
        self._p = p
        self._i = i
        self._d = d
        self._a = a


class Brake(_base.base_message):
    '''Brake operation. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        STOP = 0
        '''Name: Stop Braking'''
    
        START = 1
        '''Name: Start Braking'''
    
        REVERT = 2
        '''Name: Revert Actuation'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Guidance", id = 413, stable = None, description = "Brake the vehicle in some way, i. e., reduce forward speed.", fields = ('op',), abbrev = "Brake", name = "Brake", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Brake operation. Enumerated (Local).")
    '''Brake operation. Enumerated (Local). Type: uint8_t'''

    def __init__(self, op = None):
        '''Class constructor
        
        Brake operation. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''
        self._op = op


class DesiredLinearState(_base.base_message):
    '''Setpoint Flags Bitfield (Local).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m

            y : fp64_t, unit: m

            z : fp64_t, unit: m

            vx : fp64_t, unit: m/s

            vy : fp64_t, unit: m/s

            vz : fp64_t, unit: m/s

            ax : fp64_t, unit: m/s/s

            ay : fp64_t, unit: m/s/s

            az : fp64_t, unit: m/s/s

            flags : uint16_t, unit: Bitfield (Local)'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FL'''
    
        EMPTY = 0
        '''No active flags'''
    
        X = 1
        '''Name: Value of x is meaningful'''
    
        Y = 2
        '''Name: Value of y is meaningful'''
    
        Z = 4
        '''Name: Value of z is meaningful'''
    
        VX = 8
        '''Name: Value of vx is meaningful'''
    
        VY = 16
        '''Name: Value of vy is meaningful'''
    
        VZ = 32
        '''Name: Value of vz is meaningful'''
    
        AX = 64
        '''Name: Value of ax is meaningful'''
    
        AY = 128
        '''Name: Value of ay is meaningful'''
    
        AZ = 256
        '''Name: Value of az is meaningful'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z', '_vx', '_vy', '_vz', '_ax', '_ay', '_az', '_flags']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Guidance", id = 414, stable = None, description = "Position, velocity and acceleration setpoints in NED", fields = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'flags',), abbrev = "DesiredLinearState", name = "Desired Linear State", flags = None)

    x = _base.mutable_attr({'name': 'Desired pos in xx', 'type': 'fp64_t', 'unit': 'm'}, "Desired pos in x.")
    '''Desired pos in x. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Desired pos in yy', 'type': 'fp64_t', 'unit': 'm'}, "Desired pos in y.")
    '''Desired pos in y. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Desired pos in zz', 'type': 'fp64_t', 'unit': 'm'}, "Desired pos in z.")
    '''Desired pos in z. Type: fp64_t'''
    vx = _base.mutable_attr({'name': 'Desired Linear Speed in xx', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along NED x axis.")
    '''Desired speed along NED x axis. Type: fp64_t'''
    vy = _base.mutable_attr({'name': 'Desired Linear Speed in yy', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along NED y axis.")
    '''Desired speed along NED y axis. Type: fp64_t'''
    vz = _base.mutable_attr({'name': 'Desired Linear Speed in zz', 'type': 'fp64_t', 'unit': 'm/s'}, "Desired speed along NED z axis.")
    '''Desired speed along NED z axis. Type: fp64_t'''
    ax = _base.mutable_attr({'name': 'Desired Linear Acceleration in xx', 'type': 'fp64_t', 'unit': 'm/s/s'}, "Desired acceleration along NED x axis.")
    '''Desired acceleration along NED x axis. Type: fp64_t'''
    ay = _base.mutable_attr({'name': 'Desired Linear Acceleration in yy', 'type': 'fp64_t', 'unit': 'm/s/s'}, "Desired acceleration along NED y axis.")
    '''Desired acceleration along NED y axis. Type: fp64_t'''
    az = _base.mutable_attr({'name': 'Desired Linear Acceleration in zz', 'type': 'fp64_t', 'unit': 'm/s/s'}, "Desired acceleration along NED z axis.")
    '''Desired acceleration along NED z axis. Type: fp64_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint16_t', 'unit': 'Bitfield', 'prefix': 'FL'}, "Setpoint Flags Bitfield (Local).")
    '''Setpoint Flags Bitfield (Local). Type: uint16_t'''

    def __init__(self, x = None, y = None, z = None, vx = None, vy = None, vz = None, ax = None, ay = None, az = None, flags = None):
        '''Class constructor
        
        Setpoint Flags Bitfield (Local).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m

            y : fp64_t, unit: m

            z : fp64_t, unit: m

            vx : fp64_t, unit: m/s

            vy : fp64_t, unit: m/s

            vz : fp64_t, unit: m/s

            ax : fp64_t, unit: m/s/s

            ay : fp64_t, unit: m/s/s

            az : fp64_t, unit: m/s/s

            flags : uint16_t, unit: Bitfield (Local)'''
        self._x = x
        self._y = y
        self._z = z
        self._vx = vx
        self._vy = vy
        self._vz = vz
        self._ax = ax
        self._ay = ay
        self._az = az
        self._flags = flags


class DesiredThrottle(_base.base_message):
    '''The value of the desired throttle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Guidance", id = 415, stable = None, description = "Desired throttle e.g. for Plane in FBWA-mode.", fields = ('value',), abbrev = "DesiredThrottle", name = "Desired Throttle", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': '%'}, "The value of the desired throttle.")
    '''The value of the desired throttle. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired throttle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: %'''
        self._value = value


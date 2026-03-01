'''
IMC Sensors messages.
'''

from .. import _base
import enum as _enum

class Rpm(_base.base_message):
    '''Number of revolutions per minute.

       This message class contains the following fields and their respective types:
    value : int16_t, unit: rpm'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 250, stable = None, description = "Number of revolutions per minute.", fields = ('value',), abbrev = "Rpm", name = "Revolutions Per Minute", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'int16_t', 'unit': 'rpm'}, "Number of revolutions per minute.")
    '''Number of revolutions per minute. Type: int16_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Number of revolutions per minute.

       This message class contains the following fields and their respective types:
    value : int16_t, unit: rpm'''
        self._value = value


class Voltage(_base.base_message):
    '''The value of the internal electrical voltage as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: V'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 251, stable = None, description = "Report of electrical voltage.", fields = ('value',), abbrev = "Voltage", name = "Voltage", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Voltage Value', 'type': 'fp32_t', 'unit': 'V'}, "The value of the internal electrical voltage as measured by the sensor.")
    '''The value of the internal electrical voltage as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the internal electrical voltage as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: V'''
        self._value = value


class Current(_base.base_message):
    '''The value of the internal electrical current as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: A'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 252, stable = None, description = "Report of electrical current.", fields = ('value',), abbrev = "Current", name = "Current", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Current Value', 'type': 'fp32_t', 'unit': 'A'}, "The value of the internal electrical current as measured by the sensor.")
    '''The value of the internal electrical current as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the internal electrical current as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: A'''
        self._value = value


class GpsFix(_base.base_message):
    '''Vertical Accuracy Estimate.

       This message class contains the following fields and their respective types:
    validity : uint16_t, unit: Bitfield (Local)

            type : uint8_t, unit: Enumerated (Local)

            utc_year : uint16_t, unit: NOT FOUND

            utc_month : uint8_t, unit: NOT FOUND

            utc_day : uint8_t, unit: NOT FOUND

            utc_time : fp32_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            satellites : uint8_t, unit: NOT FOUND

            cog : fp32_t, unit: rad

            sog : fp32_t, unit: m/s

            hdop : fp32_t, unit: NOT FOUND

            vdop : fp32_t, unit: NOT FOUND

            hacc : fp32_t, unit: m

            vacc : fp32_t, unit: m'''

    class VALIDITY(_enum.IntFlag):
        '''Full name: Validity
        Prefix: GFV'''
    
        EMPTY = 0
        '''No active flags'''
    
        VALID_DATE = 1
        '''Name: Valid Date'''
    
        VALID_TIME = 2
        '''Name: Valid Time'''
    
        VALID_POS = 4
        '''Name: Valid Position'''
    
        VALID_COG = 8
        '''Name: Valid Course Over Ground'''
    
        VALID_SOG = 16
        '''Name: Valid Speed Over Ground'''
    
        VALID_HACC = 32
        '''Name: Valid Horizontal Accuracy Estimate'''
    
        VALID_VACC = 64
        '''Name: Valid Vertical Accuracy Estimate'''
    
        VALID_HDOP = 128
        '''Name: Valid Horizontal Dilution of Precision'''
    
        VALID_VDOP = 256
        '''Name: Valid Vertical Dilution of Precision'''
    
    
    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: GFT'''
    
        STANDALONE = 0
        '''Name: Stand Alone'''
    
        DIFFERENTIAL = 1
        '''Name: Differential'''
    
        DEAD_RECKONING = 2
        '''Name: Dead Reckoning'''
    
        MANUAL_INPUT = 3
        '''Name: Manual Input'''
    
        SIMULATION = 4
        '''Name: Simulation'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_validity', '_type', '_utc_year', '_utc_month', '_utc_day', '_utc_time', '_lat', '_lon', '_height', '_satellites', '_cog', '_sog', '_hdop', '_vdop', '_hacc', '_vacc']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 253, stable = None, description = "Report of a GPS fix.", fields = ('validity', 'type', 'utc_year', 'utc_month', 'utc_day', 'utc_time', 'lat', 'lon', 'height', 'satellites', 'cog', 'sog', 'hdop', 'vdop', 'hacc', 'vacc',), abbrev = "GpsFix", name = "GPS Fix", flags = "periodic")

    validity = _base.mutable_attr({'name': 'Validity', 'type': 'uint16_t', 'unit': 'Bitfield', 'prefix': 'GFV'}, "Validity of fields. Bitfield (Local).")
    '''Validity of fields. Bitfield (Local). Type: uint16_t'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'GFT'}, "Type of fix. Enumerated (Local).")
    '''Type of fix. Enumerated (Local). Type: uint8_t'''
    utc_year = _base.mutable_attr({'name': 'UTC Year', 'type': 'uint16_t'}, "UTC year.")
    '''UTC year. Type: uint16_t'''
    utc_month = _base.mutable_attr({'name': 'UTC Month', 'type': 'uint8_t'}, "UTC month.")
    '''UTC month. Type: uint8_t'''
    utc_day = _base.mutable_attr({'name': 'UTC Day', 'type': 'uint8_t'}, "UTC day.")
    '''UTC day. Type: uint8_t'''
    utc_time = _base.mutable_attr({'name': 'UTC Time of Fix', 'type': 'fp32_t', 'unit': 's'}, "UTC time of the GPS fix measured in seconds since 00:00:00 (midnight).")
    '''UTC time of the GPS fix measured in seconds since 00:00:00 (midnight). Type: fp32_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    height = _base.mutable_attr({'name': 'Height above WGS-84 ellipsoid', 'type': 'fp32_t', 'unit': 'm'}, "Height above WGS-84 ellipsoid.")
    '''Height above WGS-84 ellipsoid. Type: fp32_t'''
    satellites = _base.mutable_attr({'name': 'Number of Satellites', 'type': 'uint8_t'}, "Number of satellites used by the GPS device to compute the solution.")
    '''Number of satellites used by the GPS device to compute the solution. Type: uint8_t'''
    cog = _base.mutable_attr({'name': 'Course Over Ground', 'type': 'fp32_t', 'unit': 'rad'}, "Course Over Ground (true).")
    '''Course Over Ground (true). Type: fp32_t'''
    sog = _base.mutable_attr({'name': 'Speed Over Ground', 'type': 'fp32_t', 'unit': 'm/s'}, "Speed Over Ground.")
    '''Speed Over Ground. Type: fp32_t'''
    hdop = _base.mutable_attr({'name': 'Horizontal Dilution of Precision', 'type': 'fp32_t'}, "Horizontal dilution of precision.")
    '''Horizontal dilution of precision. Type: fp32_t'''
    vdop = _base.mutable_attr({'name': 'Vertical Dilution of Precision', 'type': 'fp32_t'}, "Vertical dilution of precision.")
    '''Vertical dilution of precision. Type: fp32_t'''
    hacc = _base.mutable_attr({'name': 'Horizontal Accuracy Estimate', 'type': 'fp32_t', 'unit': 'm'}, "Horizontal Accuracy Estimate.")
    '''Horizontal Accuracy Estimate. Type: fp32_t'''
    vacc = _base.mutable_attr({'name': 'Vertical Accuracy Estimate', 'type': 'fp32_t', 'unit': 'm'}, "Vertical Accuracy Estimate.")
    '''Vertical Accuracy Estimate. Type: fp32_t'''

    def __init__(self, validity = None, type = None, utc_year = None, utc_month = None, utc_day = None, utc_time = None, lat = None, lon = None, height = None, satellites = None, cog = None, sog = None, hdop = None, vdop = None, hacc = None, vacc = None):
        '''Class constructor
        
        Vertical Accuracy Estimate.

       This message class contains the following fields and their respective types:
    validity : uint16_t, unit: Bitfield (Local)

            type : uint8_t, unit: Enumerated (Local)

            utc_year : uint16_t, unit: NOT FOUND

            utc_month : uint8_t, unit: NOT FOUND

            utc_day : uint8_t, unit: NOT FOUND

            utc_time : fp32_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            satellites : uint8_t, unit: NOT FOUND

            cog : fp32_t, unit: rad

            sog : fp32_t, unit: m/s

            hdop : fp32_t, unit: NOT FOUND

            vdop : fp32_t, unit: NOT FOUND

            hacc : fp32_t, unit: m

            vacc : fp32_t, unit: m'''
        self._validity = validity
        self._type = type
        self._utc_year = utc_year
        self._utc_month = utc_month
        self._utc_day = utc_day
        self._utc_time = utc_time
        self._lat = lat
        self._lon = lon
        self._height = height
        self._satellites = satellites
        self._cog = cog
        self._sog = sog
        self._hdop = hdop
        self._vdop = vdop
        self._hacc = hacc
        self._vacc = vacc


class EulerAngles(_base.base_message):
    '''Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards magnetic north. In cases where the sensor cannot measure the magnetic heading, this field will have the same value as Yaw (True).

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            phi : fp64_t, unit: rad

            theta : fp64_t, unit: rad

            psi : fp64_t, unit: rad

            psi_magnetic : fp64_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_phi', '_theta', '_psi', '_psi_magnetic']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 254, stable = None, description = "Report of spatial orientation according to SNAME's notation (1950).", fields = ('time', 'phi', 'theta', 'psi', 'psi_magnetic',), abbrev = "EulerAngles", name = "Euler Angles", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    phi = _base.mutable_attr({'name': 'Roll Angle', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the vehicle longitudinal axis.")
    '''Rotation around the vehicle longitudinal axis. Type: fp64_t'''
    theta = _base.mutable_attr({'name': 'Pitch Angle', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267949, 'max': 1.5707963267949}, "Rotation around the vehicle lateral or transverse axis.")
    '''Rotation around the vehicle lateral or transverse axis. Type: fp64_t'''
    psi = _base.mutable_attr({'name': 'Yaw Angle (True)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards true north. In cases where the sensor cannot measure the true heading, this field will have the same value as Yaw (Magnetic).")
    '''Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards true north. In cases where the sensor cannot measure the true heading, this field will have the same value as Yaw (Magnetic). Type: fp64_t'''
    psi_magnetic = _base.mutable_attr({'name': 'Yaw Angle (Magnetic)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards magnetic north. In cases where the sensor cannot measure the magnetic heading, this field will have the same value as Yaw (True).")
    '''Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards magnetic north. In cases where the sensor cannot measure the magnetic heading, this field will have the same value as Yaw (True). Type: fp64_t'''

    def __init__(self, time = None, phi = None, theta = None, psi = None, psi_magnetic = None):
        '''Class constructor
        
        Rotation around the vehicle vertical axis. A value of 0 means the vehicle is oriented towards magnetic north. In cases where the sensor cannot measure the magnetic heading, this field will have the same value as Yaw (True).

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            phi : fp64_t, unit: rad

            theta : fp64_t, unit: rad

            psi : fp64_t, unit: rad

            psi_magnetic : fp64_t, unit: rad'''
        self._time = time
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._psi_magnetic = psi_magnetic


class EulerAnglesDelta(_base.base_message):
    '''Period of time of the orientation vector increments.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: rad

            y : fp64_t, unit: rad

            z : fp64_t, unit: rad

            timestep : fp32_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_x', '_y', '_z', '_timestep']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 255, stable = None, description = "Component of incremetal orientation vector over a period of time.", fields = ('time', 'x', 'y', 'z', 'timestep',), abbrev = "EulerAnglesDelta", name = "Euler Angles Delta", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'rad'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'rad'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'rad'}, "Z component.")
    '''Z component. Type: fp64_t'''
    timestep = _base.mutable_attr({'name': 'Timestep', 'type': 'fp32_t', 'unit': 's'}, "Period of time of the orientation vector increments.")
    '''Period of time of the orientation vector increments. Type: fp32_t'''

    def __init__(self, time = None, x = None, y = None, z = None, timestep = None):
        '''Class constructor
        
        Period of time of the orientation vector increments.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: rad

            y : fp64_t, unit: rad

            z : fp64_t, unit: rad

            timestep : fp32_t, unit: s'''
        self._time = time
        self._x = x
        self._y = y
        self._z = z
        self._timestep = timestep


class AngularVelocity(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: rad/s

            y : fp64_t, unit: rad/s

            z : fp64_t, unit: rad/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 256, stable = None, description = "Vector quantifying the direction and magnitude of the measured angular velocity that a device is exposed to.", fields = ('time', 'x', 'y', 'z',), abbrev = "AngularVelocity", name = "Angular Velocity", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'rad/s'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'rad/s'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'rad/s'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, time = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: rad/s

            y : fp64_t, unit: rad/s

            z : fp64_t, unit: rad/s'''
        self._time = time
        self._x = x
        self._y = y
        self._z = z


class Acceleration(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: m/s/s

            y : fp64_t, unit: m/s/s

            z : fp64_t, unit: m/s/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 257, stable = None, description = "Vector quantifying the direction and magnitude of the measured acceleration that a device is exposed to.", fields = ('time', 'x', 'y', 'z',), abbrev = "Acceleration", name = "Acceleration", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'm/s/s'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'm/s/s'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'm/s/s'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, time = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: m/s/s

            y : fp64_t, unit: m/s/s

            z : fp64_t, unit: m/s/s'''
        self._time = time
        self._x = x
        self._y = y
        self._z = z


class MagneticField(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: G

            y : fp64_t, unit: G

            z : fp64_t, unit: G'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 258, stable = None, description = "Vector quantifying the direction and magnitude of the measured magnetic field that a device is exposed to.", fields = ('time', 'x', 'y', 'z',), abbrev = "MagneticField", name = "Magnetic Field", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'G'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'G'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'G'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, time = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: G

            y : fp64_t, unit: G

            z : fp64_t, unit: G'''
        self._time = time
        self._x = x
        self._y = y
        self._z = z


class GroundVelocity(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Bitfield (Local)

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''

    class VALIDITY(_enum.IntFlag):
        '''Full name: Validity
        Prefix: VAL'''
    
        EMPTY = 0
        '''No active flags'''
    
        VEL_X = 1
        '''Name: X component is valid'''
    
        VEL_Y = 2
        '''Name: Y component is valid'''
    
        VEL_Z = 4
        '''Name: Z component is valid'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_validity', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 259, stable = None, description = "Vector quantifying the direction and magnitude of the measured velocity relative to the ground that a device is exposed to.", fields = ('validity', 'x', 'y', 'z',), abbrev = "GroundVelocity", name = "Ground Velocity", flags = "periodic")

    validity = _base.mutable_attr({'name': 'Validity', 'unit': 'Bitfield', 'type': 'uint8_t', 'prefix': 'VAL'}, "Each bit of this field represents if a given velocity component is valid. Bitfield (Local).")
    '''Each bit of this field represents if a given velocity component is valid. Bitfield (Local). Type: uint8_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'm/s'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'm/s'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'm/s'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, validity = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Bitfield (Local)

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''
        self._validity = validity
        self._x = x
        self._y = y
        self._z = z


class WaterVelocity(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Bitfield (Local)

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''

    class VALIDITY(_enum.IntFlag):
        '''Full name: Validity
        Prefix: VAL'''
    
        EMPTY = 0
        '''No active flags'''
    
        VEL_X = 1
        '''Name: X component is valid'''
    
        VEL_Y = 2
        '''Name: Y component is valid'''
    
        VEL_Z = 4
        '''Name: Z component is valid'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_validity', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 260, stable = None, description = "Vector quantifying the direction and magnitude of the measured velocity relative to the water that a device is exposed to.", fields = ('validity', 'x', 'y', 'z',), abbrev = "WaterVelocity", name = "Water Velocity", flags = "periodic")

    validity = _base.mutable_attr({'name': 'Validity', 'unit': 'Bitfield', 'type': 'uint8_t', 'prefix': 'VAL'}, "Each bit of this field represents if a given velocity component is valid. Bitfield (Local).")
    '''Each bit of this field represents if a given velocity component is valid. Bitfield (Local). Type: uint8_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'm/s'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'm/s'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'm/s'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, validity = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Bitfield (Local)

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''
        self._validity = validity
        self._x = x
        self._y = y
        self._z = z


class VelocityDelta(_base.base_message):
    '''Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_time', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 261, stable = None, description = "Component of incremetal velocity vector.", fields = ('time', 'x', 'y', 'z',), abbrev = "VelocityDelta", name = "Velocity Delta", flags = "periodic")

    time = _base.mutable_attr({'name': 'Device Time', 'type': 'fp64_t', 'unit': 's'}, "The device time.")
    '''The device time. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp64_t', 'unit': 'm/s'}, "X component.")
    '''X component. Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp64_t', 'unit': 'm/s'}, "Y component.")
    '''Y component. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp64_t', 'unit': 'm/s'}, "Z component.")
    '''Z component. Type: fp64_t'''

    def __init__(self, time = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z component.

       This message class contains the following fields and their respective types:
    time : fp64_t, unit: s

            x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''
        self._time = time
        self._x = x
        self._y = y
        self._z = z


class Distance(_base.base_message):
    '''Measured distance.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Enumerated (Local)

            location : message-list, unit: NOT FOUND

            beam_config : message-list, unit: NOT FOUND

            value : fp32_t, unit: m'''

    class VALIDITY(_enum.IntEnum):
        '''Full name: Validity
        Prefix: DV'''
    
        INVALID = 0
        '''Name: Invalid'''
    
        VALID = 1
        '''Name: Valid'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_validity', '_location', '_beam_config', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 262, stable = None, description = "Distance measurement detected by the device.", fields = ('validity', 'location', 'beam_config', 'value',), abbrev = "Distance", name = "Distance", flags = "periodic")

    validity = _base.mutable_attr({'name': 'Validity', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DV'}, "Validity of the measurement. Enumerated (Local).")
    '''Validity of the measurement. Enumerated (Local). Type: uint8_t'''
    location = _base.mutable_attr({'name': 'Location', 'type': 'message-list', 'message-type': 'DeviceState'}, "Device Location in the system.")
    '''Device Location in the system. Type: message-list'''
    beam_config = _base.mutable_attr({'name': 'Beam Configuration', 'type': 'message-list', 'message-type': 'BeamConfig'}, "Beam configuration of the device.")
    '''Beam configuration of the device. Type: message-list'''
    value = _base.mutable_attr({'name': 'Measured Distance', 'type': 'fp32_t', 'unit': 'm'}, "Measured distance.")
    '''Measured distance. Type: fp32_t'''

    def __init__(self, validity = None, location = None, beam_config = None, value = None):
        '''Class constructor
        
        Measured distance.

       This message class contains the following fields and their respective types:
    validity : uint8_t, unit: Enumerated (Local)

            location : message-list, unit: NOT FOUND

            beam_config : message-list, unit: NOT FOUND

            value : fp32_t, unit: m'''
        self._validity = validity
        self._location = location
        self._beam_config = beam_config
        self._value = value


class Temperature(_base.base_message):
    '''The value of the temperature as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: °C'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 263, stable = None, description = "Report of temperature.", fields = ('value',), abbrev = "Temperature", name = "Temperature", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Temperature', 'type': 'fp32_t', 'unit': '°C'}, "The value of the temperature as measured by the sensor.")
    '''The value of the temperature as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the temperature as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: °C'''
        self._value = value


class Pressure(_base.base_message):
    '''The value of the pressure as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: hPa'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 264, stable = None, description = "Report of external pressure.", fields = ('value',), abbrev = "Pressure", name = "Pressure", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Pressure', 'type': 'fp64_t', 'unit': 'hPa'}, "The value of the pressure as measured by the sensor.")
    '''The value of the pressure as measured by the sensor. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the pressure as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: hPa'''
        self._value = value


class Depth(_base.base_message):
    '''Depth value measured by a sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 265, stable = None, description = "Depth report.", fields = ('value',), abbrev = "Depth", name = "Depth", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Depth', 'type': 'fp32_t', 'unit': 'm'}, "Depth value measured by a sensor.")
    '''Depth value measured by a sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Depth value measured by a sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m'''
        self._value = value


class DepthOffset(_base.base_message):
    '''Depth offset.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 266, stable = None, description = "Report of Depth Offset.", fields = ('value',), abbrev = "DepthOffset", name = "Depth Offset", flags = None)

    value = _base.mutable_attr({'name': 'Measured Offset', 'type': 'fp32_t', 'unit': 'm'}, "Depth offset.")
    '''Depth offset. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Depth offset.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m'''
        self._value = value


class SoundSpeed(_base.base_message):
    '''Estimated sound speed. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 267, stable = None, description = "Sound Speed report.", fields = ('value',), abbrev = "SoundSpeed", name = "Sound Speed", flags = "periodic")

    value = _base.mutable_attr({'name': 'Computed Sound Speed', 'type': 'fp32_t', 'unit': 'm/s'}, "Estimated sound speed. Negative values denote invalid estimates.")
    '''Estimated sound speed. Negative values denote invalid estimates. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Estimated sound speed. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m/s'''
        self._value = value


class WaterDensity(_base.base_message):
    '''Computed Water Density.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: kg/m/m/m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 268, stable = None, description = "Water Density report.", fields = ('value',), abbrev = "WaterDensity", name = "Water Density", flags = "periodic")

    value = _base.mutable_attr({'name': 'Computed Water Density', 'type': 'fp32_t', 'unit': 'kg/m/m/m'}, "Computed Water Density.")
    '''Computed Water Density. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Computed Water Density.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: kg/m/m/m'''
        self._value = value


class Conductivity(_base.base_message):
    '''The value of the conductivity as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: S/m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 269, stable = None, description = "Report of conductivity.", fields = ('value',), abbrev = "Conductivity", name = "Conductivity", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Conductivity', 'type': 'fp32_t', 'unit': 'S/m'}, "The value of the conductivity as measured by the sensor.")
    '''The value of the conductivity as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the conductivity as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: S/m'''
        self._value = value


class Salinity(_base.base_message):
    '''The value of the salinity as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PSU'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 270, stable = None, description = "Report of salinity.", fields = ('value',), abbrev = "Salinity", name = "Salinity", flags = "periodic")

    value = _base.mutable_attr({'name': 'Measured Salinity', 'type': 'fp32_t', 'unit': 'PSU'}, "The value of the salinity as measured by the sensor.")
    '''The value of the salinity as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the salinity as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PSU'''
        self._value = value


class WindSpeed(_base.base_message):
    '''Wind turbulence intensity.

       This message class contains the following fields and their respective types:
    direction : fp32_t, unit: rad

            speed : fp32_t, unit: m/s

            turbulence : fp32_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_direction', '_speed', '_turbulence']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 271, stable = None, description = "Measurement of wind speed.", fields = ('direction', 'speed', 'turbulence',), abbrev = "WindSpeed", name = "Wind Speed", flags = "periodic")

    direction = _base.mutable_attr({'name': 'Direction', 'type': 'fp32_t', 'unit': 'rad'}, "Direction of the measured wind speed.")
    '''Direction of the measured wind speed. Type: fp32_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t', 'unit': 'm/s'}, "The value of the wind speed as measured by the sensor.")
    '''The value of the wind speed as measured by the sensor. Type: fp32_t'''
    turbulence = _base.mutable_attr({'name': 'Turbulence', 'type': 'fp32_t', 'unit': 'm/s'}, "Wind turbulence intensity.")
    '''Wind turbulence intensity. Type: fp32_t'''

    def __init__(self, direction = None, speed = None, turbulence = None):
        '''Class constructor
        
        Wind turbulence intensity.

       This message class contains the following fields and their respective types:
    direction : fp32_t, unit: rad

            speed : fp32_t, unit: m/s

            turbulence : fp32_t, unit: m/s'''
        self._direction = direction
        self._speed = speed
        self._turbulence = turbulence


class RelativeHumidity(_base.base_message):
    '''Value of relative humidity.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 272, stable = None, description = "Measurement of relative humidity.", fields = ('value',), abbrev = "RelativeHumidity", name = "Relative Humidity", flags = "periodic")

    value = _base.mutable_attr({'name': 'Relative Humidity Value', 'type': 'fp32_t', 'min': 0, 'max': 100}, "Value of relative humidity.")
    '''Value of relative humidity. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Value of relative humidity.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''
        self._value = value


class DevDataText(_base.base_message):
    '''Plain text data as extracted directly from the device.

       This message class contains the following fields and their respective types:
    value : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 273, stable = None, description = "Verbatim representation of device data in plain text format.", fields = ('value',), abbrev = "DevDataText", name = "Device Data (Text)", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'plaintext'}, "Plain text data as extracted directly from the device.")
    '''Plain text data as extracted directly from the device. Type: plaintext'''

    def __init__(self, value = None):
        '''Class constructor
        
        Plain text data as extracted directly from the device.

       This message class contains the following fields and their respective types:
    value : plaintext, unit: NOT FOUND'''
        self._value = value


class DevDataBinary(_base.base_message):
    '''Raw binary data as extracted directly from the device.

       This message class contains the following fields and their respective types:
    value : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 274, stable = None, description = "Verbatim representation of device data in binary format.", fields = ('value',), abbrev = "DevDataBinary", name = "Device Data (Binary)", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'rawdata'}, "Raw binary data as extracted directly from the device.")
    '''Raw binary data as extracted directly from the device. Type: rawdata'''

    def __init__(self, value = None):
        '''Class constructor
        
        Raw binary data as extracted directly from the device.

       This message class contains the following fields and their respective types:
    value : rawdata, unit: NOT FOUND'''
        self._value = value


class Force(_base.base_message):
    '''Force magnitude.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: N'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 275, stable = None, description = "Force measurement.", fields = ('value',), abbrev = "Force", name = "Force", flags = None)

    value = _base.mutable_attr({'name': 'Measured Force', 'type': 'fp32_t', 'unit': 'N'}, "Force magnitude.")
    '''Force magnitude. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Force magnitude.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: N'''
        self._value = value


class SonarData(_base.base_message):
    '''Data acquired by the measurement.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            frequency : uint32_t, unit: Hz

            min_range : uint16_t, unit: m

            max_range : uint16_t, unit: m

            bits_per_point : uint8_t, unit: bit

            scale_factor : fp32_t, unit: NOT FOUND

            beam_config : message-list, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: ST'''
    
        SIDESCAN = 0
        '''Name: Sidescan'''
    
        ECHOSOUNDER = 1
        '''Name: Echo Sounder'''
    
        MULTIBEAM = 2
        '''Name: Multibeam'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_frequency', '_min_range', '_max_range', '_bits_per_point', '_scale_factor', '_beam_config', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 276, stable = None, description = "This message contains the data acquired by a single sonar measurement. The following describes the format used to fill the data field used in this message. (Byte order is little endian.) **Sidescan:** +------+-------------------+-----------+ | Data | Name | Type | +======+===================+===========+ | A | Ranges data | uintX_t | +------+-------------------+-----------+ .. figure:: ../images/imc_sidescan.png * The type *uintX_t* will depend on the number of bits per unit, and it should be a multiple of 8. * Furthermore, for now, 32 bits is the highest value of bits per unit supported. **Multibeam:** +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | Index| Section| Name | Type | Comments | +======+========+=========================+=========+======================================================================+ | 1 | H1 | Number of points | uint16_t| Number of data points | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 2 | H2 | Start angle | fp32_t | In radians | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 3 | H3 | Flags | uint8_t | Refer to next table | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 4 | H4 ? | Angle scale factor | fp32_t | Used for angle steps in radians | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 5 | H5 ? | Intensities scale factor| fp32_t | | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 6 | D1 ? | Angle steps[H1] | uint16_t| Values in radians | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 7 | D2 | Ranges[H1] | uintX_t | Ranges data points (scale factor from common field \"Scaling Factor\") | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ | 8 | D3 ? | Intensities[H1] | uintX_t | Intensities data points | +------+--------+-------------------------+---------+----------------------------------------------------------------------+ +--------+------------------+-----+ | Section| Flag Label | Bit | +========+==================+=====+ | H3.1 | Intensities flag | 0 | +--------+------------------+-----+ | H3.2 | Angle step flag | 1 | +--------+------------------+-----+ .. figure:: ../images/imc_multibeam.png *Notes:* * Each angle at step *i* can be calculated is defined by: .. code-block:: python angle[i] = H2_start_angle + (32-bit sum of D1_angle_step[0] through D1_angle_step[i]) * H4_scaling_factor * If bit H3.1 is not set then sections H5 and D3 won't exist. * If bit H3.2 is not set then sections H4 and D1 won't exist. In case this bit is set, then the angle steps is read from field \"Beam Width\" from \"Beam Configuration\". * The type *uintX_t* will depend on the number of bits per unit, and it should be a multiple of 8. * Furthermore, for now, 32 bits is the highest value of bits per unit supported. *How to write ranges and intensities data:* .. code-block:: python :linenos: data_unit = (Integer) (data_value / scale_factor); bytes_per_unit = bits_per_unit / 8; LOOP: i = 0, until i = bytes_per_unit byte[i] = (data_unit >> 8 * i) & 0xFF); write(byte); **Common:**", fields = ('type', 'frequency', 'min_range', 'max_range', 'bits_per_point', 'scale_factor', 'beam_config', 'data',), abbrev = "SonarData", name = "Sonar Data", flags = "periodic")

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'ST'}, "Type of sonar. Enumerated (Local).")
    '''Type of sonar. Enumerated (Local). Type: uint8_t'''
    frequency = _base.mutable_attr({'name': 'Frequency', 'type': 'uint32_t', 'unit': 'Hz'}, "Operating frequency.")
    '''Operating frequency. Type: uint32_t'''
    min_range = _base.mutable_attr({'name': 'Minimum Range', 'type': 'uint16_t', 'unit': 'm'}, "Minimum range.")
    '''Minimum range. Type: uint16_t'''
    max_range = _base.mutable_attr({'name': 'Maximum Range', 'type': 'uint16_t', 'unit': 'm'}, "Maximum range.")
    '''Maximum range. Type: uint16_t'''
    bits_per_point = _base.mutable_attr({'name': 'Bits Per Data Point', 'type': 'uint8_t', 'unit': 'bit'}, "Size of the data unit. (Should be multiple of 8)")
    '''Size of the data unit. (Should be multiple of 8) Type: uint8_t'''
    scale_factor = _base.mutable_attr({'name': 'Scaling Factor', 'type': 'fp32_t'}, "Scaling factor used to multiply each data unit to restore the original floating point value.")
    '''Scaling factor used to multiply each data unit to restore the original floating point value. Type: fp32_t'''
    beam_config = _base.mutable_attr({'name': 'Beam Configuration', 'type': 'message-list', 'message-type': 'BeamConfig'}, "Beam configuration of the device.")
    '''Beam configuration of the device. Type: message-list'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Data acquired by the measurement.")
    '''Data acquired by the measurement. Type: rawdata'''

    def __init__(self, type = None, frequency = None, min_range = None, max_range = None, bits_per_point = None, scale_factor = None, beam_config = None, data = None):
        '''Class constructor
        
        Data acquired by the measurement.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            frequency : uint32_t, unit: Hz

            min_range : uint16_t, unit: m

            max_range : uint16_t, unit: m

            bits_per_point : uint8_t, unit: bit

            scale_factor : fp32_t, unit: NOT FOUND

            beam_config : message-list, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''
        self._type = type
        self._frequency = frequency
        self._min_range = min_range
        self._max_range = max_range
        self._bits_per_point = bits_per_point
        self._scale_factor = scale_factor
        self._beam_config = beam_config
        self._data = data


class Pulse(_base.base_message):
    '''Hardware pulse detection.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Sensors", id = 277, stable = None, description = "Hardware pulse detection.", fields = [], abbrev = "Pulse", name = "Pulse", flags = "periodic")


    def __init__(self, ):
        '''Class constructor
        
        Hardware pulse detection.

       This message class contains the following fields and their respective types:
'''


class PulseDetectionControl(_base.base_message):
    '''Activate or deactivate hardware pulse detection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: POP'''
    
        OFF = 0
        '''Name: Pulse Detection OFF'''
    
        ON = 1
        '''Name: Pulse Detection ON'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Sensors", id = 278, stable = None, description = "Control of hardware pulse detection.", fields = ('op',), abbrev = "PulseDetectionControl", name = "Pulse Detection Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'prefix': 'POP', 'unit': 'Enumerated'}, "Activate or deactivate hardware pulse detection. Enumerated (Local).")
    '''Activate or deactivate hardware pulse detection. Enumerated (Local). Type: uint8_t'''

    def __init__(self, op = None):
        '''Class constructor
        
        Activate or deactivate hardware pulse detection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''
        self._op = op


class FuelLevel(_base.base_message):
    '''Operation mode name and the estimated time available in that mode in hours. Example: \"Motion=1.5\"

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %

            confidence : fp32_t, unit: %

            opmodes : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_confidence', '_opmodes']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 279, stable = None, description = "Report of fuel level.", fields = ('value', 'confidence', 'opmodes',), abbrev = "FuelLevel", name = "Fuel Level", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': '%', 'min': 0, 'max': 100}, "Fuel level percentage of the system.")
    '''Fuel level percentage of the system. Type: fp32_t'''
    confidence = _base.mutable_attr({'name': 'Confidence Level', 'type': 'fp32_t', 'unit': '%', 'min': 0, 'max': 100}, "Percentage level of confidence in the estimation of the amount of energy in the batteries.")
    '''Percentage level of confidence in the estimation of the amount of energy in the batteries. Type: fp32_t'''
    opmodes = _base.mutable_attr({'name': 'Operation Modes', 'type': 'plaintext', 'unit': 'TupleList'}, "Operation mode name and the estimated time available in that mode in hours. Example: \"Motion=1.5\"")
    '''Operation mode name and the estimated time available in that mode in hours. Example: \"Motion=1.5\" Type: plaintext'''

    def __init__(self, value = None, confidence = None, opmodes = None):
        '''Class constructor
        
        Operation mode name and the estimated time available in that mode in hours. Example: \"Motion=1.5\"

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %

            confidence : fp32_t, unit: %

            opmodes : plaintext, unit: TupleList'''
        self._value = value
        self._confidence = confidence
        self._opmodes = opmodes


class GpsNavData(_base.base_message):
    '''Course / Heading Accuracy Estimate.

       This message class contains the following fields and their respective types:
    itow : uint32_t, unit: ms

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height_ell : fp32_t, unit: m

            height_sea : fp32_t, unit: m

            hacc : fp32_t, unit: m

            vacc : fp32_t, unit: m

            vel_n : fp32_t, unit: m/s

            vel_e : fp32_t, unit: m/s

            vel_d : fp32_t, unit: m/s

            speed : fp32_t, unit: m/s

            gspeed : fp32_t, unit: m/s

            heading : fp32_t, unit: rad

            sacc : fp32_t, unit: m/s

            cacc : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_itow', '_lat', '_lon', '_height_ell', '_height_sea', '_hacc', '_vacc', '_vel_n', '_vel_e', '_vel_d', '_speed', '_gspeed', '_heading', '_sacc', '_cacc']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Sensors", id = 280, stable = None, description = "Report of GPS navigation data.", fields = ('itow', 'lat', 'lon', 'height_ell', 'height_sea', 'hacc', 'vacc', 'vel_n', 'vel_e', 'vel_d', 'speed', 'gspeed', 'heading', 'sacc', 'cacc',), abbrev = "GpsNavData", name = "GPS Navigation Data", flags = None)

    itow = _base.mutable_attr({'name': 'GPS Millisecond Time of Week', 'type': 'uint32_t', 'unit': 'ms'}, "GPS Millisecond Time of Week.")
    '''GPS Millisecond Time of Week. Type: uint32_t'''
    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "Latitude.")
    '''Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "Longitude.")
    '''Longitude. Type: fp64_t'''
    height_ell = _base.mutable_attr({'name': 'Height above ellipsoid', 'type': 'fp32_t', 'unit': 'm'}, "Height Above Ellipsoid.")
    '''Height Above Ellipsoid. Type: fp32_t'''
    height_sea = _base.mutable_attr({'name': 'Height above sea level', 'type': 'fp32_t', 'unit': 'm'}, "Height Above Sea Level.")
    '''Height Above Sea Level. Type: fp32_t'''
    hacc = _base.mutable_attr({'name': 'Horizontal Accuracy Estimate', 'type': 'fp32_t', 'unit': 'm'}, "Horizontal Accuracy Estimate.")
    '''Horizontal Accuracy Estimate. Type: fp32_t'''
    vacc = _base.mutable_attr({'name': 'Vertical Accuracy Estimate', 'type': 'fp32_t', 'unit': 'm'}, "Vertical Accuracy Estimate.")
    '''Vertical Accuracy Estimate. Type: fp32_t'''
    vel_n = _base.mutable_attr({'name': 'NED North Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "NED North Velocity.")
    '''NED North Velocity. Type: fp32_t'''
    vel_e = _base.mutable_attr({'name': 'NED East Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "NED East Velocity.")
    '''NED East Velocity. Type: fp32_t'''
    vel_d = _base.mutable_attr({'name': 'NED Down Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "NED Down Velocity.")
    '''NED Down Velocity. Type: fp32_t'''
    speed = _base.mutable_attr({'name': 'Speed (3D)', 'type': 'fp32_t', 'unit': 'm/s'}, "NED Down Velocity.")
    '''NED Down Velocity. Type: fp32_t'''
    gspeed = _base.mutable_attr({'name': 'Ground Speed (2D)', 'type': 'fp32_t', 'unit': 'm/s'}, "NED Down Velocity.")
    '''NED Down Velocity. Type: fp32_t'''
    heading = _base.mutable_attr({'name': 'Heading (2D)', 'type': 'fp32_t', 'unit': 'rad'}, "NED Down Velocity.")
    '''NED Down Velocity. Type: fp32_t'''
    sacc = _base.mutable_attr({'name': 'Speed Accuracy Estimate', 'type': 'fp32_t', 'unit': 'm/s'}, "NED Down Velocity.")
    '''NED Down Velocity. Type: fp32_t'''
    cacc = _base.mutable_attr({'name': 'Course / Heading Accuracy Estimate', 'type': 'fp32_t', 'unit': 'rad'}, "Course / Heading Accuracy Estimate.")
    '''Course / Heading Accuracy Estimate. Type: fp32_t'''

    def __init__(self, itow = None, lat = None, lon = None, height_ell = None, height_sea = None, hacc = None, vacc = None, vel_n = None, vel_e = None, vel_d = None, speed = None, gspeed = None, heading = None, sacc = None, cacc = None):
        '''Class constructor
        
        Course / Heading Accuracy Estimate.

       This message class contains the following fields and their respective types:
    itow : uint32_t, unit: ms

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height_ell : fp32_t, unit: m

            height_sea : fp32_t, unit: m

            hacc : fp32_t, unit: m

            vacc : fp32_t, unit: m

            vel_n : fp32_t, unit: m/s

            vel_e : fp32_t, unit: m/s

            vel_d : fp32_t, unit: m/s

            speed : fp32_t, unit: m/s

            gspeed : fp32_t, unit: m/s

            heading : fp32_t, unit: rad

            sacc : fp32_t, unit: m/s

            cacc : fp32_t, unit: rad'''
        self._itow = itow
        self._lat = lat
        self._lon = lon
        self._height_ell = height_ell
        self._height_sea = height_sea
        self._hacc = hacc
        self._vacc = vacc
        self._vel_n = vel_n
        self._vel_e = vel_e
        self._vel_d = vel_d
        self._speed = speed
        self._gspeed = gspeed
        self._heading = heading
        self._sacc = sacc
        self._cacc = cacc


class ServoPosition(_base.base_message):
    '''Value of the servo position.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 281, stable = None, description = "Actual position of a servo.", fields = ('id', 'value',), abbrev = "ServoPosition", name = "Servo Position", flags = None)

    id = _base.mutable_attr({'name': 'Identifier', 'type': 'uint8_t'}, "Servo identifier.")
    '''Servo identifier. Type: uint8_t'''
    value = _base.mutable_attr({'name': 'Position', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "Value of the servo position.")
    '''Value of the servo position. Type: fp32_t'''

    def __init__(self, id = None, value = None):
        '''Class constructor
        
        Value of the servo position.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: rad'''
        self._id = id
        self._value = value


class DeviceState(_base.base_message):
    '''Device's rotation over the Z axis.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z', '_phi', '_theta', '_psi']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 282, stable = None, description = "Location of a specific device in the system infrastructure.", fields = ('x', 'y', 'z', 'phi', 'theta', 'psi',), abbrev = "DeviceState", name = "Device State", flags = None)

    x = _base.mutable_attr({'name': 'Device Position - X', 'type': 'fp32_t', 'unit': 'm'}, "Device's position over the X axis.")
    '''Device's position over the X axis. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Device Position - Y', 'type': 'fp32_t', 'unit': 'm'}, "Device's position over the Y axis.")
    '''Device's position over the Y axis. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Device Position - Z', 'type': 'fp32_t', 'unit': 'm'}, "Device's position over the Z axis.")
    '''Device's position over the Z axis. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Device Rotation - X', 'type': 'fp32_t', 'unit': 'rad'}, "Device's rotation over the X axis.")
    '''Device's rotation over the X axis. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Device Rotation - Y', 'type': 'fp32_t', 'unit': 'rad'}, "Device's rotation over the Y axis.")
    '''Device's rotation over the Y axis. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Device Rotation - Z', 'type': 'fp32_t', 'unit': 'rad'}, "Device's rotation over the Z axis.")
    '''Device's rotation over the Z axis. Type: fp32_t'''

    def __init__(self, x = None, y = None, z = None, phi = None, theta = None, psi = None):
        '''Class constructor
        
        Device's rotation over the Z axis.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad'''
        self._x = x
        self._y = y
        self._z = z
        self._phi = phi
        self._theta = theta
        self._psi = psi


class BeamConfig(_base.base_message):
    '''Beam height of the instrument. A negative number denotes that this information is not available or is not applicable.

       This message class contains the following fields and their respective types:
    beam_width : fp32_t, unit: rad

            beam_height : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_beam_width', '_beam_height']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 283, stable = None, description = "Beam configuration of the device.", fields = ('beam_width', 'beam_height',), abbrev = "BeamConfig", name = "Beam Configuration", flags = None)

    beam_width = _base.mutable_attr({'name': 'Beam Width', 'type': 'fp32_t', 'unit': 'rad', 'min': 0, 'max': 3.141592653589793}, "Beam width of the instrument. A negative number denotes that this information is not available or is not applicable.")
    '''Beam width of the instrument. A negative number denotes that this information is not available or is not applicable. Type: fp32_t'''
    beam_height = _base.mutable_attr({'name': 'Beam Height', 'type': 'fp32_t', 'unit': 'rad', 'min': 0, 'max': 3.141592653589793}, "Beam height of the instrument. A negative number denotes that this information is not available or is not applicable.")
    '''Beam height of the instrument. A negative number denotes that this information is not available or is not applicable. Type: fp32_t'''

    def __init__(self, beam_width = None, beam_height = None):
        '''Class constructor
        
        Beam height of the instrument. A negative number denotes that this information is not available or is not applicable.

       This message class contains the following fields and their respective types:
    beam_width : fp32_t, unit: rad

            beam_height : fp32_t, unit: rad'''
        self._beam_width = beam_width
        self._beam_height = beam_height


class DataSanity(_base.base_message):
    '''Whether the data is sane or not sane. Enumerated (Local).

       This message class contains the following fields and their respective types:
    sane : uint8_t, unit: Enumerated (Local)'''

    class SANE(_enum.IntEnum):
        '''Full name: Sanity
        Prefix: DS'''
    
        SANE = 0
        '''Name: Sane'''
    
        NOT_SANE = 1
        '''Name: Not Sane'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_sane']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Sensors", id = 284, stable = None, description = "Report sanity or lack of it in the data output by a sensor.", fields = ('sane',), abbrev = "DataSanity", name = "Data Sanity", flags = None)

    sane = _base.mutable_attr({'name': 'Sanity', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DS'}, "Whether the data is sane or not sane. Enumerated (Local).")
    '''Whether the data is sane or not sane. Enumerated (Local). Type: uint8_t'''

    def __init__(self, sane = None):
        '''Class constructor
        
        Whether the data is sane or not sane. Enumerated (Local).

       This message class contains the following fields and their respective types:
    sane : uint8_t, unit: Enumerated (Local)'''
        self._sane = sane


class RhodamineDye(_base.base_message):
    '''Amount of rhodamine dye detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 285, stable = None, description = "Rhodamine Dye measurement.", fields = ('value',), abbrev = "RhodamineDye", name = "Rhodamine Dye", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Amount of rhodamine dye detected.")
    '''Amount of rhodamine dye detected. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Amount of rhodamine dye detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class CrudeOil(_base.base_message):
    '''Amount of crude oil detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 286, stable = None, description = "Crude oil measurement.", fields = ('value',), abbrev = "CrudeOil", name = "Crude Oil", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Amount of crude oil detected.")
    '''Amount of crude oil detected. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Amount of crude oil detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class FineOil(_base.base_message):
    '''Amount of fine oil detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 287, stable = None, description = "Fine oil measurement.", fields = ('value',), abbrev = "FineOil", name = "Fine Oil", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Amount of fine oil detected.")
    '''Amount of fine oil detected. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Amount of fine oil detected.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class Turbidity(_base.base_message):
    '''Turbidity reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NTU'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 288, stable = None, description = "Turbidity measurement.", fields = ('value',), abbrev = "Turbidity", name = "Turbidity", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'NTU'}, "Turbidity reading.")
    '''Turbidity reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Turbidity reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NTU'''
        self._value = value


class Chlorophyll(_base.base_message):
    '''Chlorophyll reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: µg/L'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 289, stable = None, description = "Chlorophyll measurement.", fields = ('value',), abbrev = "Chlorophyll", name = "Chlorophyll", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'µg/L'}, "Chlorophyll reading.")
    '''Chlorophyll reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Chlorophyll reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: µg/L'''
        self._value = value


class Fluorescein(_base.base_message):
    '''Fluorescein reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 290, stable = None, description = "Fluorescein measurement.", fields = ('value',), abbrev = "Fluorescein", name = "Fluorescein", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Fluorescein reading.")
    '''Fluorescein reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Fluorescein reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class Phycocyanin(_base.base_message):
    '''Phycocyanin reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 291, stable = None, description = "Phycocyanin measurement.", fields = ('value',), abbrev = "Phycocyanin", name = "Phycocyanin", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Phycocyanin reading.")
    '''Phycocyanin reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Phycocyanin reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class Phycoerythrin(_base.base_message):
    '''Phycoerythrin reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 292, stable = None, description = "Phycoerythrin measurement.", fields = ('value',), abbrev = "Phycoerythrin", name = "Phycoerythrin", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Phycoerythrin reading.")
    '''Phycoerythrin reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Phycoerythrin reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class GpsFixRtk(_base.base_message):
    '''Quality ratio of Integer Ambiguity Resolution (bigger is better).

       This message class contains the following fields and their respective types:
    validity : uint16_t, unit: Bitfield (Local)

            type : uint8_t, unit: Enumerated (Local)

            tow : uint32_t, unit: NOT FOUND

            base_lat : fp64_t, unit: rad

            base_lon : fp64_t, unit: rad

            base_height : fp32_t, unit: m

            n : fp32_t, unit: m

            e : fp32_t, unit: m

            d : fp32_t, unit: m

            v_n : fp32_t, unit: m/s

            v_e : fp32_t, unit: m/s

            v_d : fp32_t, unit: m/s

            satellites : uint8_t, unit: NOT FOUND

            iar_hyp : uint16_t, unit: NOT FOUND

            iar_ratio : fp32_t, unit: NOT FOUND'''

    class VALIDITY(_enum.IntFlag):
        '''Full name: Validity
        Prefix: RFV'''
    
        EMPTY = 0
        '''No active flags'''
    
        VALID_TIME = 1
        '''Name: Valid Time'''
    
        VALID_BASE = 2
        '''Name: Valid Base LLH'''
    
        VALID_POS = 4
        '''Name: Valid Position'''
    
        VALID_VEL = 8
        '''Name: Valid Velocity'''
    
    
    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: RTK'''
    
        NONE = 0
        '''Name: None'''
    
        OBS = 1
        '''Name: Obs'''
    
        FLOAT = 2
        '''Name: Float'''
    
        FIXED = 3
        '''Name: Fixed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_validity', '_type', '_tow', '_base_lat', '_base_lon', '_base_height', '_n', '_e', '_d', '_v_n', '_v_e', '_v_d', '_satellites', '_iar_hyp', '_iar_ratio']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 293, stable = None, description = "Report of an RTK-GPS fix.", fields = ('validity', 'type', 'tow', 'base_lat', 'base_lon', 'base_height', 'n', 'e', 'd', 'v_n', 'v_e', 'v_d', 'satellites', 'iar_hyp', 'iar_ratio',), abbrev = "GpsFixRtk", name = "GPS Fix RTK", flags = "periodic")

    validity = _base.mutable_attr({'name': 'Validity', 'type': 'uint16_t', 'unit': 'Bitfield', 'prefix': 'RFV'}, "Validity of fields. Bitfield (Local).")
    '''Validity of fields. Bitfield (Local). Type: uint16_t'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RTK'}, "Type of fix. Enumerated (Local).")
    '''Type of fix. Enumerated (Local). Type: uint8_t'''
    tow = _base.mutable_attr({'name': 'GPS Time of Week', 'type': 'uint32_t'}, "GPS Time of Week.")
    '''GPS Time of Week. Type: uint32_t'''
    base_lat = _base.mutable_attr({'name': 'Base Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate of the base.")
    '''WGS-84 Latitude coordinate of the base. Type: fp64_t'''
    base_lon = _base.mutable_attr({'name': 'Base Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate of the base.")
    '''WGS-84 Longitude coordinate of the base. Type: fp64_t'''
    base_height = _base.mutable_attr({'name': 'Base Height above WGS-84 ellipsoid', 'type': 'fp32_t', 'unit': 'm'}, "Height above WGS-84 ellipsoid of the base.")
    '''Height above WGS-84 ellipsoid of the base. Type: fp32_t'''
    n = _base.mutable_attr({'name': 'Position North', 'type': 'fp32_t', 'unit': 'm'}, "Baseline North coordinate.")
    '''Baseline North coordinate. Type: fp32_t'''
    e = _base.mutable_attr({'name': 'Position East', 'type': 'fp32_t', 'unit': 'm'}, "Baseline East coordinate.")
    '''Baseline East coordinate. Type: fp32_t'''
    d = _base.mutable_attr({'name': 'Position Down', 'type': 'fp32_t', 'unit': 'm'}, "Baseline Down coordinate.")
    '''Baseline Down coordinate. Type: fp32_t'''
    v_n = _base.mutable_attr({'name': 'Velocity North', 'type': 'fp32_t', 'unit': 'm/s'}, "Velocity North coordinate.")
    '''Velocity North coordinate. Type: fp32_t'''
    v_e = _base.mutable_attr({'name': 'Velocity East', 'type': 'fp32_t', 'unit': 'm/s'}, "Velocity East coordinate.")
    '''Velocity East coordinate. Type: fp32_t'''
    v_d = _base.mutable_attr({'name': 'Velocity Down', 'type': 'fp32_t', 'unit': 'm/s'}, "Velocity Down coordinate.")
    '''Velocity Down coordinate. Type: fp32_t'''
    satellites = _base.mutable_attr({'name': 'Number of Satellites', 'type': 'uint8_t'}, "Number of satellites used in solution.")
    '''Number of satellites used in solution. Type: uint8_t'''
    iar_hyp = _base.mutable_attr({'name': 'IAR Hypotheses', 'type': 'uint16_t'}, "Number of hypotheses in the Integer Ambiguity Resolution (smaller is better).")
    '''Number of hypotheses in the Integer Ambiguity Resolution (smaller is better). Type: uint16_t'''
    iar_ratio = _base.mutable_attr({'name': 'IAR Ratio', 'type': 'fp32_t'}, "Quality ratio of Integer Ambiguity Resolution (bigger is better).")
    '''Quality ratio of Integer Ambiguity Resolution (bigger is better). Type: fp32_t'''

    def __init__(self, validity = None, type = None, tow = None, base_lat = None, base_lon = None, base_height = None, n = None, e = None, d = None, v_n = None, v_e = None, v_d = None, satellites = None, iar_hyp = None, iar_ratio = None):
        '''Class constructor
        
        Quality ratio of Integer Ambiguity Resolution (bigger is better).

       This message class contains the following fields and their respective types:
    validity : uint16_t, unit: Bitfield (Local)

            type : uint8_t, unit: Enumerated (Local)

            tow : uint32_t, unit: NOT FOUND

            base_lat : fp64_t, unit: rad

            base_lon : fp64_t, unit: rad

            base_height : fp32_t, unit: m

            n : fp32_t, unit: m

            e : fp32_t, unit: m

            d : fp32_t, unit: m

            v_n : fp32_t, unit: m/s

            v_e : fp32_t, unit: m/s

            v_d : fp32_t, unit: m/s

            satellites : uint8_t, unit: NOT FOUND

            iar_hyp : uint16_t, unit: NOT FOUND

            iar_ratio : fp32_t, unit: NOT FOUND'''
        self._validity = validity
        self._type = type
        self._tow = tow
        self._base_lat = base_lat
        self._base_lon = base_lon
        self._base_height = base_height
        self._n = n
        self._e = e
        self._d = d
        self._v_n = v_n
        self._v_e = v_e
        self._v_d = v_d
        self._satellites = satellites
        self._iar_hyp = iar_hyp
        self._iar_ratio = iar_ratio


class ExternalNavData(_base.base_message):
    '''The type of external navigation data Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : message, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Nav Data Type
        Prefix: EXTNAV'''
    
        FULL = 0
        '''Name: Full State'''
    
        AHRS = 1
        '''Name: Attitude Heading Reference System Only'''
    
        POSREF = 2
        '''Name: Position Reference System only'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_type']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 294, stable = None, description = "This message is a representation of the state of the vehicle, as seen by an external navigation computer. An example usage is when DUNE is used with ardupilot. The data gathered from the autopilot is a complete navigation solution. ExternalNavData contains an inline Estimated State, which is a complete description of the system in terms of parameters such as position, orientation and velocities at a particular moment in time. The Type field selects wether the navigation data is a full state estimation, or only concerns attitude or position/velocity.", fields = ('state', 'type',), abbrev = "ExternalNavData", name = "External Navigation Data", flags = "periodic")

    state = _base.mutable_attr({'name': 'Estimated State', 'type': 'message', 'message-type': 'EstimatedState'}, "External Navigation Data.")
    '''External Navigation Data. Type: message'''
    type = _base.mutable_attr({'name': 'Nav Data Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'EXTNAV'}, "The type of external navigation data Enumerated (Local).")
    '''The type of external navigation data Enumerated (Local). Type: uint8_t'''

    def __init__(self, state = None, type = None):
        '''Class constructor
        
        The type of external navigation data Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : message, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)'''
        self._state = state
        self._type = type


class DissolvedOxygen(_base.base_message):
    '''Dissolved Oxygen reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: µM'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 295, stable = None, description = "Dissolved Oxygen measurement.", fields = ('value',), abbrev = "DissolvedOxygen", name = "Dissolved Oxygen", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'µM'}, "Dissolved Oxygen reading.")
    '''Dissolved Oxygen reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Dissolved Oxygen reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: µM'''
        self._value = value


class AirSaturation(_base.base_message):
    '''Air Saturation reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 296, stable = None, description = "Air Saturation measurement.", fields = ('value',), abbrev = "AirSaturation", name = "Air Saturation", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': '%'}, "Air Saturation reading.")
    '''Air Saturation reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Air Saturation reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: %'''
        self._value = value


class Throttle(_base.base_message):
    '''The value of the desired throttle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 297, stable = None, description = "Throttle e.g. for Plane/Copter .", fields = ('value',), abbrev = "Throttle", name = "Throttle", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t', 'unit': '%'}, "The value of the desired throttle.")
    '''The value of the desired throttle. Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the desired throttle.

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: %'''
        self._value = value


class PH(_base.base_message):
    '''The value of the pH as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 298, stable = None, description = "Report of pH.", fields = ('value',), abbrev = "PH", name = "pH", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t'}, "The value of the pH as measured by the sensor.")
    '''The value of the pH as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the pH as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: NOT FOUND'''
        self._value = value


class Redox(_base.base_message):
    '''The value of the Redox as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: V'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 299, stable = None, description = "Report of Redox Potential.", fields = ('value',), abbrev = "Redox", name = "Redox Potential", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'V'}, "The value of the Redox as measured by the sensor.")
    '''The value of the Redox as measured by the sensor. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The value of the Redox as measured by the sensor.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: V'''
        self._value = value


class DissolvedOrganicMatter(_base.base_message):
    '''Type of measurement. Enumerated (Local).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB

            type : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type of measurement
        Prefix: DT'''
    
        COLORED = 0
        '''Name: Colored'''
    
        FLUORESCENT = 1
        '''Name: Fluorescent'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_type']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 903, stable = None, description = "Dissolved Organic Matter measurement.", fields = ('value', 'type',), abbrev = "DissolvedOrganicMatter", name = "Dissolved Organic Matter", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Dissolved Organic Matter reading.")
    '''Dissolved Organic Matter reading. Type: fp32_t'''
    type = _base.mutable_attr({'name': 'Type of measurement', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DT'}, "Type of measurement. Enumerated (Local).")
    '''Type of measurement. Enumerated (Local). Type: uint8_t'''

    def __init__(self, value = None, type = None):
        '''Class constructor
        
        Type of measurement. Enumerated (Local).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB

            type : uint8_t, unit: Enumerated (Local)'''
        self._value = value
        self._type = type


class OpticalBackscatter(_base.base_message):
    '''Optical Backscattering Coefficient.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: 1/m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 904, stable = None, description = "The optical backscattering coefficient refers to all the photons that have been redirected in the backward directions when a photon of light propagates in water and interacts with a \"particle\" (varying from water molecules to fish).", fields = ('value',), abbrev = "OpticalBackscatter", name = "Optical Backscattering Coefficient", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': '1/m'}, "Optical Backscattering Coefficient.")
    '''Optical Backscattering Coefficient. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Optical Backscattering Coefficient.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: 1/m'''
        self._value = value


class SadcReadings(_base.base_message):
    '''Gain value of readings. Enumerated (Local).

       This message class contains the following fields and their respective types:
    channel : int8_t, unit: NOT FOUND

            value : int32_t, unit: NOT FOUND

            gain : uint8_t, unit: Enumerated (Local)'''

    class GAIN(_enum.IntEnum):
        '''Full name: Gain
        Prefix: GAIN'''
    
        X1 = 0
        '''Name: x1'''
    
        X10 = 1
        '''Name: x10'''
    
        X100 = 2
        '''Name: x100'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_channel', '_value', '_gain']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 907, stable = None, description = "Readings from SADC board.", fields = ('channel', 'value', 'gain',), abbrev = "SadcReadings", name = "SADC Readings", flags = None)

    channel = _base.mutable_attr({'name': 'Channel', 'type': 'int8_t', 'min': 1, 'max': 4}, "Channel of SADC to read.")
    '''Channel of SADC to read. Type: int8_t'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'int32_t'}, "Value raw of sadc channel.")
    '''Value raw of sadc channel. Type: int32_t'''
    gain = _base.mutable_attr({'name': 'Gain', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'GAIN'}, "Gain value of readings. Enumerated (Local).")
    '''Gain value of readings. Enumerated (Local). Type: uint8_t'''

    def __init__(self, channel = None, value = None, gain = None):
        '''Class constructor
        
        Gain value of readings. Enumerated (Local).

       This message class contains the following fields and their respective types:
    channel : int8_t, unit: NOT FOUND

            value : int32_t, unit: NOT FOUND

            gain : uint8_t, unit: Enumerated (Local)'''
        self._channel = channel
        self._value = value
        self._gain = gain


class DmsDetection(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    ch01 : fp32_t, unit: NOT FOUND

            ch02 : fp32_t, unit: NOT FOUND

            ch03 : fp32_t, unit: NOT FOUND

            ch04 : fp32_t, unit: NOT FOUND

            ch05 : fp32_t, unit: NOT FOUND

            ch06 : fp32_t, unit: NOT FOUND

            ch07 : fp32_t, unit: NOT FOUND

            ch08 : fp32_t, unit: NOT FOUND

            ch09 : fp32_t, unit: NOT FOUND

            ch10 : fp32_t, unit: NOT FOUND

            ch11 : fp32_t, unit: NOT FOUND

            ch12 : fp32_t, unit: NOT FOUND

            ch13 : fp32_t, unit: NOT FOUND

            ch14 : fp32_t, unit: NOT FOUND

            ch15 : fp32_t, unit: NOT FOUND

            ch16 : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_ch01', '_ch02', '_ch03', '_ch04', '_ch05', '_ch06', '_ch07', '_ch08', '_ch09', '_ch10', '_ch11', '_ch12', '_ch13', '_ch14', '_ch15', '_ch16']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 908, stable = None, description = "Presence of DMS (Dimethyl Sulphide). If the value of the channel is greater than zero, it means DMS was detected.", fields = ('ch01', 'ch02', 'ch03', 'ch04', 'ch05', 'ch06', 'ch07', 'ch08', 'ch09', 'ch10', 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16',), abbrev = "DmsDetection", name = "DMS Detection", flags = None)

    ch01 = _base.mutable_attr({'name': 'Channel 1', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch02 = _base.mutable_attr({'name': 'Channel 2', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch03 = _base.mutable_attr({'name': 'Channel 3', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch04 = _base.mutable_attr({'name': 'Channel 4', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch05 = _base.mutable_attr({'name': 'Channel 5', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch06 = _base.mutable_attr({'name': 'Channel 6', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch07 = _base.mutable_attr({'name': 'Channel 7', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch08 = _base.mutable_attr({'name': 'Channel 8', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch09 = _base.mutable_attr({'name': 'Channel 9', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch10 = _base.mutable_attr({'name': 'Channel 10', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch11 = _base.mutable_attr({'name': 'Channel 11', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch12 = _base.mutable_attr({'name': 'Channel 12', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch13 = _base.mutable_attr({'name': 'Channel 13', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch14 = _base.mutable_attr({'name': 'Channel 14', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch15 = _base.mutable_attr({'name': 'Channel 15', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''
    ch16 = _base.mutable_attr({'name': 'Channel 16', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, ch01 = None, ch02 = None, ch03 = None, ch04 = None, ch05 = None, ch06 = None, ch07 = None, ch08 = None, ch09 = None, ch10 = None, ch11 = None, ch12 = None, ch13 = None, ch14 = None, ch15 = None, ch16 = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    ch01 : fp32_t, unit: NOT FOUND

            ch02 : fp32_t, unit: NOT FOUND

            ch03 : fp32_t, unit: NOT FOUND

            ch04 : fp32_t, unit: NOT FOUND

            ch05 : fp32_t, unit: NOT FOUND

            ch06 : fp32_t, unit: NOT FOUND

            ch07 : fp32_t, unit: NOT FOUND

            ch08 : fp32_t, unit: NOT FOUND

            ch09 : fp32_t, unit: NOT FOUND

            ch10 : fp32_t, unit: NOT FOUND

            ch11 : fp32_t, unit: NOT FOUND

            ch12 : fp32_t, unit: NOT FOUND

            ch13 : fp32_t, unit: NOT FOUND

            ch14 : fp32_t, unit: NOT FOUND

            ch15 : fp32_t, unit: NOT FOUND

            ch16 : fp32_t, unit: NOT FOUND'''
        self._ch01 = ch01
        self._ch02 = ch02
        self._ch03 = ch03
        self._ch04 = ch04
        self._ch05 = ch05
        self._ch06 = ch06
        self._ch07 = ch07
        self._ch08 = ch08
        self._ch09 = ch09
        self._ch10 = ch10
        self._ch11 = ch11
        self._ch12 = ch12
        self._ch13 = ch13
        self._ch14 = ch14
        self._ch15 = ch15
        self._ch16 = ch16


class CurrentProfile(_base.base_message):
    '''List of current profile measurement cells.

       This message class contains the following fields and their respective types:
    nbeams : uint8_t, unit: NOT FOUND

            ncells : uint8_t, unit: NOT FOUND

            coord_sys : uint8_t, unit: Bitfield (Local)

            profile : message-list, unit: NOT FOUND'''

    class COORD_SYS(_enum.IntFlag):
        '''Full name: Coordinate System
        Prefix: UTF'''
    
        EMPTY = 0
        '''No active flags'''
    
        XYZ = 1
        '''Name: xyz'''
    
        NED = 2
        '''Name: ned'''
    
        BEAMS = 4
        '''Name: beams'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_nbeams', '_ncells', '_coord_sys', '_profile']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 1014, stable = None, description = "Contains a profile of water velocities measured relative to the vehicle velocity, represented in the specified coordinate system.", fields = ('nbeams', 'ncells', 'coord_sys', 'profile',), abbrev = "CurrentProfile", name = "Current Profile", flags = "periodic")

    nbeams = _base.mutable_attr({'name': 'Number of Beams', 'type': 'uint8_t'}, "Number of ADCP beams.")
    '''Number of ADCP beams. Type: uint8_t'''
    ncells = _base.mutable_attr({'name': 'Number of Cells', 'type': 'uint8_t'}, "Number of ADCP cells.")
    '''Number of ADCP cells. Type: uint8_t'''
    coord_sys = _base.mutable_attr({'name': 'Coordinate System', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'UTF'}, "Coordinate system of the velocity measurement. Bitfield (Local).")
    '''Coordinate system of the velocity measurement. Bitfield (Local). Type: uint8_t'''
    profile = _base.mutable_attr({'name': 'Profile', 'type': 'message-list', 'message-type': 'CurrentProfileCell'}, "List of current profile measurement cells.")
    '''List of current profile measurement cells. Type: message-list'''

    def __init__(self, nbeams = None, ncells = None, coord_sys = None, profile = None):
        '''Class constructor
        
        List of current profile measurement cells.

       This message class contains the following fields and their respective types:
    nbeams : uint8_t, unit: NOT FOUND

            ncells : uint8_t, unit: NOT FOUND

            coord_sys : uint8_t, unit: Bitfield (Local)

            profile : message-list, unit: NOT FOUND'''
        self._nbeams = nbeams
        self._ncells = ncells
        self._coord_sys = coord_sys
        self._profile = profile


class CurrentProfileCell(_base.base_message):
    '''List of beams measurements at the current cell level.

       This message class contains the following fields and their respective types:
    cell_position : fp32_t, unit: m

            beams : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_cell_position', '_beams']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 1015, stable = None, description = "One Current measurement at a specific CellPosition.", fields = ('cell_position', 'beams',), abbrev = "CurrentProfileCell", name = "Current Profile Cell", flags = "periodic")

    cell_position = _base.mutable_attr({'name': 'Cell Position', 'type': 'fp32_t', 'unit': 'm'}, "Distance of each measurment cell along the Z-axis in the coordintate frame.")
    '''Distance of each measurment cell along the Z-axis in the coordintate frame. Type: fp32_t'''
    beams = _base.mutable_attr({'name': 'Beams Measurements', 'type': 'message-list', 'message-type': 'ADCPBeam'}, "List of beams measurements at the current cell level.")
    '''List of beams measurements at the current cell level. Type: message-list'''

    def __init__(self, cell_position = None, beams = None):
        '''Class constructor
        
        List of beams measurements at the current cell level.

       This message class contains the following fields and their respective types:
    cell_position : fp32_t, unit: m

            beams : message-list, unit: NOT FOUND'''
        self._cell_position = cell_position
        self._beams = beams


class ADCPBeam(_base.base_message):
    '''Autocorrelation of returning ping for the beam.

       This message class contains the following fields and their respective types:
    vel : fp32_t, unit: m/s

            amp : fp32_t, unit: dB

            cor : uint8_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_vel', '_amp', '_cor']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 1016, stable = None, description = "Measurement from one specific beam at the given CellPosition. Water Velocity is provided in the chosen Coordinate system. Amplitude and Correlation are always in the BEAM coordinate system.", fields = ('vel', 'amp', 'cor',), abbrev = "ADCPBeam", name = "ADCP Beam Measurements", flags = "periodic")

    vel = _base.mutable_attr({'name': 'Water Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Water velocity measured in the chosen coordinate system.")
    '''Water velocity measured in the chosen coordinate system. Type: fp32_t'''
    amp = _base.mutable_attr({'name': 'Amplitude', 'type': 'fp32_t', 'unit': 'dB'}, "Amplitude of returning ping for the beam.")
    '''Amplitude of returning ping for the beam. Type: fp32_t'''
    cor = _base.mutable_attr({'name': 'Correlation', 'type': 'uint8_t', 'unit': '%', 'max': 100, 'min': 0}, "Autocorrelation of returning ping for the beam.")
    '''Autocorrelation of returning ping for the beam. Type: uint8_t'''

    def __init__(self, vel = None, amp = None, cor = None):
        '''Class constructor
        
        Autocorrelation of returning ping for the beam.

       This message class contains the following fields and their respective types:
    vel : fp32_t, unit: m/s

            amp : fp32_t, unit: dB

            cor : uint8_t, unit: %'''
        self._vel = vel
        self._amp = amp
        self._cor = cor


class ColoredDissolvedOrganicMatter(_base.base_message):
    '''Colored Dissolved Organic Matter reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 2003, stable = None, description = "Colored Dissolved Organic Matter measurement.", fields = ('value',), abbrev = "ColoredDissolvedOrganicMatter", name = "Colored Dissolved Organic Matter", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Colored Dissolved Organic Matter reading.")
    '''Colored Dissolved Organic Matter reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Colored Dissolved Organic Matter reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class FluorescentDissolvedOrganicMatter(_base.base_message):
    '''Fluorescent Dissolved Organic Matter reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 2004, stable = None, description = "Fluorescent Dissolved Organic Matter measurement.", fields = ('value',), abbrev = "FluorescentDissolvedOrganicMatter", name = "Fluorescent Dissolved Organic Matter", flags = "periodic")

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'PPB'}, "Fluorescent Dissolved Organic Matter reading.")
    '''Fluorescent Dissolved Organic Matter reading. Type: fp32_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Fluorescent Dissolved Organic Matter reading.

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: PPB'''
        self._value = value


class TotalMagIntensity(_base.base_message):
    '''Total Magnetic Field Intensity (TMI)

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Sensors", id = 2006, stable = None, description = None, fields = ('value',), abbrev = "TotalMagIntensity", name = "Total Magnetic Field Intensity", flags = None)

    value = _base.mutable_attr({'name': 'Value', 'type': 'fp64_t'}, "Total Magnetic Field Intensity (TMI)")
    '''Total Magnetic Field Intensity (TMI) Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        Total Magnetic Field Intensity (TMI)

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: NOT FOUND'''
        self._value = value


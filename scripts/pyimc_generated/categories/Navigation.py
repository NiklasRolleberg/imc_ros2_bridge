'''
IMC Navigation messages.
'''

from .. import _base
import enum as _enum

class EstimatedState(_base.base_message):
    '''Altitude, in meters. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            u : fp32_t, unit: m/s

            v : fp32_t, unit: m/s

            w : fp32_t, unit: m/s

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_height', '_x', '_y', '_z', '_phi', '_theta', '_psi', '_u', '_v', '_w', '_vx', '_vy', '_vz', '_p', '_q', '_r', '_depth', '_alt']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 350, stable = None, description = "This message presents the estimated state of the vehicle. EstimatedState is a complete description of the system in terms of parameters such as position, orientation and velocities at a particular moment in time. The system position is given by a North-East-Down (NED) local tangent plane displacement (x, y, z) relative to an absolute WGS-84 coordinate (latitude, longitude, height above ellipsoid). The symbols for position and attitude as well as linear and angular velocities were chosen according to SNAME's notation (1950). The body-fixed reference frame and Euler angles are depicted next: .. figure:: ../images/euler-lauv.png :align: center Euler angles", fields = ('lat', 'lon', 'height', 'x', 'y', 'z', 'phi', 'theta', 'psi', 'u', 'v', 'w', 'vx', 'vy', 'vz', 'p', 'q', 'r', 'depth', 'alt',), abbrev = "EstimatedState", name = "Estimated State", flags = "periodic")

    lat = _base.mutable_attr({'name': 'Latitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    height = _base.mutable_attr({'name': 'Height (WGS-84)', 'type': 'fp32_t', 'unit': 'm'}, "Height above the WGS-84 ellipsoid.")
    '''Height above the WGS-84 ellipsoid. Type: fp32_t'''
    x = _base.mutable_attr({'name': 'Offset north', 'type': 'fp32_t', 'unit': 'm'}, "The North offset of the North/East/Down field with respect to LLH.")
    '''The North offset of the North/East/Down field with respect to LLH. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Offset east', 'type': 'fp32_t', 'unit': 'm'}, "The East offset of the North/East/Down field with respect to LLH.")
    '''The East offset of the North/East/Down field with respect to LLH. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Offset down', 'type': 'fp32_t', 'unit': 'm'}, "The Down offset of the North/East/Down field with respect to LLH.")
    '''The Down offset of the North/East/Down field with respect to LLH. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Rotation over x axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The phi Euler angle from the vehicle's attitude.")
    '''The phi Euler angle from the vehicle's attitude. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Rotation over y axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267949, 'max': 1.5707963267949}, "The theta Euler angle from the vehicle's attitude.")
    '''The theta Euler angle from the vehicle's attitude. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Rotation over z axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The psi Euler angle from the vehicle's attitude.")
    '''The psi Euler angle from the vehicle's attitude. Type: fp32_t'''
    u = _base.mutable_attr({'name': 'Body-Fixed xx Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame xx axis velocity component.")
    '''Body-fixed frame xx axis velocity component. Type: fp32_t'''
    v = _base.mutable_attr({'name': 'Body-Fixed yy Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame yy axis velocity component.")
    '''Body-fixed frame yy axis velocity component. Type: fp32_t'''
    w = _base.mutable_attr({'name': 'Body-Fixed zz Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame zz axis velocity component.")
    '''Body-fixed frame zz axis velocity component. Type: fp32_t'''
    vx = _base.mutable_attr({'name': 'Ground Velocity X (North)', 'type': 'fp32_t', 'unit': 'm/s'}, "Ground Velocity xx axis velocity component.")
    '''Ground Velocity xx axis velocity component. Type: fp32_t'''
    vy = _base.mutable_attr({'name': 'Ground Velocity Y (East)', 'type': 'fp32_t', 'unit': 'm/s'}, "Ground Velocity yy axis velocity component.")
    '''Ground Velocity yy axis velocity component. Type: fp32_t'''
    vz = _base.mutable_attr({'name': 'Ground Velocity Z (Down)', 'type': 'fp32_t', 'unit': 'm/s'}, "Ground Velocity zz axis velocity component.")
    '''Ground Velocity zz axis velocity component. Type: fp32_t'''
    p = _base.mutable_attr({'name': 'Angular Velocity in x', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed xx axis (roll).")
    '''The angular velocity over body-fixed xx axis (roll). Type: fp32_t'''
    q = _base.mutable_attr({'name': 'Angular Velocity in y', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed yy axis (pitch).")
    '''The angular velocity over body-fixed yy axis (pitch). Type: fp32_t'''
    r = _base.mutable_attr({'name': 'Angular Velocity in z', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed zz axis (yaw).")
    '''The angular velocity over body-fixed zz axis (yaw). Type: fp32_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t', 'unit': 'm'}, "Depth, in meters. To be used by underwater vehicles. Negative values denote invalid estimates.")
    '''Depth, in meters. To be used by underwater vehicles. Negative values denote invalid estimates. Type: fp32_t'''
    alt = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Altitude, in meters. Negative values denote invalid estimates.")
    '''Altitude, in meters. Negative values denote invalid estimates. Type: fp32_t'''

    def __init__(self, lat = None, lon = None, height = None, x = None, y = None, z = None, phi = None, theta = None, psi = None, u = None, v = None, w = None, vx = None, vy = None, vz = None, p = None, q = None, r = None, depth = None, alt = None):
        '''Class constructor
        
        Altitude, in meters. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            u : fp32_t, unit: m/s

            v : fp32_t, unit: m/s

            w : fp32_t, unit: m/s

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m'''
        self._lat = lat
        self._lon = lon
        self._height = height
        self._x = x
        self._y = y
        self._z = z
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._u = u
        self._v = v
        self._w = w
        self._vx = vx
        self._vy = vy
        self._vz = vz
        self._p = p
        self._q = q
        self._r = r
        self._depth = depth
        self._alt = alt


class EstimatedStreamVelocity(_base.base_message):
    '''Z component (Down).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 351, stable = None, description = "The estimated stream velocity, typically for water or air streams.", fields = ('x', 'y', 'z',), abbrev = "EstimatedStreamVelocity", name = "Estimated Stream Velocity", flags = "periodic")

    x = _base.mutable_attr({'name': 'X component (North)', 'type': 'fp64_t', 'unit': 'm/s'}, "X component (North).")
    '''X component (North). Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y component (East)', 'type': 'fp64_t', 'unit': 'm/s'}, "Y component (East).")
    '''Y component (East). Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z component (Down)', 'type': 'fp64_t', 'unit': 'm/s'}, "Z component (Down).")
    '''Z component (Down). Type: fp64_t'''

    def __init__(self, x = None, y = None, z = None):
        '''Class constructor
        
        Z component (Down).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''
        self._x = x
        self._y = y
        self._z = z


class IndicatedSpeed(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 352, stable = None, description = "Speed measured by the navigation filter.", fields = ('value',), abbrev = "IndicatedSpeed", name = "Indicated Speed", flags = None)

    value = _base.mutable_attr({'name': 'Measured speed', 'unit': 'm/s', 'type': 'fp64_t'}, "No description available")
    '''No description available Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''
        self._value = value


class TrueSpeed(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 353, stable = None, description = "Ground true speed.", fields = ('value',), abbrev = "TrueSpeed", name = "True Speed", flags = None)

    value = _base.mutable_attr({'name': 'Estimated value', 'unit': 'm/s', 'type': 'fp64_t'}, "No description available")
    '''No description available Type: fp64_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    value : fp64_t, unit: m/s'''
        self._value = value


class NavigationUncertainty(_base.base_message):
    '''The angular velocity over body-fixed zz axis bias variance from sensor.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            u : fp32_t, unit: m/s

            v : fp32_t, unit: m/s

            w : fp32_t, unit: m/s

            bias_psi : fp32_t, unit: rad

            bias_r : fp32_t, unit: rad/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z', '_phi', '_theta', '_psi', '_p', '_q', '_r', '_u', '_v', '_w', '_bias_psi', '_bias_r']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Navigation", id = 354, stable = None, description = "Report of navigation uncertainty. This is usually given by the output of the state covariance matrix of an Extended Kalman Filter.", fields = ('x', 'y', 'z', 'phi', 'theta', 'psi', 'p', 'q', 'r', 'u', 'v', 'w', 'bias_psi', 'bias_r',), abbrev = "NavigationUncertainty", name = "Navigation Uncertainty", flags = None)

    x = _base.mutable_attr({'name': 'Variance - x Position', 'type': 'fp32_t', 'unit': 'm'}, "The North offset variance of the North/East/Down field with respect to LLH.")
    '''The North offset variance of the North/East/Down field with respect to LLH. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Variance - y Position', 'type': 'fp32_t', 'unit': 'm'}, "The East offset variance of the North/East/Down field with respect to LLH.")
    '''The East offset variance of the North/East/Down field with respect to LLH. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Variance - z Position', 'type': 'fp32_t', 'unit': 'm'}, "The Down offset variance of the North/East/Down field with respect to LLH.")
    '''The Down offset variance of the North/East/Down field with respect to LLH. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Variance - Roll', 'type': 'fp32_t', 'unit': 'rad'}, "The phi Euler angle variance from the vehicle's attitude.")
    '''The phi Euler angle variance from the vehicle's attitude. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Variance - Pitch', 'type': 'fp32_t', 'unit': 'rad'}, "The theta Euler angle variance from the vehicle's attitude.")
    '''The theta Euler angle variance from the vehicle's attitude. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Variance - Yaw', 'type': 'fp32_t', 'unit': 'rad'}, "The psi Euler angle variance from the vehicle's attitude.")
    '''The psi Euler angle variance from the vehicle's attitude. Type: fp32_t'''
    p = _base.mutable_attr({'name': 'Variance - Gyro. Roll Rate', 'type': 'fp32_t', 'unit': 'rad/s'}, "The angular velocity variance over body-fixed xx axis (roll).")
    '''The angular velocity variance over body-fixed xx axis (roll). Type: fp32_t'''
    q = _base.mutable_attr({'name': 'Variance - Gyro. Pitch Rate', 'type': 'fp32_t', 'unit': 'rad/s'}, "The angular velocity variance over body-fixed yy axis (pitch).")
    '''The angular velocity variance over body-fixed yy axis (pitch). Type: fp32_t'''
    r = _base.mutable_attr({'name': 'Variance - Gyro. Yaw Rate', 'type': 'fp32_t', 'unit': 'rad/s'}, "The angular velocity variance over body-fixed zz axis (yaw).")
    '''The angular velocity variance over body-fixed zz axis (yaw). Type: fp32_t'''
    u = _base.mutable_attr({'name': 'Variance - Body-Fixed xx Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame xx axis velocity variance component.")
    '''Body-fixed frame xx axis velocity variance component. Type: fp32_t'''
    v = _base.mutable_attr({'name': 'Variance - Body-Fixed yy Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame yy axis velocity variance component.")
    '''Body-fixed frame yy axis velocity variance component. Type: fp32_t'''
    w = _base.mutable_attr({'name': 'Variance - Body-Fixed ww Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame zz axis velocity variance component.")
    '''Body-fixed frame zz axis velocity variance component. Type: fp32_t'''
    bias_psi = _base.mutable_attr({'name': 'Variance - Yaw Bias', 'type': 'fp32_t', 'unit': 'rad'}, "The psi Euler angle bias variance from the vehicle's sensed attitude.")
    '''The psi Euler angle bias variance from the vehicle's sensed attitude. Type: fp32_t'''
    bias_r = _base.mutable_attr({'name': 'Variance - Gyro. Yaw Rate Bias', 'type': 'fp32_t', 'unit': 'rad/s'}, "The angular velocity over body-fixed zz axis bias variance from sensor.")
    '''The angular velocity over body-fixed zz axis bias variance from sensor. Type: fp32_t'''

    def __init__(self, x = None, y = None, z = None, phi = None, theta = None, psi = None, p = None, q = None, r = None, u = None, v = None, w = None, bias_psi = None, bias_r = None):
        '''Class constructor
        
        The angular velocity over body-fixed zz axis bias variance from sensor.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            u : fp32_t, unit: m/s

            v : fp32_t, unit: m/s

            w : fp32_t, unit: m/s

            bias_psi : fp32_t, unit: rad

            bias_r : fp32_t, unit: rad/s'''
        self._x = x
        self._y = y
        self._z = z
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._p = p
        self._q = q
        self._r = r
        self._u = u
        self._v = v
        self._w = w
        self._bias_psi = bias_psi
        self._bias_r = bias_r


class NavigationData(_base.base_message):
    '''Custom variable.

       This message class contains the following fields and their respective types:
    bias_psi : fp32_t, unit: rad

            bias_r : fp32_t, unit: rad/s

            cog : fp32_t, unit: rad

            cyaw : fp32_t, unit: rad

            lbl_rej_level : fp32_t, unit: NOT FOUND

            gps_rej_level : fp32_t, unit: NOT FOUND

            custom_x : fp32_t, unit: NOT FOUND

            custom_y : fp32_t, unit: NOT FOUND

            custom_z : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_bias_psi', '_bias_r', '_cog', '_cyaw', '_lbl_rej_level', '_gps_rej_level', '_custom_x', '_custom_y', '_custom_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Navigation", id = 355, stable = None, description = "Report of navigation data. This is constituted by data which is not part of the vehicle estimated state but that the user may refer for more information.", fields = ('bias_psi', 'bias_r', 'cog', 'cyaw', 'lbl_rej_level', 'gps_rej_level', 'custom_x', 'custom_y', 'custom_z',), abbrev = "NavigationData", name = "Navigation Data", flags = None)

    bias_psi = _base.mutable_attr({'name': 'Yaw Bias', 'type': 'fp32_t', 'unit': 'rad'}, "The psi Euler angle bias from the vehicle's sensed attitude.")
    '''The psi Euler angle bias from the vehicle's sensed attitude. Type: fp32_t'''
    bias_r = _base.mutable_attr({'name': 'Gyro. Yaw Rate Bias', 'type': 'fp32_t', 'unit': 'rad/s'}, "The angular velocity over body-fixed zz axis bias from sensor.")
    '''The angular velocity over body-fixed zz axis bias from sensor. Type: fp32_t'''
    cog = _base.mutable_attr({'name': 'Course Over Ground', 'type': 'fp32_t', 'unit': 'rad'}, "Course over ground given by NED ground velocity vectors.")
    '''Course over ground given by NED ground velocity vectors. Type: fp32_t'''
    cyaw = _base.mutable_attr({'name': 'Continuous Yaw', 'type': 'fp32_t', 'unit': 'rad'}, "Continuous psi Euler angle (without normalizations).")
    '''Continuous psi Euler angle (without normalizations). Type: fp32_t'''
    lbl_rej_level = _base.mutable_attr({'name': 'GPS Rejection Filter Level', 'type': 'fp32_t'}, "GPS rejection filter level.")
    '''GPS rejection filter level. Type: fp32_t'''
    gps_rej_level = _base.mutable_attr({'name': 'LBL Rejection Filter Level', 'type': 'fp32_t'}, "LBL rejection filter level.")
    '''LBL rejection filter level. Type: fp32_t'''
    custom_x = _base.mutable_attr({'name': 'Variance - Custom Variable X', 'type': 'fp32_t'}, "Custom variable.")
    '''Custom variable. Type: fp32_t'''
    custom_y = _base.mutable_attr({'name': 'Variance - Custom Variable Y', 'type': 'fp32_t'}, "Custom variable.")
    '''Custom variable. Type: fp32_t'''
    custom_z = _base.mutable_attr({'name': 'Variance - Custom Variable Z', 'type': 'fp32_t'}, "Custom variable.")
    '''Custom variable. Type: fp32_t'''

    def __init__(self, bias_psi = None, bias_r = None, cog = None, cyaw = None, lbl_rej_level = None, gps_rej_level = None, custom_x = None, custom_y = None, custom_z = None):
        '''Class constructor
        
        Custom variable.

       This message class contains the following fields and their respective types:
    bias_psi : fp32_t, unit: rad

            bias_r : fp32_t, unit: rad/s

            cog : fp32_t, unit: rad

            cyaw : fp32_t, unit: rad

            lbl_rej_level : fp32_t, unit: NOT FOUND

            gps_rej_level : fp32_t, unit: NOT FOUND

            custom_x : fp32_t, unit: NOT FOUND

            custom_y : fp32_t, unit: NOT FOUND

            custom_z : fp32_t, unit: NOT FOUND'''
        self._bias_psi = bias_psi
        self._bias_r = bias_r
        self._cog = cog
        self._cyaw = cyaw
        self._lbl_rej_level = lbl_rej_level
        self._gps_rej_level = gps_rej_level
        self._custom_x = custom_x
        self._custom_y = custom_y
        self._custom_z = custom_z


class GpsFixRejection(_base.base_message):
    '''Reason for rejection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    utc_time : fp32_t, unit: s

            reason : uint8_t, unit: Enumerated (Local)'''

    class REASON(_enum.IntEnum):
        '''Full name: Reason
        Prefix: RR'''
    
        ABOVE_THRESHOLD = 0
        '''Name: Above Threshold'''
    
        INVALID = 1
        '''Name: Invalid Fix'''
    
        ABOVE_MAX_HDOP = 2
        '''Name: Above Maximum HDOP'''
    
        ABOVE_MAX_HACC = 3
        '''Name: Above Maximum HACC'''
    
        LOST_VAL_BIT = 4
        '''Name: Lost Validity Bit'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_utc_time', '_reason']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 356, stable = None, description = None, fields = ('utc_time', 'reason',), abbrev = "GpsFixRejection", name = "GPS Fix Rejection", flags = None)

    utc_time = _base.mutable_attr({'name': 'UTC Time of Fix', 'type': 'fp32_t', 'unit': 's'}, "UTC time of the rejected GPS fix measured in seconds since 00:00:00 (midnight).")
    '''UTC time of the rejected GPS fix measured in seconds since 00:00:00 (midnight). Type: fp32_t'''
    reason = _base.mutable_attr({'name': 'Reason', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RR'}, "Reason for rejection. Enumerated (Local).")
    '''Reason for rejection. Enumerated (Local). Type: uint8_t'''

    def __init__(self, utc_time = None, reason = None):
        '''Class constructor
        
        Reason for rejection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    utc_time : fp32_t, unit: s

            reason : uint8_t, unit: Enumerated (Local)'''
        self._utc_time = utc_time
        self._reason = reason


class LblRangeAcceptance(_base.base_message):
    '''Reason for acceptance/rejection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            range : fp32_t, unit: m

            acceptance : uint8_t, unit: Enumerated (Local)'''

    class ACCEPTANCE(_enum.IntEnum):
        '''Full name: Acceptance
        Prefix: RR'''
    
        ACCEPTED = 0
        '''Name: Accepted'''
    
        ABOVE_THRESHOLD = 1
        '''Name: Rejected - Above Threshold'''
    
        SINGULAR = 2
        '''Name: Rejected - Singular Point'''
    
        NO_INFO = 3
        '''Name: Rejected - Not Enough Information'''
    
        AT_SURFACE = 4
        '''Name: Rejected - Vehicle At Surface'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_range', '_acceptance']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 357, stable = None, description = "When the vehicle uses Long Base Line navigation, this message notifies that a new range was received from one of the acoustics transponders. The message fields are used to identify the range value and the transponder name. Also, this message has an acceptance field that indicates whether a LBL range was accepted or rejected, and if rejected, the reason why.", fields = ('id', 'range', 'acceptance',), abbrev = "LblRangeAcceptance", name = "LBL Range Acceptance", flags = None)

    id = _base.mutable_attr({'name': 'Beacon Identification Number', 'type': 'uint8_t'}, "Identification number of the acoustic transponder from which the range information was received.")
    '''Identification number of the acoustic transponder from which the range information was received. Type: uint8_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'fp32_t', 'unit': 'm'}, "Distance to the acoustic transponder.")
    '''Distance to the acoustic transponder. Type: fp32_t'''
    acceptance = _base.mutable_attr({'name': 'Acceptance', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RR'}, "Reason for acceptance/rejection. Enumerated (Local).")
    '''Reason for acceptance/rejection. Enumerated (Local). Type: uint8_t'''

    def __init__(self, id = None, range = None, acceptance = None):
        '''Class constructor
        
        Reason for acceptance/rejection. Enumerated (Local).

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            range : fp32_t, unit: m

            acceptance : uint8_t, unit: Enumerated (Local)'''
        self._id = id
        self._range = range
        self._acceptance = acceptance


class DvlRejection(_base.base_message):
    '''Timestep of the rejection. The timestep is 0 for an absolute rejection since it is an instantaneous reading. For innovation rejection it is the time difference between the previous accepted DVL measurement and the current one.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Bitfield (Local)

            reason : uint8_t, unit: Enumerated (Local)

            value : fp32_t, unit: m/s

            timestep : fp32_t, unit: s'''

    class TYPE(_enum.IntFlag):
        '''Full name: Type of velocity
        Prefix: TYPE'''
    
        EMPTY = 0
        '''No active flags'''
    
        GV = 1
        '''Name: Ground velocity'''
    
        WV = 2
        '''Name: Water velocity'''
    
    
    class REASON(_enum.IntEnum):
        '''Full name: Reason
        Prefix: RR'''
    
        INNOV_THRESHOLD_X = 0
        '''Name: Innovation Threshold - X'''
    
        INNOV_THRESHOLD_Y = 1
        '''Name: Innovation Threshold - Y'''
    
        ABS_THRESHOLD_X = 2
        '''Name: Absolute Threshold - X'''
    
        ABS_THRESHOLD_Y = 3
        '''Name: Absolute Threshold - Y'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_reason', '_value', '_timestep']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 358, stable = None, description = "When the vehicle uses Doppler Velocity Log sensor, this message notifies that a new measurement was locally rejected by the navigation filter.", fields = ('type', 'reason', 'value', 'timestep',), abbrev = "DvlRejection", name = "DVL Rejection", flags = None)

    type = _base.mutable_attr({'name': 'Type of velocity', 'unit': 'Bitfield', 'type': 'uint8_t', 'prefix': 'TYPE'}, "This field represents the type of the rejected velocity. Bitfield (Local).")
    '''This field represents the type of the rejected velocity. Bitfield (Local). Type: uint8_t'''
    reason = _base.mutable_attr({'name': 'Reason', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RR'}, "Reason for rejection. There are two types of DVL measurement filters. An Innovation filter checks the innovation between the current measurement and the previous measurement within a certain amount of time and an Absolute filter compares the measurement with an absolute threshold value. Those filters are tested using horizontal speed measurements, i.e., measurements in the x-axis and in the y-axis. Enumerated (Local).")
    '''Reason for rejection. There are two types of DVL measurement filters. An Innovation filter checks the innovation between the current measurement and the previous measurement within a certain amount of time and an Absolute filter compares the measurement with an absolute threshold value. Those filters are tested using horizontal speed measurements, i.e., measurements in the x-axis and in the y-axis. Enumerated (Local). Type: uint8_t'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'fp32_t', 'unit': 'm/s'}, "Value of the rejection. If it is an innovation rejection the value is the absolute difference between the previous accepted DVL measurement and the current one. If it is an absolute rejection the value is the current DVL measurement.")
    '''Value of the rejection. If it is an innovation rejection the value is the absolute difference between the previous accepted DVL measurement and the current one. If it is an absolute rejection the value is the current DVL measurement. Type: fp32_t'''
    timestep = _base.mutable_attr({'name': 'Timestep', 'type': 'fp32_t', 'unit': 's'}, "Timestep of the rejection. The timestep is 0 for an absolute rejection since it is an instantaneous reading. For innovation rejection it is the time difference between the previous accepted DVL measurement and the current one.")
    '''Timestep of the rejection. The timestep is 0 for an absolute rejection since it is an instantaneous reading. For innovation rejection it is the time difference between the previous accepted DVL measurement and the current one. Type: fp32_t'''

    def __init__(self, type = None, reason = None, value = None, timestep = None):
        '''Class constructor
        
        Timestep of the rejection. The timestep is 0 for an absolute rejection since it is an instantaneous reading. For innovation rejection it is the time difference between the previous accepted DVL measurement and the current one.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Bitfield (Local)

            reason : uint8_t, unit: Enumerated (Local)

            value : fp32_t, unit: m/s

            timestep : fp32_t, unit: s'''
        self._type = type
        self._reason = reason
        self._value = value
        self._timestep = timestep


class LblEstimate(_base.base_message):
    '''Distance between current LBL Beacon position and filter estimation.

       This message class contains the following fields and their respective types:
    beacon : message, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            var_x : fp32_t, unit: m

            var_y : fp32_t, unit: m

            distance : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_beacon', '_x', '_y', '_var_x', '_var_y', '_distance']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Navigation", id = 360, stable = None, description = "LBL Beacon position estimate.", fields = ('beacon', 'x', 'y', 'var_x', 'var_y', 'distance',), abbrev = "LblEstimate", name = "LBL Beacon Position Estimate", flags = None)

    beacon = _base.mutable_attr({'name': 'LBL Beacon Configuration', 'type': 'message', 'message-type': 'LblBeacon'}, "LBL Beacon configuration estimate.")
    '''LBL Beacon configuration estimate. Type: message'''
    x = _base.mutable_attr({'name': 'North position', 'type': 'fp32_t', 'unit': 'm'}, "The North position offset of the NED field with respect to origin.")
    '''The North position offset of the NED field with respect to origin. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'East position', 'type': 'fp32_t', 'unit': 'm'}, "The East position offset of the NED field with respect to origin.")
    '''The East position offset of the NED field with respect to origin. Type: fp32_t'''
    var_x = _base.mutable_attr({'name': 'North position variance', 'type': 'fp32_t', 'unit': 'm'}, "The North offset variance of the North/East/Down field with respect to LLH.")
    '''The North offset variance of the North/East/Down field with respect to LLH. Type: fp32_t'''
    var_y = _base.mutable_attr({'name': 'East position variance', 'type': 'fp32_t', 'unit': 'm'}, "The East offset variance of the North/East/Down field with respect to LLH.")
    '''The East offset variance of the North/East/Down field with respect to LLH. Type: fp32_t'''
    distance = _base.mutable_attr({'name': 'Distance', 'type': 'fp32_t', 'unit': 'm'}, "Distance between current LBL Beacon position and filter estimation.")
    '''Distance between current LBL Beacon position and filter estimation. Type: fp32_t'''

    def __init__(self, beacon = None, x = None, y = None, var_x = None, var_y = None, distance = None):
        '''Class constructor
        
        Distance between current LBL Beacon position and filter estimation.

       This message class contains the following fields and their respective types:
    beacon : message, unit: NOT FOUND

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            var_x : fp32_t, unit: m

            var_y : fp32_t, unit: m

            distance : fp32_t, unit: m'''
        self._beacon = beacon
        self._x = x
        self._y = y
        self._var_x = var_x
        self._var_y = var_y
        self._distance = distance


class AlignmentState(_base.base_message):
    '''Alignment State. Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: AS'''
    
        NOT_ALIGNED = 0
        '''Name: Not Aligned'''
    
        ALIGNED = 1
        '''Name: Aligned'''
    
        NOT_SUPPORTED = 2
        '''Name: Not Supported'''
    
        ALIGNING = 3
        '''Name: Aligning'''
    
        WRONG_MEDIUM = 4
        '''Name: Wrong Medium'''
    
        COARSE_ALIGNMENT = 5
        '''Name: Coarse Alignment'''
    
        FINE_ALIGNMENT = 6
        '''Name: Fine Alignment'''
    
        SYSTEM_READY = 7
        '''Name: System Ready'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 361, stable = None, description = "This message notifies the vehicle is ready for dead-reckoning missions.", fields = ('state',), abbrev = "AlignmentState", name = "Alignment State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'AS'}, "Alignment State. Enumerated (Local).")
    '''Alignment State. Enumerated (Local). Type: uint8_t'''

    def __init__(self, state = None):
        '''Class constructor
        
        Alignment State. Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''
        self._state = state


class GroupStreamVelocity(_base.base_message):
    '''Z component (Down).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Navigation", id = 362, stable = None, description = "The stream velocity estimated by a group of systems. Typically for water or air streams.", fields = ('x', 'y', 'z',), abbrev = "GroupStreamVelocity", name = "GroupStreamVelocity", flags = "periodic")

    x = _base.mutable_attr({'name': 'X component (North)', 'type': 'fp64_t', 'unit': 'm/s'}, "X component (North).")
    '''X component (North). Type: fp64_t'''
    y = _base.mutable_attr({'name': 'Y component (East)', 'type': 'fp64_t', 'unit': 'm/s'}, "Y component (East).")
    '''Y component (East). Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z component (Down)', 'type': 'fp64_t', 'unit': 'm/s'}, "Z component (Down).")
    '''Z component (Down). Type: fp64_t'''

    def __init__(self, x = None, y = None, z = None):
        '''Class constructor
        
        Z component (Down).

       This message class contains the following fields and their respective types:
    x : fp64_t, unit: m/s

            y : fp64_t, unit: m/s

            z : fp64_t, unit: m/s'''
        self._x = x
        self._y = y
        self._z = z


class Airflow(_base.base_message):
    '''Sideslip angle.

       This message class contains the following fields and their respective types:
    va : fp32_t, unit: m/s

            aoa : fp32_t, unit: rad

            ssa : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_va', '_aoa', '_ssa']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Navigation", id = 363, stable = None, description = "Airspeed along with airflow angles.", fields = ('va', 'aoa', 'ssa',), abbrev = "Airflow", name = "Airflow", flags = None)

    va = _base.mutable_attr({'name': 'Airspeed', 'type': 'fp32_t', 'unit': 'm/s'}, "Airspeed, the 2-norm of the relative velocity.")
    '''Airspeed, the 2-norm of the relative velocity. Type: fp32_t'''
    aoa = _base.mutable_attr({'name': 'Angle of attack', 'type': 'fp32_t', 'unit': 'rad'}, "Angle of attack.")
    '''Angle of attack. Type: fp32_t'''
    ssa = _base.mutable_attr({'name': 'Sideslip angle', 'type': 'fp32_t', 'unit': 'rad'}, "Sideslip angle.")
    '''Sideslip angle. Type: fp32_t'''

    def __init__(self, va = None, aoa = None, ssa = None):
        '''Class constructor
        
        Sideslip angle.

       This message class contains the following fields and their respective types:
    va : fp32_t, unit: m/s

            aoa : fp32_t, unit: rad

            ssa : fp32_t, unit: rad'''
        self._va = va
        self._aoa = aoa
        self._ssa = ssa


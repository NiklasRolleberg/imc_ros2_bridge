'''
IMC Simulation messages.
'''

from .. import _base
import enum as _enum

class SimulatedState(_base.base_message):
    '''Stream Velocity zz axis velocity component.

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

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            svx : fp32_t, unit: m/s

            svy : fp32_t, unit: m/s

            svz : fp32_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_height', '_x', '_y', '_z', '_phi', '_theta', '_psi', '_u', '_v', '_w', '_p', '_q', '_r', '_svx', '_svy', '_svz']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Simulation", id = 50, stable = None, description = "This message presents the simulated state of the vehicle. The simulated state attempts to provide a realistic state interpretation of operating various kinds of vehicles.", fields = ('lat', 'lon', 'height', 'x', 'y', 'z', 'phi', 'theta', 'psi', 'u', 'v', 'w', 'p', 'q', 'r', 'svx', 'svy', 'svz',), abbrev = "SimulatedState", name = "Simulated State", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    height = _base.mutable_attr({'name': 'Height (WGS-84)', 'type': 'fp32_t', 'unit': 'm'}, "Height above the WGS-84 ellipsoid.")
    '''Height above the WGS-84 ellipsoid. Type: fp32_t'''
    x = _base.mutable_attr({'name': 'Offset north (m)', 'type': 'fp32_t', 'unit': 'm'}, "The North offset of the North/East/Down field.")
    '''The North offset of the North/East/Down field. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Offset east (m)', 'type': 'fp32_t', 'unit': 'm'}, "The East offset of the North/East/Down field.")
    '''The East offset of the North/East/Down field. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Offset down (m)', 'type': 'fp32_t', 'unit': 'm'}, "The Down offset of the North/East/Down field.")
    '''The Down offset of the North/East/Down field. Type: fp32_t'''
    phi = _base.mutable_attr({'name': 'Rotation over x axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The phi Euler angle from the vehicle's attitude.")
    '''The phi Euler angle from the vehicle's attitude. Type: fp32_t'''
    theta = _base.mutable_attr({'name': 'Rotation over y axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267949, 'max': 1.5707963267949}, "The theta Euler angle from the vehicle's attitude.")
    '''The theta Euler angle from the vehicle's attitude. Type: fp32_t'''
    psi = _base.mutable_attr({'name': 'Rotation over z axis', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The psi Euler angle from the vehicle's attitude.")
    '''The psi Euler angle from the vehicle's attitude. Type: fp32_t'''
    u = _base.mutable_attr({'name': 'Body-Fixed xx Linear Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame xx axis linear velocity component.")
    '''Body-fixed frame xx axis linear velocity component. Type: fp32_t'''
    v = _base.mutable_attr({'name': 'Body-Fixed yy Linear Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame yy axis linear velocity component.")
    '''Body-fixed frame yy axis linear velocity component. Type: fp32_t'''
    w = _base.mutable_attr({'name': 'Body-Fixed zz Linear Velocity', 'type': 'fp32_t', 'unit': 'm/s'}, "Body-fixed frame zz axis linear velocity component.")
    '''Body-fixed frame zz axis linear velocity component. Type: fp32_t'''
    p = _base.mutable_attr({'name': 'Angular Velocity in x', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed xx axis (roll rate).")
    '''The angular velocity over body-fixed xx axis (roll rate). Type: fp32_t'''
    q = _base.mutable_attr({'name': 'Angular Velocity in y', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed yy axis (pitch rate).")
    '''The angular velocity over body-fixed yy axis (pitch rate). Type: fp32_t'''
    r = _base.mutable_attr({'name': 'Angular Velocity in z', 'type': 'fp32_t', 'unit': 'rad/s', 'min': -3.141592653589793, 'max': 3.141592653589793}, "The angular velocity over body-fixed zz axis (yaw rate).")
    '''The angular velocity over body-fixed zz axis (yaw rate). Type: fp32_t'''
    svx = _base.mutable_attr({'name': 'Stream Velocity X (North)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity xx axis velocity component.")
    '''Stream Velocity xx axis velocity component. Type: fp32_t'''
    svy = _base.mutable_attr({'name': 'Stream Velocity Y (East)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity yy axis velocity component.")
    '''Stream Velocity yy axis velocity component. Type: fp32_t'''
    svz = _base.mutable_attr({'name': 'Stream Velocity Z (Down)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity zz axis velocity component.")
    '''Stream Velocity zz axis velocity component. Type: fp32_t'''

    def __init__(self, lat = None, lon = None, height = None, x = None, y = None, z = None, phi = None, theta = None, psi = None, u = None, v = None, w = None, p = None, q = None, r = None, svx = None, svy = None, svz = None):
        '''Class constructor
        
        Stream Velocity zz axis velocity component.

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

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            svx : fp32_t, unit: m/s

            svy : fp32_t, unit: m/s

            svz : fp32_t, unit: m/s'''
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
        self._p = p
        self._q = q
        self._r = r
        self._svx = svx
        self._svy = svy
        self._svz = svz


class LeakSimulation(_base.base_message):
    '''Comma separated list of leak entities (empty for all leaks configured).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            entities : plaintext, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: LSIM'''
    
        OFF = 0
        '''Name: Leaks Off'''
    
        ON = 1
        '''Name: Leaks On'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_entities']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Simulation", id = 51, stable = None, description = "Simulate leak behavior.", fields = ('op', 'entities',), abbrev = "LeakSimulation", name = "Leak Simulation", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'prefix': 'LSIM', 'unit': 'Enumerated'}, "Indicates whether leaks have been detected or not. Enumerated (Local).")
    '''Indicates whether leaks have been detected or not. Enumerated (Local). Type: uint8_t'''
    entities = _base.mutable_attr({'name': 'Leak Entities', 'type': 'plaintext'}, "Comma separated list of leak entities (empty for all leaks configured).")
    '''Comma separated list of leak entities (empty for all leaks configured). Type: plaintext'''

    def __init__(self, op = None, entities = None):
        '''Class constructor
        
        Comma separated list of leak entities (empty for all leaks configured).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            entities : plaintext, unit: NOT FOUND'''
        self._op = op
        self._entities = entities


class UASimulation(_base.base_message):
    '''Data for transmission requests.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            speed : uint16_t, unit: bps

            data : rawdata, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: UAS'''
    
        DATA = 0
        '''Name: Data Transmission'''
    
        PING = 1
        '''Name: Ping'''
    
        PING_REPLY = 2
        '''Name: Ping Reply'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_speed', '_data']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Simulation", id = 52, stable = None, description = "Underwater acoustics simulation request.", fields = ('type', 'speed', 'data',), abbrev = "UASimulation", name = "Underwater Acoustics Simulation", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'prefix': 'UAS', 'unit': 'Enumerated'}, "Type of request. Enumerated (Local).")
    '''Type of request. Enumerated (Local). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Transmission Speed', 'unit': 'bps', 'type': 'uint16_t'}, "Transmission speed.")
    '''Transmission speed. Type: uint16_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "Data for transmission requests.")
    '''Data for transmission requests. Type: rawdata'''

    def __init__(self, type = None, speed = None, data = None):
        '''Class constructor
        
        Data for transmission requests.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            speed : uint16_t, unit: bps

            data : rawdata, unit: NOT FOUND'''
        self._type = type
        self._speed = speed
        self._data = data


class DynamicsSimParam(_base.base_message):
    '''Proportional gain from the bank angle error to the bank angular rate.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            tas2acc_pgain : fp32_t, unit: NOT FOUND

            bank2p_pgain : fp32_t, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Action on the Vehicle Simulation Parameters
        Prefix: OP'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SET = 1
        '''Name: Set'''
    
        REPORT = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_tas2acc_pgain', '_bank2p_pgain']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Simulation", id = 53, stable = None, description = "Vehicle dynamics parameters for 3DOF, 4DOF or 5DOF simulations.", fields = ('op', 'tas2acc_pgain', 'bank2p_pgain',), abbrev = "DynamicsSimParam", name = "Dynamics Simulation Parameters", flags = None)

    op = _base.mutable_attr({'name': 'Action on the Vehicle Simulation Parameters', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the vehicle simulation parameters for the formation control Enumerated (Local).")
    '''Action on the vehicle simulation parameters for the formation control Enumerated (Local). Type: uint8_t'''
    tas2acc_pgain = _base.mutable_attr({'name': 'TAS to Longitudinal Acceleration Gain', 'type': 'fp32_t'}, "Proportional gain from the TAS (True Airspeed) error to the longitudinal acceleration.")
    '''Proportional gain from the TAS (True Airspeed) error to the longitudinal acceleration. Type: fp32_t'''
    bank2p_pgain = _base.mutable_attr({'name': 'Bank to Bank Rate Gain', 'type': 'fp32_t'}, "Proportional gain from the bank angle error to the bank angular rate.")
    '''Proportional gain from the bank angle error to the bank angular rate. Type: fp32_t'''

    def __init__(self, op = None, tas2acc_pgain = None, bank2p_pgain = None):
        '''Class constructor
        
        Proportional gain from the bank angle error to the bank angular rate.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            tas2acc_pgain : fp32_t, unit: NOT FOUND

            bank2p_pgain : fp32_t, unit: NOT FOUND'''
        self._op = op
        self._tas2acc_pgain = tas2acc_pgain
        self._bank2p_pgain = bank2p_pgain


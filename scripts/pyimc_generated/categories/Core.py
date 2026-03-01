'''
IMC Core messages.
'''

from .. import _base
import enum as _enum

class EntityState(_base.base_message):
    '''Complementary human-readable description of entity state.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            flags : uint8_t, unit: Bitfield (Local)

            description : plaintext, unit: NOT FOUND'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: ESTA'''
    
        BOOT = 0
        '''Name: Bootstrapping'''
    
        NORMAL = 1
        '''Name: Normal Operation'''
    
        FAULT = 2
        '''Name: Fault'''
    
        ERROR = 3
        '''Name: Error'''
    
        FAILURE = 4
        '''Name: Failure'''
    
    
    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: EFLA'''
    
        EMPTY = 0
        '''No active flags'''
    
        HUMAN_INTERVENTION = 1
        '''Name: Human Intervention Required'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_flags', '_description']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Core", id = 1, stable = None, description = "State reported by an entity in the vehicle. The source entity is identified in the message header.", fields = ('state', 'flags', 'description',), abbrev = "EntityState", name = "Entity State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'ESTA'}, "State of entity. Enumerated (Local).")
    '''State of entity. Enumerated (Local). Type: uint8_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'EFLA'}, "Complementary entity state flags. Bitfield (Local).")
    '''Complementary entity state flags. Bitfield (Local). Type: uint8_t'''
    description = _base.mutable_attr({'name': 'Complementary description', 'type': 'plaintext'}, "Complementary human-readable description of entity state.")
    '''Complementary human-readable description of entity state. Type: plaintext'''

    def __init__(self, state = None, flags = None, description = None):
        '''Class constructor
        
        Complementary human-readable description of entity state.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            flags : uint8_t, unit: Bitfield (Local)

            description : plaintext, unit: NOT FOUND'''
        self._state = state
        self._flags = flags
        self._description = description


class QueryEntityState(_base.base_message):
    '''Request entities to report their state. Entities should respond by issuing an appropriate EntityState message.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Core", id = 2, stable = None, description = "Request entities to report their state. Entities should respond by issuing an appropriate EntityState message.", fields = [], abbrev = "QueryEntityState", name = "Query Entity State", flags = "periodic")


    def __init__(self, ):
        '''Class constructor
        
        Request entities to report their state. Entities should respond by issuing an appropriate EntityState message.

       This message class contains the following fields and their respective types:
'''


class EntityInfo(_base.base_message):
    '''Amount of time needed to properly deactivate the entity.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            label : plaintext, unit: NOT FOUND

            component : plaintext, unit: NOT FOUND

            act_time : uint16_t, unit: s

            deact_time : uint16_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_label', '_component', '_act_time', '_deact_time']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Core", id = 3, stable = None, description = "This message describes an entity.", fields = ('id', 'label', 'component', 'act_time', 'deact_time',), abbrev = "EntityInfo", name = "Entity Information", flags = None)

    id = _base.mutable_attr({'name': 'Entity Identifier', 'type': 'uint8_t'}, "Entity identifier.")
    '''Entity identifier. Type: uint8_t'''
    label = _base.mutable_attr({'name': 'Label', 'type': 'plaintext'}, "Entity label or empty if the entity id is not valid.")
    '''Entity label or empty if the entity id is not valid. Type: plaintext'''
    component = _base.mutable_attr({'name': 'Component name', 'type': 'plaintext'}, "Name of the plugin/component/subsystem associated with this entity.")
    '''Name of the plugin/component/subsystem associated with this entity. Type: plaintext'''
    act_time = _base.mutable_attr({'name': 'Activation Time', 'type': 'uint16_t', 'unit': 's'}, "Amount of time needed to properly activate the entity.")
    '''Amount of time needed to properly activate the entity. Type: uint16_t'''
    deact_time = _base.mutable_attr({'name': 'Deactivation Time', 'type': 'uint16_t', 'unit': 's'}, "Amount of time needed to properly deactivate the entity.")
    '''Amount of time needed to properly deactivate the entity. Type: uint16_t'''

    def __init__(self, id = None, label = None, component = None, act_time = None, deact_time = None):
        '''Class constructor
        
        Amount of time needed to properly deactivate the entity.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            label : plaintext, unit: NOT FOUND

            component : plaintext, unit: NOT FOUND

            act_time : uint16_t, unit: s

            deact_time : uint16_t, unit: s'''
        self._id = id
        self._label = label
        self._component = component
        self._act_time = act_time
        self._deact_time = deact_time


class QueryEntityInfo(_base.base_message):
    '''Entity identifier.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Core", id = 4, stable = None, description = "Request information about an entity identifier. The receiving system shall reply with an EntityInfo message with the details of that entity.", fields = ('id',), abbrev = "QueryEntityInfo", name = "Query Entity Information", flags = None)

    id = _base.mutable_attr({'name': 'Entity Identifier', 'type': 'uint8_t'}, "Entity identifier.")
    '''Entity identifier. Type: uint8_t'''

    def __init__(self, id = None):
        '''Class constructor
        
        Entity identifier.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND'''
        self._id = id


class EntityList(_base.base_message):
    '''Example: \"Battery=11;CTD=3\"

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            list : plaintext, unit: TupleList'''

    class OP(_enum.IntEnum):
        '''Full name: operation
        Prefix: OP'''
    
        REPORT = 0
        '''Name: Report'''
    
        QUERY = 1
        '''Name: Query'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_list']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Core", id = 5, stable = None, description = "This message describes the names and identification numbers of all entities in the system.", fields = ('op', 'list',), abbrev = "EntityList", name = "Entity List", flags = None)

    op = _base.mutable_attr({'name': 'operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    list = _base.mutable_attr({'name': 'list', 'type': 'plaintext', 'unit': 'TupleList'}, "Example: \"Battery=11;CTD=3\"")
    '''Example: \"Battery=11;CTD=3\" Type: plaintext'''

    def __init__(self, op = None, list = None):
        '''Class constructor
        
        Example: \"Battery=11;CTD=3\"

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            list : plaintext, unit: TupleList'''
        self._op = op
        self._list = list


class CpuUsage(_base.base_message):
    '''The CPU usage, in percentage, of the sending software.

       This message class contains the following fields and their respective types:
    value : uint8_t, unit: %'''

    __slots__ = ['_Attributes', '_header', '_footer', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Core", id = 7, stable = None, description = "Report of software CPU usage.", fields = ('value',), abbrev = "CpuUsage", name = "CPU Usage", flags = "periodic")

    value = _base.mutable_attr({'name': 'Usage percentage', 'type': 'uint8_t', 'max': 100, 'unit': '%'}, "The CPU usage, in percentage, of the sending software.")
    '''The CPU usage, in percentage, of the sending software. Type: uint8_t'''

    def __init__(self, value = None):
        '''Class constructor
        
        The CPU usage, in percentage, of the sending software.

       This message class contains the following fields and their respective types:
    value : uint8_t, unit: %'''
        self._value = value


class TransportBindings(_base.base_message):
    '''The id of the message to be listened to.

       This message class contains the following fields and their respective types:
    consumer : plaintext, unit: NOT FOUND

            message_id : uint16_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_consumer', '_message_id']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Core", id = 8, stable = None, description = "Message generated when tasks bind to messages.", fields = ('consumer', 'message_id',), abbrev = "TransportBindings", name = "Transport Bindings", flags = None)

    consumer = _base.mutable_attr({'name': 'Consumer name', 'type': 'plaintext'}, "The name of the consumer (e.g. task name).")
    '''The name of the consumer (e.g. task name). Type: plaintext'''
    message_id = _base.mutable_attr({'name': 'Message Identifier', 'type': 'uint16_t'}, "The id of the message to be listened to.")
    '''The id of the message to be listened to. Type: uint16_t'''

    def __init__(self, consumer = None, message_id = None):
        '''Class constructor
        
        The id of the message to be listened to.

       This message class contains the following fields and their respective types:
    consumer : plaintext, unit: NOT FOUND

            message_id : uint16_t, unit: NOT FOUND'''
        self._consumer = consumer
        self._message_id = message_id


class RestartSystem(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Restart Type
        Prefix: RSTYPE'''
    
        DUNE = 1
        '''Name: Dune'''
    
        SYSTEM = 2
        '''Name: System'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Core", id = 9, stable = None, description = "Request the destination system to restart itself.", fields = ('type',), abbrev = "RestartSystem", name = "Restart System", flags = None)

    type = _base.mutable_attr({'name': 'Restart Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RSTYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, type = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)'''
        self._type = type


class DevCalibrationControl(_base.base_message):
    '''Operation to perform. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: DCAL'''
    
        START = 0
        '''Name: Start'''
    
        STOP = 1
        '''Name: Stop'''
    
        STEP_NEXT = 2
        '''Name: Perform Next Calibration Step'''
    
        STEP_PREVIOUS = 3
        '''Name: Perform Previous Calibration Step'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 12, stable = None, description = "This message controls the calibration procedure of a given device. The destination device is selected using the destination entity identification number.", fields = ('op',), abbrev = "DevCalibrationControl", name = "Device Calibration Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DCAL'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''

    def __init__(self, op = None):
        '''Class constructor
        
        Operation to perform. Enumerated (Local).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)'''
        self._op = op


class DevCalibrationState(_base.base_message):
    '''Additional flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    total_steps : uint8_t, unit: NOT FOUND

            step_number : uint8_t, unit: NOT FOUND

            step : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: DCS'''
    
        EMPTY = 0
        '''No active flags'''
    
        PREVIOUS_NOT_SUPPORTED = 1
        '''Name: Previous Step Not Supported'''
    
        NEXT_NOT_SUPPORTED = 2
        '''Name: Next Step Not Supported'''
    
        WAITING_CONTROL = 4
        '''Name: Waiting Device Calibration Control'''
    
        ERROR = 8
        '''Name: Calibration Error'''
    
        COMPLETED = 16
        '''Name: Calibration Procedure Completed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_total_steps', '_step_number', '_step', '_flags']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 13, stable = None, description = "State of the calibration procedure.", fields = ('total_steps', 'step_number', 'step', 'flags',), abbrev = "DevCalibrationState", name = "Device Calibration State", flags = None)

    total_steps = _base.mutable_attr({'name': 'Total Steps', 'type': 'uint8_t'}, "Total number of steps of the calibration procedure.")
    '''Total number of steps of the calibration procedure. Type: uint8_t'''
    step_number = _base.mutable_attr({'name': 'Current Step Number', 'type': 'uint8_t'}, "Number of the current step being performed.")
    '''Number of the current step being performed. Type: uint8_t'''
    step = _base.mutable_attr({'name': 'Description', 'type': 'plaintext'}, "Human-readable description of the current step.")
    '''Human-readable description of the current step. Type: plaintext'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'DCS'}, "Additional flags. Bitfield (Local).")
    '''Additional flags. Bitfield (Local). Type: uint8_t'''

    def __init__(self, total_steps = None, step_number = None, step = None, flags = None):
        '''Class constructor
        
        Additional flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    total_steps : uint8_t, unit: NOT FOUND

            step_number : uint8_t, unit: NOT FOUND

            step : plaintext, unit: NOT FOUND

            flags : uint8_t, unit: Bitfield (Local)'''
        self._total_steps = total_steps
        self._step_number = step_number
        self._step = step
        self._flags = flags


class EntityActivationState(_base.base_message):
    '''Human-readable error message.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: EAS'''
    
        INACTIVE = 0
        '''Name: Entity is Inactive'''
    
        ACTIVE = 1
        '''Name: Entity is Active'''
    
        ACT_IP = 2
        '''Name: Activation in Progress'''
    
        ACT_DONE = 3
        '''Name: Activation Completed'''
    
        ACT_FAIL = 4
        '''Name: Activation Failed'''
    
        DEACT_IP = 5
        '''Name: Deactivation In Progress'''
    
        DEACT_DONE = 6
        '''Name: Deactivation Completed'''
    
        DEACT_FAIL = 7
        '''Name: Deactivation Failed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_error']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 14, stable = None, description = "State of entity activation/deactivation.", fields = ('state', 'error',), abbrev = "EntityActivationState", name = "Entity Activation State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'EAS'}, "Current state. Enumerated (Local).")
    '''Current state. Enumerated (Local). Type: uint8_t'''
    error = _base.mutable_attr({'name': 'Error', 'type': 'plaintext'}, "Human-readable error message.")
    '''Human-readable error message. Type: plaintext'''

    def __init__(self, state = None, error = None):
        '''Class constructor
        
        Human-readable error message.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''
        self._state = state
        self._error = error


class QueryEntityActivationState(_base.base_message):
    '''Query the activation/deactivation state of an entity. The recipient shall reply with an EntityActivationState message.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 15, stable = None, description = "Query the activation/deactivation state of an entity. The recipient shall reply with an EntityActivationState message.", fields = [], abbrev = "QueryEntityActivationState", name = "Query Entity Activation State", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Query the activation/deactivation state of an entity. The recipient shall reply with an EntityActivationState message.

       This message class contains the following fields and their respective types:
'''


class VehicleOperationalLimits(_base.base_message):
    '''Maximum motor RPMs' rate of change.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            speed_min : fp32_t, unit: m/s

            speed_max : fp32_t, unit: m/s

            long_accel : fp32_t, unit: m/s/s

            alt_max_msl : fp32_t, unit: m

            dive_fraction_max : fp32_t, unit: NOT FOUND

            climb_fraction_max : fp32_t, unit: NOT FOUND

            bank_max : fp32_t, unit: rad

            p_max : fp32_t, unit: rad/s

            pitch_min : fp32_t, unit: rad

            pitch_max : fp32_t, unit: rad

            q_max : fp32_t, unit: rad/s

            g_min : fp32_t, unit: g

            g_max : fp32_t, unit: g

            g_lat_max : fp32_t, unit: g

            rpm_min : fp32_t, unit: rpm

            rpm_max : fp32_t, unit: rpm

            rpm_rate_max : fp32_t, unit: rpm/s'''

    class OP(_enum.IntEnum):
        '''Full name: Action on the vehicle operational limits
        Prefix: OP'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SET = 1
        '''Name: Set'''
    
        REPORT = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_speed_min', '_speed_max', '_long_accel', '_alt_max_msl', '_dive_fraction_max', '_climb_fraction_max', '_bank_max', '_p_max', '_pitch_min', '_pitch_max', '_q_max', '_g_min', '_g_max', '_g_lat_max', '_rpm_min', '_rpm_max', '_rpm_rate_max']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 16, stable = None, description = "Vehicle opertional limits. For aircraft this should represent the flight envelope and the dynamic contraints.", fields = ('op', 'speed_min', 'speed_max', 'long_accel', 'alt_max_msl', 'dive_fraction_max', 'climb_fraction_max', 'bank_max', 'p_max', 'pitch_min', 'pitch_max', 'q_max', 'g_min', 'g_max', 'g_lat_max', 'rpm_min', 'rpm_max', 'rpm_rate_max',), abbrev = "VehicleOperationalLimits", name = "Vehicle Operational Limits", flags = None)

    op = _base.mutable_attr({'name': 'Action on the vehicle operational limits', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the vehicle operation limits Enumerated (Local).")
    '''Action on the vehicle operation limits Enumerated (Local). Type: uint8_t'''
    speed_min = _base.mutable_attr({'name': 'Minimum speed', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "Minimum operation speed. For aircraft this is equal or larger then the stall speed.")
    '''Minimum operation speed. For aircraft this is equal or larger then the stall speed. Type: fp32_t'''
    speed_max = _base.mutable_attr({'name': 'Maximum speed', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "Maximum operation speed. For aircraft this is limited by the engine power or structural contrains.")
    '''Maximum operation speed. For aircraft this is limited by the engine power or structural contrains. Type: fp32_t'''
    long_accel = _base.mutable_attr({'name': 'Longitudinal maximum acceleration', 'type': 'fp32_t', 'unit': 'm/s/s', 'min': 0}, "Maximum longitudinal acceleration.")
    '''Maximum longitudinal acceleration. Type: fp32_t'''
    alt_max_msl = _base.mutable_attr({'name': 'Maximum MSL altitude', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "Maximum altitude above mean-sea-level.")
    '''Maximum altitude above mean-sea-level. Type: fp32_t'''
    dive_fraction_max = _base.mutable_attr({'name': 'Maximum Dive Rate Speed Fraction', 'type': 'fp32_t', 'min': 0}, "Maximum dive rate (negative vertical speed) as a fraction of the longitudinal speed.")
    '''Maximum dive rate (negative vertical speed) as a fraction of the longitudinal speed. Type: fp32_t'''
    climb_fraction_max = _base.mutable_attr({'name': 'Maximum Climb Rate Speed Fraction', 'type': 'fp32_t', 'min': 0}, "Maximum climb rate (positive vertical speed) as a fraction of the longitudinal speed.")
    '''Maximum climb rate (positive vertical speed) as a fraction of the longitudinal speed. Type: fp32_t'''
    bank_max = _base.mutable_attr({'name': 'Bank limit', 'type': 'fp32_t', 'unit': 'rad', 'min': 0}, "Limit to the bank angle (roll; angle over the xx body-axis).")
    '''Limit to the bank angle (roll; angle over the xx body-axis). Type: fp32_t'''
    p_max = _base.mutable_attr({'name': 'Bank rate limit', 'type': 'fp32_t', 'unit': 'rad/s', 'min': 0}, "Limit to the bank angular rate (roll; angle over the xx body-axis).")
    '''Limit to the bank angular rate (roll; angle over the xx body-axis). Type: fp32_t'''
    pitch_min = _base.mutable_attr({'name': 'Minimum pitch angle', 'type': 'fp32_t', 'unit': 'rad'}, "Minimum pitch angle (angle over the xx body-axis).")
    '''Minimum pitch angle (angle over the xx body-axis). Type: fp32_t'''
    pitch_max = _base.mutable_attr({'name': 'Maximum pitch angle', 'type': 'fp32_t', 'unit': 'rad'}, "Maximum pitch angle (angle over the xx body-axis).")
    '''Maximum pitch angle (angle over the xx body-axis). Type: fp32_t'''
    q_max = _base.mutable_attr({'name': 'Maximum pitch rate', 'type': 'fp32_t', 'unit': 'rad/s', 'min': 0}, "Maximum pitch angular rate (angle over the xx body-axis).")
    '''Maximum pitch angular rate (angle over the xx body-axis). Type: fp32_t'''
    g_min = _base.mutable_attr({'name': 'Minimum load factor', 'type': 'fp32_t', 'unit': 'g', 'max': 0}, "Minimum load factor, i.e., maximum positive acceleration in the zz body-axis as a factor of the gravity acceleration at mean-sea-level.")
    '''Minimum load factor, i.e., maximum positive acceleration in the zz body-axis as a factor of the gravity acceleration at mean-sea-level. Type: fp32_t'''
    g_max = _base.mutable_attr({'name': 'Maximum load factor', 'type': 'fp32_t', 'unit': 'g', 'min': 0}, "Maximum load factor, i.e., maximum negative acceleration in the zz body-axis as a factor of the gravity acceleration at mean-sea-level.")
    '''Maximum load factor, i.e., maximum negative acceleration in the zz body-axis as a factor of the gravity acceleration at mean-sea-level. Type: fp32_t'''
    g_lat_max = _base.mutable_attr({'name': 'Maximum lateral load factor', 'type': 'fp32_t', 'unit': 'g', 'min': 0}, "Maximum lateral load factor, i.e., maximum acceleration in the yy body-axis as a factor of the gravity acceleration at mean-sea-level.")
    '''Maximum lateral load factor, i.e., maximum acceleration in the yy body-axis as a factor of the gravity acceleration at mean-sea-level. Type: fp32_t'''
    rpm_min = _base.mutable_attr({'name': 'Minimum RPMs', 'type': 'fp32_t', 'unit': 'rpm', 'min': 0}, "Minimum motor RPMs.")
    '''Minimum motor RPMs. Type: fp32_t'''
    rpm_max = _base.mutable_attr({'name': 'Maximum RPMs', 'type': 'fp32_t', 'unit': 'rpm', 'min': 0}, "Maximum motor RPMs.")
    '''Maximum motor RPMs. Type: fp32_t'''
    rpm_rate_max = _base.mutable_attr({'name': 'Maximum RPM rate', 'type': 'fp32_t', 'unit': 'rpm/s', 'min': 0}, "Maximum motor RPMs' rate of change.")
    '''Maximum motor RPMs' rate of change. Type: fp32_t'''

    def __init__(self, op = None, speed_min = None, speed_max = None, long_accel = None, alt_max_msl = None, dive_fraction_max = None, climb_fraction_max = None, bank_max = None, p_max = None, pitch_min = None, pitch_max = None, q_max = None, g_min = None, g_max = None, g_lat_max = None, rpm_min = None, rpm_max = None, rpm_rate_max = None):
        '''Class constructor
        
        Maximum motor RPMs' rate of change.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            speed_min : fp32_t, unit: m/s

            speed_max : fp32_t, unit: m/s

            long_accel : fp32_t, unit: m/s/s

            alt_max_msl : fp32_t, unit: m

            dive_fraction_max : fp32_t, unit: NOT FOUND

            climb_fraction_max : fp32_t, unit: NOT FOUND

            bank_max : fp32_t, unit: rad

            p_max : fp32_t, unit: rad/s

            pitch_min : fp32_t, unit: rad

            pitch_max : fp32_t, unit: rad

            q_max : fp32_t, unit: rad/s

            g_min : fp32_t, unit: g

            g_max : fp32_t, unit: g

            g_lat_max : fp32_t, unit: g

            rpm_min : fp32_t, unit: rpm

            rpm_max : fp32_t, unit: rpm

            rpm_rate_max : fp32_t, unit: rpm/s'''
        self._op = op
        self._speed_min = speed_min
        self._speed_max = speed_max
        self._long_accel = long_accel
        self._alt_max_msl = alt_max_msl
        self._dive_fraction_max = dive_fraction_max
        self._climb_fraction_max = climb_fraction_max
        self._bank_max = bank_max
        self._p_max = p_max
        self._pitch_min = pitch_min
        self._pitch_max = pitch_max
        self._q_max = q_max
        self._g_min = g_min
        self._g_max = g_max
        self._g_lat_max = g_lat_max
        self._rpm_min = rpm_min
        self._rpm_max = rpm_max
        self._rpm_rate_max = rpm_rate_max


class MsgList(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    msgs : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_msgs']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Core", id = 20, stable = None, description = None, fields = ('msgs',), abbrev = "MsgList", name = "Message List", flags = None)

    msgs = _base.mutable_attr({'name': 'Messages', 'type': 'message-list'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, msgs = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    msgs : message-list, unit: NOT FOUND'''
        self._msgs = msgs


class EntityParameter(_base.base_message):
    '''Current value of the parameter.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 801, stable = None, description = "Entity parameter.", fields = ('name', 'value',), abbrev = "EntityParameter", name = "EntityParameter", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "Name of the parameter.")
    '''Name of the parameter. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'plaintext'}, "Current value of the parameter.")
    '''Current value of the parameter. Type: plaintext'''

    def __init__(self, name = None, value = None):
        '''Class constructor
        
        Current value of the parameter.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : plaintext, unit: NOT FOUND'''
        self._name = name
        self._value = value


class EntityParameters(_base.base_message):
    '''List of parameters.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            params : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_params']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 802, stable = None, description = "List of entity parameters.", fields = ('name', 'params',), abbrev = "EntityParameters", name = "EntityParameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "Name of the entity.")
    '''Name of the entity. Type: plaintext'''
    params = _base.mutable_attr({'name': 'Parameters', 'type': 'message-list', 'message-type': 'EntityParameter'}, "List of parameters.")
    '''List of parameters. Type: message-list'''

    def __init__(self, name = None, params = None):
        '''Class constructor
        
        List of parameters.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            params : message-list, unit: NOT FOUND'''
        self._name = name
        self._params = params


class QueryEntityParameters(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            visibility : plaintext, unit: NOT FOUND

            scope : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_visibility', '_scope']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 803, stable = None, description = None, fields = ('name', 'visibility', 'scope',), abbrev = "QueryEntityParameters", name = "QueryEntityParameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    visibility = _base.mutable_attr({'name': 'Visibility', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    scope = _base.mutable_attr({'name': 'Scope', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, name = None, visibility = None, scope = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            visibility : plaintext, unit: NOT FOUND

            scope : plaintext, unit: NOT FOUND'''
        self._name = name
        self._visibility = visibility
        self._scope = scope


class SetEntityParameters(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            params : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_params']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 804, stable = None, description = None, fields = ('name', 'params',), abbrev = "SetEntityParameters", name = "SetEntityParameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    params = _base.mutable_attr({'name': 'Parameters', 'type': 'message-list', 'message-type': 'EntityParameter'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, name = None, params = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            params : message-list, unit: NOT FOUND'''
        self._name = name
        self._params = params


class SaveEntityParameters(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 805, stable = None, description = None, fields = ('name',), abbrev = "SaveEntityParameters", name = "SaveEntityParameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, name = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''
        self._name = name


class PushEntityParameters(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 811, stable = None, description = None, fields = ('name',), abbrev = "PushEntityParameters", name = "Push Entity Parameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, name = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''
        self._name = name


class PopEntityParameters(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 812, stable = None, description = None, fields = ('name',), abbrev = "PopEntityParameters", name = "Pop Entity Parameters", flags = None)

    name = _base.mutable_attr({'name': 'Entity Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, name = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''
        self._name = name


class IoEvent(_base.base_message):
    '''Human-readable error message.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: IOV_TYPE'''
    
        INPUT = 1
        '''Name: Input Available'''
    
        INPUT_ERROR = 2
        '''Name: Input Error'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_error']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Core", id = 813, stable = None, description = "Notification of an I/O event.", fields = ('type', 'error',), abbrev = "IoEvent", name = "I/O Event", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'IOV_TYPE'}, "Event type. Enumerated (Local).")
    '''Event type. Enumerated (Local). Type: uint8_t'''
    error = _base.mutable_attr({'name': 'Error Message', 'type': 'plaintext'}, "Human-readable error message.")
    '''Human-readable error message. Type: plaintext'''

    def __init__(self, type = None, error = None):
        '''Class constructor
        
        Human-readable error message.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            error : plaintext, unit: NOT FOUND'''
        self._type = type
        self._error = error


class HomePosition(_base.base_message):
    '''Altitude, in meters. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m'''

    class OP(_enum.IntEnum):
        '''Full name: Action on the vehicle home position
        Prefix: OP'''
    
        SET = 1
        '''Name: Set'''
    
        REPORT = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_lat', '_lon', '_height', '_depth', '_alt']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Core", id = 909, stable = None, description = "Vehicle Home Position.", fields = ('op', 'lat', 'lon', 'height', 'depth', 'alt',), abbrev = "HomePosition", name = "Home Position", flags = None)

    op = _base.mutable_attr({'name': 'Action on the vehicle home position', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the vehicle home Enumerated (Local).")
    '''Action on the vehicle home Enumerated (Local). Type: uint8_t'''
    lat = _base.mutable_attr({'name': 'Latitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude (WGS-84)', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    height = _base.mutable_attr({'name': 'Height (WGS-84)', 'type': 'fp32_t', 'unit': 'm'}, "Height above the WGS-84 ellipsoid.")
    '''Height above the WGS-84 ellipsoid. Type: fp32_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t', 'unit': 'm'}, "Depth, in meters. To be used by underwater vehicles. Negative values denote invalid estimates.")
    '''Depth, in meters. To be used by underwater vehicles. Negative values denote invalid estimates. Type: fp32_t'''
    alt = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Altitude, in meters. Negative values denote invalid estimates.")
    '''Altitude, in meters. Negative values denote invalid estimates. Type: fp32_t'''

    def __init__(self, op = None, lat = None, lon = None, height = None, depth = None, alt = None):
        '''Class constructor
        
        Altitude, in meters. Negative values denote invalid estimates.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m'''
        self._op = op
        self._lat = lat
        self._lon = lon
        self._height = height
        self._depth = depth
        self._alt = alt


'''
IMC Vehicle Supervision messages.
'''

from .. import _base
import enum as _enum

class VehicleState(_base.base_message):
    '''Time of last error (Epoch time).

       This message class contains the following fields and their respective types:
    op_mode : uint8_t, unit: Enumerated (Local)

            error_count : uint8_t, unit: NOT FOUND

            error_ents : plaintext, unit: NOT FOUND

            maneuver_type : uint16_t, unit: NOT FOUND

            maneuver_stime : fp64_t, unit: s

            maneuver_eta : uint16_t, unit: s

            control_loops : uint32_t, unit: Bitfield (Global)

            flags : uint8_t, unit: Bitfield (Local)

            last_error : plaintext, unit: NOT FOUND

            last_error_time : fp64_t, unit: s'''

    class OP_MODE(_enum.IntEnum):
        '''Full name: Operation Mode
        Prefix: VS'''
    
        SERVICE = 0
        '''Name: Service'''
    
        CALIBRATION = 1
        '''Name: Calibration'''
    
        ERROR = 2
        '''Name: Error'''
    
        MANEUVER = 3
        '''Name: Maneuver'''
    
        EXTERNAL = 4
        '''Name: External Control'''
    
        BOOT = 5
        '''Name: Boot'''
    
    
    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: VFLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        MANEUVER_DONE = 1
        '''Name: Maneuver Done'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op_mode', '_error_count', '_error_ents', '_maneuver_type', '_maneuver_stime', '_maneuver_eta', '_control_loops', '_flags', '_last_error', '_last_error_time']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 500, stable = None, description = "This message summarizes the overall state of the vehicle. It can contains information regarding: - The overall operation mode. - Any error conditions. - Current maneuver execution. - Active control loops.", fields = ('op_mode', 'error_count', 'error_ents', 'maneuver_type', 'maneuver_stime', 'maneuver_eta', 'control_loops', 'flags', 'last_error', 'last_error_time',), abbrev = "VehicleState", name = "Vehicle State", flags = None)

    op_mode = _base.mutable_attr({'name': 'Operation Mode', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'VS'}, "The overall operation mode. Enumerated (Local).")
    '''The overall operation mode. Enumerated (Local). Type: uint8_t'''
    error_count = _base.mutable_attr({'name': 'Errors -- Count', 'type': 'uint8_t'}, "Error count for monitored entitites.")
    '''Error count for monitored entitites. Type: uint8_t'''
    error_ents = _base.mutable_attr({'name': 'Errors -- Entities', 'type': 'plaintext'}, "The monitored entities with error conditions. This is a comma separated list of entity names.")
    '''The monitored entities with error conditions. This is a comma separated list of entity names. Type: plaintext'''
    maneuver_type = _base.mutable_attr({'name': 'Maneuver -- Type', 'type': 'uint16_t'}, "Type of maneuver being executed, when in MANEUVER mode. The value is the IMC serialization ID of the corresponding maneuver.")
    '''Type of maneuver being executed, when in MANEUVER mode. The value is the IMC serialization ID of the corresponding maneuver. Type: uint16_t'''
    maneuver_stime = _base.mutable_attr({'name': 'Maneuver -- Start Time', 'type': 'fp64_t', 'unit': 's'}, "Start time of maneuver being executed (Epoch time), when in MANEUVER mode.")
    '''Start time of maneuver being executed (Epoch time), when in MANEUVER mode. Type: fp64_t'''
    maneuver_eta = _base.mutable_attr({'name': 'Maneuver -- ETA', 'type': 'uint16_t', 'unit': 's'}, "Estimated time for maneuver completion. The value will be 65535 if the time is unknown or undefined.")
    '''Estimated time for maneuver completion. The value will be 65535 if the time is unknown or undefined. Type: uint16_t'''
    control_loops = _base.mutable_attr({'name': 'Control Loops', 'type': 'uint32_t', 'unit': 'Bitfield', 'bitfield-def': 'CLoopsMask'}, "Enabled control loops. Bitfield (Global).")
    '''Enabled control loops. Bitfield (Global). Type: uint32_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'prefix': 'VFLG', 'unit': 'Bitfield'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    last_error = _base.mutable_attr({'name': 'Last Error -- Description', 'type': 'plaintext'}, "Description of last error.")
    '''Description of last error. Type: plaintext'''
    last_error_time = _base.mutable_attr({'name': 'Last Error -- Time', 'type': 'fp64_t', 'unit': 's'}, "Time of last error (Epoch time).")
    '''Time of last error (Epoch time). Type: fp64_t'''

    def __init__(self, op_mode = None, error_count = None, error_ents = None, maneuver_type = None, maneuver_stime = None, maneuver_eta = None, control_loops = None, flags = None, last_error = None, last_error_time = None):
        '''Class constructor
        
        Time of last error (Epoch time).

       This message class contains the following fields and their respective types:
    op_mode : uint8_t, unit: Enumerated (Local)

            error_count : uint8_t, unit: NOT FOUND

            error_ents : plaintext, unit: NOT FOUND

            maneuver_type : uint16_t, unit: NOT FOUND

            maneuver_stime : fp64_t, unit: s

            maneuver_eta : uint16_t, unit: s

            control_loops : uint32_t, unit: Bitfield (Global)

            flags : uint8_t, unit: Bitfield (Local)

            last_error : plaintext, unit: NOT FOUND

            last_error_time : fp64_t, unit: s'''
        self._op_mode = op_mode
        self._error_count = error_count
        self._error_ents = error_ents
        self._maneuver_type = maneuver_type
        self._maneuver_stime = maneuver_stime
        self._maneuver_eta = maneuver_eta
        self._control_loops = control_loops
        self._flags = flags
        self._last_error = last_error
        self._last_error_time = last_error_time


class VehicleCommand(_base.base_message):
    '''Complementary human-readable information for replies.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            command : uint8_t, unit: Enumerated (Local)

            maneuver : message, unit: NOT FOUND

            calib_time : uint16_t, unit: s

            info : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: VC'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SUCCESS = 1
        '''Name: Reply -- Success'''
    
        IN_PROGRESS = 2
        '''Name: Reply -- In Progress'''
    
        FAILURE = 3
        '''Name: Reply -- Failure'''
    
    
    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: VC'''
    
        EXEC_MANEUVER = 0
        '''Name: Execute Maneuver'''
    
        STOP_MANEUVER = 1
        '''Name: Stop Maneuver'''
    
        START_CALIBRATION = 2
        '''Name: Start Calibration'''
    
        STOP_CALIBRATION = 3
        '''Name: Stop Calibration'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_request_id', '_command', '_maneuver', '_calib_time', '_info']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Vehicle Supervision", id = 501, stable = None, description = "Vehicle command.", fields = ('type', 'request_id', 'command', 'maneuver', 'calib_time', 'info',), abbrev = "VehicleCommand", name = "Vehicle Command", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'VC'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    request_id = _base.mutable_attr({'name': 'Request ID', 'type': 'uint16_t'}, "Request ID")
    '''Request ID Type: uint16_t'''
    command = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'VC'}, "The type of command/action to be performed Enumerated (Local).")
    '''The type of command/action to be performed Enumerated (Local). Type: uint8_t'''
    maneuver = _base.mutable_attr({'name': 'Maneuver', 'type': 'message', 'message-type': 'Maneuver'}, "Maneuver to be executed (for 'EXEC_MANEUVER' command)")
    '''Maneuver to be executed (for 'EXEC_MANEUVER' command) Type: message'''
    calib_time = _base.mutable_attr({'name': 'Calibration Time', 'type': 'uint16_t', 'unit': 's'}, "Amount of time to calibrate")
    '''Amount of time to calibrate Type: uint16_t'''
    info = _base.mutable_attr({'name': 'Info', 'type': 'plaintext'}, "Complementary human-readable information for replies.")
    '''Complementary human-readable information for replies. Type: plaintext'''

    def __init__(self, type = None, request_id = None, command = None, maneuver = None, calib_time = None, info = None):
        '''Class constructor
        
        Complementary human-readable information for replies.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            command : uint8_t, unit: Enumerated (Local)

            maneuver : message, unit: NOT FOUND

            calib_time : uint16_t, unit: s

            info : plaintext, unit: NOT FOUND'''
        self._type = type
        self._request_id = request_id
        self._command = command
        self._maneuver = maneuver
        self._calib_time = calib_time
        self._info = info


class MonitorEntityState(_base.base_message):
    '''Comma separated list of entity names.

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            entities : plaintext, unit: NOT FOUND'''

    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: MES'''
    
        RESET = 0
        '''Name: Reset to defaults'''
    
        ENABLE = 1
        '''Name: Enable Monitoring'''
    
        DISABLE = 2
        '''Name: Disable Monitoring'''
    
        ENABLE_EXCLUSIVE = 3
        '''Name: Enable Monitoring (exclusive - disables all others)'''
    
        STATUS = 4
        '''Name: Status Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_command', '_entities']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 502, stable = None, description = "Controls monitoring of entity states in the vehicle.", fields = ('command', 'entities',), abbrev = "MonitorEntityState", name = "Monitor Entity State", flags = None)

    command = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'MES'}, "Command. Enumerated (Local).")
    '''Command. Enumerated (Local). Type: uint8_t'''
    entities = _base.mutable_attr({'name': 'Entity Names', 'type': 'plaintext'}, "Comma separated list of entity names.")
    '''Comma separated list of entity names. Type: plaintext'''

    def __init__(self, command = None, entities = None):
        '''Class constructor
        
        Comma separated list of entity names.

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            entities : plaintext, unit: NOT FOUND'''
        self._command = command
        self._entities = entities


class EntityMonitoringState(_base.base_message):
    '''Time of last error (Epoch time).

       This message class contains the following fields and their respective types:
    mcount : uint8_t, unit: NOT FOUND

            mnames : plaintext, unit: NOT FOUND

            ecount : uint8_t, unit: NOT FOUND

            enames : plaintext, unit: NOT FOUND

            ccount : uint8_t, unit: NOT FOUND

            cnames : plaintext, unit: NOT FOUND

            last_error : plaintext, unit: NOT FOUND

            last_error_time : fp64_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_mcount', '_mnames', '_ecount', '_enames', '_ccount', '_cnames', '_last_error', '_last_error_time']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 503, stable = None, description = None, fields = ('mcount', 'mnames', 'ecount', 'enames', 'ccount', 'cnames', 'last_error', 'last_error_time',), abbrev = "EntityMonitoringState", name = "Entity Monitoring State", flags = None)

    mcount = _base.mutable_attr({'name': 'Entities monitored - Count', 'type': 'uint8_t'}, "Number of entitities being monitored.")
    '''Number of entitities being monitored. Type: uint8_t'''
    mnames = _base.mutable_attr({'name': 'Entities monitored - Names', 'type': 'plaintext'}, "Comma separated list of all entity names being monitored.")
    '''Comma separated list of all entity names being monitored. Type: plaintext'''
    ecount = _base.mutable_attr({'name': 'Entities with errors - Count', 'type': 'uint8_t'}, "Number of entitities with non-critical errors.")
    '''Number of entitities with non-critical errors. Type: uint8_t'''
    enames = _base.mutable_attr({'name': 'Entities with errors - Names', 'type': 'plaintext'}, "Comma separated list of all entity names with non-critical errors.")
    '''Comma separated list of all entity names with non-critical errors. Type: plaintext'''
    ccount = _base.mutable_attr({'name': 'Entities with critical errors - Count', 'type': 'uint8_t'}, "Number of entitities with critical errors.")
    '''Number of entitities with critical errors. Type: uint8_t'''
    cnames = _base.mutable_attr({'name': 'Entities with critical errors - Names', 'type': 'plaintext'}, "Comma separated list of all entity names with critical errors.")
    '''Comma separated list of all entity names with critical errors. Type: plaintext'''
    last_error = _base.mutable_attr({'name': 'Last Error -- Description', 'type': 'plaintext'}, "Description of last error.")
    '''Description of last error. Type: plaintext'''
    last_error_time = _base.mutable_attr({'name': 'Last Error -- Time', 'type': 'fp64_t', 'unit': 's'}, "Time of last error (Epoch time).")
    '''Time of last error (Epoch time). Type: fp64_t'''

    def __init__(self, mcount = None, mnames = None, ecount = None, enames = None, ccount = None, cnames = None, last_error = None, last_error_time = None):
        '''Class constructor
        
        Time of last error (Epoch time).

       This message class contains the following fields and their respective types:
    mcount : uint8_t, unit: NOT FOUND

            mnames : plaintext, unit: NOT FOUND

            ecount : uint8_t, unit: NOT FOUND

            enames : plaintext, unit: NOT FOUND

            ccount : uint8_t, unit: NOT FOUND

            cnames : plaintext, unit: NOT FOUND

            last_error : plaintext, unit: NOT FOUND

            last_error_time : fp64_t, unit: s'''
        self._mcount = mcount
        self._mnames = mnames
        self._ecount = ecount
        self._enames = enames
        self._ccount = ccount
        self._cnames = cnames
        self._last_error = last_error
        self._last_error_time = last_error_time


class OperationalLimits(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    mask : uint8_t, unit: Bitfield (Global)

            max_depth : fp32_t, unit: m

            min_altitude : fp32_t, unit: m

            max_altitude : fp32_t, unit: m

            min_speed : fp32_t, unit: m/s

            max_speed : fp32_t, unit: m/s

            max_vrate : fp32_t, unit: m/s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            orientation : fp32_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_mask', '_max_depth', '_min_altitude', '_max_altitude', '_min_speed', '_max_speed', '_max_vrate', '_lat', '_lon', '_orientation', '_width', '_length']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Vehicle Supervision", id = 504, stable = None, description = "Definition of operational limits.", fields = ('mask', 'max_depth', 'min_altitude', 'max_altitude', 'min_speed', 'max_speed', 'max_vrate', 'lat', 'lon', 'orientation', 'width', 'length',), abbrev = "OperationalLimits", name = "Operational Limits", flags = None)

    mask = _base.mutable_attr({'name': 'Field Indicator Mask', 'type': 'uint8_t', 'unit': 'Bitfield', 'bitfield-def': 'OpLimitsMask'}, "No description available Bitfield (Global).")
    '''No description available Bitfield (Global). Type: uint8_t'''
    max_depth = _base.mutable_attr({'name': 'Maximum Depth', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    min_altitude = _base.mutable_attr({'name': 'Minimum Altitude', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    max_altitude = _base.mutable_attr({'name': 'Maximum Altitude', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    min_speed = _base.mutable_attr({'name': 'Minimum Speed', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    max_speed = _base.mutable_attr({'name': 'Maximum Speed', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    max_vrate = _base.mutable_attr({'name': 'Maximum Vertical Rate', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "No description available")
    '''No description available Type: fp32_t'''
    lat = _base.mutable_attr({'name': 'Area -- WGS-84 Latitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Area -- WGS-84 Longitude', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "No description available")
    '''No description available Type: fp64_t'''
    orientation = _base.mutable_attr({'name': 'Area -- Orientation', 'type': 'fp32_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp32_t'''
    width = _base.mutable_attr({'name': 'Area -- Width', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    length = _base.mutable_attr({'name': 'Area -- Length', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, mask = None, max_depth = None, min_altitude = None, max_altitude = None, min_speed = None, max_speed = None, max_vrate = None, lat = None, lon = None, orientation = None, width = None, length = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    mask : uint8_t, unit: Bitfield (Global)

            max_depth : fp32_t, unit: m

            min_altitude : fp32_t, unit: m

            max_altitude : fp32_t, unit: m

            min_speed : fp32_t, unit: m/s

            max_speed : fp32_t, unit: m/s

            max_vrate : fp32_t, unit: m/s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            orientation : fp32_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m'''
        self._mask = mask
        self._max_depth = max_depth
        self._min_altitude = min_altitude
        self._max_altitude = max_altitude
        self._min_speed = min_speed
        self._max_speed = max_speed
        self._max_vrate = max_vrate
        self._lat = lat
        self._lon = lon
        self._orientation = orientation
        self._width = width
        self._length = length


class GetOperationalLimits(_base.base_message):
    '''Command to obtain the operational limits in use by the vehicle.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Vehicle Supervision", id = 505, stable = None, description = "Command to obtain the operational limits in use by the vehicle.", fields = [], abbrev = "GetOperationalLimits", name = "Get Operational Limits", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Command to obtain the operational limits in use by the vehicle.

       This message class contains the following fields and their respective types:
'''


class Calibration(_base.base_message):
    '''Duration of calibration.

       This message class contains the following fields and their respective types:
    duration : uint16_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_duration']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 506, stable = None, description = "Initiate overall calibration of a vehicle.", fields = ('duration',), abbrev = "Calibration", name = "Calibration", flags = None)

    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration of calibration.")
    '''Duration of calibration. Type: uint16_t'''

    def __init__(self, duration = None):
        '''Class constructor
        
        Duration of calibration.

       This message class contains the following fields and their respective types:
    duration : uint16_t, unit: s'''
        self._duration = duration


class ControlLoops(_base.base_message):
    '''Unsigned integer reference for the scope of the control loop message. Scope reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level controllers must inherit the same scope reference sent by maneuver. This same scope reference must be sent down to lower control layers.

       This message class contains the following fields and their respective types:
    enable : uint8_t, unit: Enumerated (Local)

            mask : uint32_t, unit: Bitfield (Global)

            scope_ref : uint32_t, unit: NOT FOUND'''

    class ENABLE(_enum.IntEnum):
        '''Full name: Enable
        Prefix: CL'''
    
        DISABLE = 0
        '''Name: Disable'''
    
        ENABLE = 1
        '''Name: Enable'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_enable', '_mask', '_scope_ref']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 507, stable = None, description = "Enable or disable control loops.", fields = ('enable', 'mask', 'scope_ref',), abbrev = "ControlLoops", name = "Control Loops", flags = None)

    enable = _base.mutable_attr({'name': 'Enable', 'type': 'uint8_t', 'prefix': 'CL', 'unit': 'Enumerated'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    mask = _base.mutable_attr({'name': 'Control Loop Mask', 'type': 'uint32_t', 'unit': 'Bitfield', 'bitfield-def': 'CLoopsMask'}, "Control loop mask. Bitfield (Global).")
    '''Control loop mask. Bitfield (Global). Type: uint32_t'''
    scope_ref = _base.mutable_attr({'name': 'Scope Time Reference', 'type': 'uint32_t'}, "Unsigned integer reference for the scope of the control loop message. Scope reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level controllers must inherit the same scope reference sent by maneuver. This same scope reference must be sent down to lower control layers.")
    '''Unsigned integer reference for the scope of the control loop message. Scope reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level controllers must inherit the same scope reference sent by maneuver. This same scope reference must be sent down to lower control layers. Type: uint32_t'''

    def __init__(self, enable = None, mask = None, scope_ref = None):
        '''Class constructor
        
        Unsigned integer reference for the scope of the control loop message. Scope reference should only be set by a maneuver. Should be set to an always increasing reference at the time of dispatching this message. Lower level controllers must inherit the same scope reference sent by maneuver. This same scope reference must be sent down to lower control layers.

       This message class contains the following fields and their respective types:
    enable : uint8_t, unit: Enumerated (Local)

            mask : uint32_t, unit: Bitfield (Global)

            scope_ref : uint32_t, unit: NOT FOUND'''
        self._enable = enable
        self._mask = mask
        self._scope_ref = scope_ref


class VehicleMedium(_base.base_message):
    '''Current medium. Enumerated (Local).

       This message class contains the following fields and their respective types:
    medium : uint8_t, unit: Enumerated (Local)'''

    class MEDIUM(_enum.IntEnum):
        '''Full name: Medium
        Prefix: VM'''
    
        GROUND = 0
        '''Name: Ground'''
    
        AIR = 1
        '''Name: Air'''
    
        WATER = 2
        '''Name: Water'''
    
        UNDERWATER = 3
        '''Name: Underwater'''
    
        UNKNOWN = 4
        '''Name: Unknown'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_medium']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 508, stable = None, description = "Detect current vehicle medium.", fields = ('medium',), abbrev = "VehicleMedium", name = "Vehicle Medium", flags = None)

    medium = _base.mutable_attr({'name': 'Medium', 'type': 'uint8_t', 'prefix': 'VM', 'unit': 'Enumerated'}, "Current medium. Enumerated (Local).")
    '''Current medium. Enumerated (Local). Type: uint8_t'''

    def __init__(self, medium = None):
        '''Class constructor
        
        Current medium. Enumerated (Local).

       This message class contains the following fields and their respective types:
    medium : uint8_t, unit: Enumerated (Local)'''
        self._medium = medium


class Collision(_base.base_message):
    '''Collision flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m/s/s

            type : uint8_t, unit: Bitfield (Local)'''

    class TYPE(_enum.IntFlag):
        '''Full name: Type
        Prefix: CD'''
    
        EMPTY = 0
        '''No active flags'''
    
        X = 1
        '''Name: X-axis'''
    
        Y = 2
        '''Name: Y-axis'''
    
        Z = 4
        '''Name: Z-axis'''
    
        IMPACT = 8
        '''Name: Impact'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_value', '_type']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 509, stable = None, description = "Detected collision.", fields = ('value', 'type',), abbrev = "Collision", name = "Collision", flags = None)

    value = _base.mutable_attr({'name': 'Collision value', 'type': 'fp32_t', 'unit': 'm/s/s'}, "Estimated collision acceleration value.")
    '''Estimated collision acceleration value. Type: fp32_t'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'CD'}, "Collision flags. Bitfield (Local).")
    '''Collision flags. Bitfield (Local). Type: uint8_t'''

    def __init__(self, value = None, type = None):
        '''Class constructor
        
        Collision flags. Bitfield (Local).

       This message class contains the following fields and their respective types:
    value : fp32_t, unit: m/s/s

            type : uint8_t, unit: Bitfield (Local)'''
        self._value = value
        self._type = type


class FormState(_base.base_message):
    '''Convergence monitoring flag. Enumerated (Local).

       This message class contains the following fields and their respective types:
    PosSimErr : fp32_t, unit: m

            Converg : fp32_t, unit: m

            Turbulence : fp32_t, unit: m/s/s

            PosSimMon : uint8_t, unit: Enumerated (Local)

            CommMon : uint8_t, unit: Enumerated (Local)

            ConvergMon : uint8_t, unit: Enumerated (Local)'''

    class POSSIMMON(_enum.IntEnum):
        '''Full name: Position Mismatch Monitor
        Prefix: POS'''
    
        OK = 0
        '''Name: Ok'''
    
        WRN = 1
        '''Name: Warning threshold'''
    
        LIM = 2
        '''Name: Limit threshold'''
    
    
    class COMMMON(_enum.IntEnum):
        '''Full name: Communications Monitor
        Prefix: COMMS'''
    
        OK = 0
        '''Name: Ok'''
    
        TIMEOUT = 1
        '''Name: Timeout'''
    
    
    class CONVERGMON(_enum.IntEnum):
        '''Full name: Convergence
        Prefix: CONV'''
    
        OK = 0
        '''Name: Ok'''
    
        TIMEOUT = 1
        '''Name: Timeout'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_PosSimErr', '_Converg', '_Turbulence', '_PosSimMon', '_CommMon', '_ConvergMon']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 510, stable = None, description = "Monitoring variables to assert the formation tracking state, i.e., the mismatch between the real and the simulated aircraft position, the convergence state, etc.", fields = ('PosSimErr', 'Converg', 'Turbulence', 'PosSimMon', 'CommMon', 'ConvergMon',), abbrev = "FormState", name = "Formation Tracking State", flags = None)

    PosSimErr = _base.mutable_attr({'name': 'Position Mismatch', 'type': 'fp32_t', 'unit': 'm'}, "Mismatch between the real and the simulated aircraft position.")
    '''Mismatch between the real and the simulated aircraft position. Type: fp32_t'''
    Converg = _base.mutable_attr({'name': 'Convergence', 'type': 'fp32_t', 'unit': 'm'}, "Convergence evalution variable. Value indicates the position error to which the system is converging, tacking into account the aircraft current position error and velocity.")
    '''Convergence evalution variable. Value indicates the position error to which the system is converging, tacking into account the aircraft current position error and velocity. Type: fp32_t'''
    Turbulence = _base.mutable_attr({'name': 'Stream Turbulence', 'type': 'fp32_t', 'unit': 'm/s/s'}, "Evaluation of the stream turbulence level, through the stream acceleration.")
    '''Evaluation of the stream turbulence level, through the stream acceleration. Type: fp32_t'''
    PosSimMon = _base.mutable_attr({'name': 'Position Mismatch Monitor', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'POS'}, "Position mismatch monitoring flag. Enumerated (Local).")
    '''Position mismatch monitoring flag. Enumerated (Local). Type: uint8_t'''
    CommMon = _base.mutable_attr({'name': 'Communications Monitor', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'COMMS'}, "Communications monitoring flag. Enumerated (Local).")
    '''Communications monitoring flag. Enumerated (Local). Type: uint8_t'''
    ConvergMon = _base.mutable_attr({'name': 'Convergence', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'CONV'}, "Convergence monitoring flag. Enumerated (Local).")
    '''Convergence monitoring flag. Enumerated (Local). Type: uint8_t'''

    def __init__(self, PosSimErr = None, Converg = None, Turbulence = None, PosSimMon = None, CommMon = None, ConvergMon = None):
        '''Class constructor
        
        Convergence monitoring flag. Enumerated (Local).

       This message class contains the following fields and their respective types:
    PosSimErr : fp32_t, unit: m

            Converg : fp32_t, unit: m

            Turbulence : fp32_t, unit: m/s/s

            PosSimMon : uint8_t, unit: Enumerated (Local)

            CommMon : uint8_t, unit: Enumerated (Local)

            ConvergMon : uint8_t, unit: Enumerated (Local)'''
        self._PosSimErr = PosSimErr
        self._Converg = Converg
        self._Turbulence = Turbulence
        self._PosSimMon = PosSimMon
        self._CommMon = CommMon
        self._ConvergMon = ConvergMon


class AutopilotMode(_base.base_message):
    '''Current mode name.

       This message class contains the following fields and their respective types:
    autonomy : uint8_t, unit: Enumerated (Local)

            mode : plaintext, unit: NOT FOUND'''

    class AUTONOMY(_enum.IntEnum):
        '''Full name: Autonomy Level
        Prefix: AL'''
    
        MANUAL = 0
        '''Name: Manual'''
    
        ASSISTED = 1
        '''Name: Assisted'''
    
        AUTO = 2
        '''Name: Auto'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_autonomy', '_mode']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 511, stable = None, description = "Reports autopilot mode.", fields = ('autonomy', 'mode',), abbrev = "AutopilotMode", name = "Autopilot Mode", flags = None)

    autonomy = _base.mutable_attr({'name': 'Autonomy Level', 'type': 'uint8_t', 'prefix': 'AL', 'unit': 'Enumerated'}, "Current mode autonomy level. Enumerated (Local).")
    '''Current mode autonomy level. Enumerated (Local). Type: uint8_t'''
    mode = _base.mutable_attr({'name': 'Mode', 'type': 'plaintext'}, "Current mode name.")
    '''Current mode name. Type: plaintext'''

    def __init__(self, autonomy = None, mode = None):
        '''Class constructor
        
        Current mode name.

       This message class contains the following fields and their respective types:
    autonomy : uint8_t, unit: Enumerated (Local)

            mode : plaintext, unit: NOT FOUND'''
        self._autonomy = autonomy
        self._mode = mode


class FormationState(_base.base_message):
    '''Convergence monitoring flag. Enumerated (Local).

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            PosSimErr : fp32_t, unit: m

            Converg : fp32_t, unit: m

            Turbulence : fp32_t, unit: m/s/s

            PosSimMon : uint8_t, unit: Enumerated (Local)

            CommMon : uint8_t, unit: Enumerated (Local)

            ConvergMon : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: FC'''
    
        REQUEST = 0
        '''Name: Request'''
    
        REPORT = 1
        '''Name: Report'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        START = 0
        '''Name: Start'''
    
        STOP = 1
        '''Name: Stop'''
    
    
    class POSSIMMON(_enum.IntEnum):
        '''Full name: Position Mismatch Monitor
        Prefix: POS'''
    
        OK = 0
        '''Name: Ok'''
    
        WRN = 1
        '''Name: Warning threshold'''
    
        LIM = 2
        '''Name: Limit threshold'''
    
    
    class COMMMON(_enum.IntEnum):
        '''Full name: Communications Monitor
        Prefix: COMMS'''
    
        OK = 0
        '''Name: Ok'''
    
        TIMEOUT = 1
        '''Name: Timeout'''
    
    
    class CONVERGMON(_enum.IntEnum):
        '''Full name: Convergence
        Prefix: CONV'''
    
        OK = 0
        '''Name: Ok'''
    
        TIMEOUT = 1
        '''Name: Timeout'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_op', '_PosSimErr', '_Converg', '_Turbulence', '_PosSimMon', '_CommMon', '_ConvergMon']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 512, stable = None, description = "Monitoring variables to assert the formation tracking state, i.e., the mismatch between the real and the simulated aircraft position, the convergence state, etc.", fields = ('type', 'op', 'PosSimErr', 'Converg', 'Turbulence', 'PosSimMon', 'CommMon', 'ConvergMon',), abbrev = "FormationState", name = "Formation Tracking State", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'FC'}, "Indicates if the message is a request, or a reply to a previous request. Enumerated (Local).")
    '''Indicates if the message is a request, or a reply to a previous request. Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    PosSimErr = _base.mutable_attr({'name': 'Position Mismatch', 'type': 'fp32_t', 'unit': 'm'}, "Mismatch between the real and the simulated aircraft position.")
    '''Mismatch between the real and the simulated aircraft position. Type: fp32_t'''
    Converg = _base.mutable_attr({'name': 'Convergence', 'type': 'fp32_t', 'unit': 'm'}, "Convergence evalution variable. Value indicates the position error to which the system is converging, tacking into account the aircraft current position error and velocity.")
    '''Convergence evalution variable. Value indicates the position error to which the system is converging, tacking into account the aircraft current position error and velocity. Type: fp32_t'''
    Turbulence = _base.mutable_attr({'name': 'Stream Turbulence', 'type': 'fp32_t', 'unit': 'm/s/s'}, "Evaluation of the stream turbulence level, through the stream acceleration.")
    '''Evaluation of the stream turbulence level, through the stream acceleration. Type: fp32_t'''
    PosSimMon = _base.mutable_attr({'name': 'Position Mismatch Monitor', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'POS'}, "Position mismatch monitoring flag. Enumerated (Local).")
    '''Position mismatch monitoring flag. Enumerated (Local). Type: uint8_t'''
    CommMon = _base.mutable_attr({'name': 'Communications Monitor', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'COMMS'}, "Communications monitoring flag. Enumerated (Local).")
    '''Communications monitoring flag. Enumerated (Local). Type: uint8_t'''
    ConvergMon = _base.mutable_attr({'name': 'Convergence', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'CONV'}, "Convergence monitoring flag. Enumerated (Local).")
    '''Convergence monitoring flag. Enumerated (Local). Type: uint8_t'''

    def __init__(self, type = None, op = None, PosSimErr = None, Converg = None, Turbulence = None, PosSimMon = None, CommMon = None, ConvergMon = None):
        '''Class constructor
        
        Convergence monitoring flag. Enumerated (Local).

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            PosSimErr : fp32_t, unit: m

            Converg : fp32_t, unit: m

            Turbulence : fp32_t, unit: m/s/s

            PosSimMon : uint8_t, unit: Enumerated (Local)

            CommMon : uint8_t, unit: Enumerated (Local)

            ConvergMon : uint8_t, unit: Enumerated (Local)'''
        self._type = type
        self._op = op
        self._PosSimErr = PosSimErr
        self._Converg = Converg
        self._Turbulence = Turbulence
        self._PosSimMon = PosSimMon
        self._CommMon = CommMon
        self._ConvergMon = ConvergMon


class ReportControl(_base.base_message):
    '''Destination Address to be filled where applicable. It should be interpreted differently depending on communication interface.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            comm_interface : uint8_t, unit: Bitfield (Local)

            period : uint16_t, unit: s

            sys_dst : plaintext, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        REQUEST_START = 0
        '''Name: Request Start of Reports'''
    
        STARTED = 1
        '''Name: Report Started'''
    
        REQUEST_STOP = 2
        '''Name: Request Stop of Reports'''
    
        STOPPED = 3
        '''Name: Report Stopped'''
    
        REQUEST_REPORT = 4
        '''Name: Request Single Reports'''
    
        REPORT_SENT = 5
        '''Name: Single Report Sent'''
    
    
    class COMM_INTERFACE(_enum.IntFlag):
        '''Full name: Communication Interface
        Prefix: CI'''
    
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
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_comm_interface', '_period', '_sys_dst']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Vehicle Supervision", id = 513, stable = None, description = "This message is sent to trigger reports to a destination system.", fields = ('op', 'comm_interface', 'period', 'sys_dst',), abbrev = "ReportControl", name = "Report Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    comm_interface = _base.mutable_attr({'name': 'Communication Interface', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'CI'}, "Communication interface to be used for reports. Bitfield (Local).")
    '''Communication interface to be used for reports. Bitfield (Local). Type: uint8_t'''
    period = _base.mutable_attr({'name': 'Period', 'type': 'uint16_t', 'unit': 's'}, "Desired periodicity for scheduled reports.")
    '''Desired periodicity for scheduled reports. Type: uint16_t'''
    sys_dst = _base.mutable_attr({'name': 'Destination System', 'type': 'plaintext'}, "Destination Address to be filled where applicable. It should be interpreted differently depending on communication interface.")
    '''Destination Address to be filled where applicable. It should be interpreted differently depending on communication interface. Type: plaintext'''

    def __init__(self, op = None, comm_interface = None, period = None, sys_dst = None):
        '''Class constructor
        
        Destination Address to be filled where applicable. It should be interpreted differently depending on communication interface.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            comm_interface : uint8_t, unit: Bitfield (Local)

            period : uint16_t, unit: s

            sys_dst : plaintext, unit: NOT FOUND'''
        self._op = op
        self._comm_interface = comm_interface
        self._period = period
        self._sys_dst = sys_dst


class StateReport(_base.base_message):
    '''Checksum of the plan being executed.

       This message class contains the following fields and their respective types:
    stime : uint32_t, unit: s

            latitude : fp32_t, unit: °

            longitude : fp32_t, unit: °

            altitude : uint16_t, unit: dm

            depth : uint16_t, unit: dm

            heading : uint16_t, unit: NOT FOUND

            speed : int16_t, unit: cm/s

            fuel : int8_t, unit: %

            exec_state : int8_t, unit: %

            plan_checksum : uint16_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_stime', '_latitude', '_longitude', '_altitude', '_depth', '_heading', '_speed', '_fuel', '_exec_state', '_plan_checksum']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vehicle Supervision", id = 514, stable = None, description = "Concise representation of entire system state.", fields = ('stime', 'latitude', 'longitude', 'altitude', 'depth', 'heading', 'speed', 'fuel', 'exec_state', 'plan_checksum',), abbrev = "StateReport", name = "State Report", flags = None)

    stime = _base.mutable_attr({'name': 'Time Stamp', 'type': 'uint32_t', 'unit': 's'}, "Time, in seconds, since January 1st 1970.")
    '''Time, in seconds, since January 1st 1970. Type: uint32_t'''
    latitude = _base.mutable_attr({'name': 'Latitude', 'type': 'fp32_t', 'unit': '°'}, "Latitude of the system, in degrees.")
    '''Latitude of the system, in degrees. Type: fp32_t'''
    longitude = _base.mutable_attr({'name': 'Longitude', 'type': 'fp32_t', 'unit': '°'}, "Longitude of the system, in degrees.")
    '''Longitude of the system, in degrees. Type: fp32_t'''
    altitude = _base.mutable_attr({'name': 'Altitude', 'type': 'uint16_t', 'unit': 'dm'}, "Altitude of the system, in decimeters. * *0xFFFF* used for unknown / not applicable value.")
    '''Altitude of the system, in decimeters. * *0xFFFF* used for unknown / not applicable value. Type: uint16_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'uint16_t', 'unit': 'dm'}, "Depth of the system, in decimeters. * *0xFFFF* used for unknown / not applicable value.")
    '''Depth of the system, in decimeters. * *0xFFFF* used for unknown / not applicable value. Type: uint16_t'''
    heading = _base.mutable_attr({'name': 'Heading', 'type': 'uint16_t'}, "Calculated as `(rads * (0xFFFF / (2 * PI))`")
    '''Calculated as `(rads * (0xFFFF / (2 * PI))` Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'int16_t', 'unit': 'cm/s'}, "Speed of the system in centimeters per second.")
    '''Speed of the system in centimeters per second. Type: int16_t'''
    fuel = _base.mutable_attr({'name': 'Fuel', 'type': 'int8_t', 'unit': '%'}, "System fuel gauge. * *-1* means unknown fuel level.")
    '''System fuel gauge. * *-1* means unknown fuel level. Type: int8_t'''
    exec_state = _base.mutable_attr({'name': 'Execution State', 'type': 'int8_t', 'unit': '%'}, "Progress of execution or idle state. * *-1* means Service mode * *-2* means Boot mode * *-3* means Calibration mode * *-4* means Error mode")
    '''Progress of execution or idle state. * *-1* means Service mode * *-2* means Boot mode * *-3* means Calibration mode * *-4* means Error mode Type: int8_t'''
    plan_checksum = _base.mutable_attr({'name': 'Plan Checksum', 'type': 'uint16_t'}, "Checksum of the plan being executed.")
    '''Checksum of the plan being executed. Type: uint16_t'''

    def __init__(self, stime = None, latitude = None, longitude = None, altitude = None, depth = None, heading = None, speed = None, fuel = None, exec_state = None, plan_checksum = None):
        '''Class constructor
        
        Checksum of the plan being executed.

       This message class contains the following fields and their respective types:
    stime : uint32_t, unit: s

            latitude : fp32_t, unit: °

            longitude : fp32_t, unit: °

            altitude : uint16_t, unit: dm

            depth : uint16_t, unit: dm

            heading : uint16_t, unit: NOT FOUND

            speed : int16_t, unit: cm/s

            fuel : int8_t, unit: %

            exec_state : int8_t, unit: %

            plan_checksum : uint16_t, unit: NOT FOUND'''
        self._stime = stime
        self._latitude = latitude
        self._longitude = longitude
        self._altitude = altitude
        self._depth = depth
        self._heading = heading
        self._speed = speed
        self._fuel = fuel
        self._exec_state = exec_state
        self._plan_checksum = plan_checksum


class VtolState(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: VTOL_STATE'''
    
        UNDEFINED = 0
        '''Name: Undefined'''
    
        TRANSITION_TO_FW = 1
        '''Name: Transition to Fixed-Wing'''
    
        TRANSITION_TO_MC = 2
        '''Name: Transition to MultiCopter'''
    
        MC = 3
        '''Name: MutiCopter'''
    
        FW = 4
        '''Name: Fixed-Wing'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vehicle Supervision", id = 519, stable = None, description = "Reports VTOL current state.", fields = ('state',), abbrev = "VtolState", name = "VTOL State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'VTOL_STATE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, state = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''
        self._state = state


class ArmingState(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: MOTORS'''
    
        ARMED = 0
        '''Name: Armed'''
    
        DISARMED = 1
        '''Name: Disarmed'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vehicle Supervision", id = 520, stable = None, description = "Reports if motors are currently armed or disarmed.", fields = ('state',), abbrev = "ArmingState", name = "Arming State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'MOTORS'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, state = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)'''
        self._state = state


class AssetReport(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            report_time : fp64_t, unit: s

            medium : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m

            sog : fp32_t, unit: m/s

            cog : fp32_t, unit: rad

            msgs : message-list, unit: NOT FOUND'''

    class MEDIUM(_enum.IntEnum):
        '''Full name: Medium
        Prefix: RM'''
    
        WIFI = 1
        '''Name: WiFi'''
    
        SATELLITE = 2
        '''Name: Satellite'''
    
        ACOUSTIC = 3
        '''Name: Acoustic'''
    
        SMS = 4
        '''Name: SMS'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_report_time', '_medium', '_lat', '_lon', '_depth', '_alt', '_sog', '_cog', '_msgs']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Vehicle Supervision", id = 525, stable = None, description = "This message is represents an Asset position / status.", fields = ('name', 'report_time', 'medium', 'lat', 'lon', 'depth', 'alt', 'sog', 'cog', 'msgs',), abbrev = "AssetReport", name = "Asset Report ", flags = None)

    name = _base.mutable_attr({'name': 'Asset Name', 'type': 'plaintext'}, "The human readable name of the asset.")
    '''The human readable name of the asset. Type: plaintext'''
    report_time = _base.mutable_attr({'name': 'Report Timestamp', 'type': 'fp64_t', 'unit': 's'}, "Time in seconds since epoch, for the generation instant.")
    '''Time in seconds since epoch, for the generation instant. Type: fp64_t'''
    medium = _base.mutable_attr({'name': 'Medium', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'RM'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp64_t'''
    depth = _base.mutable_attr({'name': 'Depth', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    alt = _base.mutable_attr({'name': 'Altitude', 'type': 'fp32_t', 'unit': 'm'}, "No description available")
    '''No description available Type: fp32_t'''
    sog = _base.mutable_attr({'name': 'Speed Over Ground', 'type': 'fp32_t', 'unit': 'm/s'}, "No description available")
    '''No description available Type: fp32_t'''
    cog = _base.mutable_attr({'name': 'Course Over Ground', 'type': 'fp32_t', 'unit': 'rad'}, "No description available")
    '''No description available Type: fp32_t'''
    msgs = _base.mutable_attr({'name': 'Additional Info', 'type': 'message-list'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, name = None, report_time = None, medium = None, lat = None, lon = None, depth = None, alt = None, sog = None, cog = None, msgs = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            report_time : fp64_t, unit: s

            medium : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            depth : fp32_t, unit: m

            alt : fp32_t, unit: m

            sog : fp32_t, unit: m/s

            cog : fp32_t, unit: rad

            msgs : message-list, unit: NOT FOUND'''
        self._name = name
        self._report_time = report_time
        self._medium = medium
        self._lat = lat
        self._lon = lon
        self._depth = depth
        self._alt = alt
        self._sog = sog
        self._cog = cog
        self._msgs = msgs


class ParametersXml(_base.base_message):
    '''The parameters XML file compressed using the GNU zip (gzip) format.

       This message class contains the following fields and their respective types:
    locale : plaintext, unit: NOT FOUND

            config : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_locale', '_config']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vehicle Supervision", id = 893, stable = "false", description = "Message containing the parameters XML of the source system.", fields = ('locale', 'config',), abbrev = "ParametersXml", name = "Parameters XML", flags = None)

    locale = _base.mutable_attr({'name': 'Locale', 'type': 'plaintext'}, "The locale used to produce this parameters XML.")
    '''The locale used to produce this parameters XML. Type: plaintext'''
    config = _base.mutable_attr({'name': 'Configuration Data', 'type': 'rawdata'}, "The parameters XML file compressed using the GNU zip (gzip) format.")
    '''The parameters XML file compressed using the GNU zip (gzip) format. Type: rawdata'''

    def __init__(self, locale = None, config = None):
        '''Class constructor
        
        The parameters XML file compressed using the GNU zip (gzip) format.

       This message class contains the following fields and their respective types:
    locale : plaintext, unit: NOT FOUND

            config : rawdata, unit: NOT FOUND'''
        self._locale = locale
        self._config = config


class GetParametersXml(_base.base_message):
    '''Request the destination system to send its parameters XML file via a :ref:`ParametersXml` message.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vehicle Supervision", id = 894, stable = "false", description = "Request the destination system to send its parameters XML file via a :ref:`ParametersXml` message.", fields = [], abbrev = "GetParametersXml", name = "Get Parameters XML", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Request the destination system to send its parameters XML file via a :ref:`ParametersXml` message.

       This message class contains the following fields and their respective types:
'''


class Tachograph(_base.base_message):
    '''The maximum recorded depth value.

       This message class contains the following fields and their respective types:
    timestamp_last_service : fp64_t, unit: s

            time_next_service : fp32_t, unit: s

            time_motor_next_service : fp32_t, unit: s

            time_idle_ground : fp32_t, unit: s

            time_idle_air : fp32_t, unit: s

            time_idle_water : fp32_t, unit: s

            time_idle_underwater : fp32_t, unit: s

            time_idle_unknown : fp32_t, unit: s

            time_motor_ground : fp32_t, unit: s

            time_motor_air : fp32_t, unit: s

            time_motor_water : fp32_t, unit: s

            time_motor_underwater : fp32_t, unit: s

            time_motor_unknown : fp32_t, unit: s

            rpm_min : int16_t, unit: rpm

            rpm_max : int16_t, unit: rpm

            depth_max : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timestamp_last_service', '_time_next_service', '_time_motor_next_service', '_time_idle_ground', '_time_idle_air', '_time_idle_water', '_time_idle_underwater', '_time_idle_unknown', '_time_motor_ground', '_time_motor_air', '_time_motor_water', '_time_motor_underwater', '_time_motor_unknown', '_rpm_min', '_rpm_max', '_depth_max']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 905, stable = None, description = "This messages is used to record system activity parameters. These parameters are mainly used for used for maintenance purposes.", fields = ('timestamp_last_service', 'time_next_service', 'time_motor_next_service', 'time_idle_ground', 'time_idle_air', 'time_idle_water', 'time_idle_underwater', 'time_idle_unknown', 'time_motor_ground', 'time_motor_air', 'time_motor_water', 'time_motor_underwater', 'time_motor_unknown', 'rpm_min', 'rpm_max', 'depth_max',), abbrev = "Tachograph", name = "Tachograph", flags = "periodic")

    timestamp_last_service = _base.mutable_attr({'name': 'Last Service Timestamp', 'type': 'fp64_t', 'unit': 's'}, "The time when the last service was performed. The number of seconds is represented in Universal Coordinated Time (UCT) in seconds since Jan 1, 1970.")
    '''The time when the last service was performed. The number of seconds is represented in Universal Coordinated Time (UCT) in seconds since Jan 1, 1970. Type: fp64_t'''
    time_next_service = _base.mutable_attr({'name': 'Time - Next Service', 'type': 'fp32_t', 'unit': 's'}, "Amount of time until the next recommended service.")
    '''Amount of time until the next recommended service. Type: fp32_t'''
    time_motor_next_service = _base.mutable_attr({'name': 'Time Motor - Next Service', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the motor can run until the next recommended service.")
    '''Amount of time the motor can run until the next recommended service. Type: fp32_t'''
    time_idle_ground = _base.mutable_attr({'name': 'Time Idle - Ground', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent idle on the ground.")
    '''Amount of time the system spent idle on the ground. Type: fp32_t'''
    time_idle_air = _base.mutable_attr({'name': 'Time Idle - Air', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent idle in the air.")
    '''Amount of time the system spent idle in the air. Type: fp32_t'''
    time_idle_water = _base.mutable_attr({'name': 'Time Idle - Water', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent idle on the water (not submerged).")
    '''Amount of time the system spent idle on the water (not submerged). Type: fp32_t'''
    time_idle_underwater = _base.mutable_attr({'name': 'Time Idle - Underwater', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent idle underwater.")
    '''Amount of time the system spent idle underwater. Type: fp32_t'''
    time_idle_unknown = _base.mutable_attr({'name': 'Time Idle - Unknown', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent idle in an unknown medium.")
    '''Amount of time the system spent idle in an unknown medium. Type: fp32_t'''
    time_motor_ground = _base.mutable_attr({'name': 'Time Motor - Ground', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent on the ground with the motor running.")
    '''Amount of time the system spent on the ground with the motor running. Type: fp32_t'''
    time_motor_air = _base.mutable_attr({'name': 'Time Motor - Air', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent in the air with the motor running.")
    '''Amount of time the system spent in the air with the motor running. Type: fp32_t'''
    time_motor_water = _base.mutable_attr({'name': 'Time Motor - Water', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent on the water (not submerged) with the motor running.")
    '''Amount of time the system spent on the water (not submerged) with the motor running. Type: fp32_t'''
    time_motor_underwater = _base.mutable_attr({'name': 'Time Motor - Underwater', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent underwater with the motor running.")
    '''Amount of time the system spent underwater with the motor running. Type: fp32_t'''
    time_motor_unknown = _base.mutable_attr({'name': 'Time Motor - Unknown', 'type': 'fp32_t', 'unit': 's'}, "Amount of time the system spent in an unknown medium with the motor running.")
    '''Amount of time the system spent in an unknown medium with the motor running. Type: fp32_t'''
    rpm_min = _base.mutable_attr({'name': 'Recorded RPMs - Minimum', 'type': 'int16_t', 'unit': 'rpm'}, "The minimum recorded RPM value.")
    '''The minimum recorded RPM value. Type: int16_t'''
    rpm_max = _base.mutable_attr({'name': 'Recorded RPMs - Maximum', 'type': 'int16_t', 'unit': 'rpm'}, "The maximum recorded RPM value.")
    '''The maximum recorded RPM value. Type: int16_t'''
    depth_max = _base.mutable_attr({'name': 'Recorded Depth - Maximum', 'type': 'fp32_t', 'unit': 'm'}, "The maximum recorded depth value.")
    '''The maximum recorded depth value. Type: fp32_t'''

    def __init__(self, timestamp_last_service = None, time_next_service = None, time_motor_next_service = None, time_idle_ground = None, time_idle_air = None, time_idle_water = None, time_idle_underwater = None, time_idle_unknown = None, time_motor_ground = None, time_motor_air = None, time_motor_water = None, time_motor_underwater = None, time_motor_unknown = None, rpm_min = None, rpm_max = None, depth_max = None):
        '''Class constructor
        
        The maximum recorded depth value.

       This message class contains the following fields and their respective types:
    timestamp_last_service : fp64_t, unit: s

            time_next_service : fp32_t, unit: s

            time_motor_next_service : fp32_t, unit: s

            time_idle_ground : fp32_t, unit: s

            time_idle_air : fp32_t, unit: s

            time_idle_water : fp32_t, unit: s

            time_idle_underwater : fp32_t, unit: s

            time_idle_unknown : fp32_t, unit: s

            time_motor_ground : fp32_t, unit: s

            time_motor_air : fp32_t, unit: s

            time_motor_water : fp32_t, unit: s

            time_motor_underwater : fp32_t, unit: s

            time_motor_unknown : fp32_t, unit: s

            rpm_min : int16_t, unit: rpm

            rpm_max : int16_t, unit: rpm

            depth_max : fp32_t, unit: m'''
        self._timestamp_last_service = timestamp_last_service
        self._time_next_service = time_next_service
        self._time_motor_next_service = time_motor_next_service
        self._time_idle_ground = time_idle_ground
        self._time_idle_air = time_idle_air
        self._time_idle_water = time_idle_water
        self._time_idle_underwater = time_idle_underwater
        self._time_idle_unknown = time_idle_unknown
        self._time_motor_ground = time_motor_ground
        self._time_motor_air = time_motor_air
        self._time_motor_water = time_motor_water
        self._time_motor_underwater = time_motor_underwater
        self._time_motor_unknown = time_motor_unknown
        self._rpm_min = rpm_min
        self._rpm_max = rpm_max
        self._depth_max = depth_max


class ApmStatus(_base.base_message):
    '''Status text message.

       This message class contains the following fields and their respective types:
    severity : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''

    class SEVERITY(_enum.IntEnum):
        '''Full name: Severity
        Prefix: APM'''
    
        EMERGENCY = 0
        '''Name: Emergency'''
    
        ALERT = 1
        '''Name: Alert'''
    
        CRITICAL = 2
        '''Name: Critical'''
    
        ERROR = 3
        '''Name: Error'''
    
        WARNING = 4
        '''Name: Warning'''
    
        NOTICE = 5
        '''Name: Notice'''
    
        INFO = 6
        '''Name: Info'''
    
        DEBUG = 7
        '''Name: Debug'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_severity', '_text']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Vehicle Supervision", id = 906, stable = None, description = "StatusText message from ardupilot.", fields = ('severity', 'text',), abbrev = "ApmStatus", name = "APM Status", flags = None)

    severity = _base.mutable_attr({'name': 'Severity', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'APM'}, "Severity of status. Enumerated (Local).")
    '''Severity of status. Enumerated (Local). Type: uint8_t'''
    text = _base.mutable_attr({'name': 'Text', 'type': 'plaintext'}, "Status text message.")
    '''Status text message. Type: plaintext'''

    def __init__(self, severity = None, text = None):
        '''Class constructor
        
        Status text message.

       This message class contains the following fields and their respective types:
    severity : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''
        self._severity = severity
        self._text = text


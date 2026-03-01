'''
IMC Actuation messages.
'''

from .. import _base
import enum as _enum

class CameraZoom(_base.base_message):
    '''The zoom action to perform. Enumerated (Local).

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            zoom : uint8_t, unit: NOT FOUND

            action : uint8_t, unit: Enumerated (Local)'''

    class ACTION(_enum.IntEnum):
        '''Full name: Action
        Prefix: ACTION'''
    
        ZOOM_RESET = 0
        '''Name: Reset Zoom'''
    
        ZOOM_IN = 1
        '''Name: Zoom In'''
    
        ZOOM_OUT = 2
        '''Name: Zoom Out'''
    
        ZOOM_STOP = 3
        '''Name: Stop Zooming'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_zoom', '_action']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 300, stable = None, description = "Camera Zoom.", fields = ('id', 'zoom', 'action',), abbrev = "CameraZoom", name = "Camera Zoom", flags = None)

    id = _base.mutable_attr({'name': 'Camera Number', 'type': 'uint8_t'}, "The identification number of the destination camera.")
    '''The identification number of the destination camera. Type: uint8_t'''
    zoom = _base.mutable_attr({'name': 'Absolute Zoom Level', 'type': 'uint8_t'}, "Absolute zoom level.")
    '''Absolute zoom level. Type: uint8_t'''
    action = _base.mutable_attr({'name': 'Action', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'ACTION'}, "The zoom action to perform. Enumerated (Local).")
    '''The zoom action to perform. Enumerated (Local). Type: uint8_t'''

    def __init__(self, id = None, zoom = None, action = None):
        '''Class constructor
        
        The zoom action to perform. Enumerated (Local).

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            zoom : uint8_t, unit: NOT FOUND

            action : uint8_t, unit: Enumerated (Local)'''
        self._id = id
        self._zoom = zoom
        self._action = action


class SetThrusterActuation(_base.base_message):
    '''Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 301, stable = None, description = "Actuate directly on a thruster.", fields = ('id', 'value',), abbrev = "SetThrusterActuation", name = "Set Thruster Actuation", flags = None)

    id = _base.mutable_attr({'name': 'Thruster Number', 'type': 'uint8_t'}, "The identification number of the destination thruster.")
    '''The identification number of the destination thruster. Type: uint8_t'''
    value = _base.mutable_attr({'name': 'Actuation Value', 'type': 'fp32_t', 'min': -1.0, 'max': 1}, "Actuation magnitude.")
    '''Actuation magnitude. Type: fp32_t'''

    def __init__(self, id = None, value = None):
        '''Class constructor
        
        Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: NOT FOUND'''
        self._id = id
        self._value = value


class SetServoPosition(_base.base_message):
    '''Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 302, stable = None, description = "Set the position of a servo.", fields = ('id', 'value',), abbrev = "SetServoPosition", name = "Set Servo Position", flags = None)

    id = _base.mutable_attr({'name': 'Identifier', 'type': 'uint8_t'}, "The identification number of the destination servo.")
    '''The identification number of the destination servo. Type: uint8_t'''
    value = _base.mutable_attr({'name': 'Position', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "Actuation magnitude.")
    '''Actuation magnitude. Type: fp32_t'''

    def __init__(self, id = None, value = None):
        '''Class constructor
        
        Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            value : fp32_t, unit: rad'''
        self._id = id
        self._value = value


class SetControlSurfaceDeflection(_base.base_message):
    '''Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            angle : fp32_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_angle']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 303, stable = None, description = "Set the deflection angle of a control surface.", fields = ('id', 'angle',), abbrev = "SetControlSurfaceDeflection", name = "Set Control Surface Deflection", flags = None)

    id = _base.mutable_attr({'name': 'Identifier', 'type': 'uint8_t'}, "The identification number of the destination control surface.")
    '''The identification number of the destination control surface. Type: uint8_t'''
    angle = _base.mutable_attr({'name': 'Angle', 'type': 'fp32_t', 'unit': 'rad'}, "Actuation magnitude.")
    '''Actuation magnitude. Type: fp32_t'''

    def __init__(self, id = None, angle = None):
        '''Class constructor
        
        Actuation magnitude.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            angle : fp32_t, unit: rad'''
        self._id = id
        self._angle = angle


class RemoteActionsRequest(_base.base_message):
    '''Example: \"Propulsion=Axis,Lights=Button\"

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            actions : plaintext, unit: TupleList'''

    class OP(_enum.IntEnum):
        '''Full name: operation
        Prefix: OP'''
    
        REPORT = 0
        '''Name: Report'''
    
        QUERY = 1
        '''Name: Query'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_actions']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Actuation", id = 304, stable = None, description = "This message is used as query to request for the possible remote actions (operation=QUERY and the list is empty in this case). The vehicle responds using the same message type returning the tuplelist with the pairs: Action,Type (operation=REPORT). The type of action can be Axis, Hat or Button. Verbs to use: +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Verb | Type | LAUV Use | Verb alternatives | Notes | +===============+==============+==========+============================================+=======================================+ | Accelerate | Button | yes | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Arm | Button | | | Should use ArmState message for state | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Decelerate | Button | yes | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Disarm | Button | | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Exit | Button | yes | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Heading | Axis | yes | Yaw, Rotate, Turning | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Lateral | Axis | | Sway, Sideways | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Pitch | Axis | | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Roll | Axis | | Bank | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Stop | Button | yes | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Thrust | Axis | yes | Surge, Forward, Throtle | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | Vertical | Axis | | Heave, Up, Ascende, VerticalRate, Depth, Z | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ | * | (type) | | | | +---------------+--------------+----------+--------------------------------------------+---------------------------------------+ Special actions for configuration: +---------------+-------------------+----------------------------------------------------+ | Verb | Special Type | Notes | +===============+===================+====================================================+ | Ranges | Decimal, Range127 | Decimal [-1.0; 1.0], Range127[-127; 127] (default) | +---------------+-------------------+----------------------------------------------------+ Types: +-------------------+------------------------------------------------------+ | Type | Note | +===================+======================================================+ | Axis | Axis like (full range) | +-------------------+------------------------------------------------------+ | Button | A button (0 for release, 1 for press) | +-------------------+------------------------------------------------------+ | Slider | Slider like (full range) | +-------------------+------------------------------------------------------+ | HalfSlider | Slider like, but only the positive half of the range | +-------------------+------------------------------------------------------+", fields = ('op', 'actions',), abbrev = "RemoteActionsRequest", name = "Remote Actions Request", flags = None)

    op = _base.mutable_attr({'name': 'operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    actions = _base.mutable_attr({'name': 'Actions', 'type': 'plaintext', 'unit': 'TupleList'}, "Example: \"Propulsion=Axis,Lights=Button\"")
    '''Example: \"Propulsion=Axis,Lights=Button\" Type: plaintext'''

    def __init__(self, op = None, actions = None):
        '''Class constructor
        
        Example: \"Propulsion=Axis,Lights=Button\"

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            actions : plaintext, unit: TupleList'''
        self._op = op
        self._actions = actions


class RemoteActions(_base.base_message):
    '''List of values for each remote action (e.g: \"Propeller=0.6,PanTilt=0.75,Lights=1\").

       This message class contains the following fields and their respective types:
    actions : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_actions']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Actuation", id = 305, stable = None, description = "This message is used to send a periodic update of values for each remote action. If the action is not on the list the assumed value is 0.", fields = ('actions',), abbrev = "RemoteActions", name = "Remote Actions", flags = "periodic")

    actions = _base.mutable_attr({'name': 'Actions', 'type': 'plaintext', 'unit': 'TupleList'}, "List of values for each remote action (e.g: \"Propeller=0.6,PanTilt=0.75,Lights=1\").")
    '''List of values for each remote action (e.g: \"Propeller=0.6,PanTilt=0.75,Lights=1\"). Type: plaintext'''

    def __init__(self, actions = None):
        '''Class constructor
        
        List of values for each remote action (e.g: \"Propeller=0.6,PanTilt=0.75,Lights=1\").

       This message class contains the following fields and their respective types:
    actions : plaintext, unit: TupleList'''
        self._actions = actions


class ButtonEvent(_base.base_message):
    '''Value of the button.

       This message class contains the following fields and their respective types:
    button : uint8_t, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_button', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 306, stable = None, description = "Event of a specific hardware button.", fields = ('button', 'value',), abbrev = "ButtonEvent", name = "Button Event", flags = None)

    button = _base.mutable_attr({'name': 'Button', 'type': 'uint8_t'}, "Button identifier.")
    '''Button identifier. Type: uint8_t'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t'}, "Value of the button.")
    '''Value of the button. Type: uint8_t'''

    def __init__(self, button = None, value = None):
        '''Class constructor
        
        Value of the button.

       This message class contains the following fields and their respective types:
    button : uint8_t, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''
        self._button = button
        self._value = value


class LcdControl(_base.base_message):
    '''Text to be written (if defined write operation).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        TURN_OFF = 0
        '''Name: Turn off display'''
    
        TURN_ON = 1
        '''Name: Turn on display'''
    
        CLEAR = 2
        '''Name: Clear display'''
    
        WRITE0 = 3
        '''Name: Write Line #0'''
    
        WRITE1 = 4
        '''Name: Write Line #1'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_text']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 307, stable = None, description = "Control LCD.", fields = ('op', 'text',), abbrev = "LcdControl", name = "LCD Control", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "The LCD action to perform Enumerated (Local).")
    '''The LCD action to perform Enumerated (Local). Type: uint8_t'''
    text = _base.mutable_attr({'name': 'Text', 'type': 'plaintext'}, "Text to be written (if defined write operation).")
    '''Text to be written (if defined write operation). Type: plaintext'''

    def __init__(self, op = None, text = None):
        '''Class constructor
        
        Text to be written (if defined write operation).

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            text : plaintext, unit: NOT FOUND'''
        self._op = op
        self._text = text


class PowerOperation(_base.base_message):
    '''Scheduled time of operation.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            time_remain : fp32_t, unit: s

            sched_time : fp64_t, unit: s'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: POP'''
    
        PWR_DOWN = 0
        '''Name: Power Down'''
    
        PWR_DOWN_IP = 1
        '''Name: Power Down in Progress'''
    
        PWR_DOWN_ABORTED = 2
        '''Name: Power Down Aborted'''
    
        SCHED_PWR_DOWN = 3
        '''Name: Schedule Power Down'''
    
        PWR_UP = 4
        '''Name: Power Up'''
    
        PWR_UP_IP = 5
        '''Name: Power Up in Progress'''
    
        SCHED_PWR_UP = 6
        '''Name: Schedule Power Up'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_time_remain', '_sched_time']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 308, stable = None, description = "This message allows controlling the system's power lines.", fields = ('op', 'time_remain', 'sched_time',), abbrev = "PowerOperation", name = "Power Operation", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'POP'}, "Operation type. Enumerated (Local).")
    '''Operation type. Enumerated (Local). Type: uint8_t'''
    time_remain = _base.mutable_attr({'name': 'Time Remaining', 'type': 'fp32_t', 'unit': 's'}, "Time remaining to complete operation.")
    '''Time remaining to complete operation. Type: fp32_t'''
    sched_time = _base.mutable_attr({'name': 'Scheduled Time', 'type': 'fp64_t', 'unit': 's'}, "Scheduled time of operation.")
    '''Scheduled time of operation. Type: fp64_t'''

    def __init__(self, op = None, time_remain = None, sched_time = None):
        '''Class constructor
        
        Scheduled time of operation.

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            time_remain : fp32_t, unit: s

            sched_time : fp64_t, unit: s'''
        self._op = op
        self._time_remain = time_remain
        self._sched_time = sched_time


class PowerChannelControl(_base.base_message):
    '''Scheduled time of operation.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)

            sched_time : fp64_t, unit: s'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: PCC_OP'''
    
        TURN_OFF = 0
        '''Name: Turn Off'''
    
        TURN_ON = 1
        '''Name: Turn On'''
    
        TOGGLE = 2
        '''Name: Toggle'''
    
        SCHED_ON = 3
        '''Name: Schedule Turn On'''
    
        SCHED_OFF = 4
        '''Name: Schedule Turn Off'''
    
        SCHED_RESET = 5
        '''Name: Reset Schedules'''
    
        SAVE = 6
        '''Name: Save Current State'''
    
        RESTART = 7
        '''Name: Restart'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_op', '_sched_time']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Actuation", id = 309, stable = None, description = "This message allows controlling power channels.", fields = ('name', 'op', 'sched_time',), abbrev = "PowerChannelControl", name = "Power Channel Control", flags = None)

    name = _base.mutable_attr({'name': 'Channel Name', 'type': 'plaintext'}, "The name of the power channel.")
    '''The name of the power channel. Type: plaintext'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'max': 6, 'prefix': 'PCC_OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    sched_time = _base.mutable_attr({'name': 'Scheduled Time', 'type': 'fp64_t', 'unit': 's'}, "Scheduled time of operation.")
    '''Scheduled time of operation. Type: fp64_t'''

    def __init__(self, name = None, op = None, sched_time = None):
        '''Class constructor
        
        Scheduled time of operation.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)

            sched_time : fp64_t, unit: s'''
        self._name = name
        self._op = op
        self._sched_time = sched_time


class QueryPowerChannelState(_base.base_message):
    '''Request the state of power channels.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Actuation", id = 310, stable = None, description = "Request the state of power channels.", fields = [], abbrev = "QueryPowerChannelState", name = "Query Power Channel State", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Request the state of power channels.

       This message class contains the following fields and their respective types:
'''


class PowerChannelState(_base.base_message):
    '''State of the Power Channel. Enumerated (Local).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: PCS'''
    
        OFF = 0
        '''Name: Off'''
    
        ON = 1
        '''Name: On'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_state']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Actuation", id = 311, stable = None, description = "Message conveying the state of a power channel.", fields = ('name', 'state',), abbrev = "PowerChannelState", name = "Power Channel State", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "Power Channel Name.")
    '''Power Channel Name. Type: plaintext'''
    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'PCS'}, "State of the Power Channel. Enumerated (Local).")
    '''State of the Power Channel. Enumerated (Local). Type: uint8_t'''

    def __init__(self, name = None, state = None):
        '''Class constructor
        
        State of the Power Channel. Enumerated (Local).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)'''
        self._name = name
        self._state = state


class LedBrightness(_base.base_message):
    '''Brightness value.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 312, stable = None, description = "Brightness value of an LED (Light-Emitting Diode).", fields = ('name', 'value',), abbrev = "LedBrightness", name = "LED Brightness", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "LED name.")
    '''LED name. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t'}, "Brightness value.")
    '''Brightness value. Type: uint8_t'''

    def __init__(self, name = None, value = None):
        '''Class constructor
        
        Brightness value.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''
        self._name = name
        self._value = value


class QueryLedBrightness(_base.base_message):
    '''LED name.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 313, stable = None, description = "Query the brightness of an LED (Light-Emitting Diode). The recipient of this message shall reply with 'LedBrightness'.", fields = ('name',), abbrev = "QueryLedBrightness", name = "Query LED Brightness", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "LED name.")
    '''LED name. Type: plaintext'''

    def __init__(self, name = None):
        '''Class constructor
        
        LED name.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''
        self._name = name


class SetLedBrightness(_base.base_message):
    '''Desired brightness value.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 314, stable = None, description = "Control the brightness of an LED (Light-Emitting Diode). The recipient of this message shall set the intensity of the LED to the desired 'value' and reply with 'LedBrightness'.", fields = ('name', 'value',), abbrev = "SetLedBrightness", name = "Set LED Brightness", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "LED name.")
    '''LED name. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t'}, "Desired brightness value.")
    '''Desired brightness value. Type: uint8_t'''

    def __init__(self, name = None, value = None):
        '''Class constructor
        
        Desired brightness value.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''
        self._name = name
        self._value = value


class SetPWM(_base.base_message):
    '''The active time of the PWM signal. The duty cycle value must be less or equal to the period.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            period : uint32_t, unit: µs

            duty_cycle : uint32_t, unit: µs'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_period', '_duty_cycle']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 315, stable = None, description = "Set properties of a PWM signal channel.", fields = ('id', 'period', 'duty_cycle',), abbrev = "SetPWM", name = "Set PWM", flags = None)

    id = _base.mutable_attr({'name': 'Channel Identifier', 'type': 'uint8_t'}, "PWM channel identifier.")
    '''PWM channel identifier. Type: uint8_t'''
    period = _base.mutable_attr({'name': 'Period', 'type': 'uint32_t', 'unit': 'µs'}, "The total period of the PWM signal (sum of active and inactive time of the PWM).")
    '''The total period of the PWM signal (sum of active and inactive time of the PWM). Type: uint32_t'''
    duty_cycle = _base.mutable_attr({'name': 'Duty Cycle', 'type': 'uint32_t', 'unit': 'µs'}, "The active time of the PWM signal. The duty cycle value must be less or equal to the period.")
    '''The active time of the PWM signal. The duty cycle value must be less or equal to the period. Type: uint32_t'''

    def __init__(self, id = None, period = None, duty_cycle = None):
        '''Class constructor
        
        The active time of the PWM signal. The duty cycle value must be less or equal to the period.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            period : uint32_t, unit: µs

            duty_cycle : uint32_t, unit: µs'''
        self._id = id
        self._period = period
        self._duty_cycle = duty_cycle


class PWM(_base.base_message):
    '''The active time of the PWM signal. The duty cycle value is less or equal to the period.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            period : uint32_t, unit: µs

            duty_cycle : uint32_t, unit: µs'''

    __slots__ = ['_Attributes', '_header', '_footer', '_id', '_period', '_duty_cycle']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 316, stable = None, description = "Properties of a PWM signal channel.", fields = ('id', 'period', 'duty_cycle',), abbrev = "PWM", name = "PWM", flags = None)

    id = _base.mutable_attr({'name': 'Channel Identifier', 'type': 'uint8_t'}, "PWM channel identifier.")
    '''PWM channel identifier. Type: uint8_t'''
    period = _base.mutable_attr({'name': 'Period', 'type': 'uint32_t', 'unit': 'µs'}, "The total period of the PWM signal (sum of active and inactive time of the PWM).")
    '''The total period of the PWM signal (sum of active and inactive time of the PWM). Type: uint32_t'''
    duty_cycle = _base.mutable_attr({'name': 'Duty Cycle', 'type': 'uint32_t', 'unit': 'µs'}, "The active time of the PWM signal. The duty cycle value is less or equal to the period.")
    '''The active time of the PWM signal. The duty cycle value is less or equal to the period. Type: uint32_t'''

    def __init__(self, id = None, period = None, duty_cycle = None):
        '''Class constructor
        
        The active time of the PWM signal. The duty cycle value is less or equal to the period.

       This message class contains the following fields and their respective types:
    id : uint8_t, unit: NOT FOUND

            period : uint32_t, unit: µs

            duty_cycle : uint32_t, unit: µs'''
        self._id = id
        self._period = period
        self._duty_cycle = duty_cycle


class GpioState(_base.base_message):
    '''Logical level of the GPIO.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 2000, stable = None, description = "Current state of a GPIO.", fields = ('name', 'value',), abbrev = "GpioState", name = "GPIO State", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "GPIO Name.")
    '''GPIO Name. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t'}, "Logical level of the GPIO.")
    '''Logical level of the GPIO. Type: uint8_t'''

    def __init__(self, name = None, value = None):
        '''Class constructor
        
        Logical level of the GPIO.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''
        self._name = name
        self._value = value


class GpioStateGet(_base.base_message):
    '''GPIO Name.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 2001, stable = None, description = "Request the state of a given GPIO. The receiving entity shall reply with a GpioState message.", fields = ('name',), abbrev = "GpioStateGet", name = "Get GPIO State", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "GPIO Name.")
    '''GPIO Name. Type: plaintext'''

    def __init__(self, name = None):
        '''Class constructor
        
        GPIO Name.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND'''
        self._name = name


class GpioStateSet(_base.base_message):
    '''Logical level of the GPIO.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Actuation", id = 2002, stable = None, description = "Set the state of a given GPIO. The receiving entity shall reply with a GpioState message.", fields = ('name', 'value',), abbrev = "GpioStateSet", name = "Set GPIO State", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "GPIO Name.")
    '''GPIO Name. Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'uint8_t'}, "Logical level of the GPIO.")
    '''Logical level of the GPIO. Type: uint8_t'''

    def __init__(self, name = None, value = None):
        '''Class constructor
        
        Logical level of the GPIO.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : uint8_t, unit: NOT FOUND'''
        self._name = name
        self._value = value


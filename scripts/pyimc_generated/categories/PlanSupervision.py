'''
IMC Plan Supervision messages.
'''

from .. import _base
import enum as _enum

class Abort(_base.base_message):
    '''Stops any executing actions and put the system in a standby mode.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Plan Supervision", id = 550, stable = None, description = "Stops any executing actions and put the system in a standby mode.", fields = [], abbrev = "Abort", name = "Abort", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Stops any executing actions and put the system in a standby mode.

       This message class contains the following fields and their respective types:
'''


class PlanSpecification(_base.base_message):
    '''Contains an optionally defined 'MessageList' for actions fired on plan termination.

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            vnamespace : plaintext, unit: NOT FOUND

            variables : message-list, unit: NOT FOUND

            start_man_id : plaintext, unit: NOT FOUND

            maneuvers : message-list, unit: NOT FOUND

            transitions : message-list, unit: NOT FOUND

            start_actions : message-list, unit: NOT FOUND

            end_actions : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_plan_id', '_description', '_vnamespace', '_variables', '_start_man_id', '_maneuvers', '_transitions', '_start_actions', '_end_actions']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Plan Supervision", id = 551, stable = None, description = "Identity and description of a plan's general parameters, associated with plan loading (i.e. load plan command in *PlanCommand*). A plan specification is defined by a plan identifier, a set of maneuver specifications and a start maneuver from that set. See the :ref:`PlanManeuver` message for details on maneuver specification.", fields = ('plan_id', 'description', 'vnamespace', 'variables', 'start_man_id', 'maneuvers', 'transitions', 'start_actions', 'end_actions',), abbrev = "PlanSpecification", name = "Plan Specification", flags = None)

    plan_id = _base.mutable_attr({'name': 'Plan ID', 'type': 'plaintext'}, "The plan's identifier.")
    '''The plan's identifier. Type: plaintext'''
    description = _base.mutable_attr({'name': 'Plan Description', 'type': 'plaintext'}, "Verbose text description of plan.")
    '''Verbose text description of plan. Type: plaintext'''
    vnamespace = _base.mutable_attr({'name': 'Namespace', 'type': 'plaintext'}, "Namespace for plan variables.")
    '''Namespace for plan variables. Type: plaintext'''
    variables = _base.mutable_attr({'name': 'Plan Variables', 'type': 'message-list', 'message-type': 'PlanVariable'}, "Plan variables.")
    '''Plan variables. Type: message-list'''
    start_man_id = _base.mutable_attr({'name': 'Starting maneuver', 'type': 'plaintext'}, "Indicates the id of the starting maneuver for this plan.")
    '''Indicates the id of the starting maneuver for this plan. Type: plaintext'''
    maneuvers = _base.mutable_attr({'name': 'Maneuvers', 'type': 'message-list', 'message-type': 'PlanManeuver'}, "List of maneuver specifications.")
    '''List of maneuver specifications. Type: message-list'''
    transitions = _base.mutable_attr({'name': 'Transitions', 'type': 'message-list', 'message-type': 'PlanTransition'}, "List of maneuver specifications.")
    '''List of maneuver specifications. Type: message-list'''
    start_actions = _base.mutable_attr({'name': 'Start Actions', 'type': 'message-list'}, "Contains an optionally defined 'MessageList' for actions fired on plan activation.")
    '''Contains an optionally defined 'MessageList' for actions fired on plan activation. Type: message-list'''
    end_actions = _base.mutable_attr({'name': 'End Actions', 'type': 'message-list'}, "Contains an optionally defined 'MessageList' for actions fired on plan termination.")
    '''Contains an optionally defined 'MessageList' for actions fired on plan termination. Type: message-list'''

    def __init__(self, plan_id = None, description = None, vnamespace = None, variables = None, start_man_id = None, maneuvers = None, transitions = None, start_actions = None, end_actions = None):
        '''Class constructor
        
        Contains an optionally defined 'MessageList' for actions fired on plan termination.

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            vnamespace : plaintext, unit: NOT FOUND

            variables : message-list, unit: NOT FOUND

            start_man_id : plaintext, unit: NOT FOUND

            maneuvers : message-list, unit: NOT FOUND

            transitions : message-list, unit: NOT FOUND

            start_actions : message-list, unit: NOT FOUND

            end_actions : message-list, unit: NOT FOUND'''
        self._plan_id = plan_id
        self._description = description
        self._vnamespace = vnamespace
        self._variables = variables
        self._start_man_id = start_man_id
        self._maneuvers = maneuvers
        self._transitions = transitions
        self._start_actions = start_actions
        self._end_actions = end_actions


class PlanManeuver(_base.base_message):
    '''Contains an optionally defined 'MessageList' for actions fired on plan termination.

       This message class contains the following fields and their respective types:
    maneuver_id : plaintext, unit: NOT FOUND

            data : message, unit: NOT FOUND

            start_actions : message-list, unit: NOT FOUND

            end_actions : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_maneuver_id', '_data', '_start_actions', '_end_actions']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Plan Supervision", id = 552, stable = None, description = "Named plan maneuver.", fields = ('maneuver_id', 'data', 'start_actions', 'end_actions',), abbrev = "PlanManeuver", name = "Plan Maneuver", flags = None)

    maneuver_id = _base.mutable_attr({'name': 'Maneuver ID', 'type': 'plaintext'}, "The maneuver ID.")
    '''The maneuver ID. Type: plaintext'''
    data = _base.mutable_attr({'name': 'Maneuver Specification', 'type': 'message', 'message-type': 'Maneuver'}, "The maneuver specification.")
    '''The maneuver specification. Type: message'''
    start_actions = _base.mutable_attr({'name': 'Start Actions', 'type': 'message-list'}, "Contains an optionally defined 'MessageList' for actions fired on plan activation.")
    '''Contains an optionally defined 'MessageList' for actions fired on plan activation. Type: message-list'''
    end_actions = _base.mutable_attr({'name': 'End Actions', 'type': 'message-list'}, "Contains an optionally defined 'MessageList' for actions fired on plan termination.")
    '''Contains an optionally defined 'MessageList' for actions fired on plan termination. Type: message-list'''

    def __init__(self, maneuver_id = None, data = None, start_actions = None, end_actions = None):
        '''Class constructor
        
        Contains an optionally defined 'MessageList' for actions fired on plan termination.

       This message class contains the following fields and their respective types:
    maneuver_id : plaintext, unit: NOT FOUND

            data : message, unit: NOT FOUND

            start_actions : message-list, unit: NOT FOUND

            end_actions : message-list, unit: NOT FOUND'''
        self._maneuver_id = maneuver_id
        self._data = data
        self._start_actions = start_actions
        self._end_actions = end_actions


class PlanTransition(_base.base_message):
    '''Messages processed when the transition is triggered.

       This message class contains the following fields and their respective types:
    source_man : plaintext, unit: NOT FOUND

            dest_man : plaintext, unit: NOT FOUND

            conditions : plaintext, unit: NOT FOUND

            actions : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_source_man', '_dest_man', '_conditions', '_actions']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Plan Supervision", id = 553, stable = None, description = "Describes a plan transition within a plan specification. A transition states the vehicle conditions that must be met to signal the transition, the maneuver that should be started as a result, and an optional set of actions triggered by the transition.", fields = ('source_man', 'dest_man', 'conditions', 'actions',), abbrev = "PlanTransition", name = "Plan Transition", flags = None)

    source_man = _base.mutable_attr({'name': 'Source', 'type': 'plaintext'}, "Comma separated list of maneuver IDs, or the special value '.' to identify a global plan transition.")
    '''Comma separated list of maneuver IDs, or the special value '.' to identify a global plan transition. Type: plaintext'''
    dest_man = _base.mutable_attr({'name': 'Destination Maneuver Name', 'type': 'plaintext'}, "Target maneuver name. If it equals the special value '_done_' then plan should terminate with a success status. If it equals the special value '_error_' then the plan should terminate with an error status.")
    '''Target maneuver name. If it equals the special value '_done_' then plan should terminate with a success status. If it equals the special value '_error_' then the plan should terminate with an error status. Type: plaintext'''
    conditions = _base.mutable_attr({'name': 'Transition conditions', 'type': 'plaintext'}, "Comma separated list of conditions for transition. Each condition identifier corresponds to a known predicate which is interpreted and tested internally by the vehicle.")
    '''Comma separated list of conditions for transition. Each condition identifier corresponds to a known predicate which is interpreted and tested internally by the vehicle. Type: plaintext'''
    actions = _base.mutable_attr({'name': 'Transition actions', 'type': 'message-list'}, "Messages processed when the transition is triggered.")
    '''Messages processed when the transition is triggered. Type: message-list'''

    def __init__(self, source_man = None, dest_man = None, conditions = None, actions = None):
        '''Class constructor
        
        Messages processed when the transition is triggered.

       This message class contains the following fields and their respective types:
    source_man : plaintext, unit: NOT FOUND

            dest_man : plaintext, unit: NOT FOUND

            conditions : plaintext, unit: NOT FOUND

            actions : message-list, unit: NOT FOUND'''
        self._source_man = source_man
        self._dest_man = dest_man
        self._conditions = conditions
        self._actions = actions


class EmergencyControl(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            plan : message, unit: NOT FOUND'''

    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: ECTL'''
    
        ENABLE = 0
        '''Name: Enable'''
    
        DISABLE = 1
        '''Name: Disable'''
    
        START = 2
        '''Name: Start'''
    
        STOP = 3
        '''Name: Stop'''
    
        QUERY = 4
        '''Name: Query'''
    
        SET_PLAN = 5
        '''Name: Set Plan'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_command', '_plan']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Plan Supervision", id = 554, stable = None, description = None, fields = ('command', 'plan',), abbrev = "EmergencyControl", name = "Emergency Control", flags = None)

    command = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'prefix': 'ECTL', 'unit': 'Enumerated'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    plan = _base.mutable_attr({'name': 'Plan Specification', 'type': 'message', 'message-type': 'PlanSpecification'}, "No description available")
    '''No description available Type: message'''

    def __init__(self, command = None, plan = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            plan : message, unit: NOT FOUND'''
        self._command = command
        self._plan = plan


class EmergencyControlState(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            comm_level : uint8_t, unit: %'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: ECS'''
    
        NOT_CONFIGURED = 0
        '''Name: Not Configured'''
    
        DISABLED = 1
        '''Name: Disabled'''
    
        ENABLED = 2
        '''Name: Enabled'''
    
        ARMED = 3
        '''Name: Armed'''
    
        ACTIVE = 4
        '''Name: Active'''
    
        STOPPING = 5
        '''Name: Stopping'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_plan_id', '_comm_level']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Plan Supervision", id = 555, stable = None, description = None, fields = ('state', 'plan_id', 'comm_level',), abbrev = "EmergencyControlState", name = "Emergency Control State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'prefix': 'ECS', 'unit': 'Enumerated'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    plan_id = _base.mutable_attr({'name': 'Plan Id', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    comm_level = _base.mutable_attr({'name': 'Communications Level', 'type': 'uint8_t', 'unit': '%', 'max': 100}, "No description available")
    '''No description available Type: uint8_t'''

    def __init__(self, state = None, plan_id = None, comm_level = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            comm_level : uint8_t, unit: %'''
        self._state = state
        self._plan_id = plan_id
        self._comm_level = comm_level


class PlanDB(_base.base_message):
    '''Human-readable complementary information. For example this may be used to detail reasons for failure, or to report in-progress information.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            arg : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: DBT'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SUCCESS = 1
        '''Name: Reply -- Success'''
    
        FAILURE = 2
        '''Name: Reply -- Failure'''
    
        IN_PROGRESS = 3
        '''Name: Reply -- In Progress'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: DBOP'''
    
        SET = 0
        '''Name: Set Plan'''
    
        DEL = 1
        '''Name: Delete Plan'''
    
        GET = 2
        '''Name: Get Plan'''
    
        GET_INFO = 3
        '''Name: Get Plan Info'''
    
        CLEAR = 4
        '''Name: Clear Database'''
    
        GET_STATE = 5
        '''Name: Get Database State (Simple)'''
    
        GET_DSTATE = 6
        '''Name: Get Database State (Detailed)'''
    
        BOOT = 7
        '''Name: Boot Notification'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_op', '_request_id', '_plan_id', '_arg', '_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 556, stable = None, description = "Request/reply to plan database.", fields = ('type', 'op', 'request_id', 'plan_id', 'arg', 'info',), abbrev = "PlanDB", name = "Plan DB", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'unit': 'Enumerated', 'type': 'uint8_t', 'prefix': 'DBT'}, "Indicates if the message is a request, or a reply to a previous request. Enumerated (Local).")
    '''Indicates if the message is a request, or a reply to a previous request. Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DBOP'}, "Indicates the operation affecting the DB. The operation may relate to a single plan or the entire plan DB. For each request, a plan DB may reply with any number of 'in progress' replies followed by a success or a failure reply. The 'op', 'request_id' and 'plan_id' fields of a request will be echoed in one or more responses to that request. The operation at stake also determines a certain type of the 'arg' field, and whether or not the 'plan_id' field needs to be set. Enumerated (Local).")
    '''Indicates the operation affecting the DB. The operation may relate to a single plan or the entire plan DB. For each request, a plan DB may reply with any number of 'in progress' replies followed by a success or a failure reply. The 'op', 'request_id' and 'plan_id' fields of a request will be echoed in one or more responses to that request. The operation at stake also determines a certain type of the 'arg' field, and whether or not the 'plan_id' field needs to be set. Enumerated (Local). Type: uint8_t'''
    request_id = _base.mutable_attr({'name': 'Request ID', 'type': 'uint16_t'}, "Request ID. This may be used by interfacing modules, e.g. using sequence counters, to annotate requests and appropriately identify replies")
    '''Request ID. This may be used by interfacing modules, e.g. using sequence counters, to annotate requests and appropriately identify replies Type: uint16_t'''
    plan_id = _base.mutable_attr({'name': 'Plan ID', 'type': 'plaintext'}, "Plan identifier for the operation, if one is required.")
    '''Plan identifier for the operation, if one is required. Type: plaintext'''
    arg = _base.mutable_attr({'name': 'Argument', 'type': 'message'}, "Request or reply argument.")
    '''Request or reply argument. Type: message'''
    info = _base.mutable_attr({'name': 'Complementary Information', 'type': 'plaintext'}, "Human-readable complementary information. For example this may be used to detail reasons for failure, or to report in-progress information.")
    '''Human-readable complementary information. For example this may be used to detail reasons for failure, or to report in-progress information. Type: plaintext'''

    def __init__(self, type = None, op = None, request_id = None, plan_id = None, arg = None, info = None):
        '''Class constructor
        
        Human-readable complementary information. For example this may be used to detail reasons for failure, or to report in-progress information.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            arg : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''
        self._type = type
        self._op = op
        self._request_id = request_id
        self._plan_id = plan_id
        self._arg = arg
        self._info = info


class PlanDBState(_base.base_message):
    '''Individual information for plans.

       This message class contains the following fields and their respective types:
    plan_count : uint16_t, unit: NOT FOUND

            plan_size : uint32_t, unit: NOT FOUND

            change_time : fp64_t, unit: s

            change_sid : uint16_t, unit: NOT FOUND

            change_sname : plaintext, unit: NOT FOUND

            md5 : rawdata, unit: NOT FOUND

            plans_info : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_plan_count', '_plan_size', '_change_time', '_change_sid', '_change_sname', '_md5', '_plans_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 557, stable = None, description = "Characterizes the state of the entire plan database.", fields = ('plan_count', 'plan_size', 'change_time', 'change_sid', 'change_sname', 'md5', 'plans_info',), abbrev = "PlanDBState", name = "Plan DB State", flags = None)

    plan_count = _base.mutable_attr({'name': 'Plan -- Count', 'type': 'uint16_t'}, "Number of stored plans.")
    '''Number of stored plans. Type: uint16_t'''
    plan_size = _base.mutable_attr({'name': 'Plan -- Size of all plans', 'type': 'uint32_t'}, "Size of all plans.The value equals the sum of the IMC payload sizes for 'PlanSpecification' stored in the DB.")
    '''Size of all plans.The value equals the sum of the IMC payload sizes for 'PlanSpecification' stored in the DB. Type: uint32_t'''
    change_time = _base.mutable_attr({'name': 'Last Change -- Time', 'type': 'fp64_t', 'unit': 's'}, "Time of last change (Epoch time).")
    '''Time of last change (Epoch time). Type: fp64_t'''
    change_sid = _base.mutable_attr({'name': 'Last Change -- Source Address', 'type': 'uint16_t'}, "IMC address for source of last DB change.")
    '''IMC address for source of last DB change. Type: uint16_t'''
    change_sname = _base.mutable_attr({'name': 'Last Change -- Source Name', 'type': 'plaintext'}, "IMC node name for source of last DB change.")
    '''IMC node name for source of last DB change. Type: plaintext'''
    md5 = _base.mutable_attr({'name': 'MD5', 'type': 'rawdata'}, "MD5 database verification code. The MD5 hash sum is computed over the stream formed by the MD5 of all plans, ordered by plan id, in compliance with RFC 1321.")
    '''MD5 database verification code. The MD5 hash sum is computed over the stream formed by the MD5 of all plans, ordered by plan id, in compliance with RFC 1321. Type: rawdata'''
    plans_info = _base.mutable_attr({'name': 'Plan info', 'type': 'message-list', 'message-type': 'PlanDBInformation'}, "Individual information for plans.")
    '''Individual information for plans. Type: message-list'''

    def __init__(self, plan_count = None, plan_size = None, change_time = None, change_sid = None, change_sname = None, md5 = None, plans_info = None):
        '''Class constructor
        
        Individual information for plans.

       This message class contains the following fields and their respective types:
    plan_count : uint16_t, unit: NOT FOUND

            plan_size : uint32_t, unit: NOT FOUND

            change_time : fp64_t, unit: s

            change_sid : uint16_t, unit: NOT FOUND

            change_sname : plaintext, unit: NOT FOUND

            md5 : rawdata, unit: NOT FOUND

            plans_info : message-list, unit: NOT FOUND'''
        self._plan_count = plan_count
        self._plan_size = plan_size
        self._change_time = change_time
        self._change_sid = change_sid
        self._change_sname = change_sname
        self._md5 = md5
        self._plans_info = plans_info


class PlanDBInformation(_base.base_message):
    '''MD5 plan verification code. The value is calculated over the message payload of the 'PlanSpecification', in compliance with RFC 1321.

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            plan_size : uint16_t, unit: NOT FOUND

            change_time : fp64_t, unit: NOT FOUND

            change_sid : uint16_t, unit: NOT FOUND

            change_sname : plaintext, unit: NOT FOUND

            md5 : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_plan_id', '_plan_size', '_change_time', '_change_sid', '_change_sname', '_md5']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 558, stable = None, description = None, fields = ('plan_id', 'plan_size', 'change_time', 'change_sid', 'change_sname', 'md5',), abbrev = "PlanDBInformation", name = "Plan DB Information", flags = None)

    plan_id = _base.mutable_attr({'name': 'Plan ID', 'type': 'plaintext'}, "Plan identifier.")
    '''Plan identifier. Type: plaintext'''
    plan_size = _base.mutable_attr({'name': 'Plan Size', 'type': 'uint16_t'}, "Plan size. The value equals the IMC message payload of the associated 'PlanSpecification' message in bytes.")
    '''Plan size. The value equals the IMC message payload of the associated 'PlanSpecification' message in bytes. Type: uint16_t'''
    change_time = _base.mutable_attr({'name': 'Last Changed -- Time', 'type': 'fp64_t'}, "Time of last change to the plan (Epoch time).")
    '''Time of last change to the plan (Epoch time). Type: fp64_t'''
    change_sid = _base.mutable_attr({'name': 'Last Change -- Source Address', 'type': 'uint16_t'}, "IMC address for source of last change to the plan.")
    '''IMC address for source of last change to the plan. Type: uint16_t'''
    change_sname = _base.mutable_attr({'name': 'Last Change -- Source Name', 'type': 'plaintext'}, "IMC node name for source of last change to the plan.")
    '''IMC node name for source of last change to the plan. Type: plaintext'''
    md5 = _base.mutable_attr({'name': 'MD5', 'type': 'rawdata'}, "MD5 plan verification code. The value is calculated over the message payload of the 'PlanSpecification', in compliance with RFC 1321.")
    '''MD5 plan verification code. The value is calculated over the message payload of the 'PlanSpecification', in compliance with RFC 1321. Type: rawdata'''

    def __init__(self, plan_id = None, plan_size = None, change_time = None, change_sid = None, change_sname = None, md5 = None):
        '''Class constructor
        
        MD5 plan verification code. The value is calculated over the message payload of the 'PlanSpecification', in compliance with RFC 1321.

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            plan_size : uint16_t, unit: NOT FOUND

            change_time : fp64_t, unit: NOT FOUND

            change_sid : uint16_t, unit: NOT FOUND

            change_sname : plaintext, unit: NOT FOUND

            md5 : rawdata, unit: NOT FOUND'''
        self._plan_id = plan_id
        self._plan_size = plan_size
        self._change_time = change_time
        self._change_sid = change_sid
        self._change_sname = change_sname
        self._md5 = md5


class PlanControl(_base.base_message):
    '''Complementary human-readable information. This is used in association to replies.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            flags : uint16_t, unit: Bitfield (Local)

            arg : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: PC'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SUCCESS = 1
        '''Name: Reply -- Success'''
    
        FAILURE = 2
        '''Name: Reply -- Failure'''
    
        IN_PROGRESS = 3
        '''Name: Reply -- In Progress'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: PC'''
    
        START = 0
        '''Name: Start Plan'''
    
        STOP = 1
        '''Name: Stop Plan'''
    
        LOAD = 2
        '''Name: Load Plan'''
    
        GET = 3
        '''Name: Get Plan'''
    
    
    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        CALIBRATE = 1
        '''Name: Calibrate Vehicle'''
    
        IGNORE_ERRORS = 2
        '''Name: Ignore Errors'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_op', '_request_id', '_plan_id', '_flags', '_arg', '_info']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Plan Supervision", id = 559, stable = None, description = "Plan control request/reply.", fields = ('type', 'op', 'request_id', 'plan_id', 'flags', 'arg', 'info',), abbrev = "PlanControl", name = "Plan Control", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'unit': 'Enumerated', 'type': 'uint8_t', 'prefix': 'PC'}, "Indicates if the message is a request or a reply to a previous request. The *op*, *request_id* and *plan_id* fields of a request will be echoed in one or more responses to that request. Enumerated (Local).")
    '''Indicates if the message is a request or a reply to a previous request. The *op*, *request_id* and *plan_id* fields of a request will be echoed in one or more responses to that request. Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'PC'}, "Plan control operation. Enumerated (Local).")
    '''Plan control operation. Enumerated (Local). Type: uint8_t'''
    request_id = _base.mutable_attr({'name': 'Request ID', 'type': 'uint16_t'}, "Request ID. This may be used by interfacing modules e.g. using sequence counters. to annotate requests and appropriately identify replies.")
    '''Request ID. This may be used by interfacing modules e.g. using sequence counters. to annotate requests and appropriately identify replies. Type: uint16_t'''
    plan_id = _base.mutable_attr({'name': 'Plan Identifier', 'type': 'plaintext'}, "The identifier for the plan to be stopped / started / loaded / retrieved according to the command requested (*op* field).")
    '''The identifier for the plan to be stopped / started / loaded / retrieved according to the command requested (*op* field). Type: plaintext'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint16_t', 'prefix': 'FLG', 'unit': 'Bitfield'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint16_t'''
    arg = _base.mutable_attr({'name': 'Request/Reply Argument', 'type': 'message'}, "Complementary message argument for requests/replies.")
    '''Complementary message argument for requests/replies. Type: message'''
    info = _base.mutable_attr({'name': 'Complementary Info', 'type': 'plaintext'}, "Complementary human-readable information. This is used in association to replies.")
    '''Complementary human-readable information. This is used in association to replies. Type: plaintext'''

    def __init__(self, type = None, op = None, request_id = None, plan_id = None, flags = None, arg = None, info = None):
        '''Class constructor
        
        Complementary human-readable information. This is used in association to replies.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            request_id : uint16_t, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            flags : uint16_t, unit: Bitfield (Local)

            arg : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''
        self._type = type
        self._op = op
        self._request_id = request_id
        self._plan_id = plan_id
        self._flags = flags
        self._arg = arg
        self._info = info


class PlanControlState(_base.base_message):
    '''Outcome of the last executed plan. Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            plan_eta : int32_t, unit: s

            plan_progress : fp32_t, unit: %

            man_id : plaintext, unit: NOT FOUND

            man_type : uint16_t, unit: NOT FOUND

            man_eta : int32_t, unit: s

            last_outcome : uint8_t, unit: Enumerated (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: PCS'''
    
        BLOCKED = 0
        '''Name: Blocked'''
    
        READY = 1
        '''Name: Ready'''
    
        INITIALIZING = 2
        '''Name: Initializing'''
    
        EXECUTING = 3
        '''Name: Executing'''
    
    
    class LAST_OUTCOME(_enum.IntEnum):
        '''Full name: Last Plan Outcome
        Prefix: LPO'''
    
        NONE = 0
        '''Name: None'''
    
        SUCCESS = 1
        '''Name: Success'''
    
        FAILURE = 2
        '''Name: Failure'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_plan_id', '_plan_eta', '_plan_progress', '_man_id', '_man_type', '_man_eta', '_last_outcome']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Plan Supervision", id = 560, stable = None, description = "State of plan control.", fields = ('state', 'plan_id', 'plan_eta', 'plan_progress', 'man_id', 'man_type', 'man_eta', 'last_outcome',), abbrev = "PlanControlState", name = "Plan Control State", flags = "periodic")

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'prefix': 'PCS', 'unit': 'Enumerated'}, "Describes overall state. Enumerated (Local).")
    '''Describes overall state. Enumerated (Local). Type: uint8_t'''
    plan_id = _base.mutable_attr({'name': 'Plan -- ID', 'type': 'plaintext'}, "Identifier of plan currently loaded.")
    '''Identifier of plan currently loaded. Type: plaintext'''
    plan_eta = _base.mutable_attr({'name': 'Plan -- ETA', 'type': 'int32_t', 'unit': 's'}, "Current plan estimated time to completion. The value will be -1 if the time is unknown or undefined.")
    '''Current plan estimated time to completion. The value will be -1 if the time is unknown or undefined. Type: int32_t'''
    plan_progress = _base.mutable_attr({'name': 'Plan -- Progress', 'type': 'fp32_t', 'unit': '%'}, "Current plan estimated progress in percent. The value will be negative if unknown or undefined.")
    '''Current plan estimated progress in percent. The value will be negative if unknown or undefined. Type: fp32_t'''
    man_id = _base.mutable_attr({'name': 'Maneuver -- ID', 'type': 'plaintext'}, "Current node ID, when executing a plan.")
    '''Current node ID, when executing a plan. Type: plaintext'''
    man_type = _base.mutable_attr({'name': 'Maneuver -- Type', 'type': 'uint16_t'}, "Type of maneuver being executed (IMC serialization id), when executing a plan.")
    '''Type of maneuver being executed (IMC serialization id), when executing a plan. Type: uint16_t'''
    man_eta = _base.mutable_attr({'name': 'Maneuver -- ETA', 'type': 'int32_t', 'unit': 's'}, "Current node estimated time to completion, when executing a plan. The value will be -1 if the time is unknown or undefined.")
    '''Current node estimated time to completion, when executing a plan. The value will be -1 if the time is unknown or undefined. Type: int32_t'''
    last_outcome = _base.mutable_attr({'name': 'Last Plan Outcome', 'type': 'uint8_t', 'prefix': 'LPO', 'unit': 'Enumerated'}, "Outcome of the last executed plan. Enumerated (Local).")
    '''Outcome of the last executed plan. Enumerated (Local). Type: uint8_t'''

    def __init__(self, state = None, plan_id = None, plan_eta = None, plan_progress = None, man_id = None, man_type = None, man_eta = None, last_outcome = None):
        '''Class constructor
        
        Outcome of the last executed plan. Enumerated (Local).

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            plan_eta : int32_t, unit: s

            plan_progress : fp32_t, unit: %

            man_id : plaintext, unit: NOT FOUND

            man_type : uint16_t, unit: NOT FOUND

            man_eta : int32_t, unit: s

            last_outcome : uint8_t, unit: Enumerated (Local)'''
        self._state = state
        self._plan_id = plan_id
        self._plan_eta = plan_eta
        self._plan_progress = plan_progress
        self._man_id = man_id
        self._man_type = man_type
        self._man_eta = man_eta
        self._last_outcome = last_outcome


class PlanVariable(_base.base_message):
    '''No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            access : uint8_t, unit: Enumerated (Local)'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: PVT'''
    
        BOOLEAN = 0
        '''Name: Boolean'''
    
        NUMBER = 1
        '''Name: Number'''
    
        TEXT = 2
        '''Name: Text'''
    
        MESSAGE = 3
        '''Name: Message'''
    
    
    class ACCESS(_enum.IntEnum):
        '''Full name: Access Type
        Prefix: PVA'''
    
        INPUT = 0
        '''Name: Input'''
    
        OUTPUT = 1
        '''Name: Output'''
    
        LOCAL = 2
        '''Name: Local'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_value', '_type', '_access']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Plan Supervision", id = 561, stable = None, description = "A plan variable.", fields = ('name', 'value', 'type', 'access',), abbrev = "PlanVariable", name = "Plan Variable", flags = None)

    name = _base.mutable_attr({'name': 'Name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    value = _base.mutable_attr({'name': 'Value', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'PVT'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    access = _base.mutable_attr({'name': 'Access Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'PVA'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''

    def __init__(self, name = None, value = None, type = None, access = None):
        '''Class constructor
        
        No description available Enumerated (Local).

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            value : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            access : uint8_t, unit: Enumerated (Local)'''
        self._name = name
        self._value = value
        self._type = type
        self._access = access


class PlanGeneration(_base.base_message):
    '''An optional list of parameters to be used by the plan generation module.

       This message class contains the following fields and their respective types:
    cmd : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            params : plaintext, unit: TupleList'''

    class CMD(_enum.IntEnum):
        '''Full name: Command
        Prefix: CMD'''
    
        GENERATE = 0
        '''Name: Generate'''
    
        EXECUTE = 1
        '''Name: Execute'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        REQUEST = 0
        '''Name: Request'''
    
        ERROR = 1
        '''Name: Error'''
    
        SUCCESS = 2
        '''Name: Success'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_cmd', '_op', '_plan_id', '_params']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Plan Supervision", id = 562, stable = None, description = "This message is used to order the generation of plans based on id and set of parameters.", fields = ('cmd', 'op', 'plan_id', 'params',), abbrev = "PlanGeneration", name = "Plan Generation", flags = None)

    cmd = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'CMD'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    plan_id = _base.mutable_attr({'name': 'Plan Identifier', 'type': 'plaintext'}, "The name of the plan to be generated.")
    '''The name of the plan to be generated. Type: plaintext'''
    params = _base.mutable_attr({'name': 'Parameters', 'type': 'plaintext', 'unit': 'TupleList'}, "An optional list of parameters to be used by the plan generation module.")
    '''An optional list of parameters to be used by the plan generation module. Type: plaintext'''

    def __init__(self, cmd = None, op = None, plan_id = None, params = None):
        '''Class constructor
        
        An optional list of parameters to be used by the plan generation module.

       This message class contains the following fields and their respective types:
    cmd : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            plan_id : plaintext, unit: NOT FOUND

            params : plaintext, unit: TupleList'''
        self._cmd = cmd
        self._op = op
        self._plan_id = plan_id
        self._params = params


class LeaderState(_base.base_message):
    '''Stream Velocity zz axis velocity component.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            svx : fp32_t, unit: m/s

            svy : fp32_t, unit: m/s

            svz : fp32_t, unit: m/s'''

    class OP(_enum.IntEnum):
        '''Full name: Action on the leader state
        Prefix: OP'''
    
        REQUEST = 0
        '''Name: Request'''
    
        SET = 1
        '''Name: Set'''
    
        REPORT = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_group_name', '_op', '_lat', '_lon', '_height', '_x', '_y', '_z', '_phi', '_theta', '_psi', '_vx', '_vy', '_vz', '_p', '_q', '_r', '_svx', '_svy', '_svz']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Plan Supervision", id = 563, stable = None, description = "This message defines the formation leader state. LeaderState is a complete description of the leader state in terms of parameters such as position, orientation and velocities at a particular moment in time. The system position is given by a North-East-Down (NED) local tangent plane displacement (x, y, z) relative to an absolute WGS-84 coordinate (latitude, longitude, height above ellipsoid). The symbols for position and attitude as well as linear and angular velocities were chosen according to SNAME's notation (1950). The body-fixed reference frame and Euler angles are depicted next: .. figure:: ../images/euler-lauv.png :align: center Euler angles", fields = ('group_name', 'op', 'lat', 'lon', 'height', 'x', 'y', 'z', 'phi', 'theta', 'psi', 'vx', 'vy', 'vz', 'p', 'q', 'r', 'svx', 'svy', 'svz',), abbrev = "LeaderState", name = "Leader State", flags = "periodic")

    group_name = _base.mutable_attr({'name': 'Group Name', 'type': 'plaintext'}, "Name for the formation group.")
    '''Name for the formation group. Type: plaintext'''
    op = _base.mutable_attr({'name': 'Action on the leader state', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the formation leader state variables Enumerated (Local).")
    '''Action on the formation leader state variables Enumerated (Local). Type: uint8_t'''
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
    svx = _base.mutable_attr({'name': 'Stream Velocity X (North)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity xx axis velocity component.")
    '''Stream Velocity xx axis velocity component. Type: fp32_t'''
    svy = _base.mutable_attr({'name': 'Stream Velocity Y (East)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity yy axis velocity component.")
    '''Stream Velocity yy axis velocity component. Type: fp32_t'''
    svz = _base.mutable_attr({'name': 'Stream Velocity Z (Down)', 'type': 'fp32_t', 'unit': 'm/s'}, "Stream Velocity zz axis velocity component.")
    '''Stream Velocity zz axis velocity component. Type: fp32_t'''

    def __init__(self, group_name = None, op = None, lat = None, lon = None, height = None, x = None, y = None, z = None, phi = None, theta = None, psi = None, vx = None, vy = None, vz = None, p = None, q = None, r = None, svx = None, svy = None, svz = None):
        '''Class constructor
        
        Stream Velocity zz axis velocity component.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            op : uint8_t, unit: Enumerated (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            height : fp32_t, unit: m

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            phi : fp32_t, unit: rad

            theta : fp32_t, unit: rad

            psi : fp32_t, unit: rad

            vx : fp32_t, unit: m/s

            vy : fp32_t, unit: m/s

            vz : fp32_t, unit: m/s

            p : fp32_t, unit: rad/s

            q : fp32_t, unit: rad/s

            r : fp32_t, unit: rad/s

            svx : fp32_t, unit: m/s

            svy : fp32_t, unit: m/s

            svz : fp32_t, unit: m/s'''
        self._group_name = group_name
        self._op = op
        self._lat = lat
        self._lon = lon
        self._height = height
        self._x = x
        self._y = y
        self._z = z
        self._phi = phi
        self._theta = theta
        self._psi = psi
        self._vx = vx
        self._vy = vy
        self._vz = vz
        self._p = p
        self._q = q
        self._r = r
        self._svx = svx
        self._svy = svy
        self._svz = svz


class PlanStatistics(_base.base_message):
    '''Amount of fuel spent, in battery percentage, by different parcels (if applicable): Total=35,Hotel=5,Payload=10,Motion=20,IMU=0

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            properties : uint8_t, unit: Bitfield (Local)

            durations : plaintext, unit: TupleList

            distances : plaintext, unit: TupleList

            actions : plaintext, unit: TupleList

            fuel : plaintext, unit: TupleList'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: TP'''
    
        PREPLAN = 0
        '''Name: Before Plan'''
    
        INPLAN = 1
        '''Name: During Plan'''
    
        POSTPLAN = 2
        '''Name: After Plan'''
    
    
    class PROPERTIES(_enum.IntFlag):
        '''Full name: Properties
        Prefix: PRP'''
    
        EMPTY = 0
        '''No active flags'''
    
        BASIC = 0
        '''Name: Basic Plan'''
    
        NONLINEAR = 1
        '''Name: Nonlinear'''
    
        INFINITE = 2
        '''Name: Infinite'''
    
        CYCLICAL = 4
        '''Name: Cyclical'''
    
        ALL = 7
        '''Name: All'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_plan_id', '_type', '_properties', '_durations', '_distances', '_actions', '_fuel']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Plan Supervision", id = 564, stable = None, description = None, fields = ('plan_id', 'type', 'properties', 'durations', 'distances', 'actions', 'fuel',), abbrev = "PlanStatistics", name = "Plan Statistics", flags = None)

    plan_id = _base.mutable_attr({'name': 'Plan Identifier', 'type': 'plaintext'}, "The name of the plan to be generated.")
    '''The name of the plan to be generated. Type: plaintext'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TP'}, "Type of plan statistics, if they are launched before, during or after the plan execution. Enumerated (Local).")
    '''Type of plan statistics, if they are launched before, during or after the plan execution. Enumerated (Local). Type: uint8_t'''
    properties = _base.mutable_attr({'name': 'Properties', 'type': 'uint8_t', 'prefix': 'PRP', 'unit': 'Bitfield'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    durations = _base.mutable_attr({'name': 'Durations', 'type': 'plaintext', 'unit': 'TupleList'}, "Maneuver and plan duration statistics in seconds, for example: Total=1000,Goto1=20,Rows=980")
    '''Maneuver and plan duration statistics in seconds, for example: Total=1000,Goto1=20,Rows=980 Type: plaintext'''
    distances = _base.mutable_attr({'name': 'Distances', 'type': 'plaintext', 'unit': 'TupleList'}, "Distances travelled in meters in each maneuver and/or total: Total=2000,Rows=1800,Elevator=200")
    '''Distances travelled in meters in each maneuver and/or total: Total=2000,Rows=1800,Elevator=200 Type: plaintext'''
    actions = _base.mutable_attr({'name': 'Actions', 'type': 'plaintext', 'unit': 'TupleList'}, "List of components active by plan actions during the plan and time active in seconds: Sidescan=100,Camera Module=150")
    '''List of components active by plan actions during the plan and time active in seconds: Sidescan=100,Camera Module=150 Type: plaintext'''
    fuel = _base.mutable_attr({'name': 'Fuel', 'type': 'plaintext', 'unit': 'TupleList'}, "Amount of fuel spent, in battery percentage, by different parcels (if applicable): Total=35,Hotel=5,Payload=10,Motion=20,IMU=0")
    '''Amount of fuel spent, in battery percentage, by different parcels (if applicable): Total=35,Hotel=5,Payload=10,Motion=20,IMU=0 Type: plaintext'''

    def __init__(self, plan_id = None, type = None, properties = None, durations = None, distances = None, actions = None, fuel = None):
        '''Class constructor
        
        Amount of fuel spent, in battery percentage, by different parcels (if applicable): Total=35,Hotel=5,Payload=10,Motion=20,IMU=0

       This message class contains the following fields and their respective types:
    plan_id : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            properties : uint8_t, unit: Bitfield (Local)

            durations : plaintext, unit: TupleList

            distances : plaintext, unit: TupleList

            actions : plaintext, unit: TupleList

            fuel : plaintext, unit: TupleList'''
        self._plan_id = plan_id
        self._type = type
        self._properties = properties
        self._durations = durations
        self._distances = distances
        self._actions = actions
        self._fuel = fuel


class SoiWaypoint(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    lat : fp32_t, unit: °

            lon : fp32_t, unit: °

            eta : uint32_t, unit: NOT FOUND

            duration : uint16_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_eta', '_duration']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 850, stable = None, description = None, fields = ('lat', 'lon', 'eta', 'duration',), abbrev = "SoiWaypoint", name = "SOI Waypoint", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp32_t', 'unit': '°'}, "No description available")
    '''No description available Type: fp32_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp32_t', 'unit': '°'}, "No description available")
    '''No description available Type: fp32_t'''
    eta = _base.mutable_attr({'name': 'Time Of Arrival', 'type': 'uint32_t'}, "No description available")
    '''No description available Type: uint32_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "No description available")
    '''No description available Type: uint16_t'''

    def __init__(self, lat = None, lon = None, eta = None, duration = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    lat : fp32_t, unit: °

            lon : fp32_t, unit: °

            eta : uint32_t, unit: NOT FOUND

            duration : uint16_t, unit: s'''
        self._lat = lat
        self._lon = lon
        self._eta = eta
        self._duration = duration


class SoiPlan(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    plan_id : uint16_t, unit: NOT FOUND

            waypoints : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_plan_id', '_waypoints']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 851, stable = None, description = None, fields = ('plan_id', 'waypoints',), abbrev = "SoiPlan", name = "SOI Plan", flags = None)

    plan_id = _base.mutable_attr({'name': 'Plan Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    waypoints = _base.mutable_attr({'name': 'Waypoints', 'type': 'message-list', 'message-type': 'SoiWaypoint'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, plan_id = None, waypoints = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    plan_id : uint16_t, unit: NOT FOUND

            waypoints : message-list, unit: NOT FOUND'''
        self._plan_id = plan_id
        self._waypoints = waypoints


class SoiCommand(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            command : uint8_t, unit: Enumerated (Local)

            settings : plaintext, unit: TupleList

            plan : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: SOITYPE'''
    
        REQUEST = 1
        '''Name: Request'''
    
        SUCCESS = 2
        '''Name: Success'''
    
        ERROR = 3
        '''Name: Error'''
    
    
    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: SOICMD'''
    
        EXEC = 1
        '''Name: Execute Plan'''
    
        STOP = 2
        '''Name: Stop Execution'''
    
        SET_PARAMS = 3
        '''Name: Set Parameters'''
    
        GET_PARAMS = 4
        '''Name: Get Parameters'''
    
        GET_PLAN = 5
        '''Name: Get Plan'''
    
        RESUME = 6
        '''Name: Resume Execution'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_command', '_settings', '_plan', '_info']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 852, stable = None, description = None, fields = ('type', 'command', 'settings', 'plan', 'info',), abbrev = "SoiCommand", name = "SOI Command", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'SOITYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    command = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'SOICMD'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    settings = _base.mutable_attr({'name': 'Settings', 'type': 'plaintext', 'unit': 'TupleList'}, "No description available")
    '''No description available Type: plaintext'''
    plan = _base.mutable_attr({'name': 'Plan', 'type': 'message', 'message-type': 'SoiPlan'}, "No description available")
    '''No description available Type: message'''
    info = _base.mutable_attr({'name': 'Extra Information', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, type = None, command = None, settings = None, plan = None, info = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            command : uint8_t, unit: Enumerated (Local)

            settings : plaintext, unit: TupleList

            plan : message, unit: NOT FOUND

            info : plaintext, unit: NOT FOUND'''
        self._type = type
        self._command = command
        self._settings = settings
        self._plan = plan
        self._info = info


class SoiState(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : uint16_t, unit: NOT FOUND

            wpt_id : uint8_t, unit: NOT FOUND

            settings_chk : uint16_t, unit: NOT FOUND'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: SOISTATE'''
    
        EXEC = 1
        '''Name: Executing'''
    
        IDLE = 2
        '''Name: Idle'''
    
        INACTIVE = 3
        '''Name: Inactive'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_plan_id', '_wpt_id', '_settings_chk']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 853, stable = None, description = None, fields = ('state', 'plan_id', 'wpt_id', 'settings_chk',), abbrev = "SoiState", name = "SOI State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'SOISTATE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    plan_id = _base.mutable_attr({'name': 'Plan Identifier', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''
    wpt_id = _base.mutable_attr({'name': 'Waypoint Identifier', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    settings_chk = _base.mutable_attr({'name': 'Settings Checksum', 'type': 'uint16_t'}, "No description available")
    '''No description available Type: uint16_t'''

    def __init__(self, state = None, plan_id = None, wpt_id = None, settings_chk = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            plan_id : uint16_t, unit: NOT FOUND

            wpt_id : uint8_t, unit: NOT FOUND

            settings_chk : uint16_t, unit: NOT FOUND'''
        self._state = state
        self._plan_id = plan_id
        self._wpt_id = wpt_id
        self._settings_chk = settings_chk


class Aborted(_base.base_message):
    '''This message signals that an :ref:`Abort` message was received and acted upon.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Plan Supervision", id = 889, stable = None, description = "This message signals that an :ref:`Abort` message was received and acted upon.", fields = [], abbrev = "Aborted", name = "Aborted", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        This message signals that an :ref:`Abort` message was received and acted upon.

       This message class contains the following fields and their respective types:
'''


'''
IMC Autonomy messages.
'''

from .. import _base
import enum as _enum

class VehicleLinks(_base.base_message):
    '''A list of Announce messages with last announces heard

       This message class contains the following fields and their respective types:
    localname : plaintext, unit: NOT FOUND

            links : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_localname', '_links']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Autonomy", id = 650, stable = None, description = "This message is sent by the TREX task which gives further information to a TREX instance about connected IMC nodes", fields = ('localname', 'links',), abbrev = "VehicleLinks", name = "Vehicle Links", flags = None)

    localname = _base.mutable_attr({'name': 'Local Name', 'type': 'plaintext'}, "The name of the vehicle being controlled")
    '''The name of the vehicle being controlled Type: plaintext'''
    links = _base.mutable_attr({'name': 'Active Links', 'type': 'message-list', 'message-type': 'Announce'}, "A list of Announce messages with last announces heard")
    '''A list of Announce messages with last announces heard Type: message-list'''

    def __init__(self, localname = None, links = None):
        '''Class constructor
        
        A list of Announce messages with last announces heard

       This message class contains the following fields and their respective types:
    localname : plaintext, unit: NOT FOUND

            links : message-list, unit: NOT FOUND'''
        self._localname = localname
        self._links = links


class TrexObservation(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    timeline : plaintext, unit: NOT FOUND

            predicate : plaintext, unit: NOT FOUND

            attributes : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeline', '_predicate', '_attributes']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 651, stable = None, description = "This message is sent to TREX to post timeline observations", fields = ('timeline', 'predicate', 'attributes',), abbrev = "TrexObservation", name = "TREX Observation", flags = "deprecated")

    timeline = _base.mutable_attr({'name': 'Timeline', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    predicate = _base.mutable_attr({'name': 'Predicate', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    attributes = _base.mutable_attr({'name': 'Attributes', 'type': 'plaintext', 'unit': 'TupleList'}, "No description available")
    '''No description available Type: plaintext'''

    def __init__(self, timeline = None, predicate = None, attributes = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    timeline : plaintext, unit: NOT FOUND

            predicate : plaintext, unit: NOT FOUND

            attributes : plaintext, unit: TupleList'''
        self._timeline = timeline
        self._predicate = predicate
        self._attributes = attributes


class TrexCommand(_base.base_message):
    '''The goal encoded as XML, if applicable (OP == POST_GOAL)

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            goal_id : plaintext, unit: NOT FOUND

            goal_xml : plaintext, unit: NOT FOUND'''

    class COMMAND(_enum.IntEnum):
        '''Full name: Command
        Prefix: OP'''
    
        DISABLE = 0
        '''Name: Disable TREX'''
    
        ENABLE = 1
        '''Name: Enable TREX'''
    
        POST_GOAL = 2
        '''Name: Post Goal'''
    
        RECALL_GOAL = 3
        '''Name: Recall Goal'''
    
        REQUEST_PLAN = 4
        '''Name: Request current plan'''
    
        REPORT_PLAN = 5
        '''Name: Report current plan'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_command', '_goal_id', '_goal_xml']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 652, stable = None, description = "This message is used to control TREX execution", fields = ('command', 'goal_id', 'goal_xml',), abbrev = "TrexCommand", name = "TREX Command", flags = "deprecated")

    command = _base.mutable_attr({'name': 'Command', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    goal_id = _base.mutable_attr({'name': 'Goal Id', 'type': 'plaintext'}, "The id of the goal, if applicable (OP == POST_GOAL || OP == RECALL_GOAL)")
    '''The id of the goal, if applicable (OP == POST_GOAL || OP == RECALL_GOAL) Type: plaintext'''
    goal_xml = _base.mutable_attr({'name': 'Goal XML', 'type': 'plaintext'}, "The goal encoded as XML, if applicable (OP == POST_GOAL)")
    '''The goal encoded as XML, if applicable (OP == POST_GOAL) Type: plaintext'''

    def __init__(self, command = None, goal_id = None, goal_xml = None):
        '''Class constructor
        
        The goal encoded as XML, if applicable (OP == POST_GOAL)

       This message class contains the following fields and their respective types:
    command : uint8_t, unit: Enumerated (Local)

            goal_id : plaintext, unit: NOT FOUND

            goal_xml : plaintext, unit: NOT FOUND'''
        self._command = command
        self._goal_id = goal_id
        self._goal_xml = goal_xml


class TrexOperation(_base.base_message):
    '''Goal / observation to post, if applicable (OP == POST_GOAL || OP == POST_TOKEN)

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            goal_id : plaintext, unit: NOT FOUND

            token : message, unit: NOT FOUND'''

    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        POST_TOKEN = 1
        '''Name: Post Token'''
    
        POST_GOAL = 2
        '''Name: Post Goal'''
    
        RECALL_GOAL = 3
        '''Name: Recall Goal'''
    
        REQUEST_PLAN = 4
        '''Name: Request current plan'''
    
        REPORT_PLAN = 5
        '''Name: Report current plan'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_op', '_goal_id', '_token']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 655, stable = None, description = "This message is used to control TREX execution", fields = ('op', 'goal_id', 'token',), abbrev = "TrexOperation", name = "TREX Operation", flags = None)

    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    goal_id = _base.mutable_attr({'name': 'Goal Id', 'type': 'plaintext'}, "The id of the goal, if applicable (OP == POST_GOAL || OP == RECALL_GOAL)")
    '''The id of the goal, if applicable (OP == POST_GOAL || OP == RECALL_GOAL) Type: plaintext'''
    token = _base.mutable_attr({'name': 'Token', 'type': 'message', 'message-type': 'TrexToken'}, "Goal / observation to post, if applicable (OP == POST_GOAL || OP == POST_TOKEN)")
    '''Goal / observation to post, if applicable (OP == POST_GOAL || OP == POST_TOKEN) Type: message'''

    def __init__(self, op = None, goal_id = None, token = None):
        '''Class constructor
        
        Goal / observation to post, if applicable (OP == POST_GOAL || OP == POST_TOKEN)

       This message class contains the following fields and their respective types:
    op : uint8_t, unit: Enumerated (Local)

            goal_id : plaintext, unit: NOT FOUND

            token : message, unit: NOT FOUND'''
        self._op = op
        self._goal_id = goal_id
        self._token = token


class TrexAttribute(_base.base_message):
    '''Upper bound of this interval. Empty text means no bound.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            attr_type : uint8_t, unit: Enumerated (Local)

            min : plaintext, unit: NOT FOUND

            max : plaintext, unit: NOT FOUND'''

    class ATTR_TYPE(_enum.IntEnum):
        '''Full name: Attribute type
        Prefix: TYPE'''
    
        BOOL = 1
        '''Name: Boolean Domain'''
    
        INT = 2
        '''Name: Integer Domain'''
    
        FLOAT = 3
        '''Name: Float Domain'''
    
        STRING = 4
        '''Name: String Domain'''
    
        ENUM = 5
        '''Name: Enumerated Domain'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_name', '_attr_type', '_min', '_max']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 656, stable = None, description = None, fields = ('name', 'attr_type', 'min', 'max',), abbrev = "TrexAttribute", name = "TREX Attribute", flags = None)

    name = _base.mutable_attr({'name': 'Attribute Name', 'type': 'plaintext'}, "Name of this attribute.")
    '''Name of this attribute. Type: plaintext'''
    attr_type = _base.mutable_attr({'name': 'Attribute type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'TYPE'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    min = _base.mutable_attr({'name': 'Minimum', 'type': 'plaintext'}, "Lower bound of this interval. Empty text means no bound.")
    '''Lower bound of this interval. Empty text means no bound. Type: plaintext'''
    max = _base.mutable_attr({'name': 'Maximum', 'type': 'plaintext'}, "Upper bound of this interval. Empty text means no bound.")
    '''Upper bound of this interval. Empty text means no bound. Type: plaintext'''

    def __init__(self, name = None, attr_type = None, min = None, max = None):
        '''Class constructor
        
        Upper bound of this interval. Empty text means no bound.

       This message class contains the following fields and their respective types:
    name : plaintext, unit: NOT FOUND

            attr_type : uint8_t, unit: Enumerated (Local)

            min : plaintext, unit: NOT FOUND

            max : plaintext, unit: NOT FOUND'''
        self._name = name
        self._attr_type = attr_type
        self._min = min
        self._max = max


class TrexToken(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    timeline : plaintext, unit: NOT FOUND

            predicate : plaintext, unit: NOT FOUND

            attributes : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeline', '_predicate', '_attributes']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 657, stable = None, description = None, fields = ('timeline', 'predicate', 'attributes',), abbrev = "TrexToken", name = "TREX Token", flags = None)

    timeline = _base.mutable_attr({'name': 'Timeline', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    predicate = _base.mutable_attr({'name': 'Predicate', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    attributes = _base.mutable_attr({'name': 'Attributes', 'type': 'message-list', 'message-type': 'TrexAttribute'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, timeline = None, predicate = None, attributes = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    timeline : plaintext, unit: NOT FOUND

            predicate : plaintext, unit: NOT FOUND

            attributes : message-list, unit: NOT FOUND'''
        self._timeline = timeline
        self._predicate = predicate
        self._attributes = attributes


class TrexPlan(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    reactor : plaintext, unit: NOT FOUND

            tokens : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_reactor', '_tokens']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 658, stable = None, description = None, fields = ('reactor', 'tokens',), abbrev = "TrexPlan", name = "TREX Plan", flags = None)

    reactor = _base.mutable_attr({'name': 'Reactor name', 'type': 'plaintext'}, "No description available")
    '''No description available Type: plaintext'''
    tokens = _base.mutable_attr({'name': 'Tokens', 'type': 'message-list', 'message-type': 'TrexToken'}, "No description available")
    '''No description available Type: message-list'''

    def __init__(self, reactor = None, tokens = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    reactor : plaintext, unit: NOT FOUND

            tokens : message-list, unit: NOT FOUND'''
        self._reactor = reactor
        self._tokens = tokens


class Event(_base.base_message):
    '''A map with additional event information.

       This message class contains the following fields and their respective types:
    topic : plaintext, unit: NOT FOUND

            data : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_topic', '_data']
    Attributes = _base.MessageAttributes(source = "vehicle,ccu", usedby = None, category = "Autonomy", id = 660, stable = None, description = "This message is used for signaling asynchronous events between different (sub) systems.", fields = ('topic', 'data',), abbrev = "Event", name = "Event", flags = None)

    topic = _base.mutable_attr({'name': 'Topic', 'type': 'plaintext'}, "The name or type of this event")
    '''The name or type of this event Type: plaintext'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'plaintext', 'unit': 'TupleList'}, "A map with additional event information.")
    '''A map with additional event information. Type: plaintext'''

    def __init__(self, topic = None, data = None):
        '''Class constructor
        
        A map with additional event information.

       This message class contains the following fields and their respective types:
    topic : plaintext, unit: NOT FOUND

            data : plaintext, unit: TupleList'''
        self._topic = topic
        self._data = data


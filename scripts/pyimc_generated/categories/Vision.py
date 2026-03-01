'''
IMC Vision messages.
'''

from .. import _base
import enum as _enum

class CompressedImage(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    frameid : uint8_t, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_frameid', '_data']
    Attributes = _base.MessageAttributes(source = "ccu, vehicle", usedby = None, category = "Vision", id = 702, stable = None, description = "", fields = ('frameid', 'data',), abbrev = "CompressedImage", name = "Compressed Image", flags = None)

    frameid = _base.mutable_attr({'name': 'Frame Id', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    data = _base.mutable_attr({'name': 'Data', 'type': 'rawdata'}, "No description available")
    '''No description available Type: rawdata'''

    def __init__(self, frameid = None, data = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    frameid : uint8_t, unit: NOT FOUND

            data : rawdata, unit: NOT FOUND'''
        self._frameid = frameid
        self._data = data


class ImageTxSettings(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    fps : uint8_t, unit: NOT FOUND

            quality : uint8_t, unit: NOT FOUND

            reps : uint8_t, unit: NOT FOUND

            tsize : uint8_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_fps', '_quality', '_reps', '_tsize']
    Attributes = _base.MessageAttributes(source = "ccu, vehicle", usedby = None, category = "Vision", id = 703, stable = None, description = "", fields = ('fps', 'quality', 'reps', 'tsize',), abbrev = "ImageTxSettings", name = "Image Transmission Settings", flags = None)

    fps = _base.mutable_attr({'name': 'Frames Per Second', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    quality = _base.mutable_attr({'name': 'Quality', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    reps = _base.mutable_attr({'name': 'Repetitions', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''
    tsize = _base.mutable_attr({'name': 'Target Size', 'type': 'uint8_t'}, "No description available")
    '''No description available Type: uint8_t'''

    def __init__(self, fps = None, quality = None, reps = None, tsize = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    fps : uint8_t, unit: NOT FOUND

            quality : uint8_t, unit: NOT FOUND

            reps : uint8_t, unit: NOT FOUND

            tsize : uint8_t, unit: NOT FOUND'''
        self._fps = fps
        self._quality = quality
        self._reps = reps
        self._tsize = tsize


class SetImageCoords(_base.base_message):
    '''Y coordinate of the target in the image frame.

       This message class contains the following fields and their respective types:
    camId : uint8_t, unit: NOT FOUND

            x : uint16_t, unit: px

            y : uint16_t, unit: px'''

    __slots__ = ['_Attributes', '_header', '_footer', '_camId', '_x', '_y']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vision", id = 895, stable = None, description = "Message containing the x and y coordinates of object to track in remote peer.", fields = ('camId', 'x', 'y',), abbrev = "SetImageCoords", name = "Set Image Coordinates", flags = None)

    camId = _base.mutable_attr({'name': 'Camera Identifier', 'type': 'uint8_t'}, "Camera identifier.")
    '''Camera identifier. Type: uint8_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'uint16_t', 'unit': 'px'}, "X coordinate of the target in the image frame.")
    '''X coordinate of the target in the image frame. Type: uint16_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'uint16_t', 'unit': 'px'}, "Y coordinate of the target in the image frame.")
    '''Y coordinate of the target in the image frame. Type: uint16_t'''

    def __init__(self, camId = None, x = None, y = None):
        '''Class constructor
        
        Y coordinate of the target in the image frame.

       This message class contains the following fields and their respective types:
    camId : uint8_t, unit: NOT FOUND

            x : uint16_t, unit: px

            y : uint16_t, unit: px'''
        self._camId = camId
        self._x = x
        self._y = y


class GetImageCoords(_base.base_message):
    '''Y coordinate of the target in the image frame.

       This message class contains the following fields and their respective types:
    camId : uint8_t, unit: NOT FOUND

            x : uint16_t, unit: px

            y : uint16_t, unit: px'''

    __slots__ = ['_Attributes', '_header', '_footer', '_camId', '_x', '_y']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vision", id = 896, stable = None, description = "Message containing the x and y coordinates of object to track in image slave.", fields = ('camId', 'x', 'y',), abbrev = "GetImageCoords", name = "Get Image Coordinates", flags = None)

    camId = _base.mutable_attr({'name': 'Camera Identifier', 'type': 'uint8_t'}, "Camera identifier.")
    '''Camera identifier. Type: uint8_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'uint16_t', 'unit': 'px'}, "X coordinate of the target in the image frame.")
    '''X coordinate of the target in the image frame. Type: uint16_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'uint16_t', 'unit': 'px'}, "Y coordinate of the target in the image frame.")
    '''Y coordinate of the target in the image frame. Type: uint16_t'''

    def __init__(self, camId = None, x = None, y = None):
        '''Class constructor
        
        Y coordinate of the target in the image frame.

       This message class contains the following fields and their respective types:
    camId : uint8_t, unit: NOT FOUND

            x : uint16_t, unit: px

            y : uint16_t, unit: px'''
        self._camId = camId
        self._x = x
        self._y = y


class GetWorldCoordinates(_base.base_message):
    '''Z offsets of the target in the real world frame.

       This message class contains the following fields and their respective types:
    tracking : uint8_t, unit: Enumerated (Global)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_tracking', '_lat', '_lon', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Vision", id = 897, stable = None, description = "Message containing the x, y and z coordinates of object in the real world.", fields = ('tracking', 'lat', 'lon', 'x', 'y', 'z',), abbrev = "GetWorldCoordinates", name = "Get World Coordinates", flags = None)

    tracking = _base.mutable_attr({'name': 'Tracking', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'Boolean'}, "True when system is tracking. Enumerated (Global).")
    '''True when system is tracking. Enumerated (Global). Type: uint8_t'''
    lat = _base.mutable_attr({'name': 'Latitude', 'type': 'fp64_t', 'unit': 'rad'}, "Latitude of the real world frame origin.")
    '''Latitude of the real world frame origin. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude', 'type': 'fp64_t', 'unit': 'rad'}, "Longitude of the real world frame origin.")
    '''Longitude of the real world frame origin. Type: fp64_t'''
    x = _base.mutable_attr({'name': 'X', 'type': 'fp32_t', 'unit': 'm'}, "X offsets of the target in the real world frame.")
    '''X offsets of the target in the real world frame. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Y', 'type': 'fp32_t', 'unit': 'm'}, "Y offsets of the target in the real world frame.")
    '''Y offsets of the target in the real world frame. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp32_t', 'unit': 'm'}, "Z offsets of the target in the real world frame.")
    '''Z offsets of the target in the real world frame. Type: fp32_t'''

    def __init__(self, tracking = None, lat = None, lon = None, x = None, y = None, z = None):
        '''Class constructor
        
        Z offsets of the target in the real world frame.

       This message class contains the following fields and their respective types:
    tracking : uint8_t, unit: Enumerated (Global)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''
        self._tracking = tracking
        self._lat = lat
        self._lon = lon
        self._x = x
        self._y = y
        self._z = z


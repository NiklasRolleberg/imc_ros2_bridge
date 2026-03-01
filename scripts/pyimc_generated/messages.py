'''
IMC messages.
'''

from . import _base
import enum as _enum
import pyimclsts.core as _core
from . import categories as _categories
from typing import Optional, Any

_message_ids = {1: 'EntityState', 2: 'QueryEntityState', 3: 'EntityInfo', 4: 'QueryEntityInfo', 5: 'EntityList', 7: 'CpuUsage', 8: 'TransportBindings', 9: 'RestartSystem', 12: 'DevCalibrationControl', 13: 'DevCalibrationState', 14: 'EntityActivationState', 15: 'QueryEntityActivationState', 16: 'VehicleOperationalLimits', 20: 'MsgList', 50: 'SimulatedState', 51: 'LeakSimulation', 52: 'UASimulation', 53: 'DynamicsSimParam', 100: 'StorageUsage', 101: 'CacheControl', 102: 'LoggingControl', 103: 'LogBookEntry', 104: 'LogBookControl', 105: 'ReplayControl', 106: 'ClockControl', 107: 'HistoricCTD', 108: 'HistoricTelemetry', 109: 'HistoricSonarData', 110: 'HistoricEvent', 111: 'VerticalProfile', 112: 'ProfileSample', 150: 'Heartbeat', 151: 'Announce', 152: 'AnnounceService', 153: 'RSSI', 154: 'VSWR', 155: 'LinkLevel', 156: 'Sms', 157: 'SmsTx', 158: 'SmsRx', 159: 'SmsState', 160: 'TextMessage', 170: 'IridiumMsgRx', 171: 'IridiumMsgTx', 172: 'IridiumTxStatus', 180: 'GroupMembershipState', 181: 'SystemGroup', 182: 'LinkLatency', 183: 'ExtendedRSSI', 184: 'HistoricData', 185: 'CompressedHistory', 186: 'HistoricSample', 187: 'HistoricDataQuery', 188: 'RemoteCommand', 189: 'CommSystemsQuery', 190: 'TelemetryMsg', 200: 'LblRange', 202: 'LblBeacon', 203: 'LblConfig', 206: 'AcousticMessage', 207: 'SimAcousticMessage', 211: 'AcousticOperation', 212: 'AcousticSystemsQuery', 213: 'AcousticSystems', 214: 'AcousticLink', 215: 'AcousticRequest', 216: 'AcousticStatus', 217: 'AcousticRelease', 250: 'Rpm', 251: 'Voltage', 252: 'Current', 253: 'GpsFix', 254: 'EulerAngles', 255: 'EulerAnglesDelta', 256: 'AngularVelocity', 257: 'Acceleration', 258: 'MagneticField', 259: 'GroundVelocity', 260: 'WaterVelocity', 261: 'VelocityDelta', 262: 'Distance', 263: 'Temperature', 264: 'Pressure', 265: 'Depth', 266: 'DepthOffset', 267: 'SoundSpeed', 268: 'WaterDensity', 269: 'Conductivity', 270: 'Salinity', 271: 'WindSpeed', 272: 'RelativeHumidity', 273: 'DevDataText', 274: 'DevDataBinary', 275: 'Force', 276: 'SonarData', 277: 'Pulse', 278: 'PulseDetectionControl', 279: 'FuelLevel', 280: 'GpsNavData', 281: 'ServoPosition', 282: 'DeviceState', 283: 'BeamConfig', 284: 'DataSanity', 285: 'RhodamineDye', 286: 'CrudeOil', 287: 'FineOil', 288: 'Turbidity', 289: 'Chlorophyll', 290: 'Fluorescein', 291: 'Phycocyanin', 292: 'Phycoerythrin', 293: 'GpsFixRtk', 294: 'ExternalNavData', 295: 'DissolvedOxygen', 296: 'AirSaturation', 297: 'Throttle', 298: 'PH', 299: 'Redox', 300: 'CameraZoom', 301: 'SetThrusterActuation', 302: 'SetServoPosition', 303: 'SetControlSurfaceDeflection', 304: 'RemoteActionsRequest', 305: 'RemoteActions', 306: 'ButtonEvent', 307: 'LcdControl', 308: 'PowerOperation', 309: 'PowerChannelControl', 310: 'QueryPowerChannelState', 311: 'PowerChannelState', 312: 'LedBrightness', 313: 'QueryLedBrightness', 314: 'SetLedBrightness', 315: 'SetPWM', 316: 'PWM', 350: 'EstimatedState', 351: 'EstimatedStreamVelocity', 352: 'IndicatedSpeed', 353: 'TrueSpeed', 354: 'NavigationUncertainty', 355: 'NavigationData', 356: 'GpsFixRejection', 357: 'LblRangeAcceptance', 358: 'DvlRejection', 360: 'LblEstimate', 361: 'AlignmentState', 362: 'GroupStreamVelocity', 363: 'Airflow', 400: 'DesiredHeading', 401: 'DesiredZ', 402: 'DesiredSpeed', 403: 'DesiredRoll', 404: 'DesiredPitch', 405: 'DesiredVerticalRate', 406: 'DesiredPath', 407: 'DesiredControl', 408: 'DesiredHeadingRate', 409: 'DesiredVelocity', 410: 'PathControlState', 411: 'AllocatedControlTorques', 412: 'ControlParcel', 413: 'Brake', 414: 'DesiredLinearState', 415: 'DesiredThrottle', 450: 'Goto', 451: 'PopUp', 452: 'Teleoperation', 453: 'Loiter', 454: 'IdleManeuver', 455: 'LowLevelControl', 456: 'Rows', 457: 'FollowPath', 458: 'PathPoint', 459: 'YoYo', 460: 'TeleoperationDone', 461: 'StationKeeping', 462: 'Elevator', 463: 'FollowTrajectory', 464: 'TrajectoryPoint', 465: 'CustomManeuver', 466: 'VehicleFormation', 467: 'VehicleFormationParticipant', 468: 'StopManeuver', 469: 'RegisterManeuver', 470: 'ManeuverControlState', 471: 'FollowSystem', 472: 'CommsRelay', 473: 'CoverArea', 474: 'PolygonVertex', 475: 'CompassCalibration', 476: 'FormationParameters', 477: 'FormationPlanExecution', 478: 'FollowReference', 479: 'Reference', 480: 'FollowRefState', 481: 'FormationMonitor', 482: 'RelativeState', 483: 'Dislodge', 484: 'Formation', 485: 'Launch', 486: 'Drop', 487: 'ScheduledGoto', 488: 'RowsCoverage', 489: 'Sample', 490: 'ImageTracking', 491: 'Takeoff', 492: 'Land', 493: 'AutonomousSection', 494: 'FollowPoint', 495: 'Alignment', 496: 'StationKeepingExtended', 497: 'ManeuverDone', 499: 'Magnetometer', 500: 'VehicleState', 501: 'VehicleCommand', 502: 'MonitorEntityState', 503: 'EntityMonitoringState', 504: 'OperationalLimits', 505: 'GetOperationalLimits', 506: 'Calibration', 507: 'ControlLoops', 508: 'VehicleMedium', 509: 'Collision', 510: 'FormState', 511: 'AutopilotMode', 512: 'FormationState', 513: 'ReportControl', 514: 'StateReport', 515: 'TransmissionRequest', 516: 'TransmissionStatus', 517: 'SmsRequest', 518: 'SmsStatus', 519: 'VtolState', 520: 'ArmingState', 521: 'TCPRequest', 522: 'TCPStatus', 525: 'AssetReport', 550: 'Abort', 551: 'PlanSpecification', 552: 'PlanManeuver', 553: 'PlanTransition', 554: 'EmergencyControl', 555: 'EmergencyControlState', 556: 'PlanDB', 557: 'PlanDBState', 558: 'PlanDBInformation', 559: 'PlanControl', 560: 'PlanControlState', 561: 'PlanVariable', 562: 'PlanGeneration', 563: 'LeaderState', 564: 'PlanStatistics', 600: 'ReportedState', 601: 'RemoteSensorInfo', 602: 'Map', 603: 'MapFeature', 604: 'MapPoint', 606: 'CcuEvent', 650: 'VehicleLinks', 651: 'TrexObservation', 652: 'TrexCommand', 655: 'TrexOperation', 656: 'TrexAttribute', 657: 'TrexToken', 658: 'TrexPlan', 660: 'Event', 702: 'CompressedImage', 703: 'ImageTxSettings', 750: 'RemoteState', 800: 'Target', 801: 'EntityParameter', 802: 'EntityParameters', 803: 'QueryEntityParameters', 804: 'SetEntityParameters', 805: 'SaveEntityParameters', 806: 'CreateSession', 807: 'CloseSession', 808: 'SessionSubscription', 809: 'SessionKeepAlive', 810: 'SessionStatus', 811: 'PushEntityParameters', 812: 'PopEntityParameters', 813: 'IoEvent', 814: 'UamTxFrame', 815: 'UamRxFrame', 816: 'UamTxStatus', 817: 'UamRxRange', 818: 'UamTxRange', 820: 'FormCtrlParam', 821: 'FormationEval', 822: 'FormationControlParams', 823: 'FormationEvaluation', 850: 'SoiWaypoint', 851: 'SoiPlan', 852: 'SoiCommand', 853: 'SoiState', 877: 'MessagePart', 888: 'NeptusBlob', 889: 'Aborted', 890: 'UsblAngles', 891: 'UsblPosition', 892: 'UsblFix', 893: 'ParametersXml', 894: 'GetParametersXml', 895: 'SetImageCoords', 896: 'GetImageCoords', 897: 'GetWorldCoordinates', 898: 'UsblAnglesExtended', 899: 'UsblPositionExtended', 900: 'UsblFixExtended', 901: 'UsblModem', 902: 'UsblConfig', 903: 'DissolvedOrganicMatter', 904: 'OpticalBackscatter', 905: 'Tachograph', 906: 'ApmStatus', 907: 'SadcReadings', 908: 'DmsDetection', 909: 'HomePosition', 1014: 'CurrentProfile', 1015: 'CurrentProfileCell', 1016: 'ADCPBeam', 2000: 'GpioState', 2001: 'GpioStateGet', 2002: 'GpioStateSet', 2003: 'ColoredDissolvedOrganicMatter', 2004: 'FluorescentDissolvedOrganicMatter', 2006: 'TotalMagIntensity', 2010: 'CommRestriction'}

# Re-export:
IMC_message = _core.IMC_message

class Unknown(_base.base_message):
    '''A (received) message whose format is not known. It has valid sync number and valid CRC but could not be parsed.

    Contains a byte string field named 'contents' and a boolean value that indicates if its big or little endian.
    On serialization, it forces the original endianness, regardless of what the user chose to use.
    '''

    __slots__ = ['_Attributes', '_header', '_footer', '_contents', '_endianness']
    Attributes = _base.MessageAttributes(fields = ('contents','endianness', ), name = "Unknown", id = id, abbrev = "Unknown", description = "A (received) message whose format is not known. It has valid sync number and valid CRC but could not be parsed.", source= None, usedby= None, category= None, stable= None, flags= None)
    contents = _base.mutable_attr({'name': 'Contents', 'type': 'rawdata'}, "contents")
    endianness = _base.mutable_attr({'name': 'endianness', 'type': 'rawdata'}, "endianness")

    def __init__(self, id, contents : bytes, endianness : bool):
        '''Class constructor
        
        A (received) message whose format is not known. It has valid sync number and valid CRC but could not be parsed.

        This message class contains the following fields and their respective types:
        contents : rawdata, unit: NOT FOUND
        '''
        self._contents : bytes = contents
        self._endianness : bool = endianness
    
    def _pack_fields(self, *, serial_functions : dict) -> bytes:
        raise NotImplemented

    def pack(self, *, is_field_message : bool = False, is_big_endian : bool = True, src : Optional[int] = None, src_ent : Optional[int] = None, 
                        dst : Optional[int] = None, dst_ent : Optional[int] = None) -> bytes:
        '''Serialize function that optionally overwrites the header, if parameters are provided.'''
        
        serial_functions = _core.pack_functions_big if self._endianness else _core.pack_functions_little
        
        s_fields = self._contents
        
        if not is_field_message:
        
            s_header = self._pack_header(serial_functions=serial_functions, size=(len(s_fields)), src=src, src_ent=src_ent, dst=dst, dst_ent=dst_ent)
            s_message = s_header + s_fields
            
            # footer:
            '''Calculates CRC-16 IBM of a bit string'''
            self._footer = _core.CRC16IMB(s_message)
            s_message = s_message + serial_functions['uint16_t'](self._footer)

            return s_message
        return serial_functions['uint16_t'](self._Attributes.id) + s_fields

        

EntityState = _categories.Core.EntityState

QueryEntityState = _categories.Core.QueryEntityState

EntityInfo = _categories.Core.EntityInfo

QueryEntityInfo = _categories.Core.QueryEntityInfo

EntityList = _categories.Core.EntityList

CpuUsage = _categories.Core.CpuUsage

TransportBindings = _categories.Core.TransportBindings

RestartSystem = _categories.Core.RestartSystem

DevCalibrationControl = _categories.Core.DevCalibrationControl

DevCalibrationState = _categories.Core.DevCalibrationState

EntityActivationState = _categories.Core.EntityActivationState

QueryEntityActivationState = _categories.Core.QueryEntityActivationState

VehicleOperationalLimits = _categories.Core.VehicleOperationalLimits

MsgList = _categories.Core.MsgList

SimulatedState = _categories.Simulation.SimulatedState

LeakSimulation = _categories.Simulation.LeakSimulation

UASimulation = _categories.Simulation.UASimulation

DynamicsSimParam = _categories.Simulation.DynamicsSimParam

StorageUsage = _categories.Storage.StorageUsage

CacheControl = _categories.Storage.CacheControl

LoggingControl = _categories.Storage.LoggingControl

LogBookEntry = _categories.Storage.LogBookEntry

LogBookControl = _categories.Storage.LogBookControl

ReplayControl = _categories.Storage.ReplayControl

ClockControl = _categories.Storage.ClockControl

HistoricCTD = _categories.Storage.HistoricCTD

HistoricTelemetry = _categories.Storage.HistoricTelemetry

HistoricSonarData = _categories.Storage.HistoricSonarData

HistoricEvent = _categories.Storage.HistoricEvent

VerticalProfile = _categories.Storage.VerticalProfile

ProfileSample = _categories.Storage.ProfileSample

Heartbeat = _categories.Networking.Heartbeat

Announce = _categories.Networking.Announce

AnnounceService = _categories.Networking.AnnounceService

RSSI = _categories.Networking.RSSI

VSWR = _categories.Networking.VSWR

LinkLevel = _categories.Networking.LinkLevel

Sms = _categories.Networking.Sms

SmsTx = _categories.Networking.SmsTx

SmsRx = _categories.Networking.SmsRx

SmsState = _categories.Networking.SmsState

TextMessage = _categories.Networking.TextMessage

IridiumMsgRx = _categories.Networking.IridiumMsgRx

IridiumMsgTx = _categories.Networking.IridiumMsgTx

IridiumTxStatus = _categories.Networking.IridiumTxStatus

GroupMembershipState = _categories.Networking.GroupMembershipState

SystemGroup = _categories.Networking.SystemGroup

LinkLatency = _categories.Networking.LinkLatency

ExtendedRSSI = _categories.Networking.ExtendedRSSI

HistoricData = _categories.Networking.HistoricData

CompressedHistory = _categories.Networking.CompressedHistory

HistoricSample = _categories.Networking.HistoricSample

HistoricDataQuery = _categories.Networking.HistoricDataQuery

RemoteCommand = _categories.Networking.RemoteCommand

CommSystemsQuery = _categories.Networking.CommSystemsQuery

TelemetryMsg = _categories.Networking.TelemetryMsg

LblRange = _categories.AcousticCommunication.LblRange

LblBeacon = _categories.AcousticCommunication.LblBeacon

LblConfig = _categories.AcousticCommunication.LblConfig

AcousticMessage = _categories.AcousticCommunication.AcousticMessage

SimAcousticMessage = _categories.AcousticCommunication.SimAcousticMessage

AcousticOperation = _categories.AcousticCommunication.AcousticOperation

AcousticSystemsQuery = _categories.AcousticCommunication.AcousticSystemsQuery

AcousticSystems = _categories.AcousticCommunication.AcousticSystems

AcousticLink = _categories.AcousticCommunication.AcousticLink

AcousticRequest = _categories.AcousticCommunication.AcousticRequest

AcousticStatus = _categories.AcousticCommunication.AcousticStatus

AcousticRelease = _categories.AcousticCommunication.AcousticRelease

Rpm = _categories.Sensors.Rpm

Voltage = _categories.Sensors.Voltage

Current = _categories.Sensors.Current

GpsFix = _categories.Sensors.GpsFix

EulerAngles = _categories.Sensors.EulerAngles

EulerAnglesDelta = _categories.Sensors.EulerAnglesDelta

AngularVelocity = _categories.Sensors.AngularVelocity

Acceleration = _categories.Sensors.Acceleration

MagneticField = _categories.Sensors.MagneticField

GroundVelocity = _categories.Sensors.GroundVelocity

WaterVelocity = _categories.Sensors.WaterVelocity

VelocityDelta = _categories.Sensors.VelocityDelta

Distance = _categories.Sensors.Distance

Temperature = _categories.Sensors.Temperature

Pressure = _categories.Sensors.Pressure

Depth = _categories.Sensors.Depth

DepthOffset = _categories.Sensors.DepthOffset

SoundSpeed = _categories.Sensors.SoundSpeed

WaterDensity = _categories.Sensors.WaterDensity

Conductivity = _categories.Sensors.Conductivity

Salinity = _categories.Sensors.Salinity

WindSpeed = _categories.Sensors.WindSpeed

RelativeHumidity = _categories.Sensors.RelativeHumidity

DevDataText = _categories.Sensors.DevDataText

DevDataBinary = _categories.Sensors.DevDataBinary

Force = _categories.Sensors.Force

SonarData = _categories.Sensors.SonarData

Pulse = _categories.Sensors.Pulse

PulseDetectionControl = _categories.Sensors.PulseDetectionControl

FuelLevel = _categories.Sensors.FuelLevel

GpsNavData = _categories.Sensors.GpsNavData

ServoPosition = _categories.Sensors.ServoPosition

DeviceState = _categories.Sensors.DeviceState

BeamConfig = _categories.Sensors.BeamConfig

DataSanity = _categories.Sensors.DataSanity

RhodamineDye = _categories.Sensors.RhodamineDye

CrudeOil = _categories.Sensors.CrudeOil

FineOil = _categories.Sensors.FineOil

Turbidity = _categories.Sensors.Turbidity

Chlorophyll = _categories.Sensors.Chlorophyll

Fluorescein = _categories.Sensors.Fluorescein

Phycocyanin = _categories.Sensors.Phycocyanin

Phycoerythrin = _categories.Sensors.Phycoerythrin

GpsFixRtk = _categories.Sensors.GpsFixRtk

ExternalNavData = _categories.Sensors.ExternalNavData

DissolvedOxygen = _categories.Sensors.DissolvedOxygen

AirSaturation = _categories.Sensors.AirSaturation

Throttle = _categories.Sensors.Throttle

PH = _categories.Sensors.PH

Redox = _categories.Sensors.Redox

CameraZoom = _categories.Actuation.CameraZoom

SetThrusterActuation = _categories.Actuation.SetThrusterActuation

SetServoPosition = _categories.Actuation.SetServoPosition

SetControlSurfaceDeflection = _categories.Actuation.SetControlSurfaceDeflection

RemoteActionsRequest = _categories.Actuation.RemoteActionsRequest

RemoteActions = _categories.Actuation.RemoteActions

ButtonEvent = _categories.Actuation.ButtonEvent

LcdControl = _categories.Actuation.LcdControl

PowerOperation = _categories.Actuation.PowerOperation

PowerChannelControl = _categories.Actuation.PowerChannelControl

QueryPowerChannelState = _categories.Actuation.QueryPowerChannelState

PowerChannelState = _categories.Actuation.PowerChannelState

LedBrightness = _categories.Actuation.LedBrightness

QueryLedBrightness = _categories.Actuation.QueryLedBrightness

SetLedBrightness = _categories.Actuation.SetLedBrightness

SetPWM = _categories.Actuation.SetPWM

PWM = _categories.Actuation.PWM

EstimatedState = _categories.Navigation.EstimatedState

EstimatedStreamVelocity = _categories.Navigation.EstimatedStreamVelocity

IndicatedSpeed = _categories.Navigation.IndicatedSpeed

TrueSpeed = _categories.Navigation.TrueSpeed

NavigationUncertainty = _categories.Navigation.NavigationUncertainty

NavigationData = _categories.Navigation.NavigationData

GpsFixRejection = _categories.Navigation.GpsFixRejection

LblRangeAcceptance = _categories.Navigation.LblRangeAcceptance

DvlRejection = _categories.Navigation.DvlRejection

LblEstimate = _categories.Navigation.LblEstimate

AlignmentState = _categories.Navigation.AlignmentState

GroupStreamVelocity = _categories.Navigation.GroupStreamVelocity

Airflow = _categories.Navigation.Airflow

DesiredHeading = _categories.Guidance.DesiredHeading

DesiredZ = _categories.Guidance.DesiredZ

DesiredSpeed = _categories.Guidance.DesiredSpeed

DesiredRoll = _categories.Guidance.DesiredRoll

DesiredPitch = _categories.Guidance.DesiredPitch

DesiredVerticalRate = _categories.Guidance.DesiredVerticalRate

DesiredPath = _categories.Guidance.DesiredPath

DesiredControl = _categories.Guidance.DesiredControl

DesiredHeadingRate = _categories.Guidance.DesiredHeadingRate

DesiredVelocity = _categories.Guidance.DesiredVelocity

PathControlState = _categories.Guidance.PathControlState

AllocatedControlTorques = _categories.Guidance.AllocatedControlTorques

ControlParcel = _categories.Guidance.ControlParcel

Brake = _categories.Guidance.Brake

DesiredLinearState = _categories.Guidance.DesiredLinearState

DesiredThrottle = _categories.Guidance.DesiredThrottle

Goto = _categories.Maneuvering.Goto

PopUp = _categories.Maneuvering.PopUp

Teleoperation = _categories.Maneuvering.Teleoperation

Loiter = _categories.Maneuvering.Loiter

IdleManeuver = _categories.Maneuvering.IdleManeuver

LowLevelControl = _categories.Maneuvering.LowLevelControl

Rows = _categories.Maneuvering.Rows

FollowPath = _categories.Maneuvering.FollowPath

PathPoint = _categories.Maneuvering.PathPoint

YoYo = _categories.Maneuvering.YoYo

TeleoperationDone = _categories.Maneuvering.TeleoperationDone

StationKeeping = _categories.Maneuvering.StationKeeping

Elevator = _categories.Maneuvering.Elevator

FollowTrajectory = _categories.Maneuvering.FollowTrajectory

TrajectoryPoint = _categories.Maneuvering.TrajectoryPoint

CustomManeuver = _categories.Maneuvering.CustomManeuver

VehicleFormation = _categories.Maneuvering.VehicleFormation

VehicleFormationParticipant = _categories.Maneuvering.VehicleFormationParticipant

StopManeuver = _categories.Maneuvering.StopManeuver

RegisterManeuver = _categories.Maneuvering.RegisterManeuver

ManeuverControlState = _categories.Maneuvering.ManeuverControlState

FollowSystem = _categories.Maneuvering.FollowSystem

CommsRelay = _categories.Maneuvering.CommsRelay

CoverArea = _categories.Maneuvering.CoverArea

PolygonVertex = _categories.Maneuvering.PolygonVertex

CompassCalibration = _categories.Maneuvering.CompassCalibration

FormationParameters = _categories.Maneuvering.FormationParameters

FormationPlanExecution = _categories.Maneuvering.FormationPlanExecution

FollowReference = _categories.Maneuvering.FollowReference

Reference = _categories.Maneuvering.Reference

FollowRefState = _categories.Maneuvering.FollowRefState

FormationMonitor = _categories.Maneuvering.FormationMonitor

RelativeState = _categories.Maneuvering.RelativeState

Dislodge = _categories.Maneuvering.Dislodge

Formation = _categories.Maneuvering.Formation

Launch = _categories.Maneuvering.Launch

Drop = _categories.Maneuvering.Drop

ScheduledGoto = _categories.Maneuvering.ScheduledGoto

RowsCoverage = _categories.Maneuvering.RowsCoverage

Sample = _categories.Maneuvering.Sample

ImageTracking = _categories.Maneuvering.ImageTracking

Takeoff = _categories.Maneuvering.Takeoff

Land = _categories.Maneuvering.Land

AutonomousSection = _categories.Maneuvering.AutonomousSection

FollowPoint = _categories.Maneuvering.FollowPoint

Alignment = _categories.Maneuvering.Alignment

StationKeepingExtended = _categories.Maneuvering.StationKeepingExtended

ManeuverDone = _categories.Maneuvering.ManeuverDone

Magnetometer = _categories.Maneuvering.Magnetometer

VehicleState = _categories.VehicleSupervision.VehicleState

VehicleCommand = _categories.VehicleSupervision.VehicleCommand

MonitorEntityState = _categories.VehicleSupervision.MonitorEntityState

EntityMonitoringState = _categories.VehicleSupervision.EntityMonitoringState

OperationalLimits = _categories.VehicleSupervision.OperationalLimits

GetOperationalLimits = _categories.VehicleSupervision.GetOperationalLimits

Calibration = _categories.VehicleSupervision.Calibration

ControlLoops = _categories.VehicleSupervision.ControlLoops

VehicleMedium = _categories.VehicleSupervision.VehicleMedium

Collision = _categories.VehicleSupervision.Collision

FormState = _categories.VehicleSupervision.FormState

AutopilotMode = _categories.VehicleSupervision.AutopilotMode

FormationState = _categories.VehicleSupervision.FormationState

ReportControl = _categories.VehicleSupervision.ReportControl

StateReport = _categories.VehicleSupervision.StateReport

TransmissionRequest = _categories.Networking.TransmissionRequest

TransmissionStatus = _categories.Networking.TransmissionStatus

SmsRequest = _categories.Networking.SmsRequest

SmsStatus = _categories.Networking.SmsStatus

VtolState = _categories.VehicleSupervision.VtolState

ArmingState = _categories.VehicleSupervision.ArmingState

TCPRequest = _categories.Networking.TCPRequest

TCPStatus = _categories.Networking.TCPStatus

AssetReport = _categories.VehicleSupervision.AssetReport

Abort = _categories.PlanSupervision.Abort

PlanSpecification = _categories.PlanSupervision.PlanSpecification

PlanManeuver = _categories.PlanSupervision.PlanManeuver

PlanTransition = _categories.PlanSupervision.PlanTransition

EmergencyControl = _categories.PlanSupervision.EmergencyControl

EmergencyControlState = _categories.PlanSupervision.EmergencyControlState

PlanDB = _categories.PlanSupervision.PlanDB

PlanDBState = _categories.PlanSupervision.PlanDBState

PlanDBInformation = _categories.PlanSupervision.PlanDBInformation

PlanControl = _categories.PlanSupervision.PlanControl

PlanControlState = _categories.PlanSupervision.PlanControlState

PlanVariable = _categories.PlanSupervision.PlanVariable

PlanGeneration = _categories.PlanSupervision.PlanGeneration

LeaderState = _categories.PlanSupervision.LeaderState

PlanStatistics = _categories.PlanSupervision.PlanStatistics

ReportedState = _categories.CCU.ReportedState

RemoteSensorInfo = _categories.CCU.RemoteSensorInfo

Map = _categories.CCU.Map

MapFeature = _categories.CCU.MapFeature

MapPoint = _categories.CCU.MapPoint

CcuEvent = _categories.CCU.CcuEvent

VehicleLinks = _categories.Autonomy.VehicleLinks

TrexObservation = _categories.Autonomy.TrexObservation

TrexCommand = _categories.Autonomy.TrexCommand

TrexOperation = _categories.Autonomy.TrexOperation

TrexAttribute = _categories.Autonomy.TrexAttribute

TrexToken = _categories.Autonomy.TrexToken

TrexPlan = _categories.Autonomy.TrexPlan

Event = _categories.Autonomy.Event

CompressedImage = _categories.Vision.CompressedImage

ImageTxSettings = _categories.Vision.ImageTxSettings

RemoteState = _categories.CCU.RemoteState

Target = _categories.Maneuvering.Target

EntityParameter = _categories.Core.EntityParameter

EntityParameters = _categories.Core.EntityParameters

QueryEntityParameters = _categories.Core.QueryEntityParameters

SetEntityParameters = _categories.Core.SetEntityParameters

SaveEntityParameters = _categories.Core.SaveEntityParameters

CreateSession = _categories.Networking.CreateSession

CloseSession = _categories.Networking.CloseSession

SessionSubscription = _categories.Networking.SessionSubscription

SessionKeepAlive = _categories.Networking.SessionKeepAlive

SessionStatus = _categories.Networking.SessionStatus

PushEntityParameters = _categories.Core.PushEntityParameters

PopEntityParameters = _categories.Core.PopEntityParameters

IoEvent = _categories.Core.IoEvent

UamTxFrame = _categories.AcousticCommunication.UamTxFrame

UamRxFrame = _categories.AcousticCommunication.UamRxFrame

UamTxStatus = _categories.AcousticCommunication.UamTxStatus

UamRxRange = _categories.AcousticCommunication.UamRxRange

UamTxRange = _categories.AcousticCommunication.UamTxRange

FormCtrlParam = _categories.Maneuvering.FormCtrlParam

FormationEval = _categories.Maneuvering.FormationEval

FormationControlParams = _categories.Maneuvering.FormationControlParams

FormationEvaluation = _categories.Maneuvering.FormationEvaluation

SoiWaypoint = _categories.PlanSupervision.SoiWaypoint

SoiPlan = _categories.PlanSupervision.SoiPlan

SoiCommand = _categories.PlanSupervision.SoiCommand

SoiState = _categories.PlanSupervision.SoiState

MessagePart = _categories.Networking.MessagePart

NeptusBlob = _categories.CCU.NeptusBlob

Aborted = _categories.PlanSupervision.Aborted

UsblAngles = _categories.AcousticCommunication.UsblAngles

UsblPosition = _categories.AcousticCommunication.UsblPosition

UsblFix = _categories.AcousticCommunication.UsblFix

ParametersXml = _categories.VehicleSupervision.ParametersXml

GetParametersXml = _categories.VehicleSupervision.GetParametersXml

SetImageCoords = _categories.Vision.SetImageCoords

GetImageCoords = _categories.Vision.GetImageCoords

GetWorldCoordinates = _categories.Vision.GetWorldCoordinates

UsblAnglesExtended = _categories.AcousticCommunication.UsblAnglesExtended

UsblPositionExtended = _categories.AcousticCommunication.UsblPositionExtended

UsblFixExtended = _categories.AcousticCommunication.UsblFixExtended

UsblModem = _categories.AcousticCommunication.UsblModem

UsblConfig = _categories.AcousticCommunication.UsblConfig

DissolvedOrganicMatter = _categories.Sensors.DissolvedOrganicMatter

OpticalBackscatter = _categories.Sensors.OpticalBackscatter

Tachograph = _categories.VehicleSupervision.Tachograph

ApmStatus = _categories.VehicleSupervision.ApmStatus

SadcReadings = _categories.Sensors.SadcReadings

DmsDetection = _categories.Sensors.DmsDetection

HomePosition = _categories.Core.HomePosition

CurrentProfile = _categories.Sensors.CurrentProfile

CurrentProfileCell = _categories.Sensors.CurrentProfileCell

ADCPBeam = _categories.Sensors.ADCPBeam

GpioState = _categories.Actuation.GpioState

GpioStateGet = _categories.Actuation.GpioStateGet

GpioStateSet = _categories.Actuation.GpioStateSet

ColoredDissolvedOrganicMatter = _categories.Sensors.ColoredDissolvedOrganicMatter

FluorescentDissolvedOrganicMatter = _categories.Sensors.FluorescentDissolvedOrganicMatter

TotalMagIntensity = _categories.Sensors.TotalMagIntensity

CommRestriction = _categories.Networking.CommRestriction
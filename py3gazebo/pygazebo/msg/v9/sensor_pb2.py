# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import pose_pb2 as pose__pb2
import camerasensor_pb2 as camerasensor__pb2
import raysensor_pb2 as raysensor__pb2
import contactsensor_pb2 as contactsensor__pb2
import logical_camera_sensor_pb2 as logical__camera__sensor__pb2
import gps_sensor_pb2 as gps__sensor__pb2
import imu_sensor_pb2 as imu__sensor__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_pb=_b('\n\x0csensor.proto\x12\x0bgazebo.msgs\x1a\npose.proto\x1a\x12\x63\x61merasensor.proto\x1a\x0fraysensor.proto\x1a\x13\x63ontactsensor.proto\x1a\x1blogical_camera_sensor.proto\x1a\x10gps_sensor.proto\x1a\x10imu_sensor.proto\"\xbf\x03\n\x06Sensor\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\n\n\x02id\x18\x02 \x01(\r\x12\x0e\n\x06parent\x18\x03 \x02(\t\x12\x11\n\tparent_id\x18\x04 \x01(\r\x12\x0c\n\x04type\x18\x05 \x02(\t\x12\x11\n\talways_on\x18\x06 \x01(\x08\x12\x13\n\x0bupdate_rate\x18\x07 \x01(\x01\x12\x1f\n\x04pose\x18\x08 \x01(\x0b\x32\x11.gazebo.msgs.Pose\x12)\n\x06\x63\x61mera\x18\t \x01(\x0b\x32\x19.gazebo.msgs.CameraSensor\x12#\n\x03ray\x18\n \x01(\x0b\x32\x16.gazebo.msgs.RaySensor\x12+\n\x07\x63ontact\x18\x0b \x01(\x0b\x32\x1a.gazebo.msgs.ContactSensor\x12\x11\n\tvisualize\x18\x0c \x01(\x08\x12\r\n\x05topic\x18\r \x01(\t\x12\x38\n\x0elogical_camera\x18\x0e \x01(\x0b\x32 .gazebo.msgs.LogicalCameraSensor\x12#\n\x03gps\x18\x0f \x01(\x0b\x32\x16.gazebo.msgs.GPSSensor\x12#\n\x03imu\x18\x10 \x01(\x0b\x32\x16.gazebo.msgs.IMUSensor')
  ,
  dependencies=[pose__pb2.DESCRIPTOR,camerasensor__pb2.DESCRIPTOR,raysensor__pb2.DESCRIPTOR,contactsensor__pb2.DESCRIPTOR,logical__camera__sensor__pb2.DESCRIPTOR,gps__sensor__pb2.DESCRIPTOR,imu__sensor__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SENSOR = _descriptor.Descriptor(
  name='Sensor',
  full_name='gazebo.msgs.Sensor',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='gazebo.msgs.Sensor.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='id', full_name='gazebo.msgs.Sensor.id', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='parent', full_name='gazebo.msgs.Sensor.parent', index=2,
      number=3, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='parent_id', full_name='gazebo.msgs.Sensor.parent_id', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='gazebo.msgs.Sensor.type', index=4,
      number=5, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='always_on', full_name='gazebo.msgs.Sensor.always_on', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='update_rate', full_name='gazebo.msgs.Sensor.update_rate', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pose', full_name='gazebo.msgs.Sensor.pose', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='camera', full_name='gazebo.msgs.Sensor.camera', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ray', full_name='gazebo.msgs.Sensor.ray', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='contact', full_name='gazebo.msgs.Sensor.contact', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='visualize', full_name='gazebo.msgs.Sensor.visualize', index=11,
      number=12, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topic', full_name='gazebo.msgs.Sensor.topic', index=12,
      number=13, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='logical_camera', full_name='gazebo.msgs.Sensor.logical_camera', index=13,
      number=14, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps', full_name='gazebo.msgs.Sensor.gps', index=14,
      number=15, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='imu', full_name='gazebo.msgs.Sensor.imu', index=15,
      number=16, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=165,
  serialized_end=612,
)

_SENSOR.fields_by_name['pose'].message_type = pose__pb2._POSE
_SENSOR.fields_by_name['camera'].message_type = camerasensor__pb2._CAMERASENSOR
_SENSOR.fields_by_name['ray'].message_type = raysensor__pb2._RAYSENSOR
_SENSOR.fields_by_name['contact'].message_type = contactsensor__pb2._CONTACTSENSOR
_SENSOR.fields_by_name['logical_camera'].message_type = logical__camera__sensor__pb2._LOGICALCAMERASENSOR
_SENSOR.fields_by_name['gps'].message_type = gps__sensor__pb2._GPSSENSOR
_SENSOR.fields_by_name['imu'].message_type = imu__sensor__pb2._IMUSENSOR
DESCRIPTOR.message_types_by_name['Sensor'] = _SENSOR

Sensor = _reflection.GeneratedProtocolMessageType('Sensor', (_message.Message,), dict(
  DESCRIPTOR = _SENSOR,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.Sensor)
  ))
_sym_db.RegisterMessage(Sensor)


# @@protoc_insertion_point(module_scope)

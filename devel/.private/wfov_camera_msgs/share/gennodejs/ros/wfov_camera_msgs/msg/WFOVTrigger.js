// Auto-generated. Do not edit!

// (in-package wfov_camera_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WFOVTrigger {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.time_reference = null;
      this.trigger_time = null;
      this.trigger_time_reference = null;
      this.shutter = null;
      this.id = null;
      this.trigger_seq = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('time_reference')) {
        this.time_reference = initObj.time_reference
      }
      else {
        this.time_reference = '';
      }
      if (initObj.hasOwnProperty('trigger_time')) {
        this.trigger_time = initObj.trigger_time
      }
      else {
        this.trigger_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('trigger_time_reference')) {
        this.trigger_time_reference = initObj.trigger_time_reference
      }
      else {
        this.trigger_time_reference = '';
      }
      if (initObj.hasOwnProperty('shutter')) {
        this.shutter = initObj.shutter
      }
      else {
        this.shutter = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('trigger_seq')) {
        this.trigger_seq = initObj.trigger_seq
      }
      else {
        this.trigger_seq = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WFOVTrigger
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [time_reference]
    bufferOffset = _serializer.string(obj.time_reference, buffer, bufferOffset);
    // Serialize message field [trigger_time]
    bufferOffset = _serializer.time(obj.trigger_time, buffer, bufferOffset);
    // Serialize message field [trigger_time_reference]
    bufferOffset = _serializer.string(obj.trigger_time_reference, buffer, bufferOffset);
    // Serialize message field [shutter]
    bufferOffset = _serializer.uint32(obj.shutter, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [trigger_seq]
    bufferOffset = _serializer.uint32(obj.trigger_seq, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WFOVTrigger
    let len;
    let data = new WFOVTrigger(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [time_reference]
    data.time_reference = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [trigger_time]
    data.trigger_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [trigger_time_reference]
    data.trigger_time_reference = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [shutter]
    data.shutter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [trigger_seq]
    data.trigger_seq = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.time_reference);
    length += _getByteLength(object.trigger_time_reference);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wfov_camera_msgs/WFOVTrigger';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e38c040150f1be3148468f6b9974f8bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message for the time synchronization packets for the WFOV cameras.
    
    Header header			# stamp is the time at which the trigger was received.
    
    string time_reference		# The name of the reference clock for this message's timestamp.
    
    time trigger_time		# The trigger time at which the data was valid.
    
    string trigger_time_reference	# The name of the reference clock for this message's trigger timestamp.
    
    uint32 shutter			# Time duration (microseconds) the shutter was open.
    
    uint32 id			# Camera ID Number
    
    uint32 trigger_seq		# Number of triggers sent to this camera ID number
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WFOVTrigger(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.time_reference !== undefined) {
      resolved.time_reference = msg.time_reference;
    }
    else {
      resolved.time_reference = ''
    }

    if (msg.trigger_time !== undefined) {
      resolved.trigger_time = msg.trigger_time;
    }
    else {
      resolved.trigger_time = {secs: 0, nsecs: 0}
    }

    if (msg.trigger_time_reference !== undefined) {
      resolved.trigger_time_reference = msg.trigger_time_reference;
    }
    else {
      resolved.trigger_time_reference = ''
    }

    if (msg.shutter !== undefined) {
      resolved.shutter = msg.shutter;
    }
    else {
      resolved.shutter = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.trigger_seq !== undefined) {
      resolved.trigger_seq = msg.trigger_seq;
    }
    else {
      resolved.trigger_seq = 0
    }

    return resolved;
    }
};

module.exports = WFOVTrigger;

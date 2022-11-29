// Auto-generated. Do not edit!

// (in-package image_exposure_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ExposureSequence {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.shutter = null;
      this.gain = null;
      this.white_balance_blue = null;
      this.white_balance_red = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('shutter')) {
        this.shutter = initObj.shutter
      }
      else {
        this.shutter = [];
      }
      if (initObj.hasOwnProperty('gain')) {
        this.gain = initObj.gain
      }
      else {
        this.gain = 0.0;
      }
      if (initObj.hasOwnProperty('white_balance_blue')) {
        this.white_balance_blue = initObj.white_balance_blue
      }
      else {
        this.white_balance_blue = 0;
      }
      if (initObj.hasOwnProperty('white_balance_red')) {
        this.white_balance_red = initObj.white_balance_red
      }
      else {
        this.white_balance_red = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExposureSequence
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [shutter]
    bufferOffset = _arraySerializer.uint32(obj.shutter, buffer, bufferOffset, null);
    // Serialize message field [gain]
    bufferOffset = _serializer.float32(obj.gain, buffer, bufferOffset);
    // Serialize message field [white_balance_blue]
    bufferOffset = _serializer.uint16(obj.white_balance_blue, buffer, bufferOffset);
    // Serialize message field [white_balance_red]
    bufferOffset = _serializer.uint16(obj.white_balance_red, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExposureSequence
    let len;
    let data = new ExposureSequence(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [shutter]
    data.shutter = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [gain]
    data.gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [white_balance_blue]
    data.white_balance_blue = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [white_balance_red]
    data.white_balance_red = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.shutter.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_exposure_msgs/ExposureSequence';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81d98e1e20eab8beb4bd07beeba6a398';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # exposure sequence (for mult-exposure HDR)
    Header header
    uint32[]  shutter    	     # Time duration (microseconds) the shutter was open.
    float32 gain                 # Gain (in dB) applied to the images in the sequence.
    uint16 white_balance_blue    # White balance in blue.
    uint16 white_balance_red     # White balance in red.
    
    
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
    const resolved = new ExposureSequence(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.shutter !== undefined) {
      resolved.shutter = msg.shutter;
    }
    else {
      resolved.shutter = []
    }

    if (msg.gain !== undefined) {
      resolved.gain = msg.gain;
    }
    else {
      resolved.gain = 0.0
    }

    if (msg.white_balance_blue !== undefined) {
      resolved.white_balance_blue = msg.white_balance_blue;
    }
    else {
      resolved.white_balance_blue = 0
    }

    if (msg.white_balance_red !== undefined) {
      resolved.white_balance_red = msg.white_balance_red;
    }
    else {
      resolved.white_balance_red = 0
    }

    return resolved;
    }
};

module.exports = ExposureSequence;

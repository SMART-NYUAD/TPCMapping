// Auto-generated. Do not edit!

// (in-package image_exposure_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let statistics_msgs = _finder('statistics_msgs');

//-----------------------------------------------------------

class ImageExposureStatistics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.time_reference = null;
      this.shutterms = null;
      this.gaindb = null;
      this.underExposed = null;
      this.overExposed = null;
      this.pixelVal = null;
      this.pixelAge = null;
      this.meanIrradiance = null;
      this.minMeasurableIrradiance = null;
      this.maxMeasurableIrradiance = null;
      this.minObservedIrradiance = null;
      this.maxObservedIrradiance = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('time_reference')) {
        this.time_reference = initObj.time_reference
      }
      else {
        this.time_reference = '';
      }
      if (initObj.hasOwnProperty('shutterms')) {
        this.shutterms = initObj.shutterms
      }
      else {
        this.shutterms = 0.0;
      }
      if (initObj.hasOwnProperty('gaindb')) {
        this.gaindb = initObj.gaindb
      }
      else {
        this.gaindb = 0.0;
      }
      if (initObj.hasOwnProperty('underExposed')) {
        this.underExposed = initObj.underExposed
      }
      else {
        this.underExposed = 0;
      }
      if (initObj.hasOwnProperty('overExposed')) {
        this.overExposed = initObj.overExposed
      }
      else {
        this.overExposed = 0;
      }
      if (initObj.hasOwnProperty('pixelVal')) {
        this.pixelVal = initObj.pixelVal
      }
      else {
        this.pixelVal = new statistics_msgs.msg.Stats1D();
      }
      if (initObj.hasOwnProperty('pixelAge')) {
        this.pixelAge = initObj.pixelAge
      }
      else {
        this.pixelAge = new statistics_msgs.msg.Stats1D();
      }
      if (initObj.hasOwnProperty('meanIrradiance')) {
        this.meanIrradiance = initObj.meanIrradiance
      }
      else {
        this.meanIrradiance = 0.0;
      }
      if (initObj.hasOwnProperty('minMeasurableIrradiance')) {
        this.minMeasurableIrradiance = initObj.minMeasurableIrradiance
      }
      else {
        this.minMeasurableIrradiance = 0.0;
      }
      if (initObj.hasOwnProperty('maxMeasurableIrradiance')) {
        this.maxMeasurableIrradiance = initObj.maxMeasurableIrradiance
      }
      else {
        this.maxMeasurableIrradiance = 0.0;
      }
      if (initObj.hasOwnProperty('minObservedIrradiance')) {
        this.minObservedIrradiance = initObj.minObservedIrradiance
      }
      else {
        this.minObservedIrradiance = 0.0;
      }
      if (initObj.hasOwnProperty('maxObservedIrradiance')) {
        this.maxObservedIrradiance = initObj.maxObservedIrradiance
      }
      else {
        this.maxObservedIrradiance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImageExposureStatistics
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [time_reference]
    bufferOffset = _serializer.string(obj.time_reference, buffer, bufferOffset);
    // Serialize message field [shutterms]
    bufferOffset = _serializer.float32(obj.shutterms, buffer, bufferOffset);
    // Serialize message field [gaindb]
    bufferOffset = _serializer.float32(obj.gaindb, buffer, bufferOffset);
    // Serialize message field [underExposed]
    bufferOffset = _serializer.uint32(obj.underExposed, buffer, bufferOffset);
    // Serialize message field [overExposed]
    bufferOffset = _serializer.uint32(obj.overExposed, buffer, bufferOffset);
    // Serialize message field [pixelVal]
    bufferOffset = statistics_msgs.msg.Stats1D.serialize(obj.pixelVal, buffer, bufferOffset);
    // Serialize message field [pixelAge]
    bufferOffset = statistics_msgs.msg.Stats1D.serialize(obj.pixelAge, buffer, bufferOffset);
    // Serialize message field [meanIrradiance]
    bufferOffset = _serializer.float64(obj.meanIrradiance, buffer, bufferOffset);
    // Serialize message field [minMeasurableIrradiance]
    bufferOffset = _serializer.float64(obj.minMeasurableIrradiance, buffer, bufferOffset);
    // Serialize message field [maxMeasurableIrradiance]
    bufferOffset = _serializer.float64(obj.maxMeasurableIrradiance, buffer, bufferOffset);
    // Serialize message field [minObservedIrradiance]
    bufferOffset = _serializer.float64(obj.minObservedIrradiance, buffer, bufferOffset);
    // Serialize message field [maxObservedIrradiance]
    bufferOffset = _serializer.float64(obj.maxObservedIrradiance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImageExposureStatistics
    let len;
    let data = new ImageExposureStatistics(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [time_reference]
    data.time_reference = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [shutterms]
    data.shutterms = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gaindb]
    data.gaindb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [underExposed]
    data.underExposed = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [overExposed]
    data.overExposed = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pixelVal]
    data.pixelVal = statistics_msgs.msg.Stats1D.deserialize(buffer, bufferOffset);
    // Deserialize message field [pixelAge]
    data.pixelAge = statistics_msgs.msg.Stats1D.deserialize(buffer, bufferOffset);
    // Deserialize message field [meanIrradiance]
    data.meanIrradiance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [minMeasurableIrradiance]
    data.minMeasurableIrradiance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [maxMeasurableIrradiance]
    data.maxMeasurableIrradiance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [minObservedIrradiance]
    data.minObservedIrradiance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [maxObservedIrradiance]
    data.maxObservedIrradiance = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.time_reference);
    return length + 148;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_exposure_msgs/ImageExposureStatistics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '334dc170ce6287d1de470f25be78dd9e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for exposure statistics reported a single image
    time stamp         # image time stamp
    string time_reference # The name of the reference clock for this message's timestamp.
    float32 shutterms  # shutter durations in ms
    float32 gaindb     # gain in decibels
    # pixel exposure and latency statistics
    uint32 underExposed # number of pixels underexposed
    uint32 overExposed  # number of pixels overexposed
    statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image
    statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames
    # irradiance = pixelval/(shutterTime*10^(gaindb/10.0))
    float64 meanIrradiance
    float64 minMeasurableIrradiance
    float64 maxMeasurableIrradiance
    float64 minObservedIrradiance
    float64 maxObservedIrradiance
    
    
    
    ================================================================================
    MSG: statistics_msgs/Stats1D
    # statistics of a 1-D distribution
    float64 min
    float64 max 
    float64 mean 
    float64 variance
    int64 N
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImageExposureStatistics(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.time_reference !== undefined) {
      resolved.time_reference = msg.time_reference;
    }
    else {
      resolved.time_reference = ''
    }

    if (msg.shutterms !== undefined) {
      resolved.shutterms = msg.shutterms;
    }
    else {
      resolved.shutterms = 0.0
    }

    if (msg.gaindb !== undefined) {
      resolved.gaindb = msg.gaindb;
    }
    else {
      resolved.gaindb = 0.0
    }

    if (msg.underExposed !== undefined) {
      resolved.underExposed = msg.underExposed;
    }
    else {
      resolved.underExposed = 0
    }

    if (msg.overExposed !== undefined) {
      resolved.overExposed = msg.overExposed;
    }
    else {
      resolved.overExposed = 0
    }

    if (msg.pixelVal !== undefined) {
      resolved.pixelVal = statistics_msgs.msg.Stats1D.Resolve(msg.pixelVal)
    }
    else {
      resolved.pixelVal = new statistics_msgs.msg.Stats1D()
    }

    if (msg.pixelAge !== undefined) {
      resolved.pixelAge = statistics_msgs.msg.Stats1D.Resolve(msg.pixelAge)
    }
    else {
      resolved.pixelAge = new statistics_msgs.msg.Stats1D()
    }

    if (msg.meanIrradiance !== undefined) {
      resolved.meanIrradiance = msg.meanIrradiance;
    }
    else {
      resolved.meanIrradiance = 0.0
    }

    if (msg.minMeasurableIrradiance !== undefined) {
      resolved.minMeasurableIrradiance = msg.minMeasurableIrradiance;
    }
    else {
      resolved.minMeasurableIrradiance = 0.0
    }

    if (msg.maxMeasurableIrradiance !== undefined) {
      resolved.maxMeasurableIrradiance = msg.maxMeasurableIrradiance;
    }
    else {
      resolved.maxMeasurableIrradiance = 0.0
    }

    if (msg.minObservedIrradiance !== undefined) {
      resolved.minObservedIrradiance = msg.minObservedIrradiance;
    }
    else {
      resolved.minObservedIrradiance = 0.0
    }

    if (msg.maxObservedIrradiance !== undefined) {
      resolved.maxObservedIrradiance = msg.maxObservedIrradiance;
    }
    else {
      resolved.maxObservedIrradiance = 0.0
    }

    return resolved;
    }
};

module.exports = ImageExposureStatistics;

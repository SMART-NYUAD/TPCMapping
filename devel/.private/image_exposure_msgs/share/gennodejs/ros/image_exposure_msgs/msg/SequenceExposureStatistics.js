// Auto-generated. Do not edit!

// (in-package image_exposure_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ImageExposureStatistics = require('./ImageExposureStatistics.js');

//-----------------------------------------------------------

class SequenceExposureStatistics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exposures = null;
      this.underExposed = null;
      this.overExposed = null;
      this.meanIrradiance = null;
      this.minMeasurableIrradiance = null;
      this.maxMeasurableIrradiance = null;
      this.minObservedIrradiance = null;
      this.maxObservedIrradiance = null;
    }
    else {
      if (initObj.hasOwnProperty('exposures')) {
        this.exposures = initObj.exposures
      }
      else {
        this.exposures = [];
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
    // Serializes a message object of type SequenceExposureStatistics
    // Serialize message field [exposures]
    // Serialize the length for message field [exposures]
    bufferOffset = _serializer.uint32(obj.exposures.length, buffer, bufferOffset);
    obj.exposures.forEach((val) => {
      bufferOffset = ImageExposureStatistics.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [underExposed]
    bufferOffset = _serializer.uint32(obj.underExposed, buffer, bufferOffset);
    // Serialize message field [overExposed]
    bufferOffset = _serializer.uint32(obj.overExposed, buffer, bufferOffset);
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
    //deserializes a message object of type SequenceExposureStatistics
    let len;
    let data = new SequenceExposureStatistics(null);
    // Deserialize message field [exposures]
    // Deserialize array length for message field [exposures]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.exposures = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.exposures[i] = ImageExposureStatistics.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [underExposed]
    data.underExposed = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [overExposed]
    data.overExposed = _deserializer.uint32(buffer, bufferOffset);
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
    object.exposures.forEach((val) => {
      length += ImageExposureStatistics.getMessageSize(val);
    });
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_exposure_msgs/SequenceExposureStatistics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a4f3187c50e7b3544984e9e28ce4328';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # exposure statistics for a multi-exposure HDR sequence
    ImageExposureStatistics[] exposures
    uint32 underExposed 		# number of pixels underexposed in longest exposure
    uint32 overExposed  		# number of pixels overexposed in shortest
    float64 meanIrradiance 		# mean irradiance of all properly exposed pixels in all images in the sequence
    float64 minMeasurableIrradiance
    float64 maxMeasurableIrradiance
    float64 minObservedIrradiance
    float64 maxObservedIrradiance
    
    
    ================================================================================
    MSG: image_exposure_msgs/ImageExposureStatistics
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
    const resolved = new SequenceExposureStatistics(null);
    if (msg.exposures !== undefined) {
      resolved.exposures = new Array(msg.exposures.length);
      for (let i = 0; i < resolved.exposures.length; ++i) {
        resolved.exposures[i] = ImageExposureStatistics.Resolve(msg.exposures[i]);
      }
    }
    else {
      resolved.exposures = []
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

module.exports = SequenceExposureStatistics;

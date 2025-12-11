// Auto-generated. Do not edit!

// (in-package rvo_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetGoalsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model = null;
      this.coordinates = null;
    }
    else {
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = '';
      }
      if (initObj.hasOwnProperty('coordinates')) {
        this.coordinates = initObj.coordinates
      }
      else {
        this.coordinates = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGoalsRequest
    // Serialize message field [model]
    bufferOffset = _serializer.string(obj.model, buffer, bufferOffset);
    // Serialize message field [coordinates]
    // Serialize the length for message field [coordinates]
    bufferOffset = _serializer.uint32(obj.coordinates.length, buffer, bufferOffset);
    obj.coordinates.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGoalsRequest
    let len;
    let data = new SetGoalsRequest(null);
    // Deserialize message field [model]
    data.model = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [coordinates]
    // Deserialize array length for message field [coordinates]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.coordinates = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.coordinates[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.model);
    length += 24 * object.coordinates.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rvo_ros/SetGoalsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d1f7652138c35224f362d4e27f4f70e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string model
    geometry_msgs/Point[] coordinates
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGoalsRequest(null);
    if (msg.model !== undefined) {
      resolved.model = msg.model;
    }
    else {
      resolved.model = ''
    }

    if (msg.coordinates !== undefined) {
      resolved.coordinates = new Array(msg.coordinates.length);
      for (let i = 0; i < resolved.coordinates.length; ++i) {
        resolved.coordinates[i] = geometry_msgs.msg.Point.Resolve(msg.coordinates[i]);
      }
    }
    else {
      resolved.coordinates = []
    }

    return resolved;
    }
};

class SetGoalsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('num_goal')) {
        this.num_goal = initObj.num_goal
      }
      else {
        this.num_goal = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGoalsResponse
    // Serialize message field [num_goal]
    bufferOffset = _serializer.int64(obj.num_goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGoalsResponse
    let len;
    let data = new SetGoalsResponse(null);
    // Deserialize message field [num_goal]
    data.num_goal = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rvo_ros/SetGoalsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7213b2fb0a7852c514b409e9c0931450';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 num_goal
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGoalsResponse(null);
    if (msg.num_goal !== undefined) {
      resolved.num_goal = msg.num_goal;
    }
    else {
      resolved.num_goal = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetGoalsRequest,
  Response: SetGoalsResponse,
  md5sum() { return '269a0b03d7a509c517f6ff76c318c9eb'; },
  datatype() { return 'rvo_ros/SetGoals'; }
};

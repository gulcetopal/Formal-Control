// Auto-generated. Do not edit!

// (in-package formal_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PathMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.m = null;
      this.y_start = null;
      this.y_finish = null;
      this.x_start = null;
      this.x_finish = null;
      this.check = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('m')) {
        this.m = initObj.m
      }
      else {
        this.m = [];
      }
      if (initObj.hasOwnProperty('y_start')) {
        this.y_start = initObj.y_start
      }
      else {
        this.y_start = [];
      }
      if (initObj.hasOwnProperty('y_finish')) {
        this.y_finish = initObj.y_finish
      }
      else {
        this.y_finish = [];
      }
      if (initObj.hasOwnProperty('x_start')) {
        this.x_start = initObj.x_start
      }
      else {
        this.x_start = [];
      }
      if (initObj.hasOwnProperty('x_finish')) {
        this.x_finish = initObj.x_finish
      }
      else {
        this.x_finish = [];
      }
      if (initObj.hasOwnProperty('check')) {
        this.check = initObj.check
      }
      else {
        this.check = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [m]
    bufferOffset = _arraySerializer.float32(obj.m, buffer, bufferOffset, null);
    // Serialize message field [y_start]
    bufferOffset = _arraySerializer.float32(obj.y_start, buffer, bufferOffset, null);
    // Serialize message field [y_finish]
    bufferOffset = _arraySerializer.float32(obj.y_finish, buffer, bufferOffset, null);
    // Serialize message field [x_start]
    bufferOffset = _arraySerializer.float32(obj.x_start, buffer, bufferOffset, null);
    // Serialize message field [x_finish]
    bufferOffset = _arraySerializer.float32(obj.x_finish, buffer, bufferOffset, null);
    // Serialize message field [check]
    bufferOffset = _serializer.bool(obj.check, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathMsg
    let len;
    let data = new PathMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [m]
    data.m = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y_start]
    data.y_start = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y_finish]
    data.y_finish = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [x_start]
    data.x_start = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [x_finish]
    data.x_finish = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [check]
    data.check = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.m.length;
    length += 4 * object.y_start.length;
    length += 4 * object.y_finish.length;
    length += 4 * object.x_start.length;
    length += 4 * object.x_finish.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'formal_control/PathMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aea262bd14c4beff32fb4111e598fc59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32[] m
    float32[] y_start
    float32[] y_finish
    float32[] x_start
    float32[] x_finish
    bool check
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.m !== undefined) {
      resolved.m = msg.m;
    }
    else {
      resolved.m = []
    }

    if (msg.y_start !== undefined) {
      resolved.y_start = msg.y_start;
    }
    else {
      resolved.y_start = []
    }

    if (msg.y_finish !== undefined) {
      resolved.y_finish = msg.y_finish;
    }
    else {
      resolved.y_finish = []
    }

    if (msg.x_start !== undefined) {
      resolved.x_start = msg.x_start;
    }
    else {
      resolved.x_start = []
    }

    if (msg.x_finish !== undefined) {
      resolved.x_finish = msg.x_finish;
    }
    else {
      resolved.x_finish = []
    }

    if (msg.check !== undefined) {
      resolved.check = msg.check;
    }
    else {
      resolved.check = false
    }

    return resolved;
    }
};

module.exports = PathMsg;

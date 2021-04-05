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

class SelfStateMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.rfdist = null;
      this.lfdist = null;
      this.bdist = null;
      this.v_relative = null;
      this.actions = null;
      this.policy = null;
      this.old_policy = null;
      this.timestep = null;
      this.v_refx = null;
      this.yaw_ref = null;
      this.got_new_plan = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('rfdist')) {
        this.rfdist = initObj.rfdist
      }
      else {
        this.rfdist = 0.0;
      }
      if (initObj.hasOwnProperty('lfdist')) {
        this.lfdist = initObj.lfdist
      }
      else {
        this.lfdist = 0.0;
      }
      if (initObj.hasOwnProperty('bdist')) {
        this.bdist = initObj.bdist
      }
      else {
        this.bdist = 0.0;
      }
      if (initObj.hasOwnProperty('v_relative')) {
        this.v_relative = initObj.v_relative
      }
      else {
        this.v_relative = 0.0;
      }
      if (initObj.hasOwnProperty('actions')) {
        this.actions = initObj.actions
      }
      else {
        this.actions = [];
      }
      if (initObj.hasOwnProperty('policy')) {
        this.policy = initObj.policy
      }
      else {
        this.policy = [];
      }
      if (initObj.hasOwnProperty('old_policy')) {
        this.old_policy = initObj.old_policy
      }
      else {
        this.old_policy = [];
      }
      if (initObj.hasOwnProperty('timestep')) {
        this.timestep = initObj.timestep
      }
      else {
        this.timestep = 0;
      }
      if (initObj.hasOwnProperty('v_refx')) {
        this.v_refx = initObj.v_refx
      }
      else {
        this.v_refx = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_ref')) {
        this.yaw_ref = initObj.yaw_ref
      }
      else {
        this.yaw_ref = 0.0;
      }
      if (initObj.hasOwnProperty('got_new_plan')) {
        this.got_new_plan = initObj.got_new_plan
      }
      else {
        this.got_new_plan = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelfStateMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [rfdist]
    bufferOffset = _serializer.float32(obj.rfdist, buffer, bufferOffset);
    // Serialize message field [lfdist]
    bufferOffset = _serializer.float32(obj.lfdist, buffer, bufferOffset);
    // Serialize message field [bdist]
    bufferOffset = _serializer.float32(obj.bdist, buffer, bufferOffset);
    // Serialize message field [v_relative]
    bufferOffset = _serializer.float32(obj.v_relative, buffer, bufferOffset);
    // Serialize message field [actions]
    bufferOffset = _arraySerializer.int32(obj.actions, buffer, bufferOffset, null);
    // Serialize message field [policy]
    bufferOffset = _arraySerializer.int32(obj.policy, buffer, bufferOffset, null);
    // Serialize message field [old_policy]
    bufferOffset = _arraySerializer.int32(obj.old_policy, buffer, bufferOffset, null);
    // Serialize message field [timestep]
    bufferOffset = _serializer.int32(obj.timestep, buffer, bufferOffset);
    // Serialize message field [v_refx]
    bufferOffset = _serializer.float32(obj.v_refx, buffer, bufferOffset);
    // Serialize message field [yaw_ref]
    bufferOffset = _serializer.float32(obj.yaw_ref, buffer, bufferOffset);
    // Serialize message field [got_new_plan]
    bufferOffset = _serializer.bool(obj.got_new_plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelfStateMsg
    let len;
    let data = new SelfStateMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [rfdist]
    data.rfdist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lfdist]
    data.lfdist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bdist]
    data.bdist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [v_relative]
    data.v_relative = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [actions]
    data.actions = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [policy]
    data.policy = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [old_policy]
    data.old_policy = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [timestep]
    data.timestep = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [v_refx]
    data.v_refx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_ref]
    data.yaw_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [got_new_plan]
    data.got_new_plan = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.actions.length;
    length += 4 * object.policy.length;
    length += 4 * object.old_policy.length;
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'formal_control/SelfStateMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8945206644fb2fbf7de854487cab7b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 rfdist
    float32 lfdist
    float32 bdist
    float32 v_relative
    
    int32[] actions
    int32[] policy
    int32[] old_policy
    int32 timestep
    
    float32 v_refx
    float32 yaw_ref
    
    bool got_new_plan
    
    
    
    
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
    const resolved = new SelfStateMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.rfdist !== undefined) {
      resolved.rfdist = msg.rfdist;
    }
    else {
      resolved.rfdist = 0.0
    }

    if (msg.lfdist !== undefined) {
      resolved.lfdist = msg.lfdist;
    }
    else {
      resolved.lfdist = 0.0
    }

    if (msg.bdist !== undefined) {
      resolved.bdist = msg.bdist;
    }
    else {
      resolved.bdist = 0.0
    }

    if (msg.v_relative !== undefined) {
      resolved.v_relative = msg.v_relative;
    }
    else {
      resolved.v_relative = 0.0
    }

    if (msg.actions !== undefined) {
      resolved.actions = msg.actions;
    }
    else {
      resolved.actions = []
    }

    if (msg.policy !== undefined) {
      resolved.policy = msg.policy;
    }
    else {
      resolved.policy = []
    }

    if (msg.old_policy !== undefined) {
      resolved.old_policy = msg.old_policy;
    }
    else {
      resolved.old_policy = []
    }

    if (msg.timestep !== undefined) {
      resolved.timestep = msg.timestep;
    }
    else {
      resolved.timestep = 0
    }

    if (msg.v_refx !== undefined) {
      resolved.v_refx = msg.v_refx;
    }
    else {
      resolved.v_refx = 0.0
    }

    if (msg.yaw_ref !== undefined) {
      resolved.yaw_ref = msg.yaw_ref;
    }
    else {
      resolved.yaw_ref = 0.0
    }

    if (msg.got_new_plan !== undefined) {
      resolved.got_new_plan = msg.got_new_plan;
    }
    else {
      resolved.got_new_plan = false
    }

    return resolved;
    }
};

module.exports = SelfStateMsg;

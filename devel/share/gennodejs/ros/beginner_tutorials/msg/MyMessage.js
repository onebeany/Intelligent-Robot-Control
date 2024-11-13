// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MyMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.nums = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = '';
      }
      if (initObj.hasOwnProperty('nums')) {
        this.nums = initObj.nums
      }
      else {
        this.nums = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyMessage
    // Serialize message field [time]
    bufferOffset = _serializer.string(obj.time, buffer, bufferOffset);
    // Serialize message field [nums]
    bufferOffset = _arraySerializer.int32(obj.nums, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyMessage
    let len;
    let data = new MyMessage(null);
    // Deserialize message field [time]
    data.time = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [nums]
    data.nums = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.time);
    length += 4 * object.nums.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/MyMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0c0cdfb81e8e5ff82904cd630ef5b88';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string time
    int32[] nums
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyMessage(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = ''
    }

    if (msg.nums !== undefined) {
      resolved.nums = msg.nums;
    }
    else {
      resolved.nums = []
    }

    return resolved;
    }
};

module.exports = MyMessage;

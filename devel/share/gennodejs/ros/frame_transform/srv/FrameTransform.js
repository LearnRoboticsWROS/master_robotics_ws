// Auto-generated. Do not edit!

// (in-package frame_transform.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class FrameTransformRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.from_camera_to_base_link = null;
    }
    else {
      if (initObj.hasOwnProperty('from_camera_to_base_link')) {
        this.from_camera_to_base_link = initObj.from_camera_to_base_link
      }
      else {
        this.from_camera_to_base_link = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrameTransformRequest
    // Serialize message field [from_camera_to_base_link]
    bufferOffset = _serializer.bool(obj.from_camera_to_base_link, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrameTransformRequest
    let len;
    let data = new FrameTransformRequest(null);
    // Deserialize message field [from_camera_to_base_link]
    data.from_camera_to_base_link = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'frame_transform/FrameTransformRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b803e7ecd898897f619a0902bc489b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool from_camera_to_base_link
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrameTransformRequest(null);
    if (msg.from_camera_to_base_link !== undefined) {
      resolved.from_camera_to_base_link = msg.from_camera_to_base_link;
    }
    else {
      resolved.from_camera_to_base_link = false
    }

    return resolved;
    }
};

class FrameTransformResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_base_link_frame = null;
      this.y_base_link_frame = null;
      this.z_base_link_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('x_base_link_frame')) {
        this.x_base_link_frame = initObj.x_base_link_frame
      }
      else {
        this.x_base_link_frame = 0.0;
      }
      if (initObj.hasOwnProperty('y_base_link_frame')) {
        this.y_base_link_frame = initObj.y_base_link_frame
      }
      else {
        this.y_base_link_frame = 0.0;
      }
      if (initObj.hasOwnProperty('z_base_link_frame')) {
        this.z_base_link_frame = initObj.z_base_link_frame
      }
      else {
        this.z_base_link_frame = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrameTransformResponse
    // Serialize message field [x_base_link_frame]
    bufferOffset = _serializer.float64(obj.x_base_link_frame, buffer, bufferOffset);
    // Serialize message field [y_base_link_frame]
    bufferOffset = _serializer.float64(obj.y_base_link_frame, buffer, bufferOffset);
    // Serialize message field [z_base_link_frame]
    bufferOffset = _serializer.float64(obj.z_base_link_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrameTransformResponse
    let len;
    let data = new FrameTransformResponse(null);
    // Deserialize message field [x_base_link_frame]
    data.x_base_link_frame = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_base_link_frame]
    data.y_base_link_frame = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_base_link_frame]
    data.z_base_link_frame = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'frame_transform/FrameTransformResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '136b4b47ffa027913397488262ef835b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_base_link_frame
    float64 y_base_link_frame
    float64 z_base_link_frame
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrameTransformResponse(null);
    if (msg.x_base_link_frame !== undefined) {
      resolved.x_base_link_frame = msg.x_base_link_frame;
    }
    else {
      resolved.x_base_link_frame = 0.0
    }

    if (msg.y_base_link_frame !== undefined) {
      resolved.y_base_link_frame = msg.y_base_link_frame;
    }
    else {
      resolved.y_base_link_frame = 0.0
    }

    if (msg.z_base_link_frame !== undefined) {
      resolved.z_base_link_frame = msg.z_base_link_frame;
    }
    else {
      resolved.z_base_link_frame = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: FrameTransformRequest,
  Response: FrameTransformResponse,
  md5sum() { return '1f0c6508a5e0cc15dc2cd25abbd47d9c'; },
  datatype() { return 'frame_transform/FrameTransform'; }
};

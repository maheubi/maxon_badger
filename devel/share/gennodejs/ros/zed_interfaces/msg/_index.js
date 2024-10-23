
"use strict";

let Object = require('./Object.js');
let BoundingBox2Di = require('./BoundingBox2Di.js');
let Skeleton2D = require('./Skeleton2D.js');
let RGBDSensors = require('./RGBDSensors.js');
let Skeleton3D = require('./Skeleton3D.js');
let PosTrackStatus = require('./PosTrackStatus.js');
let BoundingBox2Df = require('./BoundingBox2Df.js');
let ObjectsStamped = require('./ObjectsStamped.js');
let Keypoint2Df = require('./Keypoint2Df.js');
let PlaneStamped = require('./PlaneStamped.js');
let Keypoint2Di = require('./Keypoint2Di.js');
let Keypoint3D = require('./Keypoint3D.js');
let BoundingBox3D = require('./BoundingBox3D.js');

module.exports = {
  Object: Object,
  BoundingBox2Di: BoundingBox2Di,
  Skeleton2D: Skeleton2D,
  RGBDSensors: RGBDSensors,
  Skeleton3D: Skeleton3D,
  PosTrackStatus: PosTrackStatus,
  BoundingBox2Df: BoundingBox2Df,
  ObjectsStamped: ObjectsStamped,
  Keypoint2Df: Keypoint2Df,
  PlaneStamped: PlaneStamped,
  Keypoint2Di: Keypoint2Di,
  Keypoint3D: Keypoint3D,
  BoundingBox3D: BoundingBox3D,
};

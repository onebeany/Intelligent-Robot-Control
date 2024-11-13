
"use strict";

let GetKinematicsPose = require('./GetKinematicsPose.js')
let GetJointPosition = require('./GetJointPosition.js')
let SetActuatorState = require('./SetActuatorState.js')
let SetJointPosition = require('./SetJointPosition.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')
let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')

module.exports = {
  GetKinematicsPose: GetKinematicsPose,
  GetJointPosition: GetJointPosition,
  SetActuatorState: SetActuatorState,
  SetJointPosition: SetJointPosition,
  SetKinematicsPose: SetKinematicsPose,
  SetDrawingTrajectory: SetDrawingTrajectory,
};

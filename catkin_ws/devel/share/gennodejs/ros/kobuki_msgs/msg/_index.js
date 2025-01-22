
"use strict";

let SensorState = require('./SensorState.js');
let Sound = require('./Sound.js');
let KeyboardInput = require('./KeyboardInput.js');
let BumperEvent = require('./BumperEvent.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let CliffEvent = require('./CliffEvent.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let MotorPower = require('./MotorPower.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let DigitalOutput = require('./DigitalOutput.js');
let Led = require('./Led.js');
let ScanAngle = require('./ScanAngle.js');
let ControllerInfo = require('./ControllerInfo.js');
let VersionInfo = require('./VersionInfo.js');
let ExternalPower = require('./ExternalPower.js');
let DockInfraRed = require('./DockInfraRed.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');

module.exports = {
  SensorState: SensorState,
  Sound: Sound,
  KeyboardInput: KeyboardInput,
  BumperEvent: BumperEvent,
  WheelDropEvent: WheelDropEvent,
  ButtonEvent: ButtonEvent,
  CliffEvent: CliffEvent,
  DigitalInputEvent: DigitalInputEvent,
  PowerSystemEvent: PowerSystemEvent,
  MotorPower: MotorPower,
  RobotStateEvent: RobotStateEvent,
  DigitalOutput: DigitalOutput,
  Led: Led,
  ScanAngle: ScanAngle,
  ControllerInfo: ControllerInfo,
  VersionInfo: VersionInfo,
  ExternalPower: ExternalPower,
  DockInfraRed: DockInfraRed,
  AutoDockingActionResult: AutoDockingActionResult,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingAction: AutoDockingAction,
  AutoDockingResult: AutoDockingResult,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingFeedback: AutoDockingFeedback,
};

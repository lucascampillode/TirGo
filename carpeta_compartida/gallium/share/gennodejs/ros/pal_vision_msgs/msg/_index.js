
"use strict";

let Gesture = require('./Gesture.js');
let HogDetection = require('./HogDetection.js');
let DetectedPerson = require('./DetectedPerson.js');
let Rectangle = require('./Rectangle.js');
let HogDetections = require('./HogDetections.js');
let DetectedObject = require('./DetectedObject.js');
let LegDetections = require('./LegDetections.js');
let FollowMeResponse = require('./FollowMeResponse.js');
let HeadPanTilt = require('./HeadPanTilt.js');
let FaceDetection = require('./FaceDetection.js');
let FaceDetections = require('./FaceDetections.js');
let FaceRecognitionAction = require('./FaceRecognitionAction.js');
let FaceRecognitionActionResult = require('./FaceRecognitionActionResult.js');
let FaceRecognitionActionFeedback = require('./FaceRecognitionActionFeedback.js');
let FaceRecognitionResult = require('./FaceRecognitionResult.js');
let FaceRecognitionGoal = require('./FaceRecognitionGoal.js');
let FaceRecognitionFeedback = require('./FaceRecognitionFeedback.js');
let FaceRecognitionActionGoal = require('./FaceRecognitionActionGoal.js');

module.exports = {
  Gesture: Gesture,
  HogDetection: HogDetection,
  DetectedPerson: DetectedPerson,
  Rectangle: Rectangle,
  HogDetections: HogDetections,
  DetectedObject: DetectedObject,
  LegDetections: LegDetections,
  FollowMeResponse: FollowMeResponse,
  HeadPanTilt: HeadPanTilt,
  FaceDetection: FaceDetection,
  FaceDetections: FaceDetections,
  FaceRecognitionAction: FaceRecognitionAction,
  FaceRecognitionActionResult: FaceRecognitionActionResult,
  FaceRecognitionActionFeedback: FaceRecognitionActionFeedback,
  FaceRecognitionResult: FaceRecognitionResult,
  FaceRecognitionGoal: FaceRecognitionGoal,
  FaceRecognitionFeedback: FaceRecognitionFeedback,
  FaceRecognitionActionGoal: FaceRecognitionActionGoal,
};

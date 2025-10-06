
"use strict";

let GoToPOIGoal = require('./GoToPOIGoal.js');
let ExecuteParkingGoal = require('./ExecuteParkingGoal.js');
let VisualTrainingGoal = require('./VisualTrainingGoal.js');
let VisualTrainingResult = require('./VisualTrainingResult.js');
let JoyPriorityAction = require('./JoyPriorityAction.js');
let JoyTurboActionGoal = require('./JoyTurboActionGoal.js');
let ExecuteParkingFeedback = require('./ExecuteParkingFeedback.js');
let GoToResult = require('./GoToResult.js');
let GoToGoal = require('./GoToGoal.js');
let JoyTurboActionFeedback = require('./JoyTurboActionFeedback.js');
let VisualTrainingFeedback = require('./VisualTrainingFeedback.js');
let ExecuteParkingActionGoal = require('./ExecuteParkingActionGoal.js');
let GoToPOIActionGoal = require('./GoToPOIActionGoal.js');
let JoyTurboAction = require('./JoyTurboAction.js');
let GoToFeedback = require('./GoToFeedback.js');
let GoToPOIAction = require('./GoToPOIAction.js');
let GoToPOIActionFeedback = require('./GoToPOIActionFeedback.js');
let JoyPriorityActionGoal = require('./JoyPriorityActionGoal.js');
let GoToPOIActionResult = require('./GoToPOIActionResult.js');
let GoToActionResult = require('./GoToActionResult.js');
let JoyTurboResult = require('./JoyTurboResult.js');
let VisualTrainingActionFeedback = require('./VisualTrainingActionFeedback.js');
let JoyPriorityGoal = require('./JoyPriorityGoal.js');
let ExecuteParkingAction = require('./ExecuteParkingAction.js');
let GoToPOIResult = require('./GoToPOIResult.js');
let JoyTurboFeedback = require('./JoyTurboFeedback.js');
let GoToActionGoal = require('./GoToActionGoal.js');
let GoToActionFeedback = require('./GoToActionFeedback.js');
let VisualTrainingActionGoal = require('./VisualTrainingActionGoal.js');
let VisualTrainingAction = require('./VisualTrainingAction.js');
let GoToPOIFeedback = require('./GoToPOIFeedback.js');
let JoyPriorityResult = require('./JoyPriorityResult.js');
let GoToAction = require('./GoToAction.js');
let ExecuteParkingActionFeedback = require('./ExecuteParkingActionFeedback.js');
let ExecuteParkingResult = require('./ExecuteParkingResult.js');
let ExecuteParkingActionResult = require('./ExecuteParkingActionResult.js');
let JoyTurboGoal = require('./JoyTurboGoal.js');
let JoyPriorityActionFeedback = require('./JoyPriorityActionFeedback.js');
let JoyPriorityActionResult = require('./JoyPriorityActionResult.js');
let VisualTrainingActionResult = require('./VisualTrainingActionResult.js');
let JoyTurboActionResult = require('./JoyTurboActionResult.js');
let JoyPriorityFeedback = require('./JoyPriorityFeedback.js');
let LaserImage = require('./LaserImage.js');
let Emergency = require('./Emergency.js');
let POI = require('./POI.js');
let Highways = require('./Highways.js');
let PolarReading = require('./PolarReading.js');
let NiceMapTransformation = require('./NiceMapTransformation.js');
let VisualLocDB = require('./VisualLocDB.js');
let POIGroup = require('./POIGroup.js');
let EulerAnglesStamped = require('./EulerAnglesStamped.js');
let ServiceStatus = require('./ServiceStatus.js');
let AvailableMaps = require('./AvailableMaps.js');
let MapConfiguration = require('./MapConfiguration.js');
let PolarReadingScan = require('./PolarReadingScan.js');
let EulerAngles = require('./EulerAngles.js');
let TabletPOI = require('./TabletPOI.js');
let NavigationStatus = require('./NavigationStatus.js');

module.exports = {
  GoToPOIGoal: GoToPOIGoal,
  ExecuteParkingGoal: ExecuteParkingGoal,
  VisualTrainingGoal: VisualTrainingGoal,
  VisualTrainingResult: VisualTrainingResult,
  JoyPriorityAction: JoyPriorityAction,
  JoyTurboActionGoal: JoyTurboActionGoal,
  ExecuteParkingFeedback: ExecuteParkingFeedback,
  GoToResult: GoToResult,
  GoToGoal: GoToGoal,
  JoyTurboActionFeedback: JoyTurboActionFeedback,
  VisualTrainingFeedback: VisualTrainingFeedback,
  ExecuteParkingActionGoal: ExecuteParkingActionGoal,
  GoToPOIActionGoal: GoToPOIActionGoal,
  JoyTurboAction: JoyTurboAction,
  GoToFeedback: GoToFeedback,
  GoToPOIAction: GoToPOIAction,
  GoToPOIActionFeedback: GoToPOIActionFeedback,
  JoyPriorityActionGoal: JoyPriorityActionGoal,
  GoToPOIActionResult: GoToPOIActionResult,
  GoToActionResult: GoToActionResult,
  JoyTurboResult: JoyTurboResult,
  VisualTrainingActionFeedback: VisualTrainingActionFeedback,
  JoyPriorityGoal: JoyPriorityGoal,
  ExecuteParkingAction: ExecuteParkingAction,
  GoToPOIResult: GoToPOIResult,
  JoyTurboFeedback: JoyTurboFeedback,
  GoToActionGoal: GoToActionGoal,
  GoToActionFeedback: GoToActionFeedback,
  VisualTrainingActionGoal: VisualTrainingActionGoal,
  VisualTrainingAction: VisualTrainingAction,
  GoToPOIFeedback: GoToPOIFeedback,
  JoyPriorityResult: JoyPriorityResult,
  GoToAction: GoToAction,
  ExecuteParkingActionFeedback: ExecuteParkingActionFeedback,
  ExecuteParkingResult: ExecuteParkingResult,
  ExecuteParkingActionResult: ExecuteParkingActionResult,
  JoyTurboGoal: JoyTurboGoal,
  JoyPriorityActionFeedback: JoyPriorityActionFeedback,
  JoyPriorityActionResult: JoyPriorityActionResult,
  VisualTrainingActionResult: VisualTrainingActionResult,
  JoyTurboActionResult: JoyTurboActionResult,
  JoyPriorityFeedback: JoyPriorityFeedback,
  LaserImage: LaserImage,
  Emergency: Emergency,
  POI: POI,
  Highways: Highways,
  PolarReading: PolarReading,
  NiceMapTransformation: NiceMapTransformation,
  VisualLocDB: VisualLocDB,
  POIGroup: POIGroup,
  EulerAnglesStamped: EulerAnglesStamped,
  ServiceStatus: ServiceStatus,
  AvailableMaps: AvailableMaps,
  MapConfiguration: MapConfiguration,
  PolarReadingScan: PolarReadingScan,
  EulerAngles: EulerAngles,
  TabletPOI: TabletPOI,
  NavigationStatus: NavigationStatus,
};

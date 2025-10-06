
"use strict";

let ChangeSyncMap = require('./ChangeSyncMap.js')
let SetSubMapFloor = require('./SetSubMapFloor.js')
let ChangeMap = require('./ChangeMap.js')
let FinalApproachPose = require('./FinalApproachPose.js')
let RegisterSync = require('./RegisterSync.js')
let RenameMap = require('./RenameMap.js')
let GetSubMap = require('./GetSubMap.js')
let SetPOI = require('./SetPOI.js')
let SetMapConfiguration = require('./SetMapConfiguration.js')
let GetMapConfiguration = require('./GetMapConfiguration.js')
let SafetyZone = require('./SafetyZone.js')
let ListMaps = require('./ListMaps.js')
let GetPOI = require('./GetPOI.js')
let GetNodes = require('./GetNodes.js')
let VisualLocRecognize = require('./VisualLocRecognize.js')
let SaveMap = require('./SaveMap.js')
let Acknowledgment = require('./Acknowledgment.js')
let DisableEmergency = require('./DisableEmergency.js')
let ChangeBuilding = require('./ChangeBuilding.js')

module.exports = {
  ChangeSyncMap: ChangeSyncMap,
  SetSubMapFloor: SetSubMapFloor,
  ChangeMap: ChangeMap,
  FinalApproachPose: FinalApproachPose,
  RegisterSync: RegisterSync,
  RenameMap: RenameMap,
  GetSubMap: GetSubMap,
  SetPOI: SetPOI,
  SetMapConfiguration: SetMapConfiguration,
  GetMapConfiguration: GetMapConfiguration,
  SafetyZone: SafetyZone,
  ListMaps: ListMaps,
  GetPOI: GetPOI,
  GetNodes: GetNodes,
  VisualLocRecognize: VisualLocRecognize,
  SaveMap: SaveMap,
  Acknowledgment: Acknowledgment,
  DisableEmergency: DisableEmergency,
  ChangeBuilding: ChangeBuilding,
};


"use strict";

let canAppStart = require('./canAppStart.js')
let isAppRunning = require('./isAppRunning.js')
let shMemIsClosed = require('./shMemIsClosed.js')
let RegisterRemoteContinuousTest = require('./RegisterRemoteContinuousTest.js')
let shMemIsAvailable = require('./shMemIsAvailable.js')
let shMemGetUserId = require('./shMemGetUserId.js')
let finalStartupStatus = require('./finalStartupStatus.js')
let numberOfAnomalies = require('./numberOfAnomalies.js')
let tcpIpIsAvailable = require('./tcpIpIsAvailable.js')
let shMemIsTimestampValid = require('./shMemIsTimestampValid.js')
let lookupTransform = require('./lookupTransform.js')
let supervisorIsAvailable = require('./supervisorIsAvailable.js')
let getAppDependencies = require('./getAppDependencies.js')

module.exports = {
  canAppStart: canAppStart,
  isAppRunning: isAppRunning,
  shMemIsClosed: shMemIsClosed,
  RegisterRemoteContinuousTest: RegisterRemoteContinuousTest,
  shMemIsAvailable: shMemIsAvailable,
  shMemGetUserId: shMemGetUserId,
  finalStartupStatus: finalStartupStatus,
  numberOfAnomalies: numberOfAnomalies,
  tcpIpIsAvailable: tcpIpIsAvailable,
  shMemIsTimestampValid: shMemIsTimestampValid,
  lookupTransform: lookupTransform,
  supervisorIsAvailable: supervisorIsAvailable,
  getAppDependencies: getAppDependencies,
};


"use strict";

let ASRService = require('./ASRService.js')
let SoundLocalisationService = require('./SoundLocalisationService.js')
let GetSpeechDuration = require('./GetSpeechDuration.js')
let recognizerService = require('./recognizerService.js')

module.exports = {
  ASRService: ASRService,
  SoundLocalisationService: SoundLocalisationService,
  GetSpeechDuration: GetSpeechDuration,
  recognizerService: recognizerService,
};

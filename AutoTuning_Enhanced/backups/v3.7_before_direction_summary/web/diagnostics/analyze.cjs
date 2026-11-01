const fs=require('node:fs');
const C=require('../src/core.js');
const file=process.argv[2]||__dirname+'/connection-2026-10-03-three-methods.log';
const lines=fs.readFileSync(file,'utf8').split(/\r?\n/).filter(Boolean);
const summary={lines:lines.length,rxBytes:0,rxFrames:0,crcValidTelemetry:0,fast:0,slow:0,pid:0,recordedPidEvents:[],legacyErrors:[],unknown:[],events:[]};
const pieces=[];
for(let i=0;i<lines.length;i++){
  const line=lines[i];if(/ (GATT|TX_ATTEMPT|WRITE_DONE|RESULT|SYS|FIRMWARE_ERR|RX_ERROR) /.test(line))summary.events.push({line:i+1,text:line});
  if(/^\S+ PID /.test(line))summary.recordedPidEvents.push({line:i+1,text:line});
  const raw=line.match(/ RX_BYTES ([0-9a-f ]+) \|/i);if(raw){summary.rxBytes++;pieces.push(Buffer.from(raw[1].trim().replace(/ /g,''),'hex'));}
  const frame=line.match(/ RX (.*)$/);if(!frame)continue;summary.rxFrames++;
  if(frame[1]==='$ReceivePackError#'){summary.legacyErrors.push({line:i+1,text:line});continue;}
  if(!C.verify(frame[1])){summary.unknown.push({line:i+1,text:line,reason:'CRC invalid'});continue;}
  if(C.parseFast(frame[1])){summary.fast++;summary.crcValidTelemetry++;}
  else if(C.parseSlow(frame[1])){summary.slow++;summary.crcValidTelemetry++;}
  else if(C.parsePidStatus(frame[1]))summary.pid++;
  else summary.unknown.push({line:i+1,text:line,reason:'CRC valid, unclassified'});
}
const stream=Buffer.concat(pieces).toString('ascii');
summary.rawP1Occurrences=[...stream.matchAll(/\$P1,/g)].length;
summary.rawLegacyError=stream.includes('$ReceivePackError#');
console.log(JSON.stringify(summary,null,2));

const {spawnSync}=require("node:child_process");
const path=require("node:path");
const fs=require("node:fs");
const root=path.resolve(__dirname,"..");
const out=path.join(__dirname,"native-bin"); fs.mkdirSync(out,{recursive:true});
function buildAndRun(name,defines=[]) {
  const exe=path.join(out,name+".exe");
  const cc=spawnSync("gcc",["-std=c99","-O1","-Wall","-Wextra","-I",path.join(__dirname,"native_stubs"),"-I",path.join(root,"firmware","APP","PID"),...defines,path.join(__dirname,"pid_update_native.c"),"-lm","-o",exe],{encoding:"utf8"});
  if(cc.status!==0) throw new Error(cc.stdout+cc.stderr);
  if(cc.stderr) process.stdout.write(cc.stderr);
  const result=spawnSync(exe,[],{encoding:"utf8"});
  process.stdout.write(result.stdout+result.stderr);
  return result.status;
}
if(process.argv.includes("--prove-old-bug")) {
  const backup=path.join(root,"backups","20261014_before_pid_continuity_fix","firmware","APP").replace(/\\/g,"/");
  const result=buildAndRun("before_fix",['-DROUTE_SOURCE="'+backup+'/RouteA/route_a.c"','-DPID_SOURCE="'+backup+'/PID/pid_control.c"']);
  if(result===0) throw new Error("Old bug was not reproduced");
  console.log("Confirmed: pre-fix real C source fails identical-PID live-state invariant.");
}
if(buildAndRun("after_fix")!==0) process.exit(1);

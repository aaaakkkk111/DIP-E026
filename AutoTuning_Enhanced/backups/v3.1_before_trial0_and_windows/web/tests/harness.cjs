const fs=require('node:fs');
const vm=require('node:vm');
const {webcrypto}=require('node:crypto');
const path=require('node:path');
class Element {
  constructor(doc,tag,attrs={}) { this.doc=doc;this.tagName=tag;this.id=attrs.id||'';this.dataset={};for(const[k,v]of Object.entries(attrs))if(k.startsWith('data-'))this.dataset[k.slice(5)]=v;this.value=attrs.value||'';this.disabled='disabled' in attrs;this.checked=false;this.listeners={};this.textContent='';this.clientWidth=tag==='canvas'?300:0;this.clientHeight=tag==='canvas'?160:0;this.width=300;this.height=160;this.classes=new Set((attrs.class||'').split(' '));this.classList={add:k=>this.classes.add(k),remove:k=>this.classes.delete(k),toggle:(k,on)=>{on?this.classes.add(k):this.classes.delete(k);},contains:k=>this.classes.has(k)}; }
  set innerHTML(v){this._html=v;this.doc.parse(v);}
  get innerHTML(){return this._html||'';}
  addEventListener(name,fn){(this.listeners[name]??=[]).push(fn);}
  removeEventListener(name,fn){this.listeners[name]=(this.listeners[name]||[]).filter(x=>x!==fn);}
  async dispatch(name,props={}){const event={target:this,preventDefault(){},pointerId:1,...props};await Promise.all((this.listeners[name]||[]).map(fn=>fn(event)));}
  async click(){if(this.tagName==='a'){this.doc.downloads.push({name:this.download,url:this.href});return;}if(this.disabled)throw Error('Disabled element '+this.id+' '+JSON.stringify(this.dataset));await this.dispatch('click');}
  querySelector(){return this.child??=new Element(this.doc,'b');}
  getContext(){return new Proxy({},{get:()=>()=>{},set:()=>true});}
  setPointerCapture(){} remove(){} appendChild(){}
}
function documentModel(html){const doc={elements:[],downloads:[],hidden:false,listeners:{},body:{appendChild(){}},parse(text){for(const m of text.matchAll(/<(\w+)\b([^>]*?)>/g)){const attrs={};for(const a of m[2].matchAll(/([\w-]+)(?:="([^"]*)")?/g))attrs[a[1]]=a[2]??'';const e=new Element(this,m[1],attrs);if(e.id){this.elements=this.elements.filter(x=>x.id!==e.id);}this.elements.push(e);}},getElementById(id){const e=this.elements.find(x=>x.id===id);if(!e)throw Error('Missing DOM id '+id);return e;},querySelectorAll(selector){if(selector==='nav button')return this.elements.filter(e=>e.dataset.nav);if(selector==='.page')return this.elements.filter(e=>e.classes.has('page'));const m=selector.match(/^\[([\w-]+)(?:="([^"]*)")?\]$/);if(m)return this.elements.filter(e=>{const k=m[1].slice(5);return e.dataset[k]!==undefined&&(m[2]===undefined||e.dataset[k]===m[2]);});throw Error('Unknown selector '+selector);},querySelector(s){return this.querySelectorAll(s)[0]||null;},addEventListener(k,fn){(this.listeners[k]??=[]).push(fn);},createElement(tag){return new Element(this,tag);},async dispatch(k){await Promise.all((this.listeners[k]||[]).map(fn=>fn({})));}};doc.parse(html);doc.getElementById('loopReview').value='rules-auto';return doc;}
function make(options={}){
  let now=1800000000000,nextId=1;const timers=new Map(),errors=[],saved={},downloads=[];
  const doc=documentModel(fs.readFileSync(path.join(__dirname,'../src/template.html'),'utf8'));
  const setTimer=(fn,ms,interval)=>{const id=nextId++;timers.set(id,{fn,at:now+ms,ms,interval});return id;};
  class ClockDate extends Date{constructor(...args){super(...(args.length?args:[now]));}static now(){return now;}}
  const LocalURL=class extends URL{static createObjectURL(blob){const id='blob:local/'+downloads.length;downloads.push({id,blob});return id;}static revokeObjectURL(){}};
  const context=vm.createContext({document:doc,navigator:{},location:{href:'http://localhost:8787/',origin:'http://localhost:8787',protocol:'http:',search:''},Date:ClockDate,TextEncoder,TextDecoder,URL:LocalURL,URLSearchParams,Blob,AbortController,Uint8Array,Uint32Array,crypto:webcrypto,structuredClone,console,
    localStorage:{getItem:k=>saved[k]??null,setItem:(k,v)=>saved[k]=v},confirm:()=>options.confirm!==false,
    setTimeout:(f,m)=>setTimer(f,m,false),setInterval:(f,m)=>setTimer(f,m,true),clearTimeout:id=>timers.delete(id),clearInterval:id=>timers.delete(id),
    fetch:async(...args)=>options.fetch?options.fetch(...args):{ok:true,status:200,json:async()=>({choices:[{message:{content:'{"ok":true}'}}]})},isSecureContext:true,devicePixelRatio:1,
    addEventListener:(k,f)=>(context.events[k]??=[]).push(f),events:{},scrollTo(){}
  });context.window=context;
  for(const f of ['core.js','simulator.js','app.js'])vm.runInContext(fs.readFileSync(path.join(__dirname,'../src',f),'utf8'),context,{filename:f});
  const flush=async()=>{for(let i=0;i<30;i++)await Promise.resolve();};
  const advance=async ms=>{const end=now+ms;let iterations=0;while(true){let id,item;for(const[k,v]of timers)if(v.at<=end&&(!item||v.at<item.at)){id=k;item=v;}if(!item)break;if(++iterations>20000)throw Error('timer loop');now=item.at;if(item.interval)item.at+=item.ms;else timers.delete(id);try{const result=item.fn();if(result?.catch)result.catch(e=>errors.push(e));}catch(e){errors.push(e);}await flush();}now=end;await flush();};
  const action=name=>doc.querySelector(`[data-action="${name}"]`).click();
  function installBle(simOptions={}){
    let buffer='';const writes=[],ch={properties:{notify:true,writeWithoutResponse:true},listeners:{},addEventListener(k,f){this.listeners[k]=f;},removeEventListener(k){delete this.listeners[k];},async startNotifications(){},async writeValueWithoutResponse(bytes){writes.push([...bytes]);buffer+=new TextDecoder().decode(bytes);if(buffer.endsWith('#')){const raw=buffer;buffer='';sim.receive(raw);}}};
    const emit=raw=>{for(let i=0;i<raw.length;i+=11){const bytes=new TextEncoder().encode(raw.slice(i,i+11));ch.listeners.characteristicvaluechanged?.({target:{value:new DataView(bytes.buffer)}});}};
    const sim=new context.RouteASimulator.Simulator(emit,simOptions),dev={name:'YahBoom_BL-test',listeners:{},gatt:{connected:false,async connect(){this.connected=true;return {async getPrimaryService(){return {async getCharacteristic(){return ch;}};}};},disconnect(){this.connected=false;dev.listeners.gattserverdisconnected?.();}},addEventListener(k,f){this.listeners[k]=f;},removeEventListener(k){delete this.listeners[k];}};
    context.navigator.bluetooth={requestDevice:async()=>dev};doc.getElementById('chunkDelay').value='0';
    return {sim,ch,dev,writes,emit,tick:ms=>sim.tick(ms)};
  }
  return {doc,context,errors,saved,downloads,action,advance,flush,installBle,snapshot:()=>context.RouteAApp.snapshot(),set:id=>doc.getElementById(id)};
}
module.exports={make};

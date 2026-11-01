import {readFile,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
const template=await readFile(new URL('./template.html',import.meta.url),'utf8');
const scripts=await Promise.all(['../src/core.js','./get.js'].map(p=>readFile(new URL(p,import.meta.url),'utf8')));
const html=template.replace('<!-- DIAGNOSTIC_SCRIPTS -->',()=>scripts.map(s=>'<script>\n'+s+'\n</script>').join('\n'));
const output=new URL('../../routea_bluefy_diagnostics.html',import.meta.url);
await writeFile(output,html,'utf8');
console.log('Built '+fileURLToPath(output));

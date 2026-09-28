import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const manifest=JSON.parse(await fs.readFile(path.join(root,'MANIFEST.json'),'utf8'));
const errors=[];
for(const row of manifest.files){
 try{const data=await fs.readFile(path.join(root,row.path));if(data.length!==row.bytes||createHash('sha256').update(data).digest('hex')!==row.sha256)errors.push('Changed: '+row.path);}
 catch{errors.push('Missing: '+row.path);}
}
for(const filename of ['assets.json','models/standard/assets.json']){
 const data=JSON.parse(await fs.readFile(path.join(root,'app',filename),'utf8'));
 for(const mesh of Object.values(data.meshes))try{await fs.access(path.join(root,'app',mesh.file));}catch{errors.push('Mesh not found: '+mesh.file);}
}
if(errors.length){console.error(errors.join('\n'));process.exitCode=1;}
else console.log('PASS: '+manifest.files.length+' files match SHA-256; both model mesh manifests resolve.');

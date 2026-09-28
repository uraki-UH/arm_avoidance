let directions=null,metadata=null,loading=null;
export function installMeasuredScan(data,meta){if(data.length!==meta.slots*3)throw Error('MID-360参照データの長さが不正です');directions=data;metadata=meta;return meta;}
export function measuredMetadata(){return metadata;}
export async function loadMeasuredScan(){
 if(metadata)return metadata;if(loading)return loading;
 loading=(async()=>{const [a,b]=await Promise.all([fetch(new URL('./assets/mid360/measured-directions.f32',import.meta.url)),fetch(new URL('./assets/mid360/measured-directions.json',import.meta.url))]);if(!a.ok||!b.ok)throw Error('MID-360実測走査データを読み込めません');return installMeasuredScan(new Float32Array(await a.arrayBuffer()),await b.json());})().catch(e=>{loading=null;throw e;});return loading;
}
export function measuredDirection(slot,out){if(!directions)throw Error('MID-360実測走査データが未読込です');const i=((Math.round(slot)%metadata.slots)+metadata.slots)%metadata.slots;return out.fromArray(directions,i*3).normalize();}

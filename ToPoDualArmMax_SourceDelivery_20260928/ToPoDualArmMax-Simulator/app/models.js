export const ROBOT_MODELS=Object.freeze({
 standard:{id:'standard',label:'標準',title:'ToPoDualArm-Max',urdf:'models/standard/source.urdf',assets:'models/standard/assets.json',reference:'models/standard/qa-fk-reference.json',exportName:'ToPoDualArmMax'},
 long:{id:'long',label:'Long',title:'ToPoDualArm-Max Long',urdf:'source.urdf',assets:'assets.json',reference:'qa-fk-reference.json',exportName:'ToPoDualArmMax-Long'}
});
export function initialModel(){const id=new URLSearchParams(location.search).get('model');return Object.hasOwn(ROBOT_MODELS,id)?ROBOT_MODELS[id]:ROBOT_MODELS.long;}

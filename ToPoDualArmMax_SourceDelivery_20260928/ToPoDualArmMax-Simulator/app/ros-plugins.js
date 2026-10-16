import {format_fixed} from './display-format.js';
import {bridge_socket_url} from './robot-ros-state.js';

let loaded_catalog=[];
const catalog_request=()=>fetch('/api/plugins',{cache:'no-store'}).then(async response=>{
 if(!response.ok)throw Error('プラグイン一覧を取得できません。ブラウザサーバーを更新してください');
 const data=await response.json();if(data.version!==1||!Array.isArray(data.plugins))throw Error('プラグイン一覧の形式不正');
 loaded_catalog=data.plugins;return loaded_catalog;
});

// 定義の型・選択肢・範囲による設定欄の生成。方式固有の計算処理なし
export function parameter_fields(spec,on_change=()=>{}){
 const element=document.createElement('div'),inputs=new Map();element.className='plugin-fields';
 for(const [name,field] of Object.entries(spec.parameters??{})){
  if(field.is_hidden)continue;
  const label=document.createElement('label'),input=document.createElement(field.choices?'select':'input');
  label.textContent=field.label??name;input.id=field.ui_id??'plugin-'+spec.id+'-'+name;input.dataset.parameter=name;
  if(field.choices)for(const value of field.choices){const option=document.createElement('option');option.value=value;option.textContent=value;input.append(option);}
  else{input.type=field.type==='bool'?'checkbox':['number','integer'].includes(field.type)?'number':'text';if(input.type==='number'){input.step=field.type==='integer'?'1':'any';if(field.min!==undefined)input.min=field.min;if(field.max!==undefined)input.max=field.max;}}
  if(field.type==='bool')input.checked=field.default??false;else input.value=field.default??'';
  input.setAttribute('aria-label',field.label??name);input.onchange=on_change;label.append(input);element.append(label);inputs.set(name,input);
 }
 return {element,inputs,read:()=>{
  const result={};for(const [name,input] of inputs){
   const field=spec.parameters[name];let value=field.type==='bool'?(field.choices?input.value==='true':input.checked):['number','integer'].includes(field.type)?(field.choices?Number(input.value):input.valueAsNumber):input.value.trim();
   if(!input.checkValidity()||typeof value==='number'&&(!Number.isFinite(value)||field.type==='integer'&&!Number.isInteger(value)))throw Error('設定値が不正です: '+(field.label??name));
   result[name]=value;
  }return result;
 }};
}

// 状態・実速度の表示整形。回避や制御の判定への使用なし
export function format_plugin_status(frame,spec){
 const state=frame.avoidance,display=spec?.display??{},velocities=Object.entries(frame.motion?.velocity??{}).filter(([name])=>!display.velocity_pattern||new RegExp(display.velocity_pattern).test(name));
 const max_speed=velocities.length?Math.max(...velocities.map(([,value])=>Math.abs(value))):null;
 const min_motion_speed_th=.001,is_moving=max_speed!==null&&max_speed>min_motion_speed_th;
 let phase=display.phases?.[state.phase]??state.phase??'';
 if(typeof phase==='object')phase=is_moving?phase.moving:phase.holding;
 const parts=['MuJoCo',display.label??spec?.label??state.mode,phase].filter(Boolean);
 if(display.enable_motion_status)parts.push(max_speed===null?'実速度未取得':(is_moving?'動作中':'姿勢保持')+' · 腕 '+(max_speed*180/Math.PI).toFixed(1)+' deg/s');
 for(const field of display.fields??[])if(Number.isFinite(state[field.name]))parts.push(field.label+' '+format_fixed(state[field.name],field.decimals??0)+(field.unit?' '+field.unit:''));
 return parts.join(' · ')+(state.message?'\n'+state.message:'');
}
export function plugin_definition(id){return loaded_catalog.find(spec=>spec.id===id);}

export class physics_plugins{
 constructor(panel){
  this.panel=panel;this.fields=new Map();this.specs=[];this.select=document.getElementById('physics-avoidance');this.container=document.getElementById('physics-plugin-settings');
  this.select.onchange=()=>{panel.stop();this.update();};
  this.ready=catalog_request().then(catalog=>{
   this.specs=catalog.filter(spec=>spec.category==='physics');
   for(const spec of this.specs){
    const option=document.createElement('option');option.value=spec.id;option.textContent=spec.label;this.select.append(option);
    const fields=parameter_fields(spec,()=>panel.stop()),details=document.createElement('details'),summary=document.createElement('summary');
    details.id=spec.settings_id??'physics-plugin-'+spec.id;summary.textContent=spec.label+'の設定';details.append(summary,fields.element);this.container.append(details);this.fields.set(spec.id,{...fields,details});
   }
   this.launch_panel=new launch_plugins_panel(catalog.filter(spec=>spec.category==='launch'));
   this.update();
  }).catch(error=>{document.getElementById('physics-status').textContent=error.message;});
 }
 definition(){return this.specs.find(spec=>spec.id===this.select.value);}
 update(){
  const model=this.panel.robot().modelId;
  for(const option of this.select.options){const spec=this.specs.find(value=>value.id===option.value);option.disabled=!!spec?.models&&!spec.models.includes(model);}
  if(this.select.selectedOptions[0]?.disabled)this.select.value='none';
  for(const [id,fields] of this.fields){fields.details.hidden=id!==this.select.value;for(const input of fields.inputs.values())input.disabled=!this.panel.enable_robot_physics;}
 }
 settings(){const spec=this.definition();return spec?{mode:spec.id,...this.fields.get(spec.id).read()}:{mode:'none'};}
 can_set_goal(){return this.definition()?.capabilities?.can_set_goal_pose===true;}
 validate_source(source){const spec=this.definition();if(spec?.capabilities?.pose_sources&&!spec.capabilities.pose_sources.includes(source))throw Error(spec.label+'はシミュレータ操作の姿勢で開始してください');}
 format(frame){return format_plugin_status(frame,this.specs.find(spec=>spec.id===frame.avoidance.mode));}
}

export class launch_plugins_panel{
 constructor(specs){
  this.specs=specs;this.socket=null;this.phase='idle';this.fields=null;
  const section=this.section=document.createElement('section');section.className='ros-launch-plugins';
  section.innerHTML='<h3>ROS launch プラグイン</h3><label>機能<select data-field="plugin"></select></label><p data-field="description" class="sub-note"></p><div data-field="parameters"></div><div class="row-actions"><button data-field="start">開始</button><button data-field="stop" disabled>停止</button></div><p data-field="status" role="status">未起動</p><details><summary>ログ</summary><pre data-field="log"></pre></details>';
  document.getElementById('ros-panel').append(section);this.elements=Object.fromEntries([...section.querySelectorAll('[data-field]')].map(element=>[element.dataset.field,element]));
  for(const spec of specs){const option=document.createElement('option');option.value=spec.id;option.textContent=spec.label;this.elements.plugin.append(option);}
  this.elements.plugin.onchange=()=>this.select_plugin();this.elements.start.onclick=()=>this.start();this.elements.stop.onclick=()=>this.stop();this.select_plugin();
  document.getElementById('ros-endpoint').addEventListener('change',()=>this.disconnect('接続先変更のため停止'));
  window.addEventListener('pagehide',()=>this.disconnect('ページ終了'),{once:true});
 }
 select_plugin(){
  const spec=this.specs.find(value=>value.id===this.elements.plugin.value);this.elements.description.textContent=spec?.description??'登録されたlaunchがありません';
  this.fields=spec?parameter_fields(spec):null;this.elements.parameters.replaceChildren(...(this.fields?[this.fields.element]:[]));this.update_controls();
 }
 update_controls(){
  const is_active=['starting','running','stopping'].includes(this.phase);this.elements.plugin.disabled=is_active;this.elements.start.disabled=is_active||!this.fields;this.elements.stop.disabled=!is_active;
  for(const input of this.fields?.inputs.values()??[])input.disabled=is_active;
 }
 start(){
  try{
   const request={type:'start',plugin_id:this.elements.plugin.value,parameters:this.fields.read()};this.phase='starting';this.update_controls();this.elements.status.textContent='開始中';this.elements.log.textContent='';
   if(this.socket?.readyState===WebSocket.OPEN){this.socket.send(JSON.stringify(request));return;}
   const socket=this.socket=new WebSocket(bridge_socket_url(document.getElementById('ros-endpoint').value,'/plugins'));
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify(request));};
   socket.onmessage=event=>{
    if(this.socket!==socket)return;
    try{const value=JSON.parse(event.data);if(value.type==='error'){this.elements.log.textContent=value.log_tail??'';throw Error(value.error);}if(value.type!=='status'||value.phase==='idle'&&this.phase==='starting')return;
     this.phase=value.phase;const labels={idle:'未起動',starting:'準備中',running:'実行中',stopped:'停止済み',failed:'失敗',exited:'終了'};this.elements.status.textContent=labels[value.phase]??value.phase;
     this.elements.log.textContent=value.log_tail??'';this.update_controls();
    }catch(error){this.disconnect(error.message);}
   };
   socket.onerror=()=>{if(this.socket===socket)this.disconnect('プラグイン実行ブリッジへ接続できません。ブリッジを更新してください');};
   socket.onclose=()=>{if(this.socket===socket)this.disconnect('プラグイン接続終了');};
  }catch(error){this.disconnect(error.message);}
 }
 stop(){if(this.socket?.readyState===WebSocket.OPEN){this.phase='stopping';this.elements.status.textContent='停止中';this.update_controls();this.socket.send(JSON.stringify({type:'stop'}));}else this.disconnect('停止済み');}
 disconnect(message){const socket=this.socket;this.socket=null;socket?.close();this.phase='stopped';this.elements.status.textContent=message;this.update_controls();}
}

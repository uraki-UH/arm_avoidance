import fs from 'node:fs/promises';

// 実行中ブラウザへのCLI操作。GUIと共通の設定・指令検証、実機出力なし。
const args=process.argv.slice(2),take=name=>{const idx=args.indexOf(name);if(idx<0)return null;const value=args[idx+1];if(!value||value.startsWith('--'))throw Error(name+'の値が必要です');args.splice(idx,2);return value;};
try{
 const endpoint=take('--url')??'http://127.0.0.1:8877',requested=take('--session');
 const action=args.shift()??'status';
 if(action==='help'||action==='--help'){
  console.log('node tools/robot_control.mjs [--url http://127.0.0.1:8877] [--session ID] status|configure|start|stop|command|goal\nconfigure: --usage simulation|observation|preview --source browser|ros|leader|avoidance|api --mode position|position_velocity|velocity|torque --gravity true|false --topic /sim/joint_command\ncommand: --file command.json または --json \'{"mode":"position","position":{"L_joint1":0.2}}\'');
 }else{
  const url=new URL('/api/robot',endpoint),response=await fetch(url,{signal:AbortSignal.timeout(5000)});
  if(!response.ok)throw Error('画面サーバーのロボットAPIへ接続できません。サーバー更新とブラウザ再読み込みを確認してください');
  const {sessions}=await response.json();
  const active=sessions.filter(session=>session.is_online&&(!requested||session.session_id===requested));
  if(action==='status'){if(args.length)throw Error('未対応の引数: '+args.join(' '));console.log(JSON.stringify(requested?active:sessions,null,2));}
  else{
   if(active.length!==1)throw Error('操作対象を1つにしてください。statusで確認し、--session IDを指定してください');
   let value={type:action,model:active[0].state.model};
   if(action==='configure'){
    const config={};
    for(const [flag,key] of [['--usage','usage'],['--source','command_source'],['--mode','control_mode'],['--topic','command_topic']]){const item=take(flag);if(item!==null)config[key]=item;}
    const gravity=take('--gravity');if(gravity!==null){if(!['true','false'].includes(gravity))throw Error('--gravityはtrue / falseです');config.enable_gravity_compensation=gravity==='true';}
    if(config.control_mode&&!Object.hasOwn(config,'enable_gravity_compensation'))config.enable_gravity_compensation=config.control_mode!=='torque';
    value.config=config;
   }else if(action==='command'||action==='goal'){
    const file=take('--file'),json=take('--json');if((file===null)===(json===null))throw Error('--file または --jsonの一方が必要です');
    value[action==='goal'?'position':'command']=JSON.parse(file?await fs.readFile(file,'utf8'):json);
   }else if(!['start','stop'].includes(action))throw Error('未対応の操作です');
   if(args.length)throw Error('未対応の引数: '+args.join(' '));
   const result=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json','X-ToPo-Robot':'1'},body:JSON.stringify({session_id:active[0].session_id,action:value}),signal:AbortSignal.timeout(25000)});
   const body=await result.json();if(!result.ok||body.error)throw Error(body.error??'操作失敗');console.log(JSON.stringify(body.result,null,2));
  }
 }
}catch(error){console.error(error.message);process.exitCode=1;}

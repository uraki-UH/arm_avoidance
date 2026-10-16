// 行ごとの項目数をn・n+1へ均等配分。項目数の多い行は後方。
export function get_balanced_row_sizes(num_items,max_num_columns=4){
 if(!Number.isSafeInteger(num_items)||num_items<0||!Number.isSafeInteger(max_num_columns)||max_num_columns<1)throw new RangeError('項目数と最大列数の指定不正');
 if(num_items===0)return [];
 const num_rows=Math.ceil(num_items/max_num_columns),num_small=Math.floor(num_items/num_rows),num_large_rows=num_items%num_rows;
 return Array.from({length:num_rows},(_,idx)=>num_small+Number(idx>=num_rows-num_large_rows));
}

// 任意の直下要素の均等配置。追加・削除・hidden変更への追従、幅測定なし。
// 戻り値のdisconnect()による監視解除。
export function observe_balanced_layout(container,max_num_columns=4){
 get_balanced_row_sizes(0,max_num_columns);
 const update_layout=()=>{
  const items=Array.from(container.children).filter(item=>!item.hidden),rows=get_balanced_row_sizes(items.length,max_num_columns);
  if(!rows.length){container.style.removeProperty('grid-template-columns');return;}
  const num_small=rows[0],num_large=rows.at(-1),num_columns=num_small===num_large?num_small:num_small*num_large;
  container.style.display='grid';
  // 単行のラベル幅確保と、複数行の共通分割。
  container.style.gridTemplateColumns=`repeat(${num_columns},minmax(${rows.length===1?'max-content':'0'},1fr))`;
  let idx=0;
  for(const num_items of rows)for(let offset=0;offset<num_items;offset++)items[idx++].style.gridColumn=`span ${num_columns/num_items}`;
 };
 const observer=new MutationObserver(records=>{
  if(records.some(record=>record.type==='childList'?record.target===container:record.target.parentElement===container))update_layout();
 });
 observer.observe(container,{childList:true,attributes:true,attributeFilter:['hidden'],subtree:true});
 update_layout();
 return observer;
}

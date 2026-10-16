// 表示桁でゼロへ丸まる値の符号除去。計算値と表示精度の保持。
export function format_fixed(value,num_decimals=0){
 const text=value.toFixed(num_decimals);
 return text.startsWith('-')&&Number(text)===0?text.slice(1):text;
}

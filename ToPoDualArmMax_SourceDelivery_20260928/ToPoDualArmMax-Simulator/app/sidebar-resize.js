// 操作パネル左縁の幅調整。ドラッグ・キーボード操作と小画面への追従
const sidebar=document.querySelector('aside'),main=document.querySelector('main');
const handle=document.createElement('div');handle.className='sidebar-resize';handle.tabIndex=0;
handle.setAttribute('role','separator');handle.setAttribute('aria-orientation','vertical');handle.setAttribute('aria-label','操作パネルの幅');
sidebar.id||='controls-sidebar';handle.setAttribute('aria-controls',sidebar.id);handle.title='ドラッグで幅調整。左右キーでも調整、Homeで既定幅';
main.insertBefore(handle,sidebar);
const min_width=240,default_width=365;
let preferred_width=default_width,drag=null;
const max_width=()=>Math.max(min_width,Math.min(760,window.innerWidth-320));
const apply_width=width=>{
 preferred_width=Math.max(min_width,Math.min(max_width(),width));
 main.style.setProperty('--sidebar-width',preferred_width+'px');
 handle.setAttribute('aria-valuemin',String(min_width));handle.setAttribute('aria-valuemax',String(max_width()));
 handle.setAttribute('aria-valuenow',String(Math.round(preferred_width)));handle.setAttribute('aria-valuetext',Math.round(preferred_width)+' px');
};
handle.addEventListener('pointerdown',event=>{
 if(event.button!==0)return;event.preventDefault();event.stopPropagation();
 drag={pointer_id:event.pointerId,start_x:event.clientX,start_width:sidebar.getBoundingClientRect().width};
 handle.setPointerCapture(event.pointerId);document.body.classList.add('is-sidebar-resizing');
});
handle.addEventListener('pointermove',event=>{if(drag?.pointer_id===event.pointerId)apply_width(drag.start_width+drag.start_x-event.clientX);});
const finish_drag=()=>{const pointer_id=drag?.pointer_id;drag=null;document.body.classList.remove('is-sidebar-resizing');if(pointer_id!==undefined&&handle.hasPointerCapture(pointer_id))handle.releasePointerCapture(pointer_id);};
for(const event of ['pointerup','pointercancel','lostpointercapture'])handle.addEventListener(event,finish_drag);
window.addEventListener('blur',finish_drag);
handle.addEventListener('keydown',event=>{
 const step=event.shiftKey?50:10;
 if(event.key==='ArrowLeft')apply_width(preferred_width+step);
 else if(event.key==='ArrowRight')apply_width(preferred_width-step);
 else if(event.key==='Home')apply_width(default_width);
 else if(event.key==='End')apply_width(max_width());
 else return;event.preventDefault();
});
window.addEventListener('resize',()=>{finish_drag();apply_width(preferred_width);});
apply_width(preferred_width);
const stylesheet=document.createElement('link');stylesheet.rel='stylesheet';stylesheet.href=new URL('./sidebar-resize.css',import.meta.url);document.head.append(stylesheet);

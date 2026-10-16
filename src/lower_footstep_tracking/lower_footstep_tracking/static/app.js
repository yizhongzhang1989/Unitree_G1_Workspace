'use strict';
const element = id => document.getElementById(id);
const canvas = element('map');
const context = canvas.getContext('2d');
const labels = {IDLE:'未接管',STAND:'预备完成',RUNNING:'跟踪中',STOPPED:'已停止',ERROR:'故障',standing:'站立保持',starting:'升频起步',walking:'行走',closing:'收脚降频'};
let snapshot = null;
let mode = 'select';
let draft = [];
let route = [];
let routePreview = [];
let routePreviewPending = false;
let routePreviewError = '';
let routePreviewRequest = 0;
let selected = -1;
let editing = false;
let draftRevision = null;
let camera = null;
let scale = 180;
let drag = null;
let queueSignature = '';
let busy = false;
let noticeUntil = 0;
let drawnLabels = [];
let thetaPending = null;
let thetaBusy = false;
const radians = degrees => degrees * Math.PI / 180;
const degrees = radiansValue => radiansValue * 180 / Math.PI;
const clone = value => JSON.parse(JSON.stringify(value));
const icons = () => globalThis.lucide?.createIcons();
function notify(message, error = false) {
  const notice = element('notice');
  notice.textContent = message;
  notice.className = error ? 'error' : '';
  notice.hidden = false;
  noticeUntil = Date.now() + 7000;
}
async function command(action, data = {}) {
  const emergency = action === 'estop';
  if (busy && !emergency) return false;
  if (!emergency) busy = true;
  try {
    const response = await fetch('/api/command', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...data})});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    await poll();
    return true;
  } catch (error) {
    notify(error.message, true);
    return false;
  } finally {
    if (!emergency) busy = false;
  }
}
function meanYaw(feet) {
  if (!feet?.length) return snapshot?.footstep.robot_pose?.[2] || 0;
  const cosine = Math.cos(feet[0][2]) + Math.cos(feet[1][2]);
  const sine = Math.sin(feet[0][2]) + Math.sin(feet[1][2]);
  return Math.hypot(cosine, sine) <= 1e-6 ? feet[0][2] : Math.atan2(sine, cosine);
}
function currentCamera() {
  const robot = snapshot?.footstep.robot_pose || [0,0,0];
  const position = camera || robot;
  return [position[0],position[1],meanYaw(snapshot?.footstep.feet)];
}
function local(pose, anchor = currentCamera()) {
  const deltaX = pose[0] - anchor[0], deltaY = pose[1] - anchor[1];
  const cosine = Math.cos(anchor[2]), sine = Math.sin(anchor[2]);
  return [cosine*deltaX+sine*deltaY,-sine*deltaX+cosine*deltaY,pose[2]-anchor[2]];
}
function world(pose, anchor = currentCamera()) {
  const cosine = Math.cos(anchor[2]), sine = Math.sin(anchor[2]);
  return [anchor[0]+cosine*pose[0]-sine*pose[1],anchor[1]+sine*pose[0]+cosine*pose[1],anchor[2]+(pose[2]||0)];
}
function screen(pose) {
  const offset = local(pose);
  return [canvas.clientWidth/2-offset[1]*scale,canvas.clientHeight/2-offset[0]*scale];
}
function pointer(event) {
  const bounds = canvas.getBoundingClientRect();
  return world([(bounds.height/2-(event.clientY-bounds.top))/scale,(bounds.width/2-(event.clientX-bounds.left))/scale,0]);
}
function headingHandle(pose) { return [pose[0]+.16*Math.cos(pose[2]),pose[1]+.16*Math.sin(pose[2]),pose[2]]; }
function drawArrow(start, end, color, dashed = false) {
  const first = screen(start), second = screen(end);
  context.strokeStyle = color;
  context.lineWidth = 1.6;
  context.setLineDash(dashed ? [4,4] : []);
  context.beginPath(); context.moveTo(...first); context.lineTo(...second); context.stroke();
  context.setLineDash([]);
  const angle = Math.atan2(second[1]-first[1],second[0]-first[0]);
  context.beginPath(); context.moveTo(...second);
  context.lineTo(second[0]-7*Math.cos(angle-.45),second[1]-7*Math.sin(angle-.45));
  context.moveTo(...second);context.lineTo(second[0]-7*Math.cos(angle+.45),second[1]-7*Math.sin(angle+.45));context.stroke();
}
function footprint(step, type, index) {
  const [screenX,screenY] = screen(step.pose);
  const yaw = local(step.pose)[2];
  const color = type === 'hold' ? '#6d7c76' : type === 'draft' ? '#b08219' : step.side === 0 ? '#119879' : '#db6556';
  context.save();context.translate(screenX,screenY);context.rotate(-yaw);
  context.fillStyle = type === 'hold' ? '#ffffff44' : type === 'actual' ? color : type === 'draft' ? '#fff8e1' : '#ffffffdd';
  context.strokeStyle = type === 'committed' ? '#6d7c76' : color;
  context.lineWidth = type === 'actual' || (type === 'draft' && index === selected) ? 2 : 1.3;
  context.setLineDash(type === 'hold' ? [5,3] : type === 'draft' || type === 'preview' ? [3,2] : []);
  context.beginPath();context.roundRect(-.05*scale,-.11*scale,.1*scale,.22*scale,3);context.fill();context.stroke();
  context.setLineDash([]);context.beginPath();context.moveTo(0,.035*scale);context.lineTo(0,-.07*scale);context.lineTo(-4,-.045*scale);context.moveTo(0,-.07*scale);context.lineTo(4,-.045*scale);context.stroke();
  context.restore();
  context.font = '11px "IBM Plex Mono", monospace';context.textAlign='center';context.fillStyle=color;
  const labelX=type === 'actual' ? screenX : screenX+(step.side===0?-1:1)*(.05*scale+10);
  const labelY=type === 'actual' ? screenY+.11*scale+14 : screenY+4;
  if(!drawnLabels.some(position=>Math.abs(position[0]-labelX)<34&&Math.abs(position[1]-labelY)<15)) {
    const side = step.side === 0 ? 'L' : 'R';
    context.fillText(type === 'hold' ? `计划${side}` : type === 'actual' ? side : `${index+1}${side}`,labelX,labelY);
    drawnLabels.push([labelX,labelY]);
  }
  if (type === 'draft' && index === selected) {
    const handle = screen(headingHandle(step.pose));
    drawArrow(step.pose,headingHandle(step.pose),color);
    context.beginPath();context.arc(...handle,5,0,2*Math.PI);context.fillStyle='#fff';context.fill();context.strokeStyle=color;context.stroke();
  }
}
function draw() {
  drawnLabels=[];
  const width = canvas.clientWidth, height = canvas.clientHeight;
  const ratio = devicePixelRatio || 1;
  if (canvas.width !== Math.round(width*ratio) || canvas.height !== Math.round(height*ratio)) {
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
  }
  context.setTransform(ratio,0,0,ratio,0,0);
  context.clearRect(0,0,width,height);
  context.fillStyle='#f2f6f4';context.fillRect(0,0,width,height);
  const grid = scale*.1;
  context.fillStyle='#cddbd4';
  for (let horizontal=width/2%grid;horizontal<width;horizontal+=grid) {
    for (let vertical=height/2%grid;vertical<height;vertical+=grid) {
      context.beginPath();context.arc(horizontal,vertical,.8,0,2*Math.PI);context.fill();
    }
  }
  context.strokeStyle='#c1d0c8';context.lineWidth=1;context.beginPath();context.moveTo(width/2,44);context.lineTo(width/2,height-20);context.moveTo(18,height/2);context.lineTo(width-18,height/2);context.stroke();
  context.fillStyle='#6c7e75';context.font='11px "IBM Plex Mono", monospace';context.textAlign='left';context.fillText('+X',width/2+6,56);context.fillText('+Y',22,height/2-8);
  for (let position=.5;position<height/scale/2;position+=.5) context.fillText(`${position.toFixed(1)} m`,width-54,height/2-position*scale);
  const queue = snapshot?.queue;
  if (queue) {
    const steps = [...queue.committed,...queue.pending];
    for (let index=1;index<steps.length;index++) drawArrow(steps[index-1].pose,steps[index].pose,'#becac2',true);
    queue.pending.forEach((step,index)=>footprint(step,'pending',index+queue.committed.length));
    queue.committed.forEach((step,index)=>footprint(step,'committed',index));
  }
  if (snapshot?.queue) {
    (snapshot.footstep.slots?.slice(0,2)||[]).forEach((pose,side)=>footprint({pose,side},'hold',side));
    snapshot.footstep.feet.forEach((pose,side)=>footprint({pose,side},'actual',side));
    const anchor=screen(snapshot.footstep.anchor);context.strokeStyle='#08755b';context.lineWidth=1.8;context.beginPath();context.arc(...anchor,8,0,2*Math.PI);context.stroke();
    drawArrow(snapshot.footstep.robot_pose,headingHandle(snapshot.footstep.robot_pose),'#253b32');
    const robot=screen(snapshot.footstep.robot_pose);context.fillStyle='#253b32';context.beginPath();context.arc(...robot,4,0,2*Math.PI);context.fill();
  }
  if (route.length>1) for(let index=1;index<route.length;index++) drawArrow(route[index-1],route[index],'#b08219');
  routePreview.forEach((step,index)=>footprint(step,'preview',index));
  draft.forEach((step,index)=>footprint(step,'draft',index));
}
function beginDraft() {
  if (!camera) {camera=clone(snapshot?.footstep.robot_pose || [0,0,0]);draftRevision=snapshot?.revision;}
}
function resetDraft() {
  routePreviewRequest++;routePreview=[];routePreviewPending=false;routePreviewError='';
  draft=[];route=[];camera=null;selected=-1;editing=false;draftRevision=null;updateDraft();draw();
}
function routePayload() {
  return {points:route.map(pose=>pose.slice(0,2)),spacing:Number(element('spacing').value),width:Number(element('width').value),follow_heading:element('follow').checked,revision:draftRevision};
}
async function refreshRoutePreview() {
  const request=++routePreviewRequest;
  routePreview=[];routePreviewError='';routePreviewPending=route.length>=2;
  updateDraft();draw();
  if(!routePreviewPending)return;
  try {
    const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'route_preview',...routePayload()})});
    const result=await response.json();
    if(request!==routePreviewRequest)return;
    if(!response.ok)throw new Error(result.error||`HTTP ${response.status}`);
    routePreview=result.steps;
  } catch(error) {
    if(request!==routePreviewRequest)return;
    routePreviewError=error.message;
    notify(`路线无法生成脚印：${error.message}`,true);
  } finally {
    if(request===routePreviewRequest){routePreviewPending=false;updateDraft();draw();}
  }
}
function updateDraft() {
  const routeLabel=routePreviewPending?'生成脚印中':routePreviewError?'路线超界或无效':`${routePreview.length} 个脚印`;
  element('draft-count').textContent=editing?`编辑 ${draft.length} 个待执行脚步`:route.length?`路线 · ${routeLabel}`:`${draft.length || '无'}${draft.length?' 个脚步':'草稿'}`;
  element('commit').disabled=(route.length?!routePreview.length||routePreviewPending:!draft.length)||!snapshot?.queue;
  element('commit').lastChild.textContent=editing?'应用队列编辑':'加入队列';
  const step=draft[selected];element('pose-editor').hidden=!step;
  if (step) {
    element('selected-title').textContent=`草稿 ${selected+1} · ${step.side===0?'左脚':'右脚'}`;
    element('pose-x').value=step.pose[0].toFixed(3);element('pose-y').value=step.pose[1].toFixed(3);element('pose-yaw').value=degrees(step.pose[2]).toFixed(1);
    element('remove-pose').disabled=selected!==draft.length-1;
  }
}
function nextSide() {
  if(draft.length)return 1-draft[draft.length-1].side;
  const steps=[...(snapshot?.queue?.committed||[]),...(snapshot?.queue?.pending||[])];
  if(steps.length)return 1-steps[steps.length-1].side;
  const phase=snapshot?.footstep.phase??1.5*Math.PI;
  return ((Math.PI/2-phase+2*Math.PI)%(2*Math.PI))<((1.5*Math.PI-phase+2*Math.PI)%(2*Math.PI))?0:1;
}
canvas.addEventListener('pointerdown',event=>{
  if(!snapshot?.queue)return;
  beginDraft();const pose=pointer(event);
  let hit=-1,rotate=false;
  for(let index=draft.length-1;index>=0;index--) {
    const handle=headingHandle(draft[index].pose);
    if(index===selected&&Math.hypot(pose[0]-handle[0],pose[1]-handle[1])<.05){hit=index;rotate=true;break;}
    if(Math.hypot(pose[0]-draft[index].pose[0],pose[1]-draft[index].pose[1])<.10){hit=index;break;}
  }
  if(hit>=0){selected=hit;drag={type:rotate?'rotate':'move',index:hit};}
  else if(mode==='route'){if(draft.length){notify('先提交或清空脚步草稿',true);return;}routePreviewRequest++;routePreview=[];routePreviewError='';routePreviewPending=false;route=[pose];drag={type:'route'};}
  else if(mode==='custom'){if(route.length){notify('先提交或清空路线',true);return;}draft.push({side:nextSide(),pose:[pose[0],pose[1],currentCamera()[2]]});selected=draft.length-1;drag={type:'rotate',index:selected,start:[event.clientX,event.clientY]};}
  else {if(!draft.length){camera=null;draftRevision=null;}selected=-1;}
  canvas.setPointerCapture(event.pointerId);updateDraft();draw();
});
canvas.addEventListener('pointermove',event=>{
  if(!drag)return;const pose=pointer(event);
  if(drag.type==='route'){const last=route[route.length-1];if(Math.hypot(pose[0]-last[0],pose[1]-last[1])>.025)route.push(pose);}
  else {const step=draft[drag.index];if(drag.type==='rotate'){if(drag.start&&Math.hypot(event.clientX-drag.start[0],event.clientY-drag.start[1])<6)return;drag.start=null;step.pose[2]=Math.atan2(pose[1]-step.pose[1],pose[0]-step.pose[0]);}else step.pose=[pose[0],pose[1],step.pose[2]];}
  updateDraft();draw();
});
function finishDrag(){const wasRoute=drag?.type==='route';drag=null;if(wasRoute)refreshRoutePreview();}
canvas.addEventListener('pointerup',finishDrag);
canvas.addEventListener('pointercancel',finishDrag);
async function updateTheta() {
  const theta = Number(element('theta').value);
  element('theta-label').value = `${theta}°`;
  thetaPending = theta;
  if (thetaBusy) return;
  thetaBusy = true;
  try {
    while (thetaPending !== null) {
      const value = thetaPending; thetaPending = null;
      const response = await fetch('/api/command', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'theta',theta_deg:value})});
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    }
    await poll();
  } catch (error) {
    notify(error.message, true);
  } finally {
    thetaBusy = false;
  }
}
document.querySelectorAll('[data-mode]').forEach(button=>button.addEventListener('click',()=>{mode=button.dataset.mode;document.querySelectorAll('[data-mode]').forEach(item=>item.classList.toggle('selected',item===button));}));
document.querySelectorAll('[data-direction]').forEach(button=>button.addEventListener('click',()=>command('simple',{direction:button.dataset.direction,distance:Number(element('distance').value),yaw_deg:Number(element('turn').value),width:Number(element('width').value),revision:snapshot?.revision})));
document.querySelectorAll('[data-command]').forEach(button=>button.addEventListener('click',()=>command(button.dataset.command,{frequency:Number(element('frequency').value),theta_deg:Number(element('theta').value)})));
element('frequency').addEventListener('input',()=>{element('frequency-label').value=`${Number(element('frequency').value).toFixed(2)} Hz`;});
element('frequency').addEventListener('change',()=>{if(snapshot?.queue?.state==='walking')command('frequency',{frequency:Number(element('frequency').value)});});
element('theta').addEventListener('input',updateTheta);
element('localization').addEventListener('change',()=>command('localization',{mode:element('localization').value}));
for(const id of ['spacing','width','follow'])element(id).addEventListener('change',()=>{if(route.length)refreshRoutePreview();});
element('commit').addEventListener('click',async()=>{
  const result=route.length?await command('route',routePayload()):await command(editing?'replace':'custom',{steps:draft,revision:draftRevision});
  if(result)resetDraft();
});
element('discard').addEventListener('click',resetDraft);
element('undo').addEventListener('click',()=>{if(route.length){route.pop();refreshRoutePreview();}else draft.pop();selected=Math.min(selected,draft.length-1);updateDraft();draw();});
element('zoom-in').addEventListener('click',()=>{scale=Math.min(400,scale*1.2);draw();});
element('zoom-out').addEventListener('click',()=>{scale=Math.max(60,scale/1.2);draw();});
element('pose-editor').addEventListener('submit',event=>{event.preventDefault();if(selected<0)return;const pose=[Number(element('pose-x').value),Number(element('pose-y').value),radians(Number(element('pose-yaw').value))];if(!pose.every(Number.isFinite))return;draft[selected].pose=pose;draw();});
element('remove-pose').addEventListener('click',()=>{if(selected===draft.length-1){draft.pop();selected=draft.length-1;updateDraft();draw();}});
element('edit-pending').addEventListener('click',()=>{if(draft.length||route.length){notify('先提交或清空当前草稿',true);return;}beginDraft();draft=clone(snapshot.queue.pending);editing=true;selected=draft.length?0:-1;mode='select';updateDraft();draw();});
let confirmAction=null;
function confirm(title,text,callback){element('confirm-title').textContent=title;element('confirm-text').textContent=text;confirmAction=callback;element('confirm-dialog').showModal();}
element('confirm-dialog').addEventListener('close',()=>{if(element('confirm-dialog').returnValue==='ok')confirmAction?.();confirmAction=null;});
element('clear-pending').addEventListener('click',()=>confirm('清空待执行脚步？','已承诺落点不会改变。行走中的机器人仍会完成收脚。',()=>command('clear',{revision:snapshot.revision})));
element('estop').addEventListener('click',()=>command('estop'));
function renderQueue(queue) {
  const signature=JSON.stringify([queue?.committed,queue?.pending]);if(signature===queueSignature)return;queueSignature=signature;
  for(const name of ['committed','pending']) {
    const list=element(`${name}-list`);list.replaceChildren();
    (queue?.[name]||[]).forEach((step,index)=>{
      const row=document.createElement('li');row.className=step.side===1?'right':'';
      const badge=document.createElement('span');badge.className='side';badge.textContent=step.side===0?'L':'R';
      const content=document.createElement('div');const title=document.createElement('div');title.className='step-title';title.textContent=`${index+1} · ${step.closing?'收脚':step.side===0?'左脚':'右脚'}`;
      const numbers=document.createElement('div');numbers.className='numbers';numbers.textContent=`${step.pose[0].toFixed(2)}, ${step.pose[1].toFixed(2)} m / ${degrees(step.pose[2]).toFixed(0)}°`;content.append(title,numbers);
      const icon=document.createElement('i');icon.dataset.lucide=name==='committed'?'lock-keyhole':'chevron-right';icon.className='row-icon';row.append(badge,content,icon);list.append(row);
    });
  }
  element('queue-count').textContent=(queue?.committed.length||0)+(queue?.pending.length||0);element('queue-empty').hidden=Boolean(queue?.committed.length||queue?.pending.length);icons();
}
function render() {
  const queue=snapshot.queue;const foot=snapshot.footstep;
  element('task-state').textContent=snapshot.preparing?'预备插值':labels[snapshot.task_state]||snapshot.task_state;
  element('queue-state').textContent=queue?labels[queue.state]||queue.state:'未初始化';
  element('phase').textContent=foot.phase.toFixed(2);element('actual-frequency').textContent=foot.frequency.toFixed(2);
  const thetaDeg=Math.round(degrees(foot.theta??0));element('actual-theta').textContent=String(thetaDeg);
  if(!thetaBusy){element('theta').value=String(thetaDeg);element('theta-label').value=`${thetaDeg}°`;}
  element('hold').textContent=queue?.state==='standing'?`${queue.hold_elapsed.toFixed(1)} s`:'—';
  element('localization').value=snapshot.localization; element('localization').disabled=!['IDLE','STOPPED','ERROR'].includes(snapshot.task_state);
  element('localization-warning').hidden=snapshot.localization!=='planned';element('map-empty').hidden=Boolean(queue);
  const allowed={engage:!['RUNNING','STAND'].includes(snapshot.task_state),track:snapshot.task_state==='STAND'&&!snapshot.preparing&&snapshot.ready,walk:snapshot.task_state==='RUNNING'&&queue?.state==='standing'&&queue.pending.length>0,pause:queue&&['starting','walking'].includes(queue.state),release:['RUNNING','STAND','ERROR'].includes(snapshot.task_state)};
  document.querySelectorAll('[data-command]').forEach(button=>{button.disabled=!allowed[button.dataset.command];});
  document.querySelectorAll('[data-direction]').forEach(button=>{button.disabled=!queue;});
  element('edit-pending').disabled=!queue?.pending.length;element('clear-pending').disabled=!queue?.pending.length;
  if(Date.now()>noticeUntil){const reason=snapshot.error||snapshot.localization_error;element('notice').hidden=!reason;element('notice').textContent=reason;element('notice').className=reason?'error':'';}
  renderQueue(queue);updateDraft();draw();
}
async function poll() {
  try {
    const response=await fetch('/api/state',{cache:'no-store'});if(!response.ok)throw new Error(`HTTP ${response.status}`);snapshot=await response.json();element('connection').textContent='已连接';element('connection').className='status online';render();
  } catch(error) {element('connection').textContent='连接中断';element('connection').className='status';document.querySelectorAll('[data-command],[data-direction],#commit').forEach(button=>{button.disabled=true;});notify(`服务连接中断：${error.message}`,true);}
}
new ResizeObserver(draw).observe(canvas);
icons();updateDraft();poll();setInterval(poll,250);
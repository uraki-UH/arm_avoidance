import * as THREE from 'three';

const vector = (s, fallback = [0,0,0]) => s ? s.trim().split(/\s+/).map(Number) : fallback;
const direct = (e, tag) => [...e.children].filter(c => c.tagName === tag);
function origin(e, object) {
  if (!e) return;
  object.position.fromArray(vector(e.getAttribute('xyz')));
  object.quaternion.setFromEuler(new THREE.Euler(...vector(e.getAttribute('rpy')), 'ZYX'));
}

export class Robot extends THREE.Group {
  constructor(xml) {
    super();
    const doc = new DOMParser().parseFromString(xml, 'application/xml');
    if (doc.querySelector('parsererror')) throw new Error('URDFのXMLを解析できません。');
    this.urdf = doc.documentElement;
    this.name = this.urdf.getAttribute('name');
    this.links = {}; this.joints = {}; this.visuals = [];
    for (const e of direct(this.urdf, 'link')) {
      const group = new THREE.Group(); group.name = e.getAttribute('name');
      this.links[group.name] = group;
      for (const v of direct(e, 'visual')) this.visuals.push({link: group, element: v});
    }
    const children = new Set();
    for (const e of direct(this.urdf, 'joint')) {
      const frame = new THREE.Group(); origin(e.querySelector('origin'), frame);
      const name = e.getAttribute('name'), child = this.links[e.querySelector('child').getAttribute('link')];
      const parent = this.links[e.querySelector('parent').getAttribute('link')];
      if (!parent || !child) throw new Error(`URDFのリンク不整合: ${name}`);
      parent.add(frame); frame.add(child); children.add(child.name);
      const limit = e.querySelector('limit'), mimic = e.querySelector('mimic');
      const type = e.getAttribute('type');
      this.joints[name] = {name, type, frame, child, q: 0,
        axis: new THREE.Vector3(...vector(e.querySelector('axis')?.getAttribute('xyz'), [1,0,0])).normalize(),
        lower: type === 'continuous' ? -Infinity : +(limit?.getAttribute('lower') ?? 0),
        upper: type === 'continuous' ? Infinity : +(limit?.getAttribute('upper') ?? 0),
        velocity: +(limit?.getAttribute('velocity') ?? 2),
        mimic: mimic ? {joint: mimic.getAttribute('joint'), multiplier: +(mimic.getAttribute('multiplier') ?? 1), offset: +(mimic.getAttribute('offset') ?? 0)} : null
      };
    }
    for (const [name, link] of Object.entries(this.links)) if (!children.has(name)) this.add(link);
    this.actuated = Object.values(this.joints).filter(j => j.type !== 'fixed' && !j.mimic);
  }
  setJoint(name, q, updateMimic = true) {
    if(this.pose_source==='ros'&&!this.is_receiving_pose)return;
    const j = this.joints[name]; if (!j || j.type === 'fixed' || !Number.isFinite(q)) return;
    j.q = Math.max(j.lower, Math.min(j.upper, q));
    if (j.type === 'prismatic') j.child.position.copy(j.axis).multiplyScalar(j.q);
    else j.child.quaternion.setFromAxisAngle(j.axis, j.q);
    if (updateMimic) for (const m of Object.values(this.joints)) if (m.mimic?.joint === name) this.setJoint(m.name, j.q * m.mimic.multiplier + m.mimic.offset, false);
  }
  getPose() { return Object.fromEntries(this.actuated.map(j => [j.name, j.q])); }
  setPose(pose) { for (const [name,q] of Object.entries(pose)) this.setJoint(name, q); this.updateMatrixWorld(true); }
  set_received_pose(pose) {
    this.is_receiving_pose=true;try{this.setPose(pose);}finally{this.is_receiving_pose=false;}
  }
  tcp(side) { this.links[`${side}_tcp`].updateWorldMatrix(true, false); return {position: this.links[`${side}_tcp`].getWorldPosition(new THREE.Vector3()), quaternion: this.links[`${side}_tcp`].getWorldQuaternion(new THREE.Quaternion())}; }
  chain(side) { return Array.from({length: 7}, (_,i) => this.joints[`${side}_joint${i+1}`]); }
  async loadVisuals(manifest, materials, progress) {
    const cache = new Map(); let done = 0;
    const names = [...new Set(this.visuals.map(v => v.element.querySelector('mesh')?.getAttribute('filename')).filter(Boolean))];
    // Load four meshes at a time to keep memory and HTTP use bounded.
    let next = 0;
    await Promise.all(Array.from({length:4}, async () => {
      while (next < names.length) {
        const name = names[next++], info = manifest.meshes[name];
        if (!info) throw new Error(`描画メッシュがありません: ${name}`);
        const response = await fetch(info.file); if (!response.ok) throw new Error(`メッシュ読み込み: ${name} (${response.status})`);
        const buffer = await response.arrayBuffer(), header = new Uint32Array(buffer, 0, 2);
        const data = new THREE.InterleavedBuffer(new Float32Array(buffer, 8, header[0]*6), 6);
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.InterleavedBufferAttribute(data, 3, 0));
        g.setAttribute('normal', new THREE.InterleavedBufferAttribute(data, 3, 3));
        g.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer, 8 + header[0]*24, header[1]), 1));
        g.computeBoundingSphere(); g.computeBoundingBox(); cache.set(name, g);
        progress(++done / names.length);
      }
    }));
    this.renderMeshes = [];
    for (const {link, element} of this.visuals) {
      const meshElement = element.querySelector('mesh'); if (!meshElement) continue;
      const filename = meshElement.getAttribute('filename');
      let materialName = element.querySelector('material')?.getAttribute('name') || 'black';
      // Reference-image finish: green gripper shells are named *_green in the supplied CAD,
      // but their URDF visual materials are black. This affects appearance only.
      if (filename.includes('gripper_base_green')) materialName = 'green';
      // 既存の外観パレットを優先、新規材質はURDFのlinear RGBAを使用
      let material = materials[materialName];
      if (!material) {
        const rgba = vector(element.querySelector('material color')?.getAttribute('rgba'), [0.14,0.15,0.145,1]);
        material = new THREE.MeshStandardMaterial({color: new THREE.Color().setRGB(...rgba.slice(0,3)),
          opacity: rgba[3], transparent: rgba[3] < 1, metalness: .35, roughness: .4});
      }
      const mesh = new THREE.Mesh(cache.get(filename), material);
      mesh.name = filename; origin(element.querySelector('origin'), mesh);
      mesh.scale.fromArray(vector(meshElement.getAttribute('scale'), [1,1,1]));
      mesh.castShadow = true; mesh.receiveShadow = true; link.add(mesh); this.renderMeshes.push(mesh);
    }
    this.updateMatrixWorld(true);
    this.triangles = this.renderMeshes.reduce((sum,m) => sum + m.geometry.index.count / 3, 0);
  }
}

export function orientationError(target, actual) {
  const q = target.clone().multiply(actual.clone().invert()).normalize();
  if (q.w < 0) q.set(-q.x,-q.y,-q.z,-q.w);
  const v = new THREE.Vector3(q.x,q.y,q.z), n = v.length();
  return n < 1e-10 ? v.multiplyScalar(2) : v.multiplyScalar(2*Math.atan2(n, Math.max(0,q.w))/n);
}

function linearSolve(matrix, rhs) {
  const n = rhs.length, a = matrix.map((r,i) => [...r, rhs[i]]);
  for (let k=0;k<n;k++) {
    let pivot=k; for(let i=k+1;i<n;i++) if(Math.abs(a[i][k])>Math.abs(a[pivot][k])) pivot=i;
    [a[k],a[pivot]]=[a[pivot],a[k]];
    if(Math.abs(a[k][k])<1e-14) return Array(n).fill(0);
    const d=a[k][k]; for(let j=k;j<=n;j++) a[k][j]/=d;
    for(let i=0;i<n;i++) if(i!==k){const s=a[i][k];for(let j=k;j<=n;j++)a[i][j]-=s*a[k][j];}
  }
  return a.map(r=>r[n]);
}

/** Damped least-squares IK, all axes and transforms sourced from the supplied URDF. */
export function solveIK(robot, side, target, {orientation=false, iterations=24, damping=0.009} = {}) {
  const chain=robot.chain(side), weights=orientation ? [1,1,1,.12,.12,.12] : [1,1,1];
  let best=chain.map(j=>j.q), bestCost=Infinity;
  for(let iter=0;iter<iterations;iter++) {
    const tcp=robot.tcp(side), ep=target.position.clone().sub(tcp.position), er=orientationError(target.quaternion,tcp.quaternion);
    const err=[ep.x,ep.y,ep.z,...(orientation?[er.x,er.y,er.z]:[])].map((v,i)=>v*weights[i]);
    const cost=err.reduce((s,v)=>s+v*v,0);
    if(cost<bestCost){bestCost=cost;best=chain.map(j=>j.q);}
    if(ep.length()<0.0002 && (!orientation || er.length()<0.003)) break;
    const columns=chain.map(j=>{
      j.frame.updateWorldMatrix(true,false);
      const axis=j.axis.clone().applyQuaternion(j.frame.getWorldQuaternion(new THREE.Quaternion()));
      const p=j.frame.getWorldPosition(new THREE.Vector3());
      const linear=axis.clone().cross(tcp.position.clone().sub(p));
      return [linear.x,linear.y,linear.z,...(orientation?[axis.x,axis.y,axis.z]:[])].map((v,i)=>v*weights[i]);
    });
    const n=err.length;
    const a=Array.from({length:n},(_,i)=>Array.from({length:n},(_,k)=>columns.reduce((s,c)=>s+c[i]*c[k],0)+(i===k?damping*damping:0)));
    const step=linearSolve(a,err), dq=columns.map(c=>c.reduce((s,v,i)=>s+v*step[i],0));
    const scale=Math.min(1,.18/Math.max(...dq.map(Math.abs),1e-10));
    chain.forEach((j,i)=>robot.setJoint(j.name,j.q+dq[i]*scale));
  }
  const last=robot.tcp(side);
  const lastCost=last.position.distanceToSquared(target.position)+(orientation ? .0144*orientationError(target.quaternion,last.quaternion).lengthSq() : 0);
  if(lastCost>bestCost) chain.forEach((j,i)=>robot.setJoint(j.name,best[i]));
  return {position:robot.tcp(side).position.distanceTo(target.position),angle:orientationError(target.quaternion,robot.tcp(side).quaternion).length()};
}

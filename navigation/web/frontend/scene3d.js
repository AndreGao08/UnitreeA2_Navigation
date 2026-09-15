class NavigationScene3D {
  constructor(canvas) {
    this.canvas = canvas;
    this.gl = canvas.getContext("webgl", { antialias: true, alpha: false });
    if (!this.gl) throw new Error("浏览器不支持 WebGL");
    this.target = [0, 0, 0]; this.distance = 24; this.yaw = 0.7; this.pitch = 0.85;
    this.map = null; this.pose = null; this.clouds = []; this.paths = []; this.waypoints = []; this.goal = null;
    this.texture = null; this.textureUrl = null;
    this.gridTextures = {};
    this.liveMapActive = false;
    this.colorProgram = this.makeProgram(
      "attribute vec3 a_position; attribute vec3 a_color; uniform mat4 u_matrix; uniform float u_point_size; varying vec3 v_color; void main(){gl_Position=u_matrix*vec4(a_position,1.0);gl_PointSize=u_point_size;v_color=a_color;}",
      "precision mediump float; varying vec3 v_color; void main(){gl_FragColor=vec4(v_color,1.0);}"
    );
    this.textureProgram = this.makeProgram(
      "attribute vec3 a_position; attribute vec2 a_uv; uniform mat4 u_matrix; varying vec2 v_uv; void main(){gl_Position=u_matrix*vec4(a_position,1.0);v_uv=a_uv;}",
      "precision mediump float; varying vec2 v_uv; uniform sampler2D u_texture; void main(){gl_FragColor=texture2D(u_texture,v_uv);}"
    );
    this.colorBuffer = this.gl.createBuffer(); this.textureBuffer = this.gl.createBuffer();
  }

  makeProgram(vertexSource, fragmentSource) {
    const gl = this.gl;
    const compile = (type, source) => { const shader=gl.createShader(type); gl.shaderSource(shader,source); gl.compileShader(shader); if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader)); return shader; };
    const program=gl.createProgram(); gl.attachShader(program,compile(gl.VERTEX_SHADER,vertexSource)); gl.attachShader(program,compile(gl.FRAGMENT_SHADER,fragmentSource)); gl.linkProgram(program);
    if(!gl.getProgramParameter(program,gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program)); return program;
  }

  perspective(fovy, aspect, near, far) {
    const f=1/Math.tan(fovy/2), nf=1/(near-far);
    return [f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)*nf,-1, 0,0,2*far*near*nf,0];
  }

  lookAt(eye, center, up) {
    const norm=(v)=>{const n=Math.hypot(...v)||1;return v.map(x=>x/n);};
    const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
    const z=norm(eye.map((v,i)=>v-center[i])), x=norm(cross(up,z)), y=cross(z,x);
    return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0, -x.reduce((s,v,i)=>s+v*eye[i],0),-y.reduce((s,v,i)=>s+v*eye[i],0),-z.reduce((s,v,i)=>s+v*eye[i],0),1];
  }

  multiply(a,b) {
    const out=new Array(16).fill(0); for(let c=0;c<4;c++) for(let r=0;r<4;r++) for(let k=0;k<4;k++) out[c*4+r]+=a[k*4+r]*b[c*4+k]; return out;
  }

  camera() {
    const cp=Math.cos(this.pitch), eye=[this.target[0]+this.distance*cp*Math.cos(this.yaw),this.target[1]+this.distance*cp*Math.sin(this.yaw),this.target[2]+this.distance*Math.sin(this.pitch)];
    return { eye, matrix:this.multiply(this.perspective(Math.PI/4,this.canvas.width/this.canvas.height,.05,500),this.lookAt(eye,this.target,[0,0,1])) };
  }

  update({ map, pose, pointclouds, paths, waypoints, goal, preview, showMapFrame, showRobotFrame, mapFrameLength, mapFrameThickness, robotFrameLength, robotFrameThickness, tfTree, tfSelectedFrames, tfFrameLength, tfFrameThickness, costmaps, showLocalCostmap, showGlobalCostmap, liveMap }) {
    const mapChanged = map?.name !== this.map?.name;
    this.map=map; this.pose=pose; this.waypoints=waypoints||[]; this.goal=goal; this.preview=preview;
    this.showMapFrame=showMapFrame; this.showRobotFrame=showRobotFrame;
    this.mapFrameLength=Math.max(.2,Math.min(10,Number(mapFrameLength)||.5));this.mapFrameThickness=Math.max(1,Math.min(12,Number(mapFrameThickness)||4));
    this.robotFrameLength=Math.max(.1,Math.min(5,Number(robotFrameLength)||.5));this.robotFrameThickness=Math.max(1,Math.min(12,Number(robotFrameThickness)||4));
    this.tfTree=tfTree||null;this.tfSelectedFrames=Array.isArray(tfSelectedFrames)?new Set(tfSelectedFrames):null;this.tfFrameLength=Math.max(.05,Math.min(5,Number(tfFrameLength)||.35));this.tfFrameThickness=Math.max(1,Math.min(12,Number(tfFrameThickness)||3));
    this.costmaps=costmaps||{};this.showLocalCostmap=showLocalCostmap;this.showGlobalCostmap=showGlobalCostmap;
    this.liveMap=liveMap?.recent?liveMap:null;
    if(this.costmaps.local?.recent)this.updateGridTexture("local",this.costmaps.local,[0,170,255]);
    if(this.costmaps.global?.recent)this.updateGridTexture("global",this.costmaps.global,[255,35,170]);
    if(this.liveMap)this.updateLiveMapTexture(this.liveMap);
    const yaw=pose?.yaw||0, ox=pose?.x||0, oy=pose?.y||0;
    this.clouds=(pointclouds||[]).map((cloud)=>{
      const source=cloud.data?.recent?cloud.data.points:[],fixed=Boolean(cloud.data?.transformed_to_fixed);
      const color=this.hexColor(cloud.color||"#ff0000"),points=source.map(([x,y,z])=>(pose&&!fixed)?[ox+Math.cos(yaw)*x-Math.sin(yaw)*y,oy+Math.sin(yaw)*x+Math.cos(yaw)*y,z]:[x,y,z]),vertices=[];
      points.forEach(item=>vertices.push(...item,...color));
      return {color,size:Math.max(1,Math.min(12,Number(cloud.size)||3)),points,vertices:new Float32Array(vertices)};
    });
    this.paths=(paths||[]).map((path)=>{
      const source=path.data?.points||[],fixed=Boolean(path.data?.transformed_to_fixed);
      const color=this.hexColor(path.color||"#7b35d1"),points=source.map(([x,y,z])=>(pose&&!fixed)?[ox+Math.cos(yaw)*x-Math.sin(yaw)*y,oy+Math.sin(yaw)*x+Math.cos(yaw)*y,(z||0)+.13]:[x,y,(z||0)+.13]),vertices=[];
      points.forEach(item=>vertices.push(...item,...color));
      return {color,size:Math.max(1,Math.min(12,Number(path.size)||3)),kind:path.kind||"path",points,vertices:new Float32Array(vertices)};
    });
    const url=map?.pgm?.exists ? apiBase+"/maps/"+encodeURIComponent(map.name)+"/thumbnail.bmp?v="+(map.pgm.modified_ns||0) : null;
    if(url && url!==this.textureUrl) this.loadTexture(url);
    if(this.liveMap&&!this.liveMapActive){this.liveMapActive=true;this.fitGrid(this.liveMap);}
    if(!this.liveMap)this.liveMapActive=false;
    if(mapChanged) this.fit();
    this.render();
  }

  updatePose(pose) {
    this.pose=pose;
    this.render();
  }

  updateRealtime(pose,tfTree,trajectory,tfSelectedFrames) {
    this.pose=pose;
    // Realtime updates must also honor the TF component visibility.  Assigning
    // null here intentionally clears the previous tree instead of retaining a
    // stale frame set between the slower full-canvas refreshes.
    this.tfTree=tfTree||null;
    this.tfSelectedFrames=Array.isArray(tfSelectedFrames)?new Set(tfSelectedFrames):null;
    const source=(trajectory||[]).map(item=>[Number(item[0])||0,Number(item[1])||0,.13]);
    this.paths.filter(path=>path.kind==="trajectory").forEach(path=>{
      path.points=source;
      const vertices=[];source.forEach(point=>vertices.push(...point,...path.color));
      path.vertices=new Float32Array(vertices);
    });
    this.render();
  }

  hexColor(value) {
    const match=/^#([0-9a-f]{6})$/i.exec(value||"");
    if(!match)return[1,0,0];const number=parseInt(match[1],16);
    return[(number>>16&255)/255,(number>>8&255)/255,(number&255)/255];
  }

  loadTexture(url) {
    this.textureUrl=url; const image=new Image(); image.onload=()=>{const gl=this.gl;this.texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,this.texture);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,true);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,image);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);this.render();}; image.src=url;
  }

  mapCorners() {
    const m=this.map?.metadata;if(!m)return null;const [ox,oy,a]=m.origin,w=m.width*m.resolution,h=m.height*m.resolution,c=Math.cos(a),s=Math.sin(a);
    const p=(x,y)=>[ox+c*x-s*y,oy+s*x+c*y,0]; return [p(0,0),p(w,0),p(w,h),p(0,h)];
  }

  gridCorners(grid,z) {
    if(!grid)return null;const [ox,oy,a]=grid.origin,w=grid.width*grid.resolution,h=grid.height*grid.resolution,c=Math.cos(a),s=Math.sin(a);
    const p=(x,y)=>[ox+c*x-s*y,oy+s*x+c*y,z];return[p(0,0),p(w,0),p(w,h),p(0,h)];
  }

  updateGridTexture(name,grid,color) {
    if(this.gridTextures[name]?.version===grid.version)return;
    const raw=atob(grid.data_base64),rgba=new Uint8Array(grid.width*grid.height*4);
    for(let i=0;i<raw.length;i++){const value=raw.charCodeAt(i);if(value===255||value===0)continue;const alpha=Math.min(210,35+Math.round(value/100*175));rgba[i*4]=color[0];rgba[i*4+1]=color[1];rgba[i*4+2]=color[2];rgba[i*4+3]=alpha;}
    const gl=this.gl,texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,grid.width,grid.height,0,gl.RGBA,gl.UNSIGNED_BYTE,rgba);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
    if(this.gridTextures[name]?.texture)gl.deleteTexture(this.gridTextures[name].texture);this.gridTextures[name]={texture,version:grid.version};
  }

  updateLiveMapTexture(grid) {
    if(this.gridTextures.live?.version===grid.version)return;
    const raw=atob(grid.data_base64),rgba=new Uint8Array(grid.width*grid.height*4);
    for(let i=0;i<raw.length;i++){const value=raw.charCodeAt(i),shade=value===255?90:Math.max(0,255-Math.round(value/100*255));rgba[i*4]=shade;rgba[i*4+1]=shade;rgba[i*4+2]=shade;rgba[i*4+3]=255;}
    const gl=this.gl,texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,grid.width,grid.height,0,gl.RGBA,gl.UNSIGNED_BYTE,rgba);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);if(this.gridTextures.live?.texture)gl.deleteTexture(this.gridTextures.live.texture);this.gridTextures.live={texture,version:grid.version};
  }

  drawTexturedPlane(matrix,corners,texture) {
    if(!corners||!texture)return;const gl=this.gl,p=this.textureProgram;
    const data=[...corners[0],0,0,...corners[1],1,0,...corners[3],0,1,...corners[3],0,1,...corners[1],1,0,...corners[2],1,1];
    gl.useProgram(p);gl.bindBuffer(gl.ARRAY_BUFFER,this.textureBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(data),gl.DYNAMIC_DRAW);const pos=gl.getAttribLocation(p,"a_position"),uv=gl.getAttribLocation(p,"a_uv");gl.enableVertexAttribArray(pos);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,20,0);gl.enableVertexAttribArray(uv);gl.vertexAttribPointer(uv,2,gl.FLOAT,false,20,12);gl.uniformMatrix4fv(gl.getUniformLocation(p,"u_matrix"),false,new Float32Array(matrix));gl.bindTexture(gl.TEXTURE_2D,texture);gl.drawArrays(gl.TRIANGLES,0,6);
  }

  drawTexture(matrix) {
    if(this.liveMap)this.drawTexturedPlane(matrix,this.gridCorners(this.liveMap,0),this.gridTextures.live?.texture);
    else this.drawTexturedPlane(matrix,this.mapCorners(),this.texture);
  }

  drawCostmaps(matrix) {
    const gl=this.gl;gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.depthMask(false);
    if(this.showGlobalCostmap&&this.costmaps.global?.transformed_to_map)this.drawTexturedPlane(matrix,this.gridCorners(this.costmaps.global,.045),this.gridTextures.global?.texture);
    if(this.showLocalCostmap&&this.costmaps.local?.transformed_to_map)this.drawTexturedPlane(matrix,this.gridCorners(this.costmaps.local,.065),this.gridTextures.local?.texture);
    gl.depthMask(true);gl.disable(gl.BLEND);
  }

  drawColored(matrix) {
    const lines=[],points=[],axisGroups=[];const line=(a,b,c)=>lines.push(...a,...c,...b,...c);const point=(p,c)=>points.push(...p,...c);const axis=(segments,size)=>{const data=[],dots=[];segments.forEach(([a,b,c])=>{data.push(...a,...c,...b,...c);const distance=Math.hypot(b[0]-a[0],b[1]-a[1],b[2]-a[2]),count=Math.max(2,Math.ceil(distance/.02));for(let i=0;i<=count;i++){const t=i/count;dots.push(a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,a[2]+(b[2]-a[2])*t,...c);}});axisGroups.push({data,dots,size});};
    const arrow=(x,y,z,a,length,color)=>{const ex=x+Math.cos(a)*length,ey=y+Math.sin(a)*length;line([x,y,z],[ex,ey,z],color);line([ex,ey,z],[ex-Math.cos(a-.55)*length*.28,ey-Math.sin(a-.55)*length*.28,z],color);line([ex,ey,z],[ex-Math.cos(a+.55)*length*.28,ey-Math.sin(a+.55)*length*.28,z],color);};
    if(this.showMapFrame){const l=this.mapFrameLength;axis([[[0,0,.03],[l,0,.03],[1,0,0]],[[0,0,.03],[0,l,.03],[0,1,0]],[[0,0,.03],[0,0,l],[0,.45,1]]],this.mapFrameThickness);}
    this.waypoints.forEach(w=>point([w.x,w.y,.12],[1,.55,0]));
    if(this.showRobotFrame){const x=this.pose?.x||0,y=this.pose?.y||0,a=this.pose?.yaw||0,z=0,c=Math.cos(a),s=Math.sin(a),l=this.robotFrameLength;axis([[[x,y,z],[x+c*l,y+s*l,z],[1,0,0]],[[x,y,z],[x-s*l,y+c*l,z],[0,1,0]],[[x,y,z],[x,y,z+l],[0,.45,1]]],this.robotFrameThickness);}
    if(this.tfTree?.frames?.length){const placed=new Map(this.tfTree.frames.filter(f=>Array.isArray(f.translation)&&Array.isArray(f.rotation)).map(f=>[f.name,f]));const selected=this.tfSelectedFrames;const visible=(name)=>!selected||selected.has(name);const rotate=(q,v)=>{const [x,y,z,w]=q,tx=2*(y*v[2]-z*v[1]),ty=2*(z*v[0]-x*v[2]),tz=2*(x*v[1]-y*v[0]);return[v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx];};this.tfTree.frames.forEach(f=>{if(!placed.has(f.name)||!visible(f.name))return;const o=f.translation,q=f.rotation,l=this.tfFrameLength,end=(v)=>{const r=rotate(q,v);return[o[0]+r[0],o[1]+r[1],o[2]+r[2]];};axis([[o,end([l,0,0]),[1,0,0]],[o,end([0,l,0]),[0,1,0]],[o,end([0,0,l]),[0,.45,1]]],this.tfFrameThickness);const parent=placed.get(f.parent);if(parent&&visible(f.parent))line(parent.translation,o,[.35,.38,.42]);});}
    if(this.goal)arrow(this.goal.x,this.goal.y,.14,this.goal.yaw||0,.8,[.65,.2,.9]);
    if(this.preview)arrow(this.preview.x,this.preview.y,.18,this.preview.yaw||0,.9,this.preview.kind==="goal"?[.65,.2,.9]:[.1,1,.35]);
    const vertices=[...lines,...points],lineVertexCount=lines.length/6;
    const gl=this.gl,p=this.colorProgram;gl.useProgram(p);gl.bindBuffer(gl.ARRAY_BUFFER,this.colorBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(vertices),gl.DYNAMIC_DRAW);
    const pos=gl.getAttribLocation(p,"a_position"),color=gl.getAttribLocation(p,"a_color"),pointSize=gl.getUniformLocation(p,"u_point_size");gl.enableVertexAttribArray(pos);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,24,0);gl.enableVertexAttribArray(color);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.uniformMatrix4fv(gl.getUniformLocation(p,"u_matrix"),false,new Float32Array(matrix));gl.uniform1f(pointSize,3);
    if(lineVertexCount)gl.drawArrays(gl.LINES,0,lineVertexCount);const pointCount=points.length/6;if(pointCount)gl.drawArrays(gl.POINTS,lineVertexCount,pointCount);
    axisGroups.forEach(group=>{gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(group.data),gl.DYNAMIC_DRAW);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.LINES,0,group.data.length/6);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(group.dots),gl.DYNAMIC_DRAW);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.uniform1f(pointSize,group.size);gl.drawArrays(gl.POINTS,0,group.dots.length/6);});
    this.clouds.forEach(cloud=>{if(!cloud.points.length)return;gl.bufferData(gl.ARRAY_BUFFER,cloud.vertices,gl.DYNAMIC_DRAW);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.uniform1f(pointSize,cloud.size);gl.drawArrays(gl.POINTS,0,cloud.points.length);});
    this.paths.forEach(path=>{if(path.points.length<2)return;gl.bufferData(gl.ARRAY_BUFFER,path.vertices,gl.DYNAMIC_DRAW);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.lineWidth(path.size);gl.drawArrays(gl.LINE_STRIP,0,path.points.length);gl.uniform1f(pointSize,Math.max(2,path.size));gl.drawArrays(gl.POINTS,0,path.points.length);});gl.lineWidth(1);
  }

  render() {
    if(this.canvas.clientWidth<10||this.canvas.clientHeight<10)return;
    const gl=this.gl,displayWidth=Math.max(1,Math.round(this.canvas.clientWidth*devicePixelRatio)),displayHeight=Math.max(1,Math.round(this.canvas.clientHeight*devicePixelRatio));if(this.canvas.width!==displayWidth||this.canvas.height!==displayHeight){this.canvas.width=displayWidth;this.canvas.height=displayHeight;}
    gl.viewport(0,0,this.canvas.width,this.canvas.height);gl.clearColor(1,1,1,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);const matrix=this.camera().matrix;this.drawTexture(matrix);this.drawCostmaps(matrix);this.drawColored(matrix);
  }

  orbit(dx,dy){this.yaw-=dx*.012;this.pitch=Math.max(-1.48,Math.min(1.56,this.pitch+dy*.012));this.render();}
  pan(dx,dy){const scale=2*Math.tan(Math.PI/8)*this.distance/Math.max(1,this.canvas.clientHeight),rx=-Math.sin(this.yaw),ry=Math.cos(this.yaw),fx=Math.cos(this.yaw),fy=Math.sin(this.yaw);this.target[0]-=(rx*dx+fx*dy)*scale;this.target[1]-=(ry*dx+fy*dy)*scale;this.render();}
  topView(){this.pitch=Math.PI/2-.001;this.render();}
  zoom(delta){this.distance=Math.max(2,Math.min(100,this.distance*(delta<0?.88:1.14)));this.render();}
  fit(){const c=this.mapCorners();if(c){this.target=[c.reduce((s,p)=>s+p[0],0)/4,c.reduce((s,p)=>s+p[1],0)/4,0];this.distance=Math.max(8,Math.hypot(c[1][0]-c[0][0],c[1][1]-c[0][1],c[3][0]-c[0][0],c[3][1]-c[0][1])*.9);}else{this.target=[0,0,0];this.distance=20;}this.render();}
  fitGrid(grid){const c=this.gridCorners(grid,0);if(c){this.target=[c.reduce((s,p)=>s+p[0],0)/4,c.reduce((s,p)=>s+p[1],0)/4,0];this.distance=Math.max(8,Math.hypot(c[1][0]-c[0][0],c[1][1]-c[0][1],c[3][0]-c[0][0],c[3][1]-c[0][1])*.9);}this.render();}
  screenToGround(clientX,clientY){const rect=this.canvas.getBoundingClientRect(),nx=((clientX-rect.left)/rect.width)*2-1,ny=1-((clientY-rect.top)/rect.height)*2;const {eye}=this.camera(),forward=this.normalize(this.target.map((v,i)=>v-eye[i])),right=this.normalize([forward[1],-forward[0],0]),up=this.cross(right,forward),t=Math.tan(Math.PI/8),ray=this.normalize([forward[0]+right[0]*nx*t*this.canvas.width/this.canvas.height+up[0]*ny*t,forward[1]+right[1]*nx*t*this.canvas.width/this.canvas.height+up[1]*ny*t,forward[2]+right[2]*nx*t+up[2]*ny*t]);if(Math.abs(ray[2])<1e-6)return null;const k=-eye[2]/ray[2];return k>0?{x:eye[0]+ray[0]*k,y:eye[1]+ray[1]*k}:null;}
  normalize(v){const n=Math.hypot(...v)||1;return v.map(x=>x/n);}
  cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
}

window.NavigationScene3D = NavigationScene3D;

use super::allocation::{Ledger,push};
use super::{add,mul,TopologyError,FaceUse,SegmentIncidence};
use crate::mosaic::{LabelMap,OUTSIDE,BoundaryGraph,Node,Segment,SegRef,Face,Contour};
use visioncortex::PointI32;

const DV:[(i32,i32);4]=[(0,-1),(1,0),(0,1),(-1,0)];
fn mask(m:&LabelMap,x:i32,y:i32)->u8 {
 let nw=m.label(x-1,y-1);let ne=m.label(x,y-1);let sw=m.label(x-1,y);let se=m.label(x,y);
 (nw!=ne) as u8 | (((ne!=se) as u8)<<1) | (((sw!=se) as u8)<<2) | (((nw!=sw) as u8)<<3)
}
fn sides(m:&LabelMap,x:i32,y:i32,d:usize)->(u32,u32){let nw=m.label(x-1,y-1);let ne=m.label(x,y-1);let sw=m.label(x-1,y);let se=m.label(x,y);match d{0=>(nw,ne),1=>(ne,se),2=>(se,sw),_=>(sw,nw)}}
fn slot(w:usize,x:i32,y:i32,d:usize)->(bool,usize){match d{0=>(true,(y-1) as usize*(w+1)+x as usize),2=>(true,y as usize*(w+1)+x as usize),1=>(false,y as usize*w+x as usize),_=>(false,y as usize*w+(x-1) as usize)}}
#[derive(Clone,Copy)]struct Desc {x:i32,y:i32,d:usize,len:usize,endx:i32,endy:i32,last:usize,start:Option<u32>,end:Option<u32>,left:u32,right:u32}
fn walk(m:&LabelMap,x:i32,y:i32,d:usize,limit:usize,mut emit:impl FnMut(PointI32,usize)->Result<(),TopologyError>)->Result<(usize,i32,i32,usize),TopologyError>{
 let(mut cx,mut cy,mut dir)=(x,y,d);let mut len=1;
 loop {if len>limit+1{return Err(TopologyError::CountMismatch);}emit(PointI32{x:cx,y:cy},dir)?;let(dx,dy)=DV[dir];let(nx,ny)=(cx+dx,cy+dy);len+=1;
  let bits=mask(m,nx,ny);if bits.count_ones()>=3||(nx==x&&ny==y){return Ok((len,nx,ny,dir));}
  let back=(dir+2)%4;dir=(0..4).find(|&k|k!=back&&(bits&(1<<k))!=0).ok_or(TopologyError::CountMismatch)?;cx=nx;cy=ny;
 }
}
pub(super) fn extract(m:&LabelMap,edges:usize,l:&mut Ledger)->Result<BoundaryGraph,TopologyError>{
 let(w,h)=(m.width as usize,m.height as usize);let corners=mul(w+1,h+1)?;
 let mut n=0usize;let mut degree=0usize;for y in 0..=h{for x in 0..=w{let d=mask(m,x as i32,y as i32).count_ones() as usize;if d>=3{n=add(n,1)?;degree=add(degree,d)?;}}}
 let bound=edges.min(add(degree/2,edges/4)?);
 let mut node_at=l.filled(corners,u32::MAX)?;let mut nodes=l.reserve(n)?;
 for y in 0..=h{for x in 0..=w{if mask(m,x as i32,y as i32).count_ones()>=3{node_at[y*(w+1)+x]=nodes.len() as u32;push(&mut nodes,Node{corner:PointI32{x:x as i32,y:y as i32},out:[None;4]})?;}}}
 let mut vv=l.filled(mul(w+1,h)?,false)?;let mut hh=l.filled(mul(w,h+1)?,false)?;let mut desc=l.reserve(bound)?;let mut counted=0;
 // Match legacy node scan and direction order, then free ring raster scan.
 for pass in 0..2 {
  let count=if pass==0{nodes.len()}else{corners};
  for i in 0..count{let(x,y)=if pass==0{(nodes[i].corner.x,nodes[i].corner.y)}else{((i%(w+1)) as i32,(i/(w+1)) as i32)};let bits=mask(m,x,y);
   for d in 0..4{if bits&(1<<d)==0{continue;}let(v,k)=slot(w,x,y,d);if if v{vv[k]}else{hh[k]}{continue;}
    let(len,ex,ey,last)=walk(m,x,y,d,edges,|p,dir|{let(v,k)=slot(w,p.x,p.y,dir);if v{vv[k]=true}else{hh[k]=true};Ok(())})?;
    counted=add(counted,len-1)?;if counted>edges{return Err(TopologyError::CountMismatch);}
    let(left,right)=sides(m,x,y,d);let start=if pass==0{Some(i as u32)}else{None};let end=if pass==0{let id=node_at[ey as usize*(w+1)+ex as usize];if id==u32::MAX{return Err(TopologyError::CountMismatch);}Some(id)}else{None};
    push(&mut desc,Desc{x,y,d,len,endx:ex,endy:ey,last,start,end,left,right})?;
   }
  }
 }
 if counted!=edges{return Err(TopologyError::CountMismatch);}
 l.release(vv);l.release(hh);l.release(node_at);
 let mut segments=l.reserve(desc.len())?;let mut second_edges=0usize;
 for s in &desc{let mut points=l.reserve(s.len)?;let(len,ex,ey,last)=walk(m,s.x,s.y,s.d,edges,|p,_|push(&mut points,p))?;push(&mut points,PointI32{x:ex,y:ey})?;
  second_edges=add(second_edges,len-1)?;if second_edges>edges{return Err(TopologyError::CountMismatch);}
  if(len,ex,ey,last)!=(s.len,s.endx,s.endy,s.last){return Err(TopologyError::CountMismatch);}
  let id=segments.len() as u32;if let(Some(start),Some(end))=(s.start,s.end){nodes[start as usize].out[s.d]=Some(SegRef{seg:id,forward:true});nodes[end as usize].out[(s.last+2)%4]=Some(SegRef{seg:id,forward:false});}
  push(&mut segments,Segment{points,start:s.start,end:s.end,left:s.left,right:s.right,first_dir:s.d as u8,last_dir:if s.start.is_some(){s.last as u8}else{0}})?;
 }
 if second_edges!=edges{return Err(TopologyError::CountMismatch);}l.release(desc);Ok(BoundaryGraph{nodes,segments})
}

#[derive(Clone,Copy)]struct Cycle{region:u32,island:u32,ordinal:usize,start:SegRef,len:usize}
fn region(g:&BoundaryGraph,r:SegRef)->u32{let s=&g.segments[r.seg as usize];if r.forward{s.left}else{s.right}}
fn next(g:&BoundaryGraph,m:&LabelMap,r:SegRef)->Result<SegRef,TopologyError>{let s=&g.segments[r.seg as usize];let(node,d)=if r.forward{(s.end,s.last_dir as usize)}else{(s.start,(s.first_dir as usize+2)%4)};let node=node.ok_or(TopologyError::CountMismatch)?;let p=g.nodes[node as usize].corner;
 for out in [(d+1)%4,d,(d+3)%4]{if mask(m,p.x,p.y)&(1<<out)!=0&&sides(m,p.x,p.y,out).0==region(g,r){return g.nodes[node as usize].out[out].ok_or(TopologyError::CountMismatch);}}
 Err(TopologyError::CountMismatch)
}
fn island_for(g:&BoundaryGraph,m:&LabelMap,ids:&[u32],r:SegRef)->Result<u32,TopologyError>{let s=&g.segments[r.seg as usize];let(a,b)=if r.forward{(s.points[0],s.points[1])}else{(*s.points.last().unwrap(),s.points[s.points.len()-2])};let d=DV.iter().position(|&(x,y)|b.x-a.x==x&&b.y-a.y==y).ok_or(TopologyError::CountMismatch)?;
 let(x,y)=match d{0=>(a.x-1,a.y-1),1=>(a.x,a.y-1),2=>(a.x,a.y),_=>(a.x-1,a.y)};if x<0||y<0||x>=m.width as i32||y>=m.height as i32{return Err(TopologyError::CountMismatch);}Ok(ids[y as usize*m.width as usize+x as usize])}
pub(super) fn faces(g:&BoundaryGraph,m:&LabelMap,l:&mut Ledger)->Result<Vec<Face>,TopologyError>{
 let(w,h)=(m.width as usize,m.height as usize);let p=mul(w,h)?;let mut ids=l.filled(p,u32::MAX)?;let mut stack=l.reserve::<usize>(p)?;let mut island=0u32;
 for start in 0..p{if ids[start]!=u32::MAX||m.labels[start]==OUTSIDE{continue;}ids[start]=island;push(&mut stack,start)?;
  while let Some(i)=stack.pop(){let(x,y)=(i%w,i/w);for dy in -1..=1{for dx in -1..=1{if dx==0&&dy==0{continue;}let(nx,ny)=(x as i32+dx,y as i32+dy);if nx<0||ny<0||nx>=w as i32||ny>=h as i32{continue;}let j=ny as usize*w+nx as usize;if ids[j]==u32::MAX&&m.labels[j]==m.labels[start]{ids[j]=island;push(&mut stack,j)?;}}}}
  island=island.checked_add(1).ok_or(TopologyError::ArithmeticOverflow)?;
 }
 l.release(stack);
 let mut used=l.filled(g.segments.len(),[false;2])?;let mut cycles=l.reserve(mul(g.segments.len(),2)?)?;let mut second_refs=0usize;
 for rings in [false,true]{for (seg_id,s) in g.segments.iter().enumerate(){if s.is_ring()!=rings{continue;}for forward in [true,false]{let start=SegRef{seg:seg_id as u32,forward};let r=region(g,start);if r==OUTSIDE||used[seg_id][forward as usize]{continue;}
   let mut len=0;let mut cur=start;loop{if len>=g.segments.len()*2||used[cur.seg as usize][cur.forward as usize]{return Err(TopologyError::CountMismatch);}second_refs=add(second_refs,1)?;if second_refs>mul(g.segments.len(),2)?{return Err(TopologyError::CountMismatch);}used[cur.seg as usize][cur.forward as usize]=true;len+=1;if rings{break;}cur=next(g,m,cur)?;if cur==start{break;}}
   let ordinal=cycles.len();push(&mut cycles,Cycle{region:r,island:island_for(g,m,&ids,start)?,ordinal,start,len})?;
  }}}
 used.fill([false;2]);l.release(ids);let expected_refs=second_refs;let mut emitted_refs=0usize;
 // In-place unstable sort with unique ordinal tie preserves legacy BTree group
 // order and per-group contour insertion order without sort scratch allocation.
 cycles.sort_unstable_by_key(|c|(c.region,c.island,c.ordinal));
 let count=cycles.iter().enumerate().filter(|&(i,c)|i==0||(c.region,c.island)!=(cycles[i-1].region,cycles[i-1].island)).count();let mut faces=l.reserve(count)?;let mut i=0;
 while i<cycles.len(){let begin=i;let key=(cycles[i].region,cycles[i].island);while i<cycles.len()&&(cycles[i].region,cycles[i].island)==key{i+=1;}
  let mut contours=l.reserve(i-begin)?;for c in &cycles[begin..i]{let mut refs=l.reserve(c.len)?;let mut cur=c.start;for _ in 0..c.len{if used[cur.seg as usize][cur.forward as usize]{return Err(TopologyError::CountMismatch);}used[cur.seg as usize][cur.forward as usize]=true;emitted_refs=add(emitted_refs,1)?;if emitted_refs>expected_refs{return Err(TopologyError::CountMismatch);}push(&mut refs,cur)?;if !g.segments[cur.seg as usize].is_ring(){cur=next(g,m,cur)?;}}if cur!=c.start{return Err(TopologyError::CountMismatch);}push(&mut contours,Contour(refs))?;}
  push(&mut faces,Face{region:key.0,contours})?;
 }
 if emitted_refs!=expected_refs{return Err(TopologyError::CountMismatch);}l.release(used);l.release(cycles);Ok(faces)
}
pub(super) fn incidence(g:&BoundaryGraph,faces:&[Face],l:&mut Ledger)->Result<Vec<SegmentIncidence>,TopologyError>{let mut out=l.reserve(g.segments.len())?;
 for s in &g.segments{let forward=l.reserve(if s.left!=OUTSIDE{1}else{0})?;let reversed=l.reserve(if s.right!=OUTSIDE{1}else{0})?;push(&mut out,SegmentIncidence{forward,reversed})?;}
 for(face_id,f)in faces.iter().enumerate(){for(contour_id,c)in f.contours.iter().enumerate(){for(reference_id,r)in c.0.iter().enumerate(){let v=if r.forward{&mut out[r.seg as usize].forward}else{&mut out[r.seg as usize].reversed};if !v.is_empty(){return Err(TopologyError::CountMismatch);}push(v,FaceUse{face_id,contour_id,reference_id})?;}}}
 for(s,i)in g.segments.iter().zip(&out){if i.forward.len()!=usize::from(s.left!=OUTSIDE)||i.reversed.len()!=usize::from(s.right!=OUTSIDE){return Err(TopologyError::CountMismatch);}}
 Ok(out)
}

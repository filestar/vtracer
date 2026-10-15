use super::*;
use crate::ir::Paint;
use visioncortex::Color;
fn map(w:u32,h:u32,labels:Vec<u32>)->LabelMap {
 let n=labels.iter().filter(|&&r|r!=OUTSIDE).max().map(|&r|r as usize+1).unwrap_or(0);
 LabelMap{width:w,height:h,labels,paints:(0..n).map(|_|Paint::Solid(Color::new(20,50,90))).collect()}
}
fn signature(paths:&[(u32,MultiPath)])->String {format!("{paths:?}")}
fn cache(t:&PreparedTopology)->Vec<Vec<PointI32>> {t.graph().segments.iter().map(|s|s.points.clone()).collect()}
#[test]
fn staged_exact_matches_old_all_small_partitions() {
 // 3 labels incl OUTSIDE on every2x2grid:81partitions, pinch/hole/island cases.
 for mask in 0..81 {let mut n=mask;let labels=(0..4).map(|_|{let r=n%3;n/=3;if r==2{OUTSIDE}else{r}}).collect();let m=map(2,2,labels);
  let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();
  let old=super::super::compose::compose_label_map(&m,&super::super::fit::PixelSegmentFitter,&[]);
  assert_eq!(signature(&old),signature(&t.compose_integer_polylines(&cache(&t)).unwrap()));
  for (seg_id,inc) in t.incidence().iter().enumerate(){for direction in [true,false]{for u in inc.uses(direction){let r=t.faces()[u.face_id].contours[u.contour_id].0[u.reference_id];assert_eq!(r.seg as usize,seg_id);assert_eq!(r.forward,direction);}}}
 }
}
#[test]
fn connected_face_incidence_not_paint_id_and_outside_empty() {
 let m=map(3,1,vec![0,OUTSIDE,0]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();assert_eq!(t.faces().len(),2);assert!(t.faces().iter().all(|f|f.region==0));
 let ids=t.incidence().iter().flat_map(|i|i.forward.iter().chain(&i.reversed)).map(|u|u.face_id).collect::<std::collections::BTreeSet<_>>();assert_eq!(ids,[0,1].into_iter().collect());
 assert!(t.incidence().iter().all(|i|i.forward.is_empty()||i.reversed.is_empty()));
}
#[test]
fn invalid_input_and_border_preflight() {
 let m=map(1,1,vec![0]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();assert_eq!(t.boundary_edges(),4);
 let mut b=TopologyBudget::default();b.max_boundary_edges=3;assert!(matches!(PreparedTopology::prepare(&m,b),Err(TopologyError::EdgeLimitExceeded)));
 b=TopologyBudget::default();b.max_grid_slots=8;assert!(matches!(PreparedTopology::prepare(&m,b),Err(TopologyError::GridLimitExceeded)));
 b=TopologyBudget::default();b.max_estimated_bytes=1;assert!(matches!(PreparedTopology::prepare(&m,b),Err(TopologyError::EstimatedBytesLimitExceeded)));
 let bad=LabelMap{width:u32::MAX,height:2,labels:vec![],paints:vec![]};assert!(matches!(PreparedTopology::prepare(&bad,TopologyBudget::default()),Err(TopologyError::InvalidDimensions)));
 let bad=LabelMap{width:1,height:1,labels:vec![],paints:vec![]};assert!(matches!(PreparedTopology::prepare(&bad,TopologyBudget::default()),Err(TopologyError::InvalidLabelCount)));
 let bad=LabelMap{width:1,height:1,labels:vec![4],paints:vec![]};assert!(matches!(PreparedTopology::prepare(&bad,TopologyBudget::default()),Err(TopologyError::InvalidRegionId)));
}
#[test]
fn cache_validation_and_before_output_budget() {
 let m=map(2,1,vec![0,1]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();let c=cache(&t);
 assert!(matches!(t.compose_integer_polylines(&[]),Err(TopologyError::CacheLengthMismatch)));
 let mut bad=c.clone();bad[0].clear();assert!(matches!(t.compose_integer_polylines(&bad),Err(TopologyError::CacheTooShort)));
 let mut bad=c.clone();bad[0][0].x=-1;assert!(matches!(t.compose_integer_polylines(&bad),Err(TopologyError::CachePointOutsideGrid)));
 let open=t.graph().segments.iter().position(|s|!s.is_ring()).unwrap();let mut bad=c.clone();bad[open][0].x=(bad[open][0].x+1)%3;assert!(matches!(t.compose_integer_polylines(&bad),Err(TopologyError::CacheEndpointMismatch)));
 let mut b=TopologyBudget::default();b.max_cache_points=1;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::CacheLimitExceeded)));
 let mut b=TopologyBudget::default();b.max_output_commands=1;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::OutputLimitExceeded)));
}
#[test]
fn rings_rotate_close_and_ownership_not_certified() {
 let m=map(1,1,vec![0]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();let mut c=cache(&t);assert!(t.graph().segments[0].is_ring());c[0].pop();c[0].rotate_left(1);let first=c[0][0];c[0].push(first);assert!(t.compose_integer_polylines(&c).is_ok());
 c[0].pop();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::CacheRingClosureMismatch)));
}
#[test]
fn hole_contours_and_junction_return_sides() {
 let m=map(3,3,vec![0,0,0,0,1,0,0,0,0]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();
 let outer=t.faces().iter().find(|f|f.region==0).unwrap();assert_eq!(outer.contours.len(),2);
 assert!(t.incidence().iter().any(|i|!i.forward.is_empty()&&!i.reversed.is_empty()));
 assert_eq!(signature(&t.compose_integer_polylines(&cache(&t)).unwrap()),signature(&super::super::compose::compose_label_map(&m,&super::super::fit::PixelSegmentFitter,&[])));
 // Exhaustively seek degree>=3 returning segments, ensuring they pin the same
 // junction rather than being classified as free rings.
 let mut found=false;
 for mask in 0..512 {let labels=(0..9).map(|k|if mask&(1<<k)==0{0}else{1}).collect();let m=map(3,3,labels);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();
  if t.graph().segments.iter().any(|s|!s.is_ring()&&s.start==s.end){found=true;assert!(t.compose_integer_polylines(&cache(&t)).is_ok());}
 }
 assert!(found,"fixture search must include junction-return loop");
}
fn assert_directed_incidence_and_joins(t:&PreparedTopology) {
 for (id,s) in t.graph().segments.iter().enumerate(){for (forward,region) in [(true,s.left),(false,s.right)] {
  let uses=t.incidence()[id].uses(forward);
  if region==OUTSIDE {assert!(uses.is_empty());} else {assert_eq!(uses.len(),1,"seg={id} forward={forward}");assert_eq!(t.faces()[uses[0].face_id].region,region);}
 }}
 for face in t.faces(){for contour in &face.contours{assert!(!contour.0.is_empty());
  for i in 0..contour.0.len(){let a=contour.0[i];let b=contour.0[(i+1)%contour.0.len()];let sa=&t.graph().segments[a.seg as usize];let sb=&t.graph().segments[b.seg as usize];
   let end=if a.forward{sa.points.last()}else{sa.points.first()};let start=if b.forward{sb.points.first()}else{sb.points.last()};assert_eq!(end,start);
  }
 }}
}
#[test]
fn nested_checkerboard_junction_incidence_is_complete() {
 let nested=map(5,5,vec![0,0,0,0,0,0,1,1,1,0,0,1,2,1,0,0,1,1,1,0,0,0,0,0,0]);
 let fixtures=vec![nested,map(2,2,vec![0,1,1,0]),map(2,2,vec![0,1,2,2]),map(3,3,vec![0,0,0,0,1,0,0,0,0])];
 for m in fixtures {let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();assert_directed_incidence_and_joins(&t);}
 for mask in 0..512 {let m=map(3,3,(0..9).map(|k|if mask&(1<<k)==0{0}else{1}).collect());let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();assert_directed_incidence_and_joins(&t);}
}
#[test]
fn allocations_are_released_and_peak_is_bounded() {
 let m=map(300,200,vec![0;60000]);let mut b=TopologyBudget::default();b.max_estimated_bytes=8*1024*1024;
 let t=PreparedTopology::prepare(&m,b).unwrap();assert!(t.preparation_bytes<t.preparation_peak_bytes);
 // Retained size vs peak size ratio shows transient allocations were released
 assert!(t.preparation_peak_bytes>t.preparation_bytes+300*200*4);
}
#[test]
fn temporary_cache_and_output_byte_budget_preflight() {
 let m=map(2,1,vec![0,1]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();let c=cache(&t);
 let mut b=TopologyBudget::default();b.max_estimated_bytes=t.preparation_peak_bytes;
 let t=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::EstimatedBytesLimitExceeded)));
}
#[test]
fn resource_sparse_disconnected_strip_and_empty() {
 let mut sparse=vec![OUTSIDE;100*100];sparse[5000]=0;
 let mut islands=vec![OUTSIDE;30*30];for y in (0..30).step_by(3){for x in (0..30).step_by(3){islands[y*30+x]=0;}}
 for m in [map(100,100,vec![OUTSIDE;10000]),map(100,100,sparse),map(30,30,islands),map(32768,1,vec![0;32768])]{
  let mut b=TopologyBudget::default();b.max_estimated_bytes=32*1024*1024;
  let t=PreparedTopology::prepare(&m,b).unwrap();let peak=t.preparation_peak_bytes();assert!(peak<=b.max_estimated_bytes);
  let c=cache(&t);assert_eq!(signature(&t.compose_integer_polylines(&c).unwrap()),signature(&super::super::compose::compose_label_map(&m,&super::super::fit::PixelSegmentFitter,&[])));
  b.max_estimated_bytes=peak-1;assert!(matches!(PreparedTopology::prepare(&m,b),Err(TopologyError::EstimatedBytesLimitExceeded)));
 }
}
#[test]
fn per_face_command_gate_and_composition_exact_peak() {
 let m=map(2,1,vec![0,1]);let mut b=TopologyBudget::default();let t=PreparedTopology::prepare(&m,b).unwrap();let c=cache(&t);
 let paths=t.compose_integer_polylines(&c).unwrap();let face_max=paths.iter().map(|(_,p)|p.subpaths.iter().map(|s|s.commands.len()).sum::<usize>()).max().unwrap();
 b.max_path_commands=face_max-1;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::OutputLimitExceeded)));
 b.max_path_commands=face_max;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(t.compose_integer_polylines(&c).is_ok());
 // Actual output capacities plus integer cache capacities determine live peak;
 // no transient fitted cache or oriented segment clone exists in staged emission.
 let output_bytes=paths.capacity()*std::mem::size_of::<(u32,MultiPath)>()+paths.iter().map(|(_,p)|p.subpaths.capacity()*std::mem::size_of::<crate::ir::SubPath>()+p.subpaths.iter().map(|s|s.commands.capacity()*std::mem::size_of::<crate::ir::PathCmd>()).sum::<usize>()).sum::<usize>();
 let cache_bytes=c.len()*std::mem::size_of::<Vec<PointI32>>()+c.iter().map(|p|p.capacity()*std::mem::size_of::<PointI32>()).sum::<usize>();
 b.max_estimated_bytes=t.preparation_bytes+cache_bytes+output_bytes;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(t.compose_integer_polylines(&c).is_ok());
 b.max_estimated_bytes-=1;let t=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(t.compose_integer_polylines(&c),Err(TopologyError::EstimatedBytesLimitExceeded)));
}
#[test]
fn two_sided_command_prediction() {
 let m=map(2,1,vec![0,1]);let t=PreparedTopology::prepare(&m,TopologyBudget::default()).unwrap();let c=cache(&t);let out=t.compose_integer_polylines(&c).unwrap();let commands=out.iter().flat_map(|(_,p)|&p.subpaths).map(|s|s.commands.len()).sum::<usize>();
 let mut b=TopologyBudget::default();b.max_output_commands=commands;let equal=PreparedTopology::prepare(&m,b).unwrap();assert!(equal.compose_integer_polylines(&c).is_ok());
 b.max_output_commands=commands-1;let short=PreparedTopology::prepare(&m,b).unwrap();assert!(matches!(short.compose_integer_polylines(&c),Err(TopologyError::OutputLimitExceeded)));
}

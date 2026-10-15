//! Budgeted staged composition for callers owning a separate geometry certificate.
use crate::ir::{MultiPath, PathCmd};
use super::face::Face;
mod allocation;
mod bounded;
mod bounded_context;
pub mod context;
#[cfg(test)] mod context_tests;
use allocation::Ledger;
use super::graph::BoundaryGraph;
use super::{LabelMap, OUTSIDE};
#[cfg(test)] mod tests;
use visioncortex::{PointF64, PointI32};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopologyError {
    InvalidDimensions, InvalidLabelCount, InvalidRegionId,
    GridLimitExceeded, EdgeLimitExceeded, CacheLimitExceeded,
    OutputLimitExceeded, EstimatedBytesLimitExceeded, ArithmeticOverflow,
    CacheLengthMismatch, CacheRingClosureMismatch, CacheEndpointMismatch,
    CacheTooShort, CachePointOutsideGrid, AllocationFailed, CountMismatch, Cancelled, WorkLimitExceeded,
}
impl std::fmt::Display for TopologyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "mosaic topology refusal: {:?}", self)
    }
}
impl std::error::Error for TopologyError {}

/// Caller-supplied limits. Byte estimates are conservative algorithmic allocation
/// accounting, not a process RSS guarantee or an allocator/OOM recovery mechanism.
#[derive(Debug, Clone, Copy)]
pub struct TopologyBudget {
    pub max_pixels: usize,
    /// Sum of corner grid, vertical slots, horizontal slots and island grid.
    pub max_grid_slots: usize,
    /// Unit boundary edges, including borders against OUTSIDE.
    pub max_boundary_edges: usize,
    /// Replacement cache points, checked at composition before cloning. Graph
    /// source points are separately bounded by at most twice max_boundary_edges.
    pub max_cache_points: usize,
    /// Both sides' emitted commands, checked at composition before allocation.
    /// Preparation does not predict shortcut lengths supplied later by the caller.
    pub max_output_commands: usize,
    /// Limit enforced per connected face before output expansion.
    pub max_path_commands: usize,
    pub max_estimated_bytes: usize,
}
impl Default for TopologyBudget {
    fn default() -> Self {
        Self { max_pixels: 32_000_000, max_grid_slots: 130_000_000,
            max_boundary_edges: 1_000_000, max_cache_points: 2_000_000,
            max_output_commands: 2_000_000, max_path_commands: 100_000, max_estimated_bytes: 256 * 1024 * 1024 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaceUse {
    /// Connected face index, NOT a region/paint identifier.
    pub face_id: usize,
    pub contour_id: usize,
    pub reference_id: usize,
}
/// One entry per graph segment. Empty direction means no assembled face (e.g.
/// OUTSIDE). Junction-returning segments may reference the same face on both sides.
#[derive(Debug, Clone, Default)]
pub struct SegmentIncidence {
    pub forward: Vec<FaceUse>,
    pub reversed: Vec<FaceUse>,
}
impl SegmentIncidence {
    pub fn uses(&self, forward: bool) -> &[FaceUse] {
        if forward { &self.forward } else { &self.reversed }
    }
}

pub struct PreparedTopology {
    graph: BoundaryGraph,
    faces: Vec<Face>,
    incidence: Vec<SegmentIncidence>,
    budget: TopologyBudget,
    width: u32,
    height: u32,
    preparation_bytes: usize,
    preparation_peak_bytes: usize,
    boundary_edges: usize,
}
fn add(a: usize, b: usize) -> Result<usize, TopologyError> {
    a.checked_add(b).ok_or(TopologyError::ArithmeticOverflow)
}
fn mul(a: usize, b: usize) -> Result<usize, TopologyError> {
    a.checked_mul(b).ok_or(TopologyError::ArithmeticOverflow)
}

impl PreparedTopology {
    /// Validate grid arithmetic, labels, border-inclusive edges and allocation
    /// estimates BEFORE extracting graphs/islands. Old compose_label_map is unchanged.
    pub fn prepare(map: &LabelMap, budget: TopologyBudget) -> Result<Self, TopologyError> {
        if map.width == 0 || map.height == 0 || map.width > 32768 || map.height > 32768 {
            return Err(TopologyError::InvalidDimensions);
        }
        let (w,h) = (map.width as usize,map.height as usize);
        let pixels = mul(w,h)?;
        if pixels != map.labels.len() { return Err(TopologyError::InvalidLabelCount); }
        if pixels > budget.max_pixels { return Err(TopologyError::GridLimitExceeded); }
        if map.labels.iter().any(|&l| l != OUTSIDE && l as usize >= map.paints.len()) {
            return Err(TopologyError::InvalidRegionId);
        }
        let corners = mul(add(w,1)?,add(h,1)?)?;
        let vertical = mul(add(w,1)?,h)?;
        let horizontal = mul(w,add(h,1)?)?;
        let slots = add(add(corners,vertical)?,add(horizontal,pixels)?)?;
        if slots > budget.max_grid_slots { return Err(TopologyError::GridLimitExceeded); }
        // Caller subtracts its additional concurrently live FTracer allocations
        // from max_estimated_bytes. Charge the existing map capacities explicitly.
        let input_bytes=add(mul(map.labels.capacity(),std::mem::size_of::<u32>())?,mul(map.paints.capacity(),std::mem::size_of::<crate::ir::Paint>())?)?;
        let mut ledger=Ledger::new(budget.max_estimated_bytes,input_bytes)?;
        let mut edges = 0usize;
        for y in 0..h {
            for x in 0..w {
                let a = map.labels[y*w+x];
                let mut count = 0;
                if x==0 && a!=OUTSIDE { count+=1; }
                if y==0 && a!=OUTSIDE { count+=1; }
                if a != if x+1<w {map.labels[y*w+x+1]} else {OUTSIDE} { count+=1; }
                if a != if y+1<h {map.labels[(y+1)*w+x]} else {OUTSIDE} { count+=1; }
                edges = add(edges,count)?;
                if edges > budget.max_boundary_edges { return Err(TopologyError::EdgeLimitExceeded); }
            }
        }
        let graph=bounded::extract(map,edges,&mut ledger)?;
        let faces=bounded::faces(&graph,map,&mut ledger)?;
        let incidence=bounded::incidence(&graph,&faces,&mut ledger)?;
        let preparation_bytes=ledger.live;
        Ok(Self {graph,faces,incidence,budget,width:map.width,height:map.height,preparation_bytes,preparation_peak_bytes:ledger.peak,boundary_edges:edges})
    }
    pub fn graph(&self) -> &BoundaryGraph { &self.graph }
    pub fn faces(&self) -> &[Face] { &self.faces }
    pub fn incidence(&self) -> &[SegmentIncidence] { &self.incidence }
    pub fn boundary_edges(&self) -> usize { self.boundary_edges }
    pub fn estimated_preparation_bytes(&self) -> usize { self.preparation_bytes }
    /// Logical capacity peak during preparation, including retained input map.
    /// Additional caller allocations/RSS are not included; caller reserves headroom.
    pub fn preparation_peak_bytes(&self) -> usize { self.preparation_peak_bytes }

    /// Structural cache validation ONLY; does not certify geometry/topology,
    /// nonintersection, winding, area or pixel ownership. The caller supplies that
    /// certificate. Shared references compose the SAME cache entry in reverse.
    pub fn compose_integer_polylines(&self, cache: &[Vec<PointI32>]) -> Result<Vec<(u32,MultiPath)>,TopologyError> {
        if cache.len()!=self.graph.segments.len() {return Err(TopologyError::CacheLengthMismatch);}
        let mut points_total=0;
        for (points,seg) in cache.iter().zip(&self.graph.segments) {
            if points.len()<2 {return Err(TopologyError::CacheTooShort);}
            points_total=add(points_total,points.len())?;
            if points_total>self.budget.max_cache_points {return Err(TopologyError::CacheLimitExceeded);}
            if points.iter().any(|p|p.x<0||p.y<0||p.x as u32>self.width||p.y as u32>self.height) {
                return Err(TopologyError::CachePointOutsideGrid);
            }
            if seg.is_ring() && points.first()!=points.last() {return Err(TopologyError::CacheRingClosureMismatch);}
            // Ring start rotation is allowed; geometric equivalence is certified
            // by the caller. Open junction-return loops still pin both endpoints.
            if !seg.is_ring() && (points.first()!=seg.points.first() || points.last()!=seg.points.last()) {
                return Err(TopologyError::CacheEndpointMismatch);
            }
        }
        let mut emitted=0;
        let mut contours=0;
        for face in &self.faces {
            let before=emitted;
            for contour in &face.contours {
                contours=add(contours,1)?;
                emitted=add(emitted,2)?; // M + Z
                for r in &contour.0 {emitted=add(emitted,cache[r.seg as usize].len()-1)?;}
            }
            if emitted-before>self.budget.max_path_commands{return Err(TopologyError::OutputLimitExceeded);}
        }
        if emitted>self.budget.max_output_commands {return Err(TopologyError::OutputLimitExceeded);}
        // A borrowed slice cannot reveal excess capacity of its caller-owned
        // outer Vec. Caller must subtract that excess from its headroom allowance.
        let cache_bytes=cache.iter().try_fold(mul(cache.len(),std::mem::size_of::<Vec<PointI32>>())?,|n,p|add(n,mul(p.capacity(),std::mem::size_of::<PointI32>())?))?;
        let mut ledger=Ledger::new(self.budget.max_estimated_bytes,add(self.preparation_bytes,cache_bytes)?)?;
        // Counted reservations and direct oriented iteration avoid F64 fitted
        // caches and per-reference ordered clones on the staged path only.
        let mut out=ledger.reserve(self.faces.len())?;
        for face in &self.faces {
            let mut subpaths=ledger.reserve(face.contours.len())?;
            for contour in &face.contours {
                let mut n=2usize;for r in &contour.0{n=add(n,cache[r.seg as usize].len()-1)?;}
                let mut commands=ledger.reserve(n)?;
                for (i,r) in contour.0.iter().enumerate(){let points=&cache[r.seg as usize];
                    if i==0{let p=if r.forward{points[0]}else{*points.last().unwrap()};allocation::push(&mut commands,PathCmd::MoveTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}
                    if r.forward{for p in &points[1..]{allocation::push(&mut commands,PathCmd::LineTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}}
                    else{for p in points[..points.len()-1].iter().rev(){allocation::push(&mut commands,PathCmd::LineTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}}
                }
                allocation::push(&mut commands,PathCmd::Close)?;allocation::push(&mut subpaths,crate::ir::SubPath{commands})?;
            }
            allocation::push(&mut out,(face.region,MultiPath{subpaths}))?;
        }
        Ok(out)
    }
}

impl PreparedTopology {
    pub fn prepare_with_context(map: &LabelMap, budget: TopologyBudget,ctx:&context::PipelineContext) -> Result<Self, TopologyError> {
        let mut phase=ctx.phase()?;
        if map.width == 0 || map.height == 0 || map.width > 32768 || map.height > 32768 {
            return Err(TopologyError::InvalidDimensions);
        }
        let (w,h) = (map.width as usize,map.height as usize);
        let pixels = mul(w,h)?;
        if pixels != map.labels.len() { return Err(TopologyError::InvalidLabelCount); }
        if pixels > budget.max_pixels { return Err(TopologyError::GridLimitExceeded); }
        let mut invalid=false;for &label in &map.labels{ctx.action(context::Action::Grid)?;ctx.action(context::Action::Grid)?;if label!=OUTSIDE&&label as usize>=map.paints.len(){invalid=true;break;}}if invalid {
            return Err(TopologyError::InvalidRegionId);
        }
        let corners = mul(add(w,1)?,add(h,1)?)?;
        let vertical = mul(add(w,1)?,h)?;
        let horizontal = mul(w,add(h,1)?)?;
        let slots = add(add(corners,vertical)?,add(horizontal,pixels)?)?;
        if slots > budget.max_grid_slots { return Err(TopologyError::GridLimitExceeded); }
        // Caller subtracts its additional concurrently live FTracer allocations
        // from max_estimated_bytes. Charge the existing map capacities explicitly.
        let input_bytes=add(mul(map.labels.capacity(),std::mem::size_of::<u32>())?,mul(map.paints.capacity(),std::mem::size_of::<crate::ir::Paint>())?)?;
        if ctx.capacity.live.get()<input_bytes{return Err(TopologyError::CountMismatch);}let baseline=ctx.capacity.live.get();let mut ledger=context::Ledger::new(ctx,input_bytes,budget.max_estimated_bytes)?;
        let mut edges = 0usize;
        for y in 0..h {ctx.action(context::Action::Grid)?;
            for x in 0..w {ctx.action(context::Action::Grid)?;
                let a = map.labels[y*w+x];
                let mut count = 0;
                if x==0 && a!=OUTSIDE { count+=1; }
                if y==0 && a!=OUTSIDE { count+=1; }
                if a != if x+1<w {map.labels[y*w+x+1]} else {OUTSIDE} { count+=1; }
                if a != if y+1<h {map.labels[(y+1)*w+x]} else {OUTSIDE} { count+=1; }
                edges = add(edges,count)?;
                if edges > budget.max_boundary_edges { return Err(TopologyError::EdgeLimitExceeded); }
            }
        }
        let graph=bounded_context::extract(map,edges,&mut ledger)?;
        let faces=bounded_context::faces(&graph,map,&mut ledger)?;
        let incidence=bounded_context::incidence(&graph,&faces,&mut ledger)?;
        let preparation_bytes=add(input_bytes,ctx.capacity.live.get().checked_sub(baseline).ok_or(TopologyError::ArithmeticOverflow)?)?;
        ctx.poll()?;phase.success();Ok(Self {graph,faces,incidence,budget,width:map.width,height:map.height,preparation_bytes,preparation_peak_bytes:ledger.phase_peak,boundary_edges:edges})
    }
    pub fn compose_integer_polylines_with_context(&self, cache: &[Vec<PointI32>],ctx:&context::PipelineContext) -> Result<Vec<(u32,MultiPath)>,TopologyError> {
        let mut phase=ctx.phase()?;
        if cache.len()!=self.graph.segments.len() {return Err(TopologyError::CacheLengthMismatch);}
        let mut points_total=0;
        for (points,seg) in cache.iter().zip(&self.graph.segments) {ctx.action(context::Action::Compose)?;
            if points.len()<2 {return Err(TopologyError::CacheTooShort);}
            points_total=add(points_total,points.len())?;
            if points_total>self.budget.max_cache_points {return Err(TopologyError::CacheLimitExceeded);}
            let mut outside=false;for p in points{ctx.action(context::Action::Compose)?;ctx.action(context::Action::Compose)?;if p.x<0||p.y<0||p.x as u32>self.width||p.y as u32>self.height{outside=true;break;}}if outside {
                return Err(TopologyError::CachePointOutsideGrid);
            }
            if seg.is_ring() && points.first()!=points.last() {return Err(TopologyError::CacheRingClosureMismatch);}
            // Ring start rotation is allowed; geometric equivalence is certified
            // by the caller. Open junction-return loops still pin both endpoints.
            if !seg.is_ring() && (points.first()!=seg.points.first() || points.last()!=seg.points.last()) {
                return Err(TopologyError::CacheEndpointMismatch);
            }
        }
        let mut emitted=0;
        let mut contours=0;
        for face in &self.faces {ctx.action(context::Action::Compose)?;
            let before=emitted;
            for contour in &face.contours {ctx.action(context::Action::Compose)?;
                contours=add(contours,1)?;
                emitted=add(emitted,2)?; // M + Z
                for r in &contour.0 {ctx.action(context::Action::Compose)?;emitted=add(emitted,cache[r.seg as usize].len()-1)?;}
            }
            if emitted-before>self.budget.max_path_commands{return Err(TopologyError::OutputLimitExceeded);}
        }
        if emitted>self.budget.max_output_commands {return Err(TopologyError::OutputLimitExceeded);}
        // A borrowed slice cannot reveal excess capacity of its caller-owned
        // outer Vec. Caller must subtract that excess from its headroom allowance.
        let mut cache_bytes=mul(cache.len(),std::mem::size_of::<Vec<PointI32>>())?;for p in cache{ctx.action(context::Action::Compose)?;cache_bytes=add(cache_bytes,mul(p.capacity(),std::mem::size_of::<PointI32>())?)?;}
        if ctx.capacity.live.get()<add(self.preparation_bytes,cache_bytes)?{return Err(TopologyError::CountMismatch);}let mut ledger=context::Ledger::new(ctx,add(self.preparation_bytes,cache_bytes)?,self.budget.max_estimated_bytes)?;
        // Counted reservations and direct oriented iteration avoid F64 fitted
        // caches and per-reference ordered clones on the staged path only.
        let mut out=ledger.reserve(self.faces.len())?;
        for face in &self.faces {ctx.action(context::Action::Compose)?;
            let mut subpaths=ledger.reserve(face.contours.len())?;
            for contour in &face.contours {ctx.action(context::Action::Compose)?;
                let mut n=2usize;for r in &contour.0{ctx.action(context::Action::Compose)?;n=add(n,cache[r.seg as usize].len()-1)?;}
                let mut commands=ledger.reserve(n)?;
                for (i,r) in contour.0.iter().enumerate(){ctx.action(context::Action::Compose)?;let points=&cache[r.seg as usize];
                    if i==0{let p=if r.forward{points[0]}else{*points.last().unwrap()};ledger.push(&mut commands,PathCmd::MoveTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}
                    if r.forward{for p in &points[1..]{ctx.action(context::Action::Compose)?;ledger.push(&mut commands,PathCmd::LineTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}}
                    else{for p in points[..points.len()-1].iter().rev(){ctx.action(context::Action::Compose)?;ledger.push(&mut commands,PathCmd::LineTo(PointF64{x:p.x as f64,y:p.y as f64}))?;}}
                }
                ledger.push(&mut commands,PathCmd::Close)?;ledger.push(&mut subpaths,crate::ir::SubPath{commands})?;
            }
            ledger.push(&mut out,(face.region,MultiPath{subpaths}))?;
        }
        ctx.poll()?;phase.success();Ok(out)
    }
}

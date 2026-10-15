//! Staged-only exact reservations. No legacy extractor/assembler is changed.
//! The ledger charges requested allocations before reserve and actual capacities
//! afterwards. An allocator may over-allocate before that second check; this is
//! a logical envelope with caller headroom, not a hard RSS/allocator guarantee.
use super::{add,mul,TopologyError};
use std::mem::size_of;

pub(super) struct Ledger { pub live:usize, pub peak:usize, limit:usize }
impl Ledger {
 pub fn new(limit:usize,live:usize)->Result<Self,TopologyError>{if live>limit{return Err(TopologyError::EstimatedBytesLimitExceeded);}Ok(Self{live,peak:live,limit})}
 pub fn charge(&mut self,bytes:usize)->Result<(),TopologyError>{let next=add(self.live,bytes)?;if next>self.limit{return Err(TopologyError::EstimatedBytesLimitExceeded);}self.live=next;self.peak=self.peak.max(next);Ok(())}
 pub fn release<T>(&mut self,v:Vec<T>){self.live-=v.capacity()*size_of::<T>();drop(v);}
 pub fn reserve<T>(&mut self,len:usize)->Result<Vec<T>,TopologyError>{
  let bytes=mul(len,size_of::<T>())?;self.charge(bytes)?;
  let mut v=Vec::new();v.try_reserve_exact(len).map_err(|_|TopologyError::AllocationFailed)?;
  let actual=mul(v.capacity(),size_of::<T>())?;
  if actual>bytes{self.charge(actual-bytes)?;}
  Ok(v)
 }
 pub fn filled<T:Clone>(&mut self,len:usize,value:T)->Result<Vec<T>,TopologyError>{let mut v=self.reserve(len)?;v.resize(len,value);Ok(v)}
}
/// Never implicitly grow a pre-counted buffer. Counter discrepancies fail closed.
pub(super) fn push<T>(v:&mut Vec<T>,value:T)->Result<(),TopologyError>{if v.len()==v.capacity(){return Err(TopologyError::CountMismatch);}v.push(value);Ok(())}

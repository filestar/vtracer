//! PRIVATE shared backing/work context; no global runtime governance claim.
use super::TopologyError as E;
use std::{cell::Cell,mem::size_of,sync::atomic::{AtomicBool,Ordering}};
#[derive(Debug,Clone,Copy)]pub enum Action{Grid,Walk,Component,Contour,Incidence,Sort,Compose,Allocation,Initialization,Write}
impl Action{fn index(self)->usize{self as usize}}
/// Caller declares all retained backing, including map exactly once. Any failed
/// contextual phase poisons this context; discard it and all partial outputs.
/// Context capacity applies alongside each legacy phase-local topology byte cap.
/// Global historical peak remains available only on capacity.peak.
pub struct PipelineContext<'a>{pub capacity:PipelineCapacity,pub work:OrchestrationWork,cancel:&'a AtomicBool,poisoned:Cell<bool>}
pub(super) struct PhaseGuard<'a,'b>{ctx:&'a PipelineContext<'b>,success:bool}
impl PhaseGuard<'_,'_>{pub fn success(&mut self){self.success=true;}}
impl Drop for PhaseGuard<'_,'_>{fn drop(&mut self){if !self.success{self.ctx.poisoned.set(true);}}}
#[cfg(test)]thread_local!{static INJECT:std::cell::Cell<Option<(usize,usize)>>=const{std::cell::Cell::new(None)};}
#[cfg(test)]pub(super) fn inject(action:Action,at:usize){INJECT.with(|x|x.set(Some((action.index(),at))));}
#[cfg(test)]pub(super) fn clear(){INJECT.with(|x|x.set(None));}
pub struct PipelineCapacity{pub live:Cell<usize>,pub peak:Cell<usize>,pub retained:usize,pub limit:usize}
pub struct OrchestrationWork{pub used:Cell<usize>,pub categories:[Cell<usize>;10],pub provisional_test_limit:usize}
impl<'a> PipelineContext<'a>{
 pub fn new(retained:usize,bytes:usize,provisional_test_limit:usize,cancel:&'a AtomicBool)->Result<Self,E>{if retained>bytes{return Err(E::EstimatedBytesLimitExceeded)}Ok(Self{capacity:PipelineCapacity{live:Cell::new(retained),peak:Cell::new(retained),retained,limit:bytes},work:OrchestrationWork{used:Cell::new(0),categories:std::array::from_fn(|_|Cell::new(0)),provisional_test_limit},cancel,poisoned:Cell::new(false)})}
 pub(super) fn phase(&self)->Result<PhaseGuard<'_,'a>,E>{self.poll()?;Ok(PhaseGuard{ctx:self,success:false})}
 pub fn poll(&self)->Result<(),E>{if self.poisoned.get(){return Err(E::CountMismatch)}if self.cancel.load(Ordering::Acquire){self.poisoned.set(true);Err(E::Cancelled)}else{Ok(())}}
 pub fn action(&self,kind:Action)->Result<(),E>{#[cfg(test)]INJECT.with(|x|{if let Some((k,at))=x.get(){if k==kind.index()&&self.work.categories[k].get()>=at{self.cancel.store(true,Ordering::Release);}}});let mut phase=self.phase()?;let n=self.work.used.get().checked_add(1).ok_or(E::ArithmeticOverflow)?;if n>self.work.provisional_test_limit{self.poisoned.set(true);return Err(E::WorkLimitExceeded)}let c=&self.work.categories[kind.index()];c.set(c.get().checked_add(1).ok_or(E::ArithmeticOverflow)?);self.work.used.set(n);phase.success();Ok(())}
 pub fn charge(&self,bytes:usize)->Result<(),E>{let mut phase=self.phase()?;let n=self.capacity.live.get().checked_add(bytes).ok_or(E::ArithmeticOverflow)?;if n>self.capacity.limit{self.poisoned.set(true);return Err(E::EstimatedBytesLimitExceeded)}self.capacity.live.set(n);self.capacity.peak.set(self.capacity.peak.get().max(n));phase.success();Ok(())}
 pub fn release_after_drop(&self,bytes:usize)->Result<(),E>{let mut phase=self.phase()?;let n=self.capacity.live.get().checked_sub(bytes).ok_or(E::ArithmeticOverflow)?;if n<self.capacity.retained{return Err(E::CountMismatch)}self.capacity.live.set(n);phase.success();Ok(())}
}
pub(super) struct Ledger<'a,'b>{pub context:&'a PipelineContext<'b>,baseline:usize,input_bytes:usize,pub phase_peak:usize,phase_limit:usize}
impl<'a,'b> Ledger<'a,'b>{
 pub fn new(context:&'a PipelineContext<'b>,input_bytes:usize,phase_limit:usize)->Result<Self,E>{if input_bytes>phase_limit{return Err(E::EstimatedBytesLimitExceeded)}Ok(Self{context,baseline:context.capacity.live.get(),input_bytes,phase_peak:input_bytes,phase_limit})}
 fn observe(&mut self)->Result<(),E>{let own=self.context.capacity.live.get().checked_sub(self.baseline).and_then(|n|n.checked_add(self.input_bytes)).ok_or(E::ArithmeticOverflow)?;if own>self.phase_limit{return Err(E::EstimatedBytesLimitExceeded)}self.phase_peak=self.phase_peak.max(own);Ok(())}
 pub fn action(&self,k:Action)->Result<(),E>{self.context.action(k)}
 pub fn reserve<T>(&mut self,n:usize)->Result<Vec<T>,E>{self.action(Action::Allocation)?;let bytes=n.checked_mul(size_of::<T>()).ok_or(E::ArithmeticOverflow)?;let own=self.context.capacity.live.get().checked_sub(self.baseline).and_then(|n|n.checked_add(self.input_bytes)).and_then(|n|n.checked_add(bytes)).ok_or(E::ArithmeticOverflow)?;if own>self.phase_limit{return Err(E::EstimatedBytesLimitExceeded)}self.context.charge(bytes)?;self.observe()?;let mut v=Vec::new();v.try_reserve_exact(n).map_err(|_|E::AllocationFailed)?;self.context.charge((v.capacity()-n).checked_mul(size_of::<T>()).ok_or(E::ArithmeticOverflow)?)?;self.observe()?;Ok(v)}
 pub fn push<T>(&mut self,v:&mut Vec<T>,x:T)->Result<(),E>{self.action(Action::Write)?;push(v,x)}
 pub fn filled<T:Copy>(&mut self,n:usize,x:T)->Result<Vec<T>,E>{let mut v=self.reserve(n)?;for _ in 0..n{self.action(Action::Initialization)?;push(&mut v,x)?;}Ok(v)}
 pub fn release<T>(&mut self,v:Vec<T>)->Result<(),E>{let bytes=v.capacity().checked_mul(size_of::<T>()).ok_or(E::ArithmeticOverflow)?;drop(v);self.context.release_after_drop(bytes)}
}
pub(super) fn push<T>(v:&mut Vec<T>,x:T)->Result<(),E>{if v.len()==v.capacity(){return Err(E::CountMismatch)}v.push(x);Ok(())}
pub(super) fn sort<T:Copy,K:Ord,F:Fn(&T)->K>(v:&mut[T],key:F,l:&mut Ledger)->Result<(),E>{
 fn down<T:Copy,K:Ord,F:Fn(&T)->K>(v:&mut[T],mut r:usize,end:usize,key:&F,l:&mut Ledger)->Result<(),E>{while r<end/2{l.action(Action::Sort)?;let mut c=r*2+1;if c+1<end{l.action(Action::Sort)?;if key(&v[c])<key(&v[c+1]){c+=1}}l.action(Action::Sort)?;if key(&v[r])>=key(&v[c]){break}l.action(Action::Sort)?;v.swap(r,c);r=c;}Ok(())}
 for r in (0..v.len()/2).rev(){down(v,r,v.len(),&key,l)?}for end in (1..v.len()).rev(){l.action(Action::Sort)?;v.swap(0,end);down(v,0,end,&key,l)?}Ok(())
}

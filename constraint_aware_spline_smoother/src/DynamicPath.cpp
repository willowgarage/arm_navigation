/*****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2009, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#include "constraint_aware_spline_smoother/DynamicPath.h"
#include <list>
#include <algorithm>
using namespace std;

const static Real EpsilonT = 1e-8;
//maximum iterations for distance-based line segment checker
const static int checkMaxIters = 1000;

inline Real LInfDistance(const Vector& a,const Vector& b)
{
  assert(a.size()==b.size());
  Real d=0;
  for(size_t i=0;i<a.size();i++)
    d = Max(d,Abs(a[i]-b[i]));
  return d;
}


inline Real MaxBBLInfDistance(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  assert(x.size()==bmin.size());
  assert(x.size()==bmax.size());
  Real d=0;
  for(size_t i=0;i<x.size();i++)
    d = Max(d,Max(Abs(x[i]-bmin[i]),Abs(x[i]-bmax[i])));
  return d;
}

DynamicPath::DynamicPath()
{}

void DynamicPath::Init(const Vector& _velMax,const Vector& _accMax)
{
  velMax = _velMax;
  accMax = _accMax;
  assert(velMax.size() == accMax.size());
}

Real DynamicPath::GetTotalTime() const
{
  Real t=0;
  for(size_t i=0;i<ramps.size();i++) t+=ramps[i].endTime;
  return t;
}

void DynamicPath::Evaluate(Real t,Vector& x)
{
  assert(!ramps.empty());
  if(t < 0) {
    x = ramps.front().x0;
  }
  else {
    for(size_t i=0;i<ramps.size();i++) {
      if(t <= ramps[i].endTime) {
	ramps[i].Evaluate(t,x);
	return;
      }
      t -= ramps[i].endTime;
    }
    x = ramps.back().x1;
  }
}

void DynamicPath::Derivative(Real t,Vector& dx)
{
  assert(!ramps.empty());
  if(t < 0) {
    dx = ramps.front().dx0;
  }
  else {
    for(size_t i=0;i<ramps.size();i++) {
      if(t <= ramps[i].endTime) {
	ramps[i].Derivative(t,dx);
	return;
      }
      t -= ramps[i].endTime;
    }
    dx = ramps.back().dx1;
  }
}

void DynamicPath::SetMilestones(const vector<Vector>& x)
{
  if(x.empty()) {
    ramps.resize(0);
  }
  else if(x.size()==1) {
    ramps.resize(1);
    ramps[0].SetConstant(x[0]);
  }
  else {
    Vector zero(x[0].size(),0.0);
    ramps.resize(x.size()-1);
    for(size_t i=0;i<ramps.size();i++) {
      ramps[i].x0 = x[i];
      ramps[i].x1 = x[i+1];
      ramps[i].dx0 = zero;
      ramps[i].dx1 = zero;
      bool res=ramps[i].SolveMinTimeLinear(accMax,velMax);
      assert(res);
    }
  }
}

void DynamicPath::SetMilestones(const vector<Vector>& x,const vector<Vector>& dx)
{
  if(x.empty()) {
    ramps.resize(0);
  }
  else if(x.size()==1) {
    ramps.resize(1);
    ramps[0].SetConstant(x[0]);
  }
  else {
    ramps.resize(x.size()-1);
    for(size_t i=0;i<ramps.size();i++) {
      ramps[i].x0 = x[i];
      ramps[i].x1 = x[i+1];
      ramps[i].dx0 = dx[i];
      ramps[i].dx1 = dx[i+1];
      bool res=ramps[i].SolveMinTime(accMax,velMax);
      assert(res);
    }
  }
}

void DynamicPath::GetMilestones(vector<Vector>& x,vector<Vector>& dx) const
{
  if(ramps.empty()) {
    x.resize(0);
    dx.resize(0);
    return;
  }
  x.resize(ramps.size()+1);
  dx.resize(ramps.size()+1);
  x[0] = ramps[0].x0;
  dx[0] = ramps[0].dx0;
  for(size_t i=0;i<ramps.size();i++) {
    x[i+1] = ramps[i].x1;
    dx[i+1] = ramps[i].dx1;
  }
}

void DynamicPath::Append(const Vector& x)
{
  size_t n=ramps.size();
  size_t p=n-1;
  ramps.resize(ramps.size()+1);
  if(ramps.size()==1) {
    ramps[0].SetConstant(x);
  }
  else {
    ramps[n].x0 = ramps[p].x1;
    ramps[n].dx0 = ramps[p].dx1;
    ramps[n].x1 = x;
    ramps[n].dx1.resize(x.size());
    fill(ramps[n].dx1.begin(),ramps[n].dx1.end(),0);
    bool res=ramps[n].SolveMinTime(accMax,velMax);
    assert(res);
  }
}

void DynamicPath::Append(const Vector& x,const Vector& dx)
{
  size_t n=ramps.size();
  size_t p=n-1;
  ramps.resize(ramps.size()+1);
  if(ramps.size()==1) {
    fprintf(stderr,"Can't append milestone with a nonzero velocity to an empty path\n");
    abort();
  }
  else {
    ramps[n].x0 = ramps[p].x1;
    ramps[n].dx0 = ramps[p].dx1;
    ramps[n].x1 = x;
    ramps[n].dx1 = dx;
    bool res=ramps[n].SolveMinTime(accMax,velMax);
    assert(res);
  }
}

struct RampSection
{
  Real ta,tb;
  Vector xa,xb;
  Real da,db;
};


void BoundingBox(const ParabolicRamp1D& ramp,Real ta,Real tb,Real& bmin,Real& bmax)
{
  if(ta > tb) {  //orient the interval
    return BoundingBox(ramp,tb,ta,bmin,bmax);
  }
  if(ta < 0) ta = 0;
  if(tb <= 0) {
    bmin = bmax = ramp.x0;
    return;
  }
  if(tb > ramp.ttotal) tb=ramp.ttotal;
  if(ta >= ramp.ttotal) {
    bmin = bmax = ramp.x1;
    return;
  }

  bmin = ramp.Evaluate(ta);
  bmax = ramp.Evaluate(tb);
  if(bmin > bmax) Swap(bmin,bmax);

  Real tflip1=0,tflip2=0;
  if(ta < ramp.tswitch1) {
    //x' = a1*t + v0 = 0 => t = -v0/a1
    tflip1 = -ramp.dx0/ramp.a1;
    if(tflip1 > ramp.tswitch1) tflip1 = 0;
  }
  if(tb > ramp.tswitch2) {
    //x' = a2*(T-t) + v1 = 0 => (T-t) = v1/a2
    tflip2 = ramp.ttotal-ramp.dx1/ramp.a2;
    if(tflip2 < ramp.tswitch2) tflip2 = 0;
  }
  if(ta < tflip1 && tb > tflip1) {
    Real xflip = ramp.Evaluate(tflip1);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }
  if(ta < tflip2 && tb > tflip2) {
    Real xflip = ramp.Evaluate(tflip2);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }
}

void BoundingBox(const ParabolicRampND& ramp,Real ta,Real tb,Vector& bmin,Vector& bmax)
{
  bmin.resize(ramp.ramps.size());
  bmax.resize(ramp.ramps.size());
  for(size_t i=0;i<ramp.ramps.size();i++) {
    BoundingBox(ramp.ramps[i],ta,tb,bmin[i],bmax[i]);
  }
}

    

bool CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* feas,DistanceCheckerBase* distance,int maxiters)
{
  if(!feas->ConfigFeasible(ramp.x0)) return false;
  if(!feas->ConfigFeasible(ramp.x1)) return false;
  assert(distance->ObstacleDistanceNorm()==Inf);
  RampSection section;
  section.ta = 0;
  section.tb = ramp.endTime;
  section.xa = ramp.x0;
  section.xb = ramp.x1;
  section.da = distance->ObstacleDistance(ramp.x0);
  section.db = distance->ObstacleDistance(ramp.x1);
  if(section.da <= 0.0) return false;
  if(section.db <= 0.0) return false;
  list<RampSection> queue;
  queue.push_back(section);
  int iters=0;
  while(!queue.empty()) {
    section = queue.front();
    queue.erase(queue.begin());

    //check the bounds around this section
    if(LInfDistance(section.xa,section.xb) <= section.da+section.db) {
      Vector bmin,bmax;
      BoundingBox(ramp,section.ta,section.tb,bmin,bmax);
      if(MaxBBLInfDistance(section.xa,bmin,bmax) < section.da && 
	 MaxBBLInfDistance(section.xb,bmin,bmax) < section.db)
	//can cull out the section
	continue;
    }
    Real tc = (section.ta+section.tb)*0.5;
    Vector xc;
    ramp.Evaluate(tc,xc);
    if(!feas->ConfigFeasible(xc)) return false;  //infeasible config
    //subdivide
    Real dc = distance->ObstacleDistance(xc);
    RampSection sa,sb;
    sa.ta = section.ta;
    sa.xa = section.xa;
    sa.da = section.da;
    sa.tb = sb.ta = tc;
    sa.xb = sb.xa = xc;
    sa.db = sb.da = dc;
    sb.tb = section.tb;
    sb.xb = section.xb;
    sb.db = section.db;

    //recurse on segments
    queue.push_back(sa);
    queue.push_back(sb);

    if(iters++ >= maxiters) return false;
  }
  return true;
}

bool CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* space,Real tol)
{
  assert(tol > 0);
  if(!space->ConfigFeasible(ramp.x0)) return false;
  if(!space->ConfigFeasible(ramp.x1)) return false;
  //assert(space->ConfigFeasible(ramp.x0));
  //assert(space->ConfigFeasible(ramp.x1));

  //for a parabola of form f(x) = a x^2 + b x, and the straight line 
  //of form g(X,u) = u*f(X)
  //d^2(g(X,u),p) = |p - <f(X),p>/<f(X),f(X)> f(X)|^2 < tol^2
  //<p,p> - <f(X),p>^2/<f(X),f(X)>  = p^T (I-f(X)f(X)^T/f(X)^T f(X)) p
  //max_x d^2(f(x)) => f(x)^T (I-f(X)f(X)^T/f(X)^T f(X)) f'(x) = 0
  //(x^2 a^T + x b^T) A (2a x + b) = 0
  //(x a^T + b^T) A (2a x + b) = 0
  //2 x^2 a^T A a + 3 x b^T A a + b^T A b = 0

  //the max X for which f(x) deviates from g(X,x) by at most tol is...
  //max_x |g(X,x)-f(x)| = max_x x/X f(X)-f(x)
  //=> f(X)/X - f'(x) = 0
  //=>  X/2 = x 
  //=> max_x |g(X,x)-f(x)| = |(X/2)/X f(X)-f(X/2)|
  //= |1/2 (aX^2+bX) - a(X/2)^2 - b(X/2) + c |
  //= |a| X^2 / 4
  //so... max X st max_x |g(X,x)-f(x)| < tol => X = 2*sqrt(tol/|a|)
  vector<Real> divs;
  Real t=0;
  divs.push_back(t);
  while(t < ramp.endTime) {
    Real tnext=t;
    Real amax = 0;
    Real switchNext=ramp.endTime;
    for(size_t i=0;i<ramp.ramps.size();i++) {
      if(t < ramp.ramps[i].tswitch1) {  //ramp up
	switchNext =  Min(switchNext, ramp.ramps[i].tswitch1);
	amax = Max(amax,Max(Abs(ramp.ramps[i].a1),Abs(ramp.ramps[i].a2)));
      }
      else if(t < ramp.ramps[i].tswitch2) {  //constant vel
	switchNext = Min(switchNext, ramp.ramps[i].tswitch2);
      }
      else if(t < ramp.ramps[i].ttotal) {  //ramp down
	amax = Max(amax,Max(Abs(ramp.ramps[i].a1),Abs(ramp.ramps[i].a2)));     
      }
    }
    Real dt = 2.0*Sqrt(tol/amax);
    if(t+dt > switchNext) tnext = switchNext;
    else tnext = t+dt;

    t = tnext;
    divs.push_back(tnext);
  }
  divs.push_back(ramp.endTime);

  //do a bisection thingie
  list<pair<int,int> > segs;
  segs.push_back(pair<int,int>(0,divs.size()-1));
  Vector q1,q2;
  while(!segs.empty()) {
    int i=segs.front().first;
    int j=segs.front().second;
    segs.erase(segs.begin());
    if(j == i+1) {
      //check path from t to tnext
      ramp.Evaluate(divs[i],q1);
      ramp.Evaluate(divs[j],q2);
      if(!space->SegmentFeasible(q1,q2)) return false;
    }
    else {
      int k=(i+j)/2;
      ramp.Evaluate(divs[k],q1);
      if(!space->ConfigFeasible(q1)) return false;
      segs.push_back(pair<int,int>(i,k));
      segs.push_back(pair<int,int>(k,j));
    }
  }
  return true;
}

bool DynamicPath::TryShortcut(Real t1,Real t2,FeasibilityCheckerBase* space,Real tol)
{
  if(t1 > t2) Swap(t1,t2);
  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
  int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
  if(i1 == i2) return false;
  Real u1 = t1-rampStartTime[i1];
  Real u2 = t2-rampStartTime[i2];
  assert(u1 >= 0);
  assert(u1 <= ramps[i1].endTime+EpsilonT);
  assert(u2 >= 0);
  assert(u2 <= ramps[i2].endTime+EpsilonT);
  u1 = Min(u1,ramps[i1].endTime);
  u2 = Min(u2,ramps[i2].endTime);
  ParabolicRampND test;
  ramps[i1].Evaluate(u1,test.x0);
  ramps[i2].Evaluate(u2,test.x1);
  ramps[i1].Derivative(u1,test.dx0);
  ramps[i2].Derivative(u2,test.dx1);
  bool res=test.SolveMinTime(accMax,velMax);
  if(!res) return false;
  assert(test.endTime >= 0);
  assert(test.IsValid());
  if(!CheckRamp(test,space,tol)) return false;

  //perform shortcut
  //crop i1 and i2
  ramps[i1].TrimBack(ramps[i1].endTime-u1);
  ramps[i1].x1 = test.x0;
  ramps[i1].dx1 = test.dx0;
  ramps[i2].TrimFront(u2);
  ramps[i2].x0 = test.x1;
  ramps[i2].dx0 = test.dx1;
  
  //test the in-between ramps
  ParabolicRampND temp;
  temp = ramps[i1];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  temp = ramps[i2];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  
  //replace intermediate ramps with test
  for(int i=0;i<i2-i1-1;i++)
    ramps.erase(ramps.begin()+i1+1);
  ramps.insert(ramps.begin()+i1+1,test);
  
  //check for consistency
  for(size_t i=0;i+1<ramps.size();i++) {
    assert(ramps[i].x1 == ramps[i+1].x0);
    assert(ramps[i].dx1 == ramps[i+1].dx0);
  }
  return true;
}

bool DynamicPath::TryShortcut(Real t1,Real t2,FeasibilityCheckerBase* feas,DistanceCheckerBase* dist)
{
  if(t1 > t2) Swap(t1,t2);
  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
  int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
  if(i1 == i2) return false;
  Real u1 = t1-rampStartTime[i1];
  Real u2 = t2-rampStartTime[i2];
  assert(u1 >= 0);
  assert(u1 <= ramps[i1].endTime+EpsilonT);
  assert(u2 >= 0);
  assert(u2 <= ramps[i2].endTime+EpsilonT);
  u1 = Min(u1,ramps[i1].endTime);
  u2 = Min(u2,ramps[i2].endTime);
  ParabolicRampND test;
  ramps[i1].Evaluate(u1,test.x0);
  ramps[i2].Evaluate(u2,test.x1);
  ramps[i1].Derivative(u1,test.dx0);
  ramps[i2].Derivative(u2,test.dx1);
  bool res=test.SolveMinTime(accMax,velMax);
  if(!res) return false;
  assert(test.endTime >= 0);
  assert(test.IsValid());
  if(!CheckRamp(test,feas,dist,checkMaxIters)) return false;

  //perform shortcut
  //crop i1 and i2
  ramps[i1].TrimBack(ramps[i1].endTime-u1);
  ramps[i1].x1 = test.x0;
  ramps[i1].dx1 = test.dx0;
  ramps[i2].TrimFront(u2);
  ramps[i2].x0 = test.x1;
  ramps[i2].dx0 = test.dx1;
  
  //test the in-between ramps
  ParabolicRampND temp;
  temp = ramps[i1];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  temp = ramps[i2];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  
  //replace intermediate ramps with test
  for(int i=0;i<i2-i1-1;i++)
    ramps.erase(ramps.begin()+i1+1);
  ramps.insert(ramps.begin()+i1+1,test);
  
  //check for consistency
  for(size_t i=0;i+1<ramps.size();i++) {
    assert(ramps[i].x1 == ramps[i+1].x0);
    assert(ramps[i].dx1 == ramps[i+1].dx0);
  }
  return true;
}

int DynamicPath::Shortcut(int numIters,FeasibilityCheckerBase* space,Real tol)
{
  int shortcuts = 0;
  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  for(int iters=0;iters<numIters;iters++) {
    Real t1=Rand()*endTime,t2=Rand()*endTime;
    if(t1 > t2) Swap(t1,t2);
    int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
    int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
    if(i1 == i2) continue; //same ramp
    Real u1 = t1-rampStartTime[i1];
    Real u2 = t2-rampStartTime[i2];
    assert(u1 >= 0);
    assert(u1 <= ramps[i1].endTime+EpsilonT);
    assert(u2 >= 0);
    assert(u2 <= ramps[i2].endTime+EpsilonT);
    u1 = Min(u1,ramps[i1].endTime);
    u2 = Min(u2,ramps[i2].endTime);
    ParabolicRampND test;
    ramps[i1].Evaluate(u1,test.x0);
    ramps[i2].Evaluate(u2,test.x1);
    ramps[i1].Derivative(u1,test.dx0);
    ramps[i2].Derivative(u2,test.dx1);
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    assert(test.endTime >= 0);
    assert(test.IsValid());
    if(CheckRamp(test,space,tol)) {  //perform shortcut
      shortcuts++;
      //crop i1 and i2
      ramps[i1].TrimBack(ramps[i1].endTime-u1);
      ramps[i1].x1 = test.x0;
      ramps[i1].dx1 = test.dx0;
      ramps[i2].TrimFront(u2);
      ramps[i2].x0 = test.x1;
      ramps[i2].dx0 = test.dx1;

      //test the in-between ramps
      ParabolicRampND temp;
      temp = ramps[i1];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }
      temp = ramps[i2];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }

      //replace intermediate ramps with test
      for(int i=0;i<i2-i1-1;i++)
	ramps.erase(ramps.begin()+i1+1);
      ramps.insert(ramps.begin()+i1+1,test);

      //check for consistency
      for(size_t i=0;i+1<ramps.size();i++) {
	assert(ramps[i].x1 == ramps[i+1].x0);
	assert(ramps[i].dx1 == ramps[i+1].dx0);
      }

      //revise the timing
      rampStartTime.resize(ramps.size());
      endTime=0;
      for(size_t i=0;i<ramps.size();i++) {
	rampStartTime[i] = endTime;
	endTime += ramps[i].endTime;
      }
    }
  }
  return shortcuts;
}

int DynamicPath::ShortCircuit(FeasibilityCheckerBase* space,Real tol)
{
  int shortcuts=0;
  ParabolicRampND test;
  for(size_t i=0;i+1<ramps.size();i++) {
    test.x0 = ramps[i].x0;
    test.dx0 = ramps[i].dx0;
    test.x1 = ramps[i+1].x1;    
    test.dx1 = ramps[i+1].dx1;    
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    assert(test.endTime >= 0);
    assert(test.IsValid());
    if(CheckRamp(test,space,tol)) {  //perform shortcut    
      ramps[i] = test;
      ramps.erase(ramps.begin()+i+1);
      i--;
      shortcuts++;
    }
  }
  return shortcuts;
}


int DynamicPath::Shortcut(int numIters,FeasibilityCheckerBase* space,DistanceCheckerBase* distance)
{
  int shortcuts = 0;
  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  for(int iters=0;iters<numIters;iters++) {
    Real t1=Rand()*endTime,t2=Rand()*endTime;
    if(t1 > t2) Swap(t1,t2);
    int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
    int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
    if(i1 == i2) continue; //same ramp
    Real u1 = t1-rampStartTime[i1];
    Real u2 = t2-rampStartTime[i2];
    assert(u1 >= 0);
    assert(u1 <= ramps[i1].endTime+EpsilonT);
    assert(u2 >= 0);
    assert(u2 <= ramps[i2].endTime+EpsilonT);
    u1 = Min(u1,ramps[i1].endTime);
    u2 = Min(u2,ramps[i2].endTime);
    ParabolicRampND test;
    ramps[i1].Evaluate(u1,test.x0);
    ramps[i2].Evaluate(u2,test.x1);
    ramps[i1].Derivative(u1,test.dx0);
    ramps[i2].Derivative(u2,test.dx1);
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    assert(test.endTime >= 0);
    assert(test.IsValid());
    if(CheckRamp(test,space,distance,checkMaxIters)) {  //perform shortcut
      shortcuts++;
      //crop i1 and i2
      ramps[i1].TrimBack(ramps[i1].endTime-u1);
      ramps[i1].x1 = test.x0;
      ramps[i1].dx1 = test.dx0;
      ramps[i2].TrimFront(u2);
      ramps[i2].x0 = test.x1;
      ramps[i2].dx0 = test.dx1;

      //test the in-between ramps
      ParabolicRampND temp;
      temp = ramps[i1];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }
      temp = ramps[i2];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }

      //replace intermediate ramps with test
      for(int i=0;i<i2-i1-1;i++)
	ramps.erase(ramps.begin()+i1+1);
      ramps.insert(ramps.begin()+i1+1,test);

      //check for consistency
      for(size_t i=0;i+1<ramps.size();i++) {
	assert(ramps[i].x1 == ramps[i+1].x0);
	assert(ramps[i].dx1 == ramps[i+1].dx0);
      }

      //revise the timing
      rampStartTime.resize(ramps.size());
      endTime=0;
      for(size_t i=0;i<ramps.size();i++) {
	rampStartTime[i] = endTime;
	endTime += ramps[i].endTime;
      }
    }
  }
  return shortcuts;
}

int DynamicPath::ShortCircuit(FeasibilityCheckerBase* space,DistanceCheckerBase* dist)
{
  int shortcuts=0;
  ParabolicRampND test;
  for(size_t i=0;i+1<ramps.size();i++) {
    test.x0 = ramps[i].x0;
    test.dx0 = ramps[i].dx0;
    test.x1 = ramps[i+1].x1;    
    test.dx1 = ramps[i+1].dx1;    
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    assert(test.endTime >= 0);
    assert(test.IsValid());
    if(CheckRamp(test,space,dist,checkMaxIters)) {  //perform shortcut    
      ramps[i] = test;
      ramps.erase(ramps.begin()+i+1);
      i--;
      shortcuts++;
    }
  }
  return shortcuts;
}



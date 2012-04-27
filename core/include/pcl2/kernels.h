/**
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <math.h>
#include <vector>

//  ========================================================================
//  SSE implementations: ssebool, ssefloat
//

#include <pmmintrin.h>

class ssebool {
  public:
  __m128i m128i;
  ssebool(void)                      { }
  ssebool(__m128i v)                 { m128i = v; }
  ssebool(int i)                     { m128i = _mm_set_epi32(i, i, i, i); }
  int mask4() {
    return _mm_movemask_ps((__m128)m128i);
  }
  ssebool operator&(ssebool b)      { return ssebool(_mm_and_si128(m128i, b.m128i)); }
  ssebool operator|(ssebool b)      { return ssebool(_mm_or_si128(m128i, b.m128i)); }
  ssebool operator^(ssebool b)      { return ssebool(_mm_xor_si128(m128i, b.m128i)); }
};

class sseflt {
  public:
  static const int stride = 4;
  __m128 m128;
  sseflt(void)                      { }
  sseflt(__m128 v)                  { m128 = v; }
  sseflt(__m128i iv)                { m128 = (__m128)iv; }
  sseflt(float ff)                  { m128 = _mm_set_ps(ff, ff, ff, ff); }
  sseflt sqrt()                     { return sseflt(_mm_sqrt_ps(m128)); }
  sseflt recip()                    { return sseflt(_mm_rcp_ps(m128)); }
  ssebool lessthan(sseflt b)         { return ssebool((__m128i)_mm_cmplt_ps(m128, b.m128)); }
  ssebool lessthan(float f)         { return lessthan(sseflt(f)); }
  ssebool greaterthan(sseflt b)         { return ssebool((__m128i)_mm_cmpgt_ps(m128, b.m128)); }
  ssebool greaterthan(float f)         { return greaterthan(sseflt(f)); }
  float sum()                       {
    __m128 s = _mm_hadd_ps(m128, m128);
    return _mm_cvtss_f32(_mm_hadd_ps(s, s));
  }
  int mask4() {
    return _mm_movemask_ps(m128);
  }
  void write(float*d)               { _mm_store_ps(d, m128); }
  // sseflt operator+(const sseflt &o) { return sseflt(m128 + o.m128); }
  // sseflt operator*(const sseflt &o) { return sseflt(m128 * o.m128); }
  ssebool equals(const sseflt &b) {
    return ssebool((__m128i)_mm_cmpeq_ps(m128, b.m128));
  }
};

static sseflt operator&(const sseflt &a, const sseflt &b)
{
  return sseflt(_mm_and_ps(a.m128, b.m128));
}
static sseflt operator^(const sseflt &a, const sseflt &b)
{
  return sseflt(_mm_xor_ps(a.m128, b.m128));
}
static sseflt operator|(const sseflt &a, const sseflt &b)
{
  return sseflt(_mm_or_ps(a.m128, b.m128));
}

static sseflt operator+(const sseflt &a, const sseflt &b)
{
  return sseflt(a.m128 + b.m128);
}

static sseflt operator+(const sseflt &a, float f)
{
  return a + sseflt(f);
}

static sseflt operator*(const sseflt &a, const sseflt &b)
{
  return sseflt(a.m128 * b.m128);
}

static sseflt operator/(float f, const sseflt &b)
{
  sseflt a(f);
  return sseflt(a.m128 / b.m128);
}

class sse {
  public:
    typedef sseflt tfloat;
    typedef ssebool tbool;
};

// ========================================================================
//  vanilla implementations: vanbool, vanfloat
//

#include <math.h>

static float SQRT(float x) { return sqrt(x); }

class vanbool {
  public:
  bool v;
  vanbool(void)                      { }
  vanbool(bool b)                    { v = b; }
  vanbool(int i)                     { v = i; }
  vanbool operator&(vanbool b)      { return vanbool(v & b.v); }
  vanbool operator|(vanbool b)      { return vanbool(v | b.v); }
  vanbool operator^(vanbool b)      { return vanbool(v ^ b.v); }
};

class vanflt {
  public:
  float v;
  vanflt(void)                      { }
  vanflt(float _v)                        { v = _v; }
  vanflt sqrt()                     { return vanflt(SQRT(v)); }
  vanflt recip()                    { return vanflt(1.0 / v); }
  vanbool lessthan(vanflt b)        { return vanbool(v < b.v); }
  vanbool lessthan(float f)         { return lessthan(vanflt(f)); }
  vanbool greaterthan(vanflt b)     { return vanbool(v > b.v); }
  vanbool greaterthan(float f)      { return greaterthan(vanflt(f)); }
  vanbool equals(const vanflt &b)   { return vanbool(v == b.v); }
};

static vanflt operator+(const vanflt &a, const vanflt &b)
{
  return vanflt(a.v + b.v);
}

static vanflt operator+(const vanflt &a, float f)
{
  return a + vanflt(f);
}

static vanflt operator*(const vanflt &a, const vanflt &b)
{
  return vanflt(a.v * b.v);
}

static vanflt operator/(float f, const vanflt &b)
{
  vanflt a(f);
  return vanflt(a.v / b.v);
}

class van {
  public:
    typedef vanflt tfloat;
    typedef vanbool tbool;
};

// ========================================================================
// application kernels, templated on a type family, TF
//

template <class TF>
class kernel {
  public:
};

template <class TF>
class transform_3x3 : public kernel<TF> {
    float *m;
  public:
    typedef typename TF::tfloat mfloat;
    transform_3x3(float *_m) : m(_m) {}
    void work(mfloat &xn, mfloat &yn, mfloat &zn, mfloat x, mfloat y, mfloat z) {
      xn = x * m[0] + y * m[1] + z * m[2];
      yn = x * m[3] + y * m[4] + z * m[5];
      zn = x * m[6] + y * m[7] + z * m[8];
    }
};

template <class TF>
class dot3 : public kernel<TF> {
    float *m;
  public:
    typedef typename TF::tfloat mfloat;
    dot3(float *_m) { m = _m; }
    mfloat work(mfloat x, mfloat y, mfloat z) {
      return x * m[0] + y * m[1] + z * m[2];
    }
};

template <class TF>
class xband : public kernel<TF> {
    float lo, hi;
  public:
    typedef typename TF::tfloat mfloat;
    typedef typename TF::tbool mbool;
    xband(float _lo, float _hi) { lo = _lo; hi = _hi; }
    mbool work(mfloat x, mfloat y, mfloat z) {
      return x.greaterthan(lo) & x.lessthan(hi);
    }
};

// ========================================================================
// apply a kernel to points
//
// xyz_sum  for a kernel that returns a scalar, return the sum
// xyz_any  for a kernel that returns a predicate, return true if any points are true
// xyz_all  for a kernel that returns a predicate, return true if all points are true
// xyz_min
// xyz_max
// xyz_xyz  for a kernel that transforms xyz, xyz_xyz transforms coordinates


// ========================================================================
// apply a kernel to xyzs, type family van
//

template <class kernel>
static float xyz_sum(vanflt *xi, vanflt *yi, vanflt *zi, size_t sz, kernel &wk)
{
  float total = 0.0;
  while (sz--)
    total += wk.work(*xi++, *yi++, *zi++).v;
  return total;
}

template <class kernel>
static float xyz_sum2(vanflt *xi, vanflt *yi, vanflt *zi, size_t sz, kernel &wk)
{
  float total = 0.0;
  while (sz--) {
    float v = wk.work(*xi++, *yi++, *zi++).v;
    total += (v * v);
  }
  return total;
}

template <class kernel>
static int xyz_any(vanflt *xi, vanflt *yi, vanflt *zi, size_t sz, kernel &wk)
{
  while (sz--)
    if (wk.work(*xi++, *yi++, *zi++).v)
      return 1;
  return 0;
}

template <class kernel>
static int xyz_all(vanflt *xi, vanflt *yi, vanflt *zi, size_t sz, kernel &wk)
{
  while (sz--)
    if (!wk.work(*xi++, *yi++, *zi++).v)
      return 0;
  return 1;
}

template <class kernel>
static std::vector<size_t> xyz_pick(vanflt *xi, vanflt *yi, vanflt *zi, size_t sz, kernel &wk)
{
  std::vector<size_t> r;
  for (size_t i = 0; i < sz; i++)
    if (wk.work(*xi++, *yi++, *zi++).v)
      r.push_back(i);
  return r;
}

template <class kernel>
static void xyz_xyz(vanflt *xo, vanflt *yo, vanflt *zo,
             vanflt *xi, vanflt *yi, vanflt *zi,
             size_t sz, kernel &wk)
{
  while (sz--)
    wk.work(*xo++, *yo++, *zo++, *xi++, *yi++, *zi++);
}

// ========================================================================
// apply a kernel to xyzs, type family sse
//

// How unrolling works.  For n points, want to do ((n + 3) / 4) iterations
// through the main loop.  The final iteration gives 4,3,2 or 1 answers.

static sseflt flt_masks[4] = {
  _mm_set_epi32(~0, ~0, ~0, ~0),
  _mm_set_epi32(0, 0, 0, ~0),
  _mm_set_epi32(0, 0, ~0, ~0),
  _mm_set_epi32(0, ~0, ~0, ~0)
};

static ssebool bool_masks[4] = {
  _mm_set_epi32(~0, ~0, ~0, ~0),
  _mm_set_epi32(0, 0, 0, ~0),
  _mm_set_epi32(0, 0, ~0, ~0),
  _mm_set_epi32(0, ~0, ~0, ~0)
};


template <class kernel>
static float xyz_sum(sseflt *xi, sseflt *yi, sseflt *zi, size_t sz, kernel &wk)
{
  sseflt total(0.0), t(0.0);
  size_t fours = ((sz + 3) >> 2);
  while (fours--) {
    total = total + t;
    t = wk.work(*xi++, *yi++, *zi++);
  }
  total = total + (flt_masks[sz & 3] & t);
  return total.sum();
}

// for a kernel that returns a scalar, return the sum of squares
template <class kernel>
static float xyz_sum2(sseflt *xi, sseflt *yi, sseflt *zi, size_t sz, kernel &wk)
{
  sseflt total(0.0), t(0.0);
  size_t fours = ((sz + 3) >> 2);
  while (fours--) {
    total = total + (t * t);
    t = wk.work(*xi++, *yi++, *zi++);
  }
  total = total + (flt_masks[sz & 3] & (t * t));
  return total.sum();
}

template <class kernel>
static int xyz_any(sseflt *xi, sseflt *yi, sseflt *zi, size_t sz, kernel &wk)
{
  ssebool t(0);
  size_t fours = ((sz + 3) >> 2);
  while (fours--) {
    if (t.mask4() != 0)
      return 1;
    t = wk.work(*(sseflt*)xi++, *(sseflt*)yi++, *(sseflt*)zi++);
  }
  return ((bool_masks[sz & 3] & t).mask4() != 0);
}

template <class kernel>
static int xyz_all(sseflt *xi, sseflt *yi, sseflt *zi, size_t sz, kernel &wk)
{
  ssebool t(~0);
  size_t fours = ((sz + 3) >> 2);
  while (fours--) {
    if (t.mask4() != 15)
      return 0;
    t = wk.work(*xi++, *yi++, *zi++);
  }
  ssebool mask = (bool_masks[sz & 3] ^ ssebool(~0));
  return ((mask | t).mask4() == 15);
}

// for a kernel that returns a predicate, xyz_pick returns a vector of all the true indices

static void push4(std::vector<size_t> &v, size_t i, int msk)   // Append i, i+1, i+2, i+3 if bits in msk are set
{
  if (msk & 1)
    v.push_back(i+0);
  if (msk & 2)
    v.push_back(i+1);
  if (msk & 4)
    v.push_back(i+2);
  if (msk & 8)
    v.push_back(i+3);
}

template <class kernel>
static std::vector<size_t> xyz_pick(sseflt *xi, sseflt *yi, sseflt *zi, size_t sz, kernel &wk)
{
  std::vector<size_t> r;
  ssebool t(0);
  size_t fours = ((sz + 3) >> 2);
  size_t i = 0;
  while (fours--) {
    push4(r, i, t.mask4());
    t = wk.work(*xi++, *yi++, *zi++);
    i += 4;
  }
  ssebool mask = (bool_masks[sz & 3]);
  push4(r, i, (mask & t).mask4());
  return r;
}

template <class kernel>
static void xyz_xyz(sseflt *xo, sseflt *yo, sseflt *zo,
             sseflt *xi, sseflt *yi, sseflt *zi,
             size_t sz, kernel &wk)
{
  size_t fours = ((sz + 3) >> 2);
  while (fours--)
    wk.work(*xo++, *yo++, *zo++, *xi++, *yi++, *zi++);
}

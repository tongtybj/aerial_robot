/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of:
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
#define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
#define _CASADI_NAMESPACE_CONCAT(NS, ID) NS##ID
#define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
#define CASADI_PREFIX(ID) qd_full_model_cost_y_hess_##ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#if defined(STATIC_LINKED)
#define CASADI_SYMBOL_EXPORT
#else
#define CASADI_SYMBOL_EXPORT __declspec(dllexport)
#endif
#elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
#define CASADI_SYMBOL_EXPORT __attribute__((visibility("default")))
#else
#define CASADI_SYMBOL_EXPORT
#endif
#endif

static const casadi_int casadi_s0[17] = { 13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
static const casadi_int casadi_s1[8] = { 4, 1, 0, 4, 0, 1, 2, 3 };
static const casadi_int casadi_s2[3] = { 0, 0, 0 };
static const casadi_int casadi_s3[21] = { 17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
static const casadi_int casadi_s4[20] = { 17, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* qd_full_model_cost_y_hess:(i0[13],i1[4],i2[],i3[17],i4[],i5[4])->(o0[17x17,0nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem)
{
  return 0;
}

CASADI_SYMBOL_EXPORT int qd_full_model_cost_y_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw,
                                                   casadi_real* w, int mem)
{
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int qd_full_model_cost_y_hess_alloc_mem(void)
{
  return 0;
}

CASADI_SYMBOL_EXPORT int qd_full_model_cost_y_hess_init_mem(int mem)
{
  return 0;
}

CASADI_SYMBOL_EXPORT void qd_full_model_cost_y_hess_free_mem(int mem)
{
}

CASADI_SYMBOL_EXPORT int qd_full_model_cost_y_hess_checkout(void)
{
  return 0;
}

CASADI_SYMBOL_EXPORT void qd_full_model_cost_y_hess_release(int mem)
{
}

CASADI_SYMBOL_EXPORT void qd_full_model_cost_y_hess_incref(void)
{
}

CASADI_SYMBOL_EXPORT void qd_full_model_cost_y_hess_decref(void)
{
}

CASADI_SYMBOL_EXPORT casadi_int qd_full_model_cost_y_hess_n_in(void)
{
  return 6;
}

CASADI_SYMBOL_EXPORT casadi_int qd_full_model_cost_y_hess_n_out(void)
{
  return 1;
}

CASADI_SYMBOL_EXPORT casadi_real qd_full_model_cost_y_hess_default_in(casadi_int i)
{
  switch (i)
  {
    default:
      return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* qd_full_model_cost_y_hess_name_in(casadi_int i)
{
  switch (i)
  {
    case 0:
      return "i0";
    case 1:
      return "i1";
    case 2:
      return "i2";
    case 3:
      return "i3";
    case 4:
      return "i4";
    case 5:
      return "i5";
    default:
      return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* qd_full_model_cost_y_hess_name_out(casadi_int i)
{
  switch (i)
  {
    case 0:
      return "o0";
    default:
      return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* qd_full_model_cost_y_hess_sparsity_in(casadi_int i)
{
  switch (i)
  {
    case 0:
      return casadi_s0;
    case 1:
      return casadi_s1;
    case 2:
      return casadi_s2;
    case 3:
      return casadi_s3;
    case 4:
      return casadi_s2;
    case 5:
      return casadi_s1;
    default:
      return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* qd_full_model_cost_y_hess_sparsity_out(casadi_int i)
{
  switch (i)
  {
    case 0:
      return casadi_s4;
    default:
      return 0;
  }
}

CASADI_SYMBOL_EXPORT int qd_full_model_cost_y_hess_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw,
                                                        casadi_int* sz_w)
{
  if (sz_arg)
    *sz_arg = 6;
  if (sz_res)
    *sz_res = 1;
  if (sz_iw)
    *sz_iw = 0;
  if (sz_w)
    *sz_w = 0;
  return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

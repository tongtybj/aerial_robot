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
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) qd_body_rate_model_cost_y_fun_jac_ut_xt_ ## ID
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
#define casadi_s5 CASADI_PREFIX(s5)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s4[39] = {14, 14, 0, 1, 2, 3, 4, 5, 6, 6, 10, 14, 18, 19, 20, 21, 22, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 0, 1, 2, 3};
static const casadi_int casadi_s5[3] = {14, 0, 0};

/* qd_body_rate_model_cost_y_fun_jac_ut_xt:(i0[10],i1[4],i2[],i3[4])->(o0[14],o1[14x14,22nz],o2[14x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][1] : 0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][2] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][3] : 0;
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[3]? arg[3][0] : 0;
  if (res[0]!=0) res[0][6]=a0;
  a1=arg[0]? arg[0][7] : 0;
  a2=(a0*a1);
  a3=arg[0]? arg[0][6] : 0;
  a4=arg[3]? arg[3][1] : 0;
  a5=(a3*a4);
  a2=(a2-a5);
  a5=arg[3]? arg[3][2] : 0;
  a6=arg[0]? arg[0][9] : 0;
  a7=(a5*a6);
  a2=(a2+a7);
  a7=arg[0]? arg[0][8] : 0;
  a8=arg[3]? arg[3][3] : 0;
  a9=(a7*a8);
  a2=(a2-a9);
  a2=(a2+a4);
  if (res[0]!=0) res[0][7]=a2;
  a2=(a0*a7);
  a9=(a3*a5);
  a2=(a2-a9);
  a9=(a4*a6);
  a2=(a2-a9);
  a9=(a1*a8);
  a2=(a2+a9);
  a2=(a2+a5);
  if (res[0]!=0) res[0][8]=a2;
  a7=(a4*a7);
  a1=(a1*a5);
  a7=(a7-a1);
  a6=(a0*a6);
  a7=(a7+a6);
  a3=(a3*a8);
  a7=(a7-a3);
  a7=(a7+a8);
  if (res[0]!=0) res[0][9]=a7;
  a7=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][10]=a7;
  a7=arg[1]? arg[1][1] : 0;
  if (res[0]!=0) res[0][11]=a7;
  a7=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][12]=a7;
  a7=arg[1]? arg[1][3] : 0;
  if (res[0]!=0) res[0][13]=a7;
  a7=1.;
  if (res[1]!=0) res[1][0]=a7;
  if (res[1]!=0) res[1][1]=a7;
  if (res[1]!=0) res[1][2]=a7;
  if (res[1]!=0) res[1][3]=a7;
  if (res[1]!=0) res[1][4]=a7;
  if (res[1]!=0) res[1][5]=a7;
  a3=(-a4);
  if (res[1]!=0) res[1][6]=a3;
  if (res[1]!=0) res[1][7]=a0;
  a3=(-a8);
  if (res[1]!=0) res[1][8]=a3;
  if (res[1]!=0) res[1][9]=a5;
  a3=(-a5);
  if (res[1]!=0) res[1][10]=a3;
  if (res[1]!=0) res[1][11]=a8;
  if (res[1]!=0) res[1][12]=a0;
  a3=(-a4);
  if (res[1]!=0) res[1][13]=a3;
  a8=(-a8);
  if (res[1]!=0) res[1][14]=a8;
  a5=(-a5);
  if (res[1]!=0) res[1][15]=a5;
  if (res[1]!=0) res[1][16]=a4;
  if (res[1]!=0) res[1][17]=a0;
  if (res[1]!=0) res[1][18]=a7;
  if (res[1]!=0) res[1][19]=a7;
  if (res[1]!=0) res[1][20]=a7;
  if (res[1]!=0) res[1][21]=a7;
  return 0;
}

CASADI_SYMBOL_EXPORT int qd_body_rate_model_cost_y_fun_jac_ut_xt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int qd_body_rate_model_cost_y_fun_jac_ut_xt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int qd_body_rate_model_cost_y_fun_jac_ut_xt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void qd_body_rate_model_cost_y_fun_jac_ut_xt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int qd_body_rate_model_cost_y_fun_jac_ut_xt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void qd_body_rate_model_cost_y_fun_jac_ut_xt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void qd_body_rate_model_cost_y_fun_jac_ut_xt_incref(void) {
}

CASADI_SYMBOL_EXPORT void qd_body_rate_model_cost_y_fun_jac_ut_xt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int qd_body_rate_model_cost_y_fun_jac_ut_xt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int qd_body_rate_model_cost_y_fun_jac_ut_xt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real qd_body_rate_model_cost_y_fun_jac_ut_xt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* qd_body_rate_model_cost_y_fun_jac_ut_xt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* qd_body_rate_model_cost_y_fun_jac_ut_xt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* qd_body_rate_model_cost_y_fun_jac_ut_xt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* qd_body_rate_model_cost_y_fun_jac_ut_xt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int qd_body_rate_model_cost_y_fun_jac_ut_xt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif

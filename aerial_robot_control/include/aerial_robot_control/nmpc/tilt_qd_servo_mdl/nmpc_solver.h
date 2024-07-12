//
// Created by lijinjie on 23/11/29.
// Refactored by lijinjie on 24/07/11.
//

#ifndef TILT_QD_SERVO_MDL_NMPC_SOLVER_H
#define TILT_QD_SERVO_MDL_NMPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/c_generated_code/acados_solver_tilt_qd_servo_mdl.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoMdlMPCSolver : public BaseMPCSolver
{
public:
  TiltQdServoMdlMPCSolver()
  {
    // acados macro
    NN_ = TILT_QD_SERVO_MDL_N;
    NX_ = TILT_QD_SERVO_MDL_NX;
    NZ_ = TILT_QD_SERVO_MDL_NZ;
    NU_ = TILT_QD_SERVO_MDL_NU;
    NP_ = TILT_QD_SERVO_MDL_NP;
    NBX_ = TILT_QD_SERVO_MDL_NBX;
    NBX0_ = TILT_QD_SERVO_MDL_NBX0;
    NBU_ = TILT_QD_SERVO_MDL_NBU;
    NSBX_ = TILT_QD_SERVO_MDL_NSBX;
    NSBU_ = TILT_QD_SERVO_MDL_NSBU;
    NSH_ = TILT_QD_SERVO_MDL_NSH;
    NSH0_ = TILT_QD_SERVO_MDL_NSH0;
    NSG_ = TILT_QD_SERVO_MDL_NSG;
    NSPHI_ = TILT_QD_SERVO_MDL_NSPHI;
    NSHN_ = TILT_QD_SERVO_MDL_NSHN;
    NSGN_ = TILT_QD_SERVO_MDL_NSGN;
    NSPHIN_ = TILT_QD_SERVO_MDL_NSPHIN;
    NSPHI0_ = TILT_QD_SERVO_MDL_NSPHI0;
    NSBXN_ = TILT_QD_SERVO_MDL_NSBXN;
    NS_ = TILT_QD_SERVO_MDL_NS;
    NS0_ = TILT_QD_SERVO_MDL_NS0;
    NSN_ = TILT_QD_SERVO_MDL_NSN;
    NG_ = TILT_QD_SERVO_MDL_NG;
    NBXN_ = TILT_QD_SERVO_MDL_NBXN;
    NGN_ = TILT_QD_SERVO_MDL_NGN;
    NY0_ = TILT_QD_SERVO_MDL_NY0;
    NY_ = TILT_QD_SERVO_MDL_NY;
    NYN_ = TILT_QD_SERVO_MDL_NYN;
    NH_ = TILT_QD_SERVO_MDL_NH;
    NHN_ = TILT_QD_SERVO_MDL_NHN;
    NH0_ = TILT_QD_SERVO_MDL_NH0;
    NPHI0_ = TILT_QD_SERVO_MDL_NPHI0;
    NPHI_ = TILT_QD_SERVO_MDL_NPHI;
    NPHIN_ = TILT_QD_SERVO_MDL_NPHIN;
    NR_ = TILT_QD_SERVO_MDL_NR;

    // acados functions that only using once
    acados_ocp_capsule_ = tilt_qd_servo_mdl_acados_create_capsule();

    int status = tilt_qd_servo_mdl_acados_create_with_discretization(acados_ocp_capsule_, NN_, new_time_steps);
    if (status)
      throw std::runtime_error("tilt_qd_servo_mdl_acados_create_with_discretization() returned status " +
                               std::to_string(status) + ". Exiting.");

    nlp_config_ = tilt_qd_servo_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = tilt_qd_servo_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = tilt_qd_servo_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = tilt_qd_servo_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = tilt_qd_servo_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = tilt_qd_servo_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  };

  ~TiltQdServoMdlMPCSolver()
  {
    int status = tilt_qd_servo_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_mdl_acados_free() returned status " << status << ". \n" << std::endl;

    status = tilt_qd_servo_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_mdl_acados_free_capsule() returned status " << status << ". \n" << std::endl;
  };

protected:
  tilt_qd_servo_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  // acados functions that using multiple times
  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return tilt_qd_servo_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, int* idx, double* p, int n_update) override
  {
    return tilt_qd_servo_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx, p, n_update);
  }

  inline int acadosSolve() override
  {
    return tilt_qd_servo_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    tilt_qd_servo_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_MDL_NMPC_SOLVER_H

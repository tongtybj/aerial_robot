// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "aerial_robot_control/control/base/mpc_solver.h"

MPC::MPCSolver::MPCSolver(const aerial_robot_msgs::PredXUConstPtr& x_u_init)
{
  /* construct optimizer */
  acados_ocp_capsule = qd_body_rate_model_acados_create_capsule();

  // allocate the array and fill it accordingly
  double* new_time_steps = nullptr;
  status = qd_body_rate_model_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
  if (status)
  {
    printf("qd_body_rate_model_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  nlp_config = qd_body_rate_model_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = qd_body_rate_model_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = qd_body_rate_model_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = qd_body_rate_model_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = qd_body_rate_model_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = qd_body_rate_model_acados_get_nlp_opts(acados_ocp_capsule);

  //  set constraints idx for x0
  int idxbx0[NBX0];
  for (int i = 0; i < NBX0; i++)
  {
    idxbx0[i] = i;
  }
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);

  // set rti_phase TODO: check this according to the paper
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);

  /* reset constraints and initial guess */
  reset(x_u_init);
}

MPC::MPCSolver::~MPCSolver()
{
  // free solver
  status = qd_body_rate_model_acados_free(acados_ocp_capsule);
  if (status)
  {
    printf("qd_body_rate_model_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = qd_body_rate_model_acados_free_capsule(acados_ocp_capsule);
  if (status)
  {
    printf("qd_body_rate_model_acados_free_capsule() returned status %d. \n", status);
  }
}

void MPC::MPCSolver::reset(const aerial_robot_msgs::PredXUConstPtr& x_u)
{
  const unsigned int x_stride = x_u->x.layout.dim[1].stride;
  const unsigned int u_stride = x_u->u.layout.dim[1].stride;

  // reset initial guess
  double x[NX];
  double u[NU];
  double p[NP];
  for (int i = 0; i < N; i++)
  {
    std::copy(x_u->x.data.begin() + x_stride * i, x_u->x.data.begin() + x_stride * (i + 1), x);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x);

    std::copy(x_u->u.data.begin() + u_stride * i, x_u->u.data.begin() + u_stride * (i + 1), u);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u);

    // quaternions
    std::copy(x_u->x.data.begin() + x_stride * i + 6, x_u->x.data.begin() + x_stride * i + 10, p);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "p", p);
  }
  std::copy(x_u->x.data.begin() + x_stride * N, x_u->x.data.begin() + x_stride * (N + 1), x);
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x);

  std::copy(x_u->x.data.begin() + x_stride * N + 6, x_u->x.data.begin() + x_stride * N + 10, p);
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "p", p);

  // set feedback constraints
  double lbx0[NBX0];
  std::copy(x_u->x.data.begin() + x_stride * 0 + 0, x_u->x.data.begin() + x_stride * 0 + NBX0, lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  double ubx0[NBX0];
  std::copy(x_u->x.data.begin() + x_stride * 0 + 0, x_u->x.data.begin() + x_stride * 0 + NBX0, ubx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
}

int MPC::MPCSolver::solve(const nav_msgs::OdometryConstPtr& odom_now, const aerial_robot_msgs::PredXUConstPtr& x_u_ref)
{
  /* prepare evaluation */
  int N_timings = 1;
  double min_time = 1e12;
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;

  /* set values for reference */
  for (int i = 0; i < N; i++)
  {
    // yr = np.concatenate((xr[i, :], ur[i, :]))
    double yr[NX + NU];
    for (int j = 0; j < NX; j++)
    {
      yr[j] = xr[i * NX + j];
    }
    for (int j = 0; j < NU; j++)
    {
      yr[NX + j] = ur[i * NU + j];
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "y_ref", yr);

    // quaternion_r = xr[i, 6:10]
    // self.solver.set(i, "p", quaternion_r)  # for nonlinear quaternion error
    double quaternion_r[4];
    for (int j = 0; j < 4; j++)
    {
      quaternion_r[j] = xr[i * NX + 6 + j];
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "p", quaternion_r);
  }
  // final state of x, no u
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "y_ref", xr + N * NX);

  /* solve ocp in loop */
  for (int i = 0; i < N_timings; i++)
  {
    status = qd_body_rate_model_acados_solve(acados_ocp_capsule);

    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);
  }

  if (status != ACADOS_SUCCESS)
  {
    ROS_WARN("qd_body_rate_model_acados_solve() failed with status %d.\n", status);
  }

  /* get solutions */
  for (int ii = 0; ii <= nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &out_xu.x_traj[ii * NX]);
  for (int ii = 0; ii < nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &out_xu.u_traj[ii * NU]);

  /* print statistics in DEBUG mode */
  if (getenv("ROS_DEBUG"))
  {
    std::cout << "\n--- x_traj ---\n" << std::endl;

    for (int i = 0; i <= N; i++)
    {
      std::cout << "X Row " << i << ":" << std::endl;
      for (int j = 0; j < NX; j++)
      {
        int index = i * NX + j;
        std::cout << out_xu.x_traj[index] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "\n--- u_traj ---\n" << std::endl;

    for (int i = 0; i < N; i++)
    {
      std::cout << "U Row " << i << ":" << std::endl;
      for (int j = 0; j < NU; j++)
      {
        int index = i * NU + j;
        std::cout << out_xu.u_traj[index] << " ";
      }
      std::cout << std::endl;
    }

    ROS_DEBUG("\nsolved ocp %d times, solution printed above\n\n", N_timings);

    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    qd_body_rate_model_acados_print_stats(acados_ocp_capsule);
    ROS_DEBUG("\nSolver info:\n");
    ROS_DEBUG(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, N_timings,
              min_time * 1000, kkt_norm_inf);
  }

  return status;
}

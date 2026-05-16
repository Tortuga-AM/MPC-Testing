#include <jni.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include "frc_robot_Drive_TinyMPCJNI.h"
#include "tinympc/tiny_api.hpp"

using namespace Eigen;

TinySolver *solver = nullptr;

JNIEXPORT void JNICALL Java_frc_robot_Drive_TinyMPCJNI_initializeNative(JNIEnv *env, jclass clazz) {
    if (solver != nullptr) return;

    double dt = 0.02;
    // Dynamics
    Matrix<double, 6, 6> Adyn = Matrix<double, 6, 6>::Identity();
    Adyn(0, 3) = dt; Adyn(1, 4) = dt; Adyn(2, 5) = dt;

    Matrix<double, 6, 3> Bdyn = Matrix<double, 6, 3>::Zero();
    double hdt2 = 0.5 * dt * dt;
    // Ensure each input ONLY affects its own state
    Bdyn.setZero();
    Bdyn(0, 0) = hdt2; // ax -> x
    Bdyn(1, 1) = hdt2; // ay -> y
    Bdyn(2, 2) = hdt2; // alpha -> theta
    Bdyn(3, 0) = dt;   // ax -> vx
    Bdyn(4, 1) = dt;   // ay -> vy
    Bdyn(5, 2) = dt;   // alpha -> omega

    // VERY HIGH Q, LOW R
    Matrix<double, 6, 1> q_diag;
    q_diag << 8000000.0, 8000000.0, 8000000.0, 1000000.0, 1000000.0, 1000000.0;
    Matrix<double, 6, 6> Q = q_diag.asDiagonal();

    Matrix<double, 3, 1> r_diag;
    r_diag << 0.000000000001, 0.000000000001, 0.000000000001; 
    Matrix<double, 3, 3> R = r_diag.asDiagonal();

    Matrix<double, 6, 30> x_min, x_max;
    x_min.setConstant(-1e6); x_max.setConstant(1e6);
    Matrix<double, 3, 29> u_min, u_max;
    u_min.setConstant(-9); u_max.setConstant(9);

    // Setup with VERBOSE = 1 to see the precomputation log
    int status = tiny_setup(&solver, Adyn, Bdyn, Q, R, 0.03, 6, 3, 30, x_min, x_max, u_min, u_max, 1);
    if (status == 0 && solver != nullptr) {
        solver->settings->max_iter = 100;
        solver->settings->check_termination = 5; // Larger than max_iter means it never runs
        solver->settings->abs_pri_tol = 1e-4;
        solver->settings->abs_dua_tol = 1e-4;    // FORCE it to run all max_iter for testing
        
        printf("Solver settings forced. Max Iter: 200, Check Term: Enabled(2)\n");
    }
    if (status == 0 && solver != nullptr) {
        // Check if Kinf is zero. If Kinf is 0, the solver will NEVER move.
        double k_sum = solver->cache->Kinf.array().abs().sum();
        printf("Precompute Check: Kinf L1 Norm = %f\n", k_sum);
    }
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_Drive_TinyMPCJNI_solveNative(
    JNIEnv *env, jclass clazz, jdoubleArray x0_java, jdoubleArray xref_flat_java) {
    
    if (solver == nullptr) return nullptr;

    jdouble* x0_ptr = env->GetDoubleArrayElements(x0_java, NULL);
    jdouble* xref_ptr = env->GetDoubleArrayElements(xref_flat_java, NULL);

    // Map into double matrices
    for (int i = 0; i < 6; i++) {
        solver->work->x(i, 0) = x0_ptr[i];
    }
    for (int i = 0; i < 180; i++) {
        solver->work->Xref.data()[i] = xref_ptr[i];
    }
    //printf("Native Debug: x0[0]=%f, Xref[0,0]=%f, Xref[0,1]=%f\n", solver->work->x(0,0), solver->work->Xref(0,0), solver->work->Xref(0,1));
    solver->work->Uref.setZero();
    solver->work->u.setZero();
    solver->work->v.setZero();
    solver->work->y.setZero();
    solver->work->g.setZero();

    // 2. Ensure constraints are actually disabled if we aren't using them
    solver->settings->en_state_bound = 0; 
    solver->settings->en_input_bound = 1;
    tiny_solve(solver);
    //printf("Post-Solve: u0=%f, u1=%f, iter=%d\n", 
    //        solver->solution->u(0,0), solver->solution->u(1,0), solver->solution->iter);

    jdoubleArray result = env->NewDoubleArray(3);
    // Everything is already double!
    double u0_val[3] = {
        solver->solution->u(0, 0), 
        solver->solution->u(1, 0), 
        solver->solution->u(2, 0)
    };
    env->SetDoubleArrayRegion(result, 0, 3, u0_val);
    // double u0[3] = {
    //     solver->solution->u(0, 0), 
    //     solver->solution->u(1, 0), 
    //     solver->solution->u(2, 0)
    // };
    // env->SetDoubleArrayRegion(result, 0, 3, u0);

    env->ReleaseDoubleArrayElements(x0_java, x0_ptr, JNI_ABORT);
    env->ReleaseDoubleArrayElements(xref_flat_java, xref_ptr, JNI_ABORT);

    return result;
}
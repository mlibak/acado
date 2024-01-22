#include <acado_code_generation.hpp>
#include <acado_toolkit.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cstring>
#include <cstdlib>

USING_NAMESPACE_ACADO
using namespace std;

int main(int argc, char *const argv[])
{

    // ***************************************************
    // SETTING UP SYSTEM DYNAMICS AND FORMULATING THE OCP
    // ***************************************************

    // -----------------------------
    // SYSTEM VARIABLES
    // -----------------------------

    // X coordinate
    DifferentialState x;
    // Y coordinate
    DifferentialState y;
    // Velocity
    DifferentialState v;
    // Yaw angle (heading)
    DifferentialState psy;
    // Front wheel steering angle rate
    DifferentialState delta_dot;

    // Front wheel steering angle
    Control delta_f;
    // Acceleration
    Control a;

    // -----------------------------
    // VEHICLE PARAMETERS
    // -----------------------------

    const double l_f = 1.46205; // In Unity -> 7 times larger
    const double l_r = 1.41395; // In Unity -> 7 times larger

    // Wheelbase, [m]
    const double l = l_f + l_r;

    // -----------------------------
    // DIFFERENTIAL EEQUATION
    // -----------------------------

    DifferentialEquation f;

    f << dot(x) == v * cos(psy + atan((l_r / l) * tan(delta_f)));
    f << dot(y) == v * sin(psy + atan((l_r / l) * tan(delta_f)));
    f << dot(v) == 20 * a;
    f << dot(psy) == (v / l_r) * sin(atan((l_r / l) * tan(delta_f)));

    // -----------------------------
    // LEAST SQUARES FUNCTION
    // -----------------------------

    Function h, hN;

    // Running cost vector (states and inputs)
    h << x << y << v << psy << delta_f << a;

    // End cost vector (no inputs at last state)
    hN << x << y << v << psy;

    // Weighting matrices
    BMatrix Q = eye<bool>(h.getDim());
    BMatrix P = eye<bool>(hN.getDim());

    // -----------------------------
    // OPTIMAL CONTROL PROBLEM
    // -----------------------------

    const double t_start = 0.0;
    const double t_end = 3.0;

    // Discretization
    const double dt = 0.1;

    // Horizon
    const int N = round(t_end / dt);

    // Define OCP
    OCP ocp(t_start, t_end, N);

    // Add system dynamics to the OCP
    ocp.subjectTo(f);

    // Add least squares minimization
    ocp.minimizeLSQ(Q, h);
    ocp.minimizeLSQEndTerm(P, hN);

    // -----------------------------
    // STATIC CONSTRAINTS
    // -----------------------------

    // Velocity up to 108 [kmh] -> 30 [m/s]
    ocp.subjectTo(v <= 30);
    ocp.subjectTo(-1 <= a <= 1);

    // Steering angle ±28 [degrees] -> ±0.4886921906 [rad]
    ocp.subjectTo(-0.4886921906 <= delta_f <= 0.4886921906);

    // -----------------------------
    // DYNAMIC CONSTRAINTS
    // -----------------------------

    // Constraints for lane boundaries

    // X coordinate of the track's centerline
    OnlineData x_cl;
    // // Y coordinate of the track's centerline
    OnlineData y_cl;
    // Race track's width
    // OnlineData racetrack_width;
    // double racetrack_width = 7;
    double racetrack_width = 4 * 4;

    // ocp.subjectTo(sqrt((x - x_cl) * (x - x_cl) + (y - y_cl) * (y - y_cl)) + racetrack_width >= 0);
    // ocp.subjectTo(sqrt((x - x_cl) * (x - x_cl) + (y - y_cl) * (y - y_cl)) - racetrack_width <= 0);

    ocp.subjectTo((x - x_cl) * (x - x_cl) + (y - y_cl) * (y - y_cl) - racetrack_width <= 0);

    // Set number of OnlineData(OD) manualy
    // Corrects a bug in ACADO where wrong number of ODs is stored
    ocp.setNOD(2);

    // ***************************************************
    // CODE GENERATION
    // ***************************************************

    // Define an export module
    OCPexport mpc(ocp);

    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4); // accurate
    mpc.set(NUM_INTEGRATOR_STEPS, N);
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, 'NO');
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, 'YES');

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(QP_SOLVER, QP_QPOASES);

    // Do not generate tests, makes or matlab interfaces
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Export to the package in the ROS2 workspace
    if (mpc.exportCode("../../../ros_ws/src/speedcar_mpc/externals/acado") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);
    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
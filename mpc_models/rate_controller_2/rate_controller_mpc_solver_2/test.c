/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// Initialize OnlineData matrix
const int size_temp_od = (N + 1) * NX;

/* A template for testing of the solver. */
int main( )
{
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

	// Initialize Online Data
	for (i = 0; i < size_temp_od; ++i)
	{
		acadoVariables.od[i] = 0;
	}
	// Set online data to their respective values
	for (i = 0; i < N + 1; ++i)
	{
		// These states work
		// acadoVariables.od[i * NOD] = 15;
		// acadoVariables.od[i * NOD + 1] = 0.034;
		// acadoVariables.od[i * NOD + 2] = 0;
		// acadoVariables.od[i * NOD + 3] = 0;
		// acadoVariables.od[i * NOD + 4] = 0.034;
		// These states don't work
		acadoVariables.od[i * NOD] = 15.2088;
		acadoVariables.od[i * NOD + 1] = -0.0359;
		acadoVariables.od[i * NOD + 2] = 0.02039;
		acadoVariables.od[i * NOD + 3] = 0.00147;
		acadoVariables.od[i * NOD + 4] = 0.0564;
	}

	/* Initialize the reference. */
	for (i = 0; i < N; ++i)
		acadoVariables.y[i * NY] = 0.02137;
		acadoVariables.y[i * NY + 1] = 0.0;
		acadoVariables.y[i * NY + 2] = 0.0;
		acadoVariables.y[i * NY + 3] = 0.0;
		acadoVariables.y[i * NY + 4] = 0.0;
		acadoVariables.y[i * NY + 5] = 0.0;
	for (i = 0; i < NYN; ++i)
		acadoVariables.yN[i] = 0.0;
	acadoVariables.yN[0] = acadoVariables.y[0];

	// Initialize nodes by propagating the first state instance by the given inputs and online data
	printf("Propagating state with Forward Simulation\n");
	acado_initializeNodesByForwardSimulation();

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;
#endif

	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
		/* Perform the feedback step. */
		for (i = 0; i < NX; ++i)
		{
			acadoVariables.x0[i] = acadoVariables.x[i];
		}
		// Change reference values with time
		// for (i = 0; i < NY * N; ++i)
		// 	acadoVariables.y[i] = iter * 0.05;
		// for (i = 0; i < NYN; ++i)
		// 	acadoVariables.yN[i] = iter * 0.05;

		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		printf("Current states:\n");
		for (i = 0; i < NX; ++i)
		{
			printf("%f, ", acadoVariables.x[i]);
		}
		printf("\n");
		printf("New control inputs:\n");
		for (i = 0; i < NX; ++i)
		{
			printf("%f, ", acadoVariables.u[i]);
		}
		printf("\n");

		if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

		/* Optional: shift the initialization (look at acado_common.h). */
        // acado_shiftStates(2, 0, 0);
		// acado_shiftControls( 0 );

		/* Prepare for the next step. */
		acado_preparationStep();
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

    return 0;
}

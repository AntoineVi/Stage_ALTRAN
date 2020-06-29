##
##  This file is part of qpOASES.
##
##  qpOASES -- An Implementation of the Online Active Set Strategy.
##  Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
##  Christian Kirches et al. All rights reserved.
##
##  qpOASES is free software; you can redistribute it and/or
##  modify it under the terms of the GNU Lesser General Public
##  License as published by the Free Software Foundation; either
##  version 2.1 of the License, or (at your option) any later version.
##
##  qpOASES is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##  See the GNU Lesser General Public License for more details.
##
##  You should have received a copy of the GNU Lesser General Public
##  License along with qpOASES; if not, write to the Free Software
##  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##

## author of this file: Sebastian F. Walter

"""
Python interface to qpOASES
using Cython
:author: Sebastian F. Walter, Manuel Kudruss
"""

import numpy as np
cimport numpy as np

from cython.operator cimport dereference as deref

cimport qpoases

cdef class PyBooleanType:
    FALSE = BT_FALSE
    TRUE  = BT_TRUE

cdef class PyPrintLevel:
    DEBUG_ITER = PL_DEBUG_ITER
    TABULAR    = PL_TABULAR
    NONE       = PL_NONE
    LOW        = PL_LOW
    MEDIUM     = PL_MEDIUM
    HIGH       = PL_HIGH

cdef class PyHessianType:
    ZERO               = HST_ZERO
    IDENTITY           = HST_IDENTITY
    POSDEF             = HST_POSDEF
    POSDEF_NULLSPACE   = HST_POSDEF_NULLSPACE
    SEMIDEF            = HST_SEMIDEF
    UNKNOWN            = HST_UNKNOWN

cdef class PySubjectToStatus:
    LOWER              = ST_LOWER
    INACTIVE           = ST_INACTIVE
    UPPER              = ST_UPPER
    INFEASIBLE_LOWER   = ST_INFEASIBLE_LOWER
    INFEASIBLE_UPPER   = ST_INFEASIBLE_UPPER
    UNDEFINED          = ST_UNDEFINED

cdef class PyReturnValue:
    TERMINAL_LIST_ELEMENT                 = -1
    SUCCESSFUL_RETURN                     = 0
    DIV_BY_ZERO                           = RET_DIV_BY_ZERO
    INDEX_OUT_OF_BOUNDS                   = RET_INDEX_OUT_OF_BOUNDS
    INVALID_ARGUMENTS                     = RET_INVALID_ARGUMENTS
    ERROR_UNDEFINED                       = RET_ERROR_UNDEFINED
    WARNING_UNDEFINED                     = RET_WARNING_UNDEFINED
    INFO_UNDEFINED                        = RET_INFO_UNDEFINED
    EWI_UNDEFINED                         = RET_EWI_UNDEFINED
    AVAILABLE_WITH_LINUX_ONLY             = RET_AVAILABLE_WITH_LINUX_ONLY
    UNKNOWN_BUG                           = RET_UNKNOWN_BUG
    PRINTLEVEL_CHANGED                    = RET_PRINTLEVEL_CHANGED
    NOT_YET_IMPLEMENTED                   = RET_NOT_YET_IMPLEMENTED
    INDEXLIST_MUST_BE_REORDERD            = RET_INDEXLIST_MUST_BE_REORDERD
    INDEXLIST_EXCEEDS_MAX_LENGTH          = RET_INDEXLIST_EXCEEDS_MAX_LENGTH
    INDEXLIST_CORRUPTED                   = RET_INDEXLIST_CORRUPTED
    INDEXLIST_OUTOFBOUNDS                 = RET_INDEXLIST_OUTOFBOUNDS
    INDEXLIST_ADD_FAILED                  = RET_INDEXLIST_ADD_FAILED
    INDEXLIST_INTERSECT_FAILED            = RET_INDEXLIST_INTERSECT_FAILED
    INDEX_ALREADY_OF_DESIRED_STATUS       = RET_INDEX_ALREADY_OF_DESIRED_STATUS
    ADDINDEX_FAILED                       = RET_ADDINDEX_FAILED
    REMOVEINDEX_FAILED                    = RET_REMOVEINDEX_FAILED
    SWAPINDEX_FAILED                      = RET_SWAPINDEX_FAILED
    NOTHING_TO_DO                         = RET_NOTHING_TO_DO
    SETUP_BOUND_FAILED                    = RET_SETUP_BOUND_FAILED
    SETUP_CONSTRAINT_FAILED               = RET_SETUP_CONSTRAINT_FAILED
    MOVING_BOUND_FAILED                   = RET_MOVING_BOUND_FAILED
    MOVING_CONSTRAINT_FAILED              = RET_MOVING_CONSTRAINT_FAILED
    SHIFTING_FAILED                       = RET_SHIFTING_FAILED
    ROTATING_FAILED                       = RET_ROTATING_FAILED
    QPOBJECT_NOT_SETUP                    = RET_QPOBJECT_NOT_SETUP
    QP_ALREADY_INITIALISED                = RET_QP_ALREADY_INITIALISED
    NO_INIT_WITH_STANDARD_SOLVER          = RET_NO_INIT_WITH_STANDARD_SOLVER
    RESET_FAILED                          = RET_RESET_FAILED
    INIT_FAILED                           = RET_INIT_FAILED
    INIT_FAILED_TQ                        = RET_INIT_FAILED_TQ
    INIT_FAILED_CHOLESKY                  = RET_INIT_FAILED_CHOLESKY
    INIT_FAILED_HOTSTART                  = RET_INIT_FAILED_HOTSTART
    INIT_FAILED_INFEASIBILITY             = RET_INIT_FAILED_INFEASIBILITY
    INIT_FAILED_UNBOUNDEDNESS             = RET_INIT_FAILED_UNBOUNDEDNESS
    INIT_FAILED_REGULARISATION            = RET_INIT_FAILED_REGULARISATION
    INIT_SUCCESSFUL                       = RET_INIT_SUCCESSFUL
    OBTAINING_WORKINGSET_FAILED           = RET_OBTAINING_WORKINGSET_FAILED
    SETUP_WORKINGSET_FAILED               = RET_SETUP_WORKINGSET_FAILED
    SETUP_AUXILIARYQP_FAILED              = RET_SETUP_AUXILIARYQP_FAILED
    NO_EXTERN_SOLVER                      = RET_NO_EXTERN_SOLVER
    QP_UNBOUNDED                          = RET_QP_UNBOUNDED
    QP_INFEASIBLE                         = RET_QP_INFEASIBLE
    QP_NOT_SOLVED                         = RET_QP_NOT_SOLVED
    QP_SOLVED                             = RET_QP_SOLVED
    UNABLE_TO_SOLVE_QP                    = RET_UNABLE_TO_SOLVE_QP
    INITIALISATION_STARTED                = RET_INITIALISATION_STARTED
    HOTSTART_FAILED                       = RET_HOTSTART_FAILED
    HOTSTART_FAILED_TO_INIT               = RET_HOTSTART_FAILED_TO_INIT
    HOTSTART_FAILED_AS_QP_NOT_INITIALISED = RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED
    ITERATION_STARTED                     = RET_ITERATION_STARTED
    SHIFT_DETERMINATION_FAILED            = RET_SHIFT_DETERMINATION_FAILED
    STEPDIRECTION_DETERMINATION_FAILED    = RET_STEPDIRECTION_DETERMINATION_FAILED
    STEPLENGTH_DETERMINATION_FAILED       = RET_STEPLENGTH_DETERMINATION_FAILED
    OPTIMAL_SOLUTION_FOUND                = RET_OPTIMAL_SOLUTION_FOUND
    HOMOTOPY_STEP_FAILED                  = RET_HOMOTOPY_STEP_FAILED
    HOTSTART_STOPPED_INFEASIBILITY        = RET_HOTSTART_STOPPED_INFEASIBILITY
    HOTSTART_STOPPED_UNBOUNDEDNESS        = RET_HOTSTART_STOPPED_UNBOUNDEDNESS
    WORKINGSET_UPDATE_FAILED              = RET_WORKINGSET_UPDATE_FAILED
    MAX_NWSR_REACHED                      = RET_MAX_NWSR_REACHED
    CONSTRAINTS_NOT_SPECIFIED             = RET_CONSTRAINTS_NOT_SPECIFIED
    INVALID_FACTORISATION_FLAG            = RET_INVALID_FACTORISATION_FLAG
    UNABLE_TO_SAVE_QPDATA                 = RET_UNABLE_TO_SAVE_QPDATA
    STEPDIRECTION_FAILED_TQ               = RET_STEPDIRECTION_FAILED_TQ
    STEPDIRECTION_FAILED_CHOLESKY         = RET_STEPDIRECTION_FAILED_CHOLESKY
    CYCLING_DETECTED                      = RET_CYCLING_DETECTED
    CYCLING_NOT_RESOLVED                  = RET_CYCLING_NOT_RESOLVED
    CYCLING_RESOLVED                      = RET_CYCLING_RESOLVED
    STEPSIZE                              = RET_STEPSIZE
    STEPSIZE_NONPOSITIVE                  = RET_STEPSIZE_NONPOSITIVE
    SETUPSUBJECTTOTYPE_FAILED             = RET_SETUPSUBJECTTOTYPE_FAILED
    ADDCONSTRAINT_FAILED                  = RET_ADDCONSTRAINT_FAILED
    ADDCONSTRAINT_FAILED_INFEASIBILITY    = RET_ADDCONSTRAINT_FAILED_INFEASIBILITY
    ADDBOUND_FAILED                       = RET_ADDBOUND_FAILED
    ADDBOUND_FAILED_INFEASIBILITY         = RET_ADDBOUND_FAILED_INFEASIBILITY
    REMOVECONSTRAINT_FAILED               = RET_REMOVECONSTRAINT_FAILED
    REMOVEBOUND_FAILED                    = RET_REMOVEBOUND_FAILED
    REMOVE_FROM_ACTIVESET                 = RET_REMOVE_FROM_ACTIVESET
    ADD_TO_ACTIVESET                      = RET_ADD_TO_ACTIVESET
    REMOVE_FROM_ACTIVESET_FAILED          = RET_REMOVE_FROM_ACTIVESET_FAILED
    ADD_TO_ACTIVESET_FAILED               = RET_ADD_TO_ACTIVESET_FAILED
    CONSTRAINT_ALREADY_ACTIVE             = RET_CONSTRAINT_ALREADY_ACTIVE
    ALL_CONSTRAINTS_ACTIVE                = RET_ALL_CONSTRAINTS_ACTIVE
    LINEARLY_DEPENDENT                    = RET_LINEARLY_DEPENDENT
    LINEARLY_INDEPENDENT                  = RET_LINEARLY_INDEPENDENT
    LI_RESOLVED                           = RET_LI_RESOLVED
    ENSURELI_FAILED                       = RET_ENSURELI_FAILED
    ENSURELI_FAILED_TQ                    = RET_ENSURELI_FAILED_TQ
    ENSURELI_FAILED_NOINDEX               = RET_ENSURELI_FAILED_NOINDEX
    ENSURELI_FAILED_CYCLING               = RET_ENSURELI_FAILED_CYCLING
    BOUND_ALREADY_ACTIVE                  = RET_BOUND_ALREADY_ACTIVE
    ALL_BOUNDS_ACTIVE                     = RET_ALL_BOUNDS_ACTIVE
    CONSTRAINT_NOT_ACTIVE                 = RET_CONSTRAINT_NOT_ACTIVE
    BOUND_NOT_ACTIVE                      = RET_BOUND_NOT_ACTIVE
    HESSIAN_NOT_SPD                       = RET_HESSIAN_NOT_SPD
    HESSIAN_INDEFINITE                    = RET_HESSIAN_INDEFINITE
    MATRIX_SHIFT_FAILED                   = RET_MATRIX_SHIFT_FAILED
    MATRIX_FACTORISATION_FAILED           = RET_MATRIX_FACTORISATION_FAILED
    PRINT_ITERATION_FAILED                = RET_PRINT_ITERATION_FAILED
    NO_GLOBAL_MESSAGE_OUTPUTFILE          = RET_NO_GLOBAL_MESSAGE_OUTPUTFILE
    DISABLECONSTRAINTS_FAILED             = RET_DISABLECONSTRAINTS_FAILED
    ENABLECONSTRAINTS_FAILED              = RET_ENABLECONSTRAINTS_FAILED
    ALREADY_ENABLED                       = RET_ALREADY_ENABLED
    ALREADY_DISABLED                      = RET_ALREADY_DISABLED
    NO_HESSIAN_SPECIFIED                  = RET_NO_HESSIAN_SPECIFIED
    USING_REGULARISATION                  = RET_USING_REGULARISATION
    EPS_MUST_BE_POSITVE                   = RET_EPS_MUST_BE_POSITVE
    REGSTEPS_MUST_BE_POSITVE              = RET_REGSTEPS_MUST_BE_POSITVE
    HESSIAN_ALREADY_REGULARISED           = RET_HESSIAN_ALREADY_REGULARISED
    CANNOT_REGULARISE_IDENTITY            = RET_CANNOT_REGULARISE_IDENTITY
    CANNOT_REGULARISE_SPARSE              = RET_CANNOT_REGULARISE_SPARSE
    NO_REGSTEP_NWSR                       = RET_NO_REGSTEP_NWSR
    FEWER_REGSTEPS_NWSR                   = RET_FEWER_REGSTEPS_NWSR
    CHOLESKY_OF_ZERO_HESSIAN              = RET_CHOLESKY_OF_ZERO_HESSIAN
    CONSTRAINTS_ARE_NOT_SCALED            = RET_CONSTRAINTS_ARE_NOT_SCALED
    INITIAL_BOUNDS_STATUS_NYI             = RET_INITIAL_BOUNDS_STATUS_NYI
    ERROR_IN_CONSTRAINTPRODUCT            = RET_ERROR_IN_CONSTRAINTPRODUCT
    UPDATEMATRICES_FAILED                 = RET_UPDATEMATRICES_FAILED
    UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED= RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED
    UNABLE_TO_OPEN_FILE                   = RET_UNABLE_TO_OPEN_FILE
    UNABLE_TO_WRITE_FILE                  = RET_UNABLE_TO_WRITE_FILE
    UNABLE_TO_READ_FILE                   = RET_UNABLE_TO_READ_FILE
    FILEDATA_INCONSISTENT                 = RET_FILEDATA_INCONSISTENT
    UNABLE_TO_ANALYSE_QPROBLEM            = RET_UNABLE_TO_ANALYSE_QPROBLEM
    NWSR_SET_TO_ONE                       = RET_NWSR_SET_TO_ONE
    UNABLE_TO_READ_BENCHMARK              = RET_UNABLE_TO_READ_BENCHMARK
    BENCHMARK_ABORTED                     = RET_BENCHMARK_ABORTED
    INITIAL_QP_SOLVED                     = RET_INITIAL_QP_SOLVED
    QP_SOLUTION_STARTED                   = RET_QP_SOLUTION_STARTED
    BENCHMARK_SUCCESSFUL                  = RET_BENCHMARK_SUCCESSFUL
    NO_DIAGONAL_AVAILABLE                 = RET_NO_DIAGONAL_AVAILABLE
    ENSURELI_DROPPED                      = RET_ENSURELI_DROPPED
    SIMPLE_STATUS_P1                      = RET_SIMPLE_STATUS_P1
    SIMPLE_STATUS_P0                      = RET_SIMPLE_STATUS_P0
    SIMPLE_STATUS_M1                      = RET_SIMPLE_STATUS_M1
    SIMPLE_STATUS_M2                      = RET_SIMPLE_STATUS_M2
    SIMPLE_STATUS_M3                      = RET_SIMPLE_STATUS_M3



cdef class PyOptions:
    cdef Options *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self):
        # FIXME: add support for the other constructors
        self.thisptr = new Options()

    def __dealloc__(self):
        del self.thisptr

    def setToDefault(self):
        return self.thisptr.setToDefault()

    def setToReliable(self):
        return self.thisptr.setToReliable()

    def setToMPC(self):
        return self.thisptr.setToMPC()

    def setToFast(self):
        return self.thisptr.setToFast()

    def ensureConsistency(self):
        return self.thisptr.ensureConsistency()

    property printLevel:
        def __get__(self): return self.thisptr.printLevel
        def __set__(self, printLevel): self.thisptr.printLevel = printLevel

    property enableRamping:
        def __get__(self): return self.thisptr.enableRamping
        def __set__(self, enableRamping): self.thisptr.enableRamping = enableRamping

    property enableFarBounds:
        def __get__(self): return self.thisptr.enableFarBounds
        def __set__(self, enableFarBounds): self.thisptr.enableFarBounds = enableFarBounds

    property enableFlippingBounds:
        def __get__(self): return self.thisptr.enableFlippingBounds
        def __set__(self, enableFlippingBounds): self.thisptr.enableFlippingBounds = enableFlippingBounds

    property enableRegularisation:
        def __get__(self): return self.thisptr.enableRegularisation
        def __set__(self, enableRegularisation): self.thisptr.enableRegularisation = enableRegularisation

    property enableFullLITests:
        def __get__(self): return self.thisptr.enableFullLITests
        def __set__(self, enableFullLITests): self.thisptr.enableFullLITests = enableFullLITests

    property enableNZCTests:
        def __get__(self): return self.thisptr.enableNZCTests
        def __set__(self, enableNZCTests): self.thisptr.enableNZCTests = enableNZCTests

    property enableDriftCorrection:
        def __get__(self): return self.thisptr.enableDriftCorrection
        def __set__(self, enableDriftCorrection): self.thisptr.enableDriftCorrection = enableDriftCorrection

    property enableCholeskyRefactorisation:
        def __get__(self): return self.thisptr.enableCholeskyRefactorisation
        def __set__(self, enableCholeskyRefactorisation): self.thisptr.enableCholeskyRefactorisation = enableCholeskyRefactorisation

    property enableEqualities:
        def __get__(self): return self.thisptr.enableEqualities
        def __set__(self, enableEqualities): self.thisptr.enableEqualities = enableEqualities

    property terminationTolerance:
        def __get__(self): return self.thisptr.terminationTolerance
        def __set__(self, terminationTolerance): self.thisptr.terminationTolerance = terminationTolerance

    property boundTolerance:
        def __get__(self): return self.thisptr.boundTolerance
        def __set__(self, boundTolerance): self.thisptr.boundTolerance = boundTolerance

    property boundRelaxation:
        def __get__(self): return self.thisptr.boundRelaxation
        def __set__(self, boundRelaxation): self.thisptr.boundRelaxation = boundRelaxation

    property epsNum:
        def __get__(self): return self.thisptr.epsNum
        def __set__(self, epsNum): self.thisptr.epsNum = epsNum

    property epsDen:
        def __get__(self): return self.thisptr.epsDen
        def __set__(self, epsDen): self.thisptr.epsDen = epsDen

    property maxPrimalJump:
        def __get__(self): return self.thisptr.maxPrimalJump
        def __set__(self, maxPrimalJump): self.thisptr.maxPrimalJump = maxPrimalJump

    property maxDualJump:
        def __get__(self): return self.thisptr.maxDualJump
        def __set__(self, maxDualJump): self.thisptr.maxDualJump = maxDualJump

    property initialRamping:
        def __get__(self): return self.thisptr.initialRamping
        def __set__(self, initialRamping): self.thisptr.initialRamping = initialRamping

    property finalRamping:
        def __get__(self): return self.thisptr.finalRamping
        def __set__(self, finalRamping): self.thisptr.finalRamping = finalRamping

    property initialFarBounds:
        def __get__(self): return self.thisptr.initialFarBounds
        def __set__(self, initialFarBounds): self.thisptr.initialFarBounds = initialFarBounds

    property growFarBounds:
        def __get__(self): return self.thisptr.growFarBounds
        def __set__(self, growFarBounds): self.thisptr.growFarBounds = growFarBounds

    property initialStatusBounds:
        def __get__(self): return self.thisptr.initialStatusBounds
        def __set__(self, initialStatusBounds): self.thisptr.initialStatusBounds = initialStatusBounds

    property epsFlipping:
        def __get__(self): return self.thisptr.epsFlipping
        def __set__(self, epsFlipping): self.thisptr.epsFlipping = epsFlipping

    property numRegularisationSteps:
        def __get__(self): return self.thisptr.numRegularisationSteps
        def __set__(self, numRegularisationSteps): self.thisptr.numRegularisationSteps = numRegularisationSteps

    property epsRegularisation:
        def __get__(self): return self.thisptr.epsRegularisation
        def __set__(self, epsRegularisation): self.thisptr.epsRegularisation = epsRegularisation

    property numRefinementSteps:
        def __get__(self): return self.thisptr.numRefinementSteps
        def __set__(self, numRefinementSteps): self.thisptr.numRefinementSteps = numRefinementSteps

    property epsIterRef:
        def __get__(self): return self.thisptr.epsIterRef
        def __set__(self, epsIterRef): self.thisptr.epsIterRef = epsIterRef

    property epsLITests:
        def __get__(self): return self.thisptr.epsLITests
        def __set__(self, epsLITests): self.thisptr.epsLITests = epsLITests

    property epsNZCTests:
        def __get__(self): return self.thisptr.epsNZCTests
        def __set__(self, epsNZCTests): self.thisptr.epsNZCTests = epsNZCTests

    property dropBoundPriority:
        def __get__(self): return self.thisptr.dropBoundPriority
        def __set__(self, dropBoundPriority): self.thisptr.dropBoundPriority = dropBoundPriority

    property dropEqConPriority:
        def __get__(self): return self.thisptr.dropEqConPriority
        def __set__(self, dropEqConPriority): self.thisptr.dropEqConPriority = dropEqConPriority

    property dropIneqConPriority:
        def __get__(self): return self.thisptr.dropIneqConPriority
        def __set__(self, dropIneqConPriority): self.thisptr.dropIneqConPriority = dropIneqConPriority



cdef class PyQProblemB:
    cdef QProblemB *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self, int nV):
        # FIXME: allow other HessianTypes!
        self.thisptr = new QProblemB(nV, HST_UNKNOWN)
    def __dealloc__(self):
        del self.thisptr

    def init(self,
             np.ndarray[np.double_t, ndim=2] H,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.init(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    nWSR,
                    &cputime
                    )

        return self.thisptr.init(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    nWSR)

    def hotstart(self,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.hotstart(
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    nWSR,
                    &cputime
                    )

        return self.thisptr.hotstart(
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    nWSR)

    def getPrimalSolution(self, np.ndarray[np.double_t, ndim=1] xOpt):
        return self.thisptr.getPrimalSolution(<double*> xOpt.data)

    def getDualSolution(self, np.ndarray[np.double_t, ndim=1] yOpt):
        return self.thisptr.getDualSolution(<double*> yOpt.data)

    def getObjVal(self):
        return self.thisptr.getObjVal()

    def printOptions(self):
        return self.thisptr.printOptions()

    def getOptions(self):
        # FIXME: memory management? who deallocates o
        cdef Options *o = new Options(self.thisptr.getOptions())
        retval = PyOptions()
        retval.thisptr = o
        return retval

    def setOptions(self, PyOptions options):
        self.thisptr.setOptions(deref(options.thisptr))


cdef class PyQProblem:
    cdef QProblem *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self, int nV, int nC):
        self.thisptr = new QProblem(nV, nC, HST_UNKNOWN)
    def __dealloc__(self):
        del self.thisptr

    cpdef init(self,
             np.ndarray[np.double_t, ndim=2] H,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=2] A,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             np.ndarray[np.double_t, ndim=1] lbA,
             np.ndarray[np.double_t, ndim=1] ubA,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.init(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> A.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR,
                    &cputime)

        return self.thisptr.init(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> A.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR)

    cpdef hotstart(self,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             np.ndarray[np.double_t, ndim=1] lbA,
             np.ndarray[np.double_t, ndim=1] ubA,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.hotstart(
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR,
                    &cputime
                    )

        return self.thisptr.hotstart(
                    <double*> g.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR)

    cpdef getPrimalSolution(self, np.ndarray[np.double_t, ndim=1] xOpt):
        return self.thisptr.getPrimalSolution(<double*> xOpt.data)

    cpdef getDualSolution(self, np.ndarray[np.double_t, ndim=1] yOpt):
        return self.thisptr.getDualSolution(<double*> yOpt.data)

    cpdef getObjVal(self):
        return self.thisptr.getObjVal()

    cpdef printOptions(self):
        return self.thisptr.printOptions()

    cpdef setOptions(self, PyOptions options):
        self.thisptr.setOptions(deref(options.thisptr))

cdef class PySQProblem:
    cdef SQProblem *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self, int nV, int nC):
        self.thisptr = new SQProblem(nV, nC, HST_UNKNOWN)
    def __dealloc__(self):
        del self.thisptr

    cpdef init(self,
             np.ndarray[np.double_t, ndim=2] H,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=2] A,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             np.ndarray[np.double_t, ndim=1] lbA,
             np.ndarray[np.double_t, ndim=1] ubA,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.init(
                        <double*> H.data,
                        <double*> g.data,
                        <double*> A.data,
                        <double*> lb.data,
                        <double*> ub.data,
                        <double*> lbA.data,
                        <double*> ubA.data,
                        nWSR,
                        &cputime)


        return self.thisptr.init(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> A.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR)

    cpdef hotstart(self,
             np.ndarray[np.double_t, ndim=2] H,
             np.ndarray[np.double_t, ndim=1] g,
             np.ndarray[np.double_t, ndim=2] A,
             np.ndarray[np.double_t, ndim=1] lb,
             np.ndarray[np.double_t, ndim=1] ub,
             np.ndarray[np.double_t, ndim=1] lbA,
             np.ndarray[np.double_t, ndim=1] ubA,
             nWSR,
             double cputime=0):

        # FIXME: add asserts

        if cputime > 1.e-16:
            return self.thisptr.hotstart(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> A.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR,
                    &cputime)

        return self.thisptr.hotstart(
                    <double*> H.data,
                    <double*> g.data,
                    <double*> A.data,
                    <double*> lb.data,
                    <double*> ub.data,
                    <double*> lbA.data,
                    <double*> ubA.data,
                    nWSR)

    cpdef getPrimalSolution(self, np.ndarray[np.double_t, ndim=1] xOpt):
        return self.thisptr.getPrimalSolution(<double*> xOpt.data)

    cpdef getDualSolution(self, np.ndarray[np.double_t, ndim=1] yOpt):
        return self.thisptr.getDualSolution(<double*> yOpt.data)

    cpdef getObjVal(self):
        return self.thisptr.getObjVal()

    cpdef printOptions(self):
        return self.thisptr.printOptions()

    cpdef setOptions(self, PyOptions options):
        self.thisptr.setOptions(deref(options.thisptr))


cdef class PySolutionAnalysis:
    cdef SolutionAnalysis *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new SolutionAnalysis()
    def __dealloc__(self):
        del self.thisptr

    cpdef getMaxKKTviolation(self, qp, np.ndarray[np.double_t, ndim=1] maxKKTviolation):

        if isinstance(qp, PyQProblemB):
            return self._getMaxKKTviolation_QProblemB(qp, maxKKTviolation)

        elif isinstance(qp, PyQProblem):
            return self._getMaxKKTviolation_QProblem(qp, maxKKTviolation)

        elif isinstance(qp, PySQProblem):
            return self._getMaxKKTviolation_SQProblem(qp, maxKKTviolation)

        else:
            raise ValueError('argument 1 must be QProblemB, QProblem or SQProblem')


    cpdef _getMaxKKTviolation_QProblemB(self, PyQProblemB qp, np.ndarray[np.double_t, ndim=1] maxKKTviolation):
        return self.thisptr.getMaxKKTviolation(qp.thisptr, <double&> maxKKTviolation.data[0])

    cpdef _getMaxKKTviolation_QProblem(self, PyQProblem qp, np.ndarray[np.double_t, ndim=1] maxKKTviolation):
        return self.thisptr.getMaxKKTviolation(qp.thisptr, <double&> maxKKTviolation.data[0])

    cpdef _getMaxKKTviolation_SQProblem(self, PySQProblem qp, np.ndarray[np.double_t, ndim=1] maxKKTviolation):
        return self.thisptr.getMaxKKTviolation(qp.thisptr, <double&> maxKKTviolation.data[0])



    cpdef getVarianceCovariance(self,
                              qp,
                              np.ndarray[np.double_t, ndim=1] g_b_bA_VAR,
                              np.ndarray[np.double_t, ndim=1] Primal_Dual_VAR ):

        if isinstance(qp, PyQProblemB):
            return self._getVarianceCovariance_QProblemB(qp, g_b_bA_VAR, Primal_Dual_VAR)

        elif isinstance(qp, PyQProblem):
            return self._getVarianceCovariance_QProblem(qp, g_b_bA_VAR, Primal_Dual_VAR)

        elif isinstance(qp, PySQProblem):
            return self._getVarianceCovariance_SQProblem(qp, g_b_bA_VAR, Primal_Dual_VAR)

        else:
            raise ValueError('argument 1 must be QProblemB, QProblem or SQProblem')

    cpdef _getVarianceCovariance_QProblemB(self,
                              PyQProblemB qp,
                              np.ndarray[np.double_t, ndim=1] g_b_bA_VAR,
                              np.ndarray[np.double_t, ndim=1] Primal_Dual_VAR ):
        return self.thisptr.getVarianceCovariance(qp.thisptr,
                                                  <double*> g_b_bA_VAR.data,
                                                  <double*> Primal_Dual_VAR.data)

    cpdef _getVarianceCovariance_QProblem(self,
                              PyQProblem qp,
                              np.ndarray[np.double_t, ndim=1] g_b_bA_VAR,
                              np.ndarray[np.double_t, ndim=1] Primal_Dual_VAR ):
        return self.thisptr.getVarianceCovariance(qp.thisptr,
                                                  <double*> g_b_bA_VAR.data,
                                                  <double*> Primal_Dual_VAR.data)

    cpdef _getVarianceCovariance_SQProblem(self,
                              PySQProblem qp,
                              np.ndarray[np.double_t, ndim=1] g_b_bA_VAR,
                              np.ndarray[np.double_t, ndim=1] Primal_Dual_VAR ):
        return self.thisptr.getVarianceCovariance(qp.thisptr,
                                                  <double*> g_b_bA_VAR.data,
                                                  <double*> Primal_Dual_VAR.data)

# Wrapped some utility functions for unit testing
cpdef py_runOQPbenchmark(path,               # Full path of the benchmark files (without trailing slash!).
                       isSparse,           # Shall convert matrices to sparse format before solution?
                       useHotstarts,       # Shall QP solution be hotstarted?
                       PyOptions options,  # QP solver options to be used while solving benchmark problems.
                       int maxAllowedNWSR, # Maximum number of working set recalculations to be performed.
                       double maxCPUTime,  # Maximum allowed CPU time for qp solving.
                       ):
    """run a QP benchmark example"""
    maxNWSR            = 0.0 # Output: Maximum number of performed working set recalculations.
    avgNWSR            = 0.0 # Output: Average number of performed working set recalculations.
    maxCPUtime         = 0.0 # Output: Maximum CPU time required for solving each QP.
    avgCPUtime         = 0.0 # Output: Average CPU time required for solving each QP.
    maxStationarity    = 0.0 # Output: Maximum residual of stationarity condition.
    maxFeasibility     = 0.0 # Output: Maximum residual of primal feasibility condition.
    maxComplementarity = 0.0 # Output: Maximum residual of complementarity condition.

    maxCPUtime = maxCPUTime

    p = path.encode()
    returnValue = runOQPbenchmark(p,
                                  isSparse,
                                  useHotstarts,
                                  deref(options.thisptr),
                                  maxAllowedNWSR,
                                  maxNWSR,
                                  avgNWSR,
                                  maxCPUtime,
                                  avgCPUtime,
                                  maxStationarity,
                                  maxFeasibility,
                                  maxComplementarity)

    return returnValue, maxNWSR, avgNWSR, maxCPUtime, avgCPUtime, \
           maxStationarity, maxFeasibility, maxComplementarity


def py_getKKTResidual(int nV,                              # Number of variables.
                      int nC,                              # Number of constraints.
                      np.ndarray[np.double_t, ndim=2] H,   # Hessian matrix.
                      np.ndarray[np.double_t, ndim=1] g,   # Sequence of gradient vectors.
                      np.ndarray[np.double_t, ndim=2] A,   # Constraint matrix.
                      np.ndarray[np.double_t, ndim=1] lb,  # Sequence of lower bound vectors (on variables).
                      np.ndarray[np.double_t, ndim=1] ub,  # Sequence of upper bound vectors (on variables).
                      np.ndarray[np.double_t, ndim=1] lbA, # Sequence of lower constraints' bound vectors.
                      np.ndarray[np.double_t, ndim=1] ubA, # Sequence of upper constraints' bound vectors.
                      np.ndarray[np.double_t, ndim=1] x,   # Sequence of primal trial vectors.
                      np.ndarray[np.double_t, ndim=1] y,   # Sequence of dual trial vectors.
                      ):
    stat = 0.0 # Maximum value of stationarity condition residual.
    feas = 0.0 # Maximum value of primal feasibility violation.
    cmpl = 0.0 # Maximum value of complementarity residual.
    getKKTResidual(nV,
                   nC,
                   <double*> H.data,
                   <double*> g.data,
                   <double*> A.data,
                   <double*> lb.data,
                   <double*> ub.data,
                   <double*> lbA.data,
                   <double*> ubA.data,
                   <double*> x.data,
                   <double*> y.data,
                   stat,
                   feas,
                   cmpl
                   )
    return stat, feas, cmpl


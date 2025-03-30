#include "solvers/Solver.h"

Solver::Solver(RigidBodySystem* rigidBodySystem)
    : m_rigidBodySystem(rigidBodySystem),
      m_maxIter(20),
      m_parallelEnabled(true),
      m_measurePerformance(false),
      m_lastSolveTime(0.0),
      m_numThreads(0)
{
}

void Solver::setNumThreads(int numThreads)
{
    m_numThreads = numThreads;
    
#ifdef USE_OPENMP
    if (numThreads > 0) {
        omp_set_num_threads(numThreads);
    } else {
        // Reset to default (uses OMP_NUM_THREADS environment variable or max available)
        omp_set_num_threads(omp_get_max_threads());
    }
#endif
}

int Solver::getNumThreads() const
{
#ifdef USE_OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

void Solver::startTiming()
{
    if (m_measurePerformance) {
        m_startTime = std::chrono::high_resolution_clock::now();
    }
}

void Solver::endTiming()
{
    if (m_measurePerformance) {
        auto endTime = std::chrono::high_resolution_clock::now();
        m_lastSolveTime = std::chrono::duration<double, std::milli>(endTime - m_startTime).count();
    }
}

int Solver::getEffectiveThreadCount() const
{
    if (!m_parallelEnabled) {
        return 1;
    }
    
#ifdef USE_OPENMP
    return m_numThreads > 0 ? m_numThreads : omp_get_max_threads();
#else
    return 1;
#endif
}
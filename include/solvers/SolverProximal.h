#pragma once
#include "solvers/Solver.h"
#include <Eigen/Dense>
#include <vector>

// NOTE: This is for the PROX PGS solver.
class SolverProximal : public Solver
{
public:
	SolverProximal(RigidBodySystem* _rigidBodySystem);
	virtual void solve(float h) override;

	// Callbacks.
	void setCSV(std::ofstream& fileName, char* filePath);
	virtual void writeCSV(std::ofstream& fileName, char* filePath);

protected:

	//SolverStartFunc m_startFunc;

	//const char* m_path;
	//std::ofstream m_file;
};


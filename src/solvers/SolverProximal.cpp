#include "solvers/SolverProximal.h"
#include <Eigen/Dense>
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "utils/Serializer.h"
#include <chrono>
#include <filesystem>
#include <utility> 

namespace {

    //// Export matrices
    //// https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format
    //const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    //template <typename Derived>
    //void writeCSVfile(std::string name, const Eigen::MatrixBase<Derived>& matrix)
    //{
    //    std::ofstream file(name.c_str());
    //    file.open(name.c_str(), std::ios::out | std::ios::app);
    //    file << matrix.format(CSVFormat) << ",";
    //}
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

    template <typename Derived>
    void writeToCSVfile(std::string name, const Eigen::MatrixBase<Derived>& matrix)
    {
        std::ofstream file(name.c_str());
        file << matrix.format(CSVFormat) << ",";
    }



    void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<int>>> dataset) {
        // Make a CSV file with one or more columns of integer values
        // Each column of data is represented by the pair <column name, column data>
        //   as std::pair<std::string, std::vector<int>>
        // The dataset is represented as a vector of these columns
        // Note that all columns should be the same size

        // Create an output filestream object
        std::ofstream myFile(filename);

        // Send column names to the stream
        for (int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).first;
            if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";

        // Send data to the stream
        for (int i = 0; i < dataset.at(0).second.size(); ++i)
        {
            for (int j = 0; j < dataset.size(); ++j)
            {
                myFile << dataset.at(j).second.at(i);
                if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
            }
            myFile << "\n";
        }

        // Close the file
        myFile.close();
    }
    // Count active set
    static inline int countActiveConstraints(std::vector<Eigen::Vector3f> lambdaCandidate, float mu)
    {
        int count = 0;
        for (int i = 0; i < lambdaCandidate.size(); ++i)
        {
            for (int j = 0; j < lambdaCandidate[i].size(); ++j)
            {
                Eigen::Vector3f upper = { FLT_MAX, mu * lambdaCandidate[i][0], mu * lambdaCandidate[i][0] };
                Eigen::Vector3f lower = { 0.0f, -mu * lambdaCandidate[i][0], -mu * lambdaCandidate[i][0] };
                if (lambdaCandidate[i][j] > lower[j] && lambdaCandidate[i][j] < upper[j])
                {
                    ++count;
                }
            }
        }
        return count;
    }

    // Compute the LCP error 
    static inline float  computeLCPError(Eigen::Vector3f lambdaContact, Eigen::Matrix3f A, Eigen::VectorXf w, const float mu)
    {
        float err = 0.0f;
        // float threshold = 1e-5f;

        Eigen::Vector3f upper = { FLT_MAX, mu * lambdaContact[0], mu * lambdaContact[0] };
        Eigen::Vector3f lower = { 0.0f, -mu * lambdaContact[0], -mu * lambdaContact[0] };

        for (int i = 0; i < lambdaContact.size(); ++i)
        {
            float wplus = std::max(w[i], 0.0f);
            float wminus = -std::min(w[i], 0.0f);
            err += std::max(std::fabs(std::min((lambdaContact[i] - lower[i]), wplus)), std::fabs(std::min((upper[i] - lambdaContact[i]), wminus)));
            // err += std::max(std::fabs(std::min(A(i, i) * (lambdaContact[i] - lower[i]), wplus)), std::fabs(std::min(A(i, i) * (upper[i] - lambdaContact[i]), wminus)));
        }
        return err;
    }

    static inline void multAndAdd(const Eigen::MatrixXf& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float a, Eigen::Vector3f& b)
    {
        b += a * G.col(0) * x(0);
        b += a * G.col(1) * x(1);
        b += a * G.col(2) * x(2);
        b += a * G.col(3) * y(0);
        b += a * G.col(4) * y(1);
        b += a * G.col(5) * y(2);
    }

    // Computes the right-hand side vector of the Schur complement system: 
    static inline void buildRHS(Contact* c, float h, Eigen::Vector3f& b)
    {
        const float gamma = h * c->k / (h * c->k + c->b);       // error reduction parameter
        b = gamma * c->phi / h;
        // std::cout << "Phi" << c->phi << std::endl;

        // e-coefficients
        float e = 0.2f;

        if (!c->body0->fixed)
        {
            multAndAdd(c->J0, c->body0->xdot, c->body0->omega, 1.0f + e, b);
            multAndAdd(c->J0Minv, c->body0->f, c->body0->tau, h, b);
        }
        if (!c->body1->fixed)
        {
            multAndAdd(c->J1, c->body1->xdot, c->body1->omega, 1.0f + e, b);
            multAndAdd(c->J1Minv, c->body1->f, c->body1->tau, h, b);
        }
    }

    static inline void accumulateCoupledContacts(Contact* c, const JBlock& JMinv, RigidBody* body, Eigen::VectorXf& x)
    {
        if (body->fixed)
            return;

        for (Contact* contact : body->contacts)
        {
            if (contact != c)
            {
                if (body == contact->body0)
                    x += JMinv * (contact->J0.transpose() * contact->lambda);
                else
                    x += JMinv * (contact->J1.transpose() * contact->lambda);
            }
        }
    }


    static inline float infiniteNorm(const std::vector<Eigen::Vector3f>& v) {
        float max = -1;
        for (size_t i = 0; i < v.size(); i++) {
            for (int j = 0; j < v[i].size(); j++) {
                if (std::fabs(v[i][j]) > max) {
                    max = std::fabs(v[i][j]);
                }
            }
        }
        return max;
    }

    // z_k = x_k - R_kk * ( wbar + b_k )
    static inline void computeZk(Eigen::Vector3f& x_k, Eigen::VectorXf& w0, Eigen::VectorXf& w1, const Eigen::MatrixXf& R_k,
        const Eigen::Vector3f& b, Eigen::Vector3f& z_k, JBlock& J0, JBlock& J1) {

        z_k = Eigen::Vector3f::Zero();

        z_k = x_k - R_k * (J0 * w0 + J1 * w1 + b);

    }

    static inline void initializeR(const std::vector<Eigen::Matrix3f>& A, std::vector<Eigen::MatrixXf>& R, std::vector<Eigen::MatrixXf>& nu)
    {
        // Static frame counter to track initialization strategy
        static int frameCount = 0;
        frameCount++;

        R.resize(A.size());
        nu.resize(A.size());

        // Adaptive initialization strategy
        for (size_t i = 0; i < A.size(); i++) {
            // Compute trace-based scaling factor
            float traceFactor = std::max(0.1f, std::min(1.0f, A[i].trace() / 9.0f));
            
            // More aggressive adaptation in early frames
            if (frameCount < 5) {
                // Quickly converging initial guess
                R[i] = Eigen::MatrixXf::Identity(3, 3) * (0.5f * traceFactor);
                nu[i] = Eigen::MatrixXf::Identity(3, 3) * (0.8f * traceFactor);
            } 
            else if (frameCount < 10) {
                // Gradual refinement
                R[i] = Eigen::MatrixXf::Identity(3, 3) * (0.3f * traceFactor);
                nu[i] = Eigen::MatrixXf::Identity(3, 3) * (0.9f * traceFactor);
            }
            else {
                // Standard initialization
                R[i] = Eigen::MatrixXf::Identity(3, 3) * 0.2f;
                nu[i] = Eigen::MatrixXf::Identity(3, 3) * 0.9f;
            }
        }
    }

    // lambda_n = prox_{R^+} (lambda_n - r (A * lambda_n + b))
    static inline void proxN(Eigen::Vector3f& z_k, Eigen::Vector3f& x_k) {
        x_k[0] = std::max(0.0f, z_k[0]);
    }

    // lambda_f = prox_C (lambda_f - r (A * lambda_f + b))
    static inline void proxF(Eigen::Vector3f& z_k, Eigen::Vector3f& x_k, Eigen::Vector4f& coeffs) {
        // 0: n
        // 1: s
        // 2: t
        // Set all directions to the same mu for now

        const float z_s = z_k[1];
        const float z_t = z_k[2];

        // Set contraints to zero
        x_k[1] = 0; x_k[2] = 0;

        if (x_k[0] <= 0) return;

        // Compute a,b,c constants
        const float a = coeffs[0] * x_k[0];
        const float b = coeffs[1] * x_k[0];

        if (a <= 0 || b <= 0) return;

        const float tol = 10e-10f;
        const size_t max_k = 100;

        // Scale problem
        const float scale = std::max(1.0f, std::max(a, std::max(b, std::max(std::fabs(z_s), std::fabs(z_t)))));

        const float sx = z_s / scale;
        const float sy = z_t / scale;

        const float sa = a / scale;
        const float sb = b / scale;

        const float aa = sa * sa;
        const float bb = sb * sb;

        const float xx = sx * sx;
        const float yy = sy * sy;

        const float f0 = (xx / aa) + (yy / bb) - 1;

        if (f0 < tol) {
            x_k[1] = z_s;
            x_k[2] = z_t;
            return;
        }

        const float aaxx = aa * xx;
        const float bbyy = bb * yy;

        float t0 = 0.0f;
        float t1 = std::max(sa, sb) * std::sqrt(xx + yy);
        float g0 = (aaxx) / ((aa + t0) * (aa + t0)) + (bbyy) / ((bb + t0) * (bb + t0)) - 1.0f;
        float g1 = (aaxx) / ((aa + t1) * (aa + t1)) + (bbyy) / ((bb + t1) * (bb + t1)) - 1.0f;

        // Scalar expansion coefficient of the interval
        const float expansion = 1.5f;
        while (g1 > 0) {
            t1 *= expansion;
            g1 = (aaxx) / ((aa + t1) * (aa + t1)) + (bbyy) / ((bb + t1) * (bb + t1)) - 1.0f;
        }

        // Perform binary search
        float t_k = (t0 + t1) * 0.5f;
        for (size_t k = 0; k < max_k; k++) {
            if (std::fabs(t1 - t0) < tol) break;

            const float aat = aa + t_k;
            const float bbt = bb + t_k;
            const float g_k = aaxx / (aat * aat) + bbyy / (bbt * bbt) - 1.0f;
            if (std::fabs(g_k) < tol) break;
            if (g_k > 0) {
                t0 = t_k; g0 = g_k;
            }
            else {
                t1 = t_k; g1 = g_k;
            }
            t_k = (t0 + t1) * 0.5;
        }

        // Convert root to unscaled problem
        t_k *= scale * scale;

        // Compute closest point
        x_k[1] = (a * a * z_s) / (a * a + t_k);
        x_k[2] = (b * b * z_t) / (b * b + t_k);
    }

    static inline void updateW(Eigen::VectorXf& w, JBlockTranspose& MinvJT, const Eigen::Vector3f& lambda)
    {
         w += MinvJT * lambda;
    }
}

SolverProximal::SolverProximal(RigidBodySystem* _rigidBodySystem) 
    : Solver(_rigidBodySystem)
{
    // Default configuration
}

SolverProximal::~SolverProximal() 
{
    // Ensure data logger is properly closed
}

void SolverProximal::enableDataExport(bool enable) 
{
    m_exportEnabled = enable;
}

void SolverProximal::setExportPath(const std::string& basePath) 
{
    m_exportBasePath = basePath;
    
    // Create directory if it doesn't exist
    std::filesystem::create_directories(m_exportBasePath);
}

void SolverProximal::setLogEnabled(LogType type, bool enabled) 
{
    m_logEnabled[type] = enabled;
}

void SolverProximal::initializeDataLogger(const std::string& exampleName)
{
    m_dataLogger = std::make_unique<slrbs::SimDataLogger>();
    m_dataLogger->initialize(m_exportBasePath);
    m_dataLogger->startLogging(exampleName + "_prox");
}

void SolverProximal::exportMatrices(const std::vector<Eigen::Matrix3f>& A, const std::vector<Contact*>& contacts) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::MATRICES] || !m_dataLogger)
        return;
        
    for (size_t i = 0; i < A.size(); i++) {
        m_dataLogger->logMatrix3f("A_" + std::to_string(i), A[i]);
        
        Contact* c = contacts[i];
        if (c) {
            m_dataLogger->logVector<Eigen::Matrix<float, 3, 6>>("J0_" + std::to_string(i), c->J0);
            m_dataLogger->logVector<Eigen::Matrix<float, 3, 6>>("J1_" + std::to_string(i), c->J1);
            m_dataLogger->logVector<Eigen::Matrix<float, 3, 6>>("J0Minv_" + std::to_string(i), c->J0Minv);
            m_dataLogger->logVector<Eigen::Matrix<float, 3, 6>>("J1Minv_" + std::to_string(i), c->J1Minv);
            m_dataLogger->logScalar("Phi_" + std::to_string(i), c->phi);
            m_dataLogger->logScalar("Mu_" + std::to_string(i), c->mu);
        }
    }
}

void SolverProximal::exportActiveConstraints(const std::vector<Eigen::Vector3f>& lambdaCandidate, float mu) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::ACTIVE_CONSTRAINTS] || !m_dataLogger)
        return;
        
    int activeCount = countActiveConstraints(lambdaCandidate, mu);
    int totalConstraints = lambdaCandidate.size() * 3;
    
    m_dataLogger->logInt("ActiveConstraints", activeCount);
    m_dataLogger->logInt("InactiveConstraints", totalConstraints - activeCount);
    m_dataLogger->logInt("TotalConstraints", totalConstraints);
}

void SolverProximal::exportResidual(float residualNorm) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::RESIDUAL] || !m_dataLogger)
        return;
        
    m_dataLogger->logScalar("ResidualNorm", residualNorm);
}

void SolverProximal::exportLCPError(float lcpError) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::LCP_ERROR] || !m_dataLogger)
        return;
        
    m_dataLogger->logScalar("LCPError", lcpError);
}

void SolverProximal::exportRValues(const std::vector<Eigen::MatrixXf>& R) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::MATRIX_R] || !m_dataLogger)
        return;
        
    float sumR = 0.0f;
    for (size_t i = 0; i < R.size(); i++) {
        for (int j = 0; j < R[i].rows(); j++) {
            sumR += R[i](j, j);
        }
        
        m_dataLogger->logMatrix("R_" + std::to_string(i), R[i]);
    }
    
    m_dataLogger->logScalar("SumR", sumR);
}

void SolverProximal::exportPerformanceData(double solveTimeMs, int iterations) 
{
    if (!m_exportEnabled || !m_logEnabled[LogType::PERFORMANCE] || !m_dataLogger)
        return;
        
    m_dataLogger->logScalar("SolveTimeMs", static_cast<float>(solveTimeMs));
    m_dataLogger->logInt("Iterations", iterations);
}

void SolverProximal::solve(float h)
{
    // Setup timing measurement
    auto startTime = std::chrono::high_resolution_clock::now();
    int iterations = 0;
    
    // Initialize data export if enabled
    if (m_exportEnabled && !m_dataLogger) {
        initializeDataLogger("simulation");
    }
    
    if (m_exportEnabled && m_dataLogger) {
        m_dataLogger->beginFrame();
    }

    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<RigidBody*>& bodies = m_rigidBodySystem->getBodies();
    const int numBodies = bodies.size();
    const int numContacts = contacts.size();
    int frameId = m_rigidBodySystem->m_frameCounter;
    
    // Export basic frame data
    if (m_exportEnabled && m_dataLogger) {
        m_dataLogger->logInt("FrameId", frameId);
        m_dataLogger->logInt("NumBodies", numBodies);
        m_dataLogger->logInt("NumContacts", numContacts);
        m_dataLogger->logScalar("TimeStep", h);
    }
    
    // Give an identifier to each body to be able to index the w matrix
    for (int i = 0; i < numBodies; i++) {
        bodies[i]->index = i;
    }

    // Build diagonal matrices of contacts
    std::vector<Eigen::Matrix3f> A;
    std::vector<Eigen::Vector3f> lambdasContacts;
    std::vector<Eigen::Vector4f> coeffs;
    std::vector<Eigen::Vector3f> b;

    if (numContacts > 0)
    {
        // Build diagonal matrices
        A.resize(numContacts);
        lambdasContacts.resize(numContacts);
        coeffs.resize(numContacts);
        b.resize(numContacts);

        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            c->tag = i;

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            //
            A[i] = Eigen::Matrix3f::Zero();
            for (int j = 0; j < 3; ++j)
            {
                A[i](j, j) += 1.0f / (h * h * c->k + h * c->b); // constraint force mixing
            }

            if (!c->body0->fixed)
            {
                A[i] += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {                
                A[i] += c->J1Minv * c->J1.transpose();
            }

            lambdasContacts[i] = c->lambda;

            b[i] = Eigen::Vector3f::Zero();
            buildRHS(c, h, b[i]);
            c->lambda.setZero();

            // Isotropic friction
            coeffs[i][0] = c->mu;
            coeffs[i][1] = c->mu;
            coeffs[i][2] = c->mu;
            coeffs[i][3] = c->mu;
        }
        
        // Export matrices if enabled
        if (m_exportEnabled && m_logEnabled[LogType::MATRICES] && m_dataLogger) {
            exportMatrices(A, contacts);
        }
    }

    // PROX solver
    if (numContacts > 0) {
        float residual_norm = 0.0f;
        float last_residual_norm = FLT_MAX;
        Eigen::Vector3f z_k = Eigen::Vector3f::Zero();

        float relative_threshold = 1e-5;
        float absolute_threshold = 1e-5;

        std::vector<Eigen::Vector3f> lambdaCandidate;
        std::vector<Eigen::MatrixXf> R;
        std::vector<Eigen::MatrixXf> nu;
        std::vector<Eigen::Vector3f> residual;
        std::vector<Eigen::VectorXf> w;

        lambdaCandidate.resize(A.size());
        residual.resize(A.size());
        R.resize(A.size());
        nu.resize(A.size());
        w.resize(numBodies);

        initializeR(A, R, nu);

        for (iterations = 0; iterations < m_maxIter; ++iterations) {
            for (int i = 0; i < numBodies; i++) {
                w[i] = Eigen::VectorXf::Zero(6);
            }

            // Initialize w = Minv * JT * lambda_k
            for (int i = 0; i < numContacts; ++i) {
                lambdaCandidate[i] = Eigen::Vector3f::Zero();
                updateW(w[contacts[i]->body0->index], contacts[i]->MinvJ0T, lambdasContacts[i]);
                updateW(w[contacts[i]->body1->index], contacts[i]->MinvJ1T, lambdasContacts[i]);
            }

            // Solve for each contact
            for (int i = 0; i < numContacts; ++i) {
                computeZk(lambdasContacts[i], w[contacts[i]->body0->index], w[contacts[i]->body1->index], R[i], b[i], z_k,
                    contacts[i]->J0, contacts[i]->J1);

                proxN(z_k, lambdaCandidate[i]);
                proxF(z_k, lambdaCandidate[i], coeffs[i]);

                residual[i] = lambdaCandidate[i] - lambdasContacts[i];

                // Update w = Minv * JT * (lambda(k+1) - lambda(k))
                updateW(w[contacts[i]->body0->index], contacts[i]->MinvJ0T, residual[i]);
                updateW(w[contacts[i]->body1->index], contacts[i]->MinvJ1T, residual[i]);
            }

            // Export active constraints data if enabled
            if (m_exportEnabled && m_logEnabled[LogType::ACTIVE_CONSTRAINTS] && m_dataLogger) {
                exportActiveConstraints(lambdaCandidate, 0.4f);
            }

            residual_norm = infiniteNorm(residual);
            
            // Export residual data if enabled
            if (m_exportEnabled && m_logEnabled[LogType::RESIDUAL] && m_dataLogger) {
                exportResidual(residual_norm);
            }

            // Check for convergence for early termination
            if (residual_norm < absolute_threshold)
                break;
            if (std::fabs(residual_norm - last_residual_norm) < relative_threshold * last_residual_norm)
                break;

            // Only update lambda if solver is converging
            if (residual_norm > last_residual_norm) {
                for (int j = 0; j < R.size(); j++) {
                    for (int k = 0; k < 3; ++k)
                    {
                        R[j](k, k) = 0.2f;
                    }
                }
            }
            else {
                last_residual_norm = residual_norm;
                for (int j = 0; j < numContacts; j++) {
                    lambdasContacts[j] = lambdaCandidate[j];
                }
            }
            
            // Export R values if enabled
            if (m_exportEnabled && m_logEnabled[LogType::MATRIX_R] && m_dataLogger && iterations % 10 == 0) {
                exportRValues(R);
            }
        }

        // Calculate average LCP error at the end
        float total_lcp_error = 0.0f;
        for (int i = 0; i < numContacts; i++) {
            Contact* c = contacts[i];
            if (!c) continue;
            
            // Compute w = A * lambda + b for LCP error
            Eigen::Vector3f w_lcp = A[i] * c->lambda + b[i];
            total_lcp_error += computeLCPError(c->lambda, A[i], w_lcp, c->mu);
        }
        float avg_lcp_error = numContacts > 0 ? total_lcp_error / numContacts : 0.0f;
        
        // Export LCP error if enabled
        if (m_exportEnabled && m_logEnabled[LogType::LCP_ERROR] && m_dataLogger) {
            exportLCPError(avg_lcp_error);
        }

        // Set the new lambdas to the old contact lambdas
        for (int i = 0; i < lambdasContacts.size(); i++) {
            contacts[i]->lambda = lambdasContacts[i];
        }
        
        // Report convergence information for debugging
        std::cout << "PROX solver: " << iterations << " iterations, error: " << avg_lcp_error << std::endl;
    }
    
    // Calculate solve time and export performance data
    auto endTime = std::chrono::high_resolution_clock::now();
    double solveTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    
    if (m_exportEnabled && m_logEnabled[LogType::PERFORMANCE] && m_dataLogger) {
        exportPerformanceData(solveTimeMs, iterations);
    }
    
    // End frame in data file if enabled
    if (m_exportEnabled && m_dataLogger) {
        m_dataLogger->endFrame();
    }
}
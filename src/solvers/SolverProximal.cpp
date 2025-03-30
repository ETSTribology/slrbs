#include "solvers/SolverProximal.h"
#include <Eigen/Dense>
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
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

SolverProximal::SolverProximal(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverProximal::setCSV(std::ofstream& fileName, char* filePath)
{
    m_path = filePath;
    m_file = &fileName;
}

void SolverProximal::writeCSV(std::ofstream& fileName, char* filePath)
{
    fileName.open(filePath, std::ios::out | std::ios::app);
}

void SolverProximal::solve(float h)
{
    // Files for evaluating the matrices:
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    
    /*
    std::string example_name = "marble_box";
    std::string solver_name = "prox";
    std::string R_strategy = "local_A";

    std::ofstream prox_lcp_error;
    std::string prox_lcp_error_string = solver_name + "_lcp_error_" + R_strategy + example_name + ".csv";
    char* prox_lcp_error_str = &prox_lcp_error_string[0];
    setCSV(prox_lcp_error, prox_lcp_error_str);
    writeCSV(*m_file, m_path);

    std::ofstream prox_residual;
    std::string prox_residual_string = solver_name + "_residual_" + example_name + ".csv";
    char* prox_residual_str = &prox_residual_string[0];
    prox_residual.open(prox_residual_str, std::ios::out | std::ios::app);

    // Sequence of R
    std::ofstream proxR;
    std::string proxR_string = solver_name + "_R_" + example_name + ".csv";
    char* proxR_str = &proxR_string[0];
    proxR.open(proxR_str, std::ios::out | std::ios::app);
    // Sequence of active set
    std::ofstream proxActiveSet;
    std::string proxActiveSet_string = solver_name + "_active_set_" + example_name + ".csv";
    char* proxActiveSet_str = &proxActiveSet_string[0];
    proxActiveSet.open(proxActiveSet_str, std::ios::out | std::ios::app);
    // Sequence of inactive set
    std::ofstream proxInActiveSet;
    std::string proxInActiveSet_string = solver_name + "_inactive_set_" + example_name + ".csv";
    char* proxInActiveSet_str = &proxInActiveSet_string[0];
    proxInActiveSet.open(proxInActiveSet_str, std::ios::out | std::ios::app);
    */

    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<RigidBody*>& bodies = m_rigidBodySystem->getBodies();
    const int numBodies = bodies.size();
    const int numContacts = contacts.size();
    int frameId = m_rigidBodySystem->m_frameCounter;
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

            // std::cout << "h:" << h << std::endl;

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

        std::cout << "The number of frame: " << frameId << std::endl;

        for (int iter = 0; iter < m_maxIter; ++iter) {

            if (iter == m_maxIter - 1 && frameId == 25)
            {
                std::cout << "It cannot converge!" << std::endl;
            }

            for (int i = 0; i < numBodies; i++) {
                w[i] = Eigen::VectorXf::Zero(6);
            }

            if (iter == 0)
            {
                //prox_lcp_error << 0.5f << ",";
            }

            if (frameId == 4) // && iter == 50)
            {
                // // Number of contacts and bodies
                // std::ofstream dimension;
                // dimension.open("dimension.csv", std::ios::out | std::ios::app);
                // dimension << "Contact Number: " << numContacts << "\n";
                // dimension << "Body Number: " << numBodies << "\n";

                // Contact data
                for (int i = 0; i < numContacts; ++i)
                {
                    Contact* c = contacts[i];

                    /*std::ofstream contactIndex;
                    contactIndex.open("contactIndex.csv", std::ios::out | std::ios::app);
                    contactIndex << c->index << "\n";*/

                    // std::ofstream matrixPhi;
                    // matrixPhi.open("Phi.csv", std::ios::out | std::ios::app);
                    // matrixPhi << c->phi << "\n";

                    // std::ofstream matrixJ0;
                    // matrixJ0.open("J0.csv", std::ios::out | std::ios::app);
                    // matrixJ0 << c->J0 << "\n";

                    // std::ofstream matrixJ0Minv;
                    // matrixJ0Minv.open("J0Minv.csv", std::ios::out | std::ios::app);
                    // matrixJ0Minv << c->J0Minv << "\n";

                    // std::ofstream matrixMinvJ0T;
                    // matrixMinvJ0T.open("MinvJ0T.csv", std::ios::out | std::ios::app);
                    // matrixMinvJ0T << c->MinvJ0T << "\n";                    

                    // std::ofstream matrixJ1;
                    // matrixJ1.open("J1.csv", std::ios::out | std::ios::app);
                    // matrixJ1 << c->J1 << "\n";

                    // std::ofstream matrixJ1Minv;
                    // matrixJ1Minv.open("J1Minv.csv", std::ios::out | std::ios::app);
                    // matrixJ1Minv << c->J1Minv << "\n";

                    // std::ofstream matrixMinvJ1T;
                    // matrixMinvJ1T.open("MinvJ1T.csv", std::ios::out | std::ios::app);
                    // matrixMinvJ1T << c->MinvJ1T << "\n";                 

                    // std::ofstream matrixbody;
                    // matrixbody.open("body.csv", std::ios::out | std::ios::app);
                    // matrixbody << c->body0->index << "," << c->body1->index << "\n";

                    /*std::ofstream matrixxdot1;
                    matrixxdot1.open("xdot1.csv", std::ios::out | std::ios::app);
                    matrixxdot1 << c->body1->xdot << "\n";

                    std::ofstream matrixomega1;
                    matrixomega1.open("omega1.csv", std::ios::out | std::ios::app);
                    matrixomega1 << c->body1->omega << "\n";

                    std::ofstream matrixf1;
                    matrixf1.open("f1.csv", std::ios::out | std::ios::app);
                    matrixf1 << c->body1->f << "\n";

                    std::ofstream matrixtau1;
                    matrixtau1.open("tau1.csv", std::ios::out | std::ios::app);
                    matrixtau1 << c->body1->tau << "\n";*/

                    // body 1 is always fixed so body 0 is not fixed.
                    /*if (c->body1->fixed)
                        std::cout << 1 << std::endl;*/
                }

                // Body data
                for (int i = 0; i < numBodies; ++i)
                {
                    RigidBody* body = bodies[i];

                    std::ofstream bodyIndex;
                    bodyIndex.open("bodyIndex.csv", std::ios::out | std::ios::app);
                    bodyIndex << body->index << "\n";

                    std::ofstream fixed;
                    fixed.open("fixed.csv", std::ios::out | std::ios::app);
                    fixed << body->fixed << "\n";

                    std::ofstream matrixxdot;
                    matrixxdot.open("xdot.csv", std::ios::out | std::ios::app);
                    matrixxdot << body->xdot << "\n";

                    std::ofstream matrixomega;
                    matrixomega.open("omega.csv", std::ios::out | std::ios::app);
                    matrixomega << body->omega << "\n";

                    std::ofstream matrixf;
                    matrixf.open("f.csv", std::ios::out | std::ios::app);
                    matrixf << body->f << "\n";

                    std::ofstream matrixtau;
                    matrixtau.open("tau.csv", std::ios::out | std::ios::app);
                    matrixtau << body->tau << "\n";

                }
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

            // Export the number of the constraints in active or inactive set;
            int num_active = countActiveConstraints(lambdaCandidate, 0.4f);
            int num_inactive = 3 * numContacts - num_active;

            residual_norm = infiniteNorm(residual);
            //prox_residual << residual_norm << ",";

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

            // Export R 
            /*float sumR = 0.0f;
            for (int i = 0; i < R.size(); ++i)
            {
                int numCol = R[i].cols();
                int numRow = R[i].rows();
                // Only count the diagonal elements
                for (int j = 0; j < numRow; ++j)
                {
                    sumR += R[i](j, j);
                }                                
            }   
            //proxR << sumR << ",";

            //proxActiveSet << num_active << ",";
            //proxInActiveSet << num_inactive << ",";

            //(*m_file) << lcpError << ",";*/
        }

        // Calculate average LCP error at the end
        float total_lcp_error = 0.0f;
        for (int i = 0; i < numContacts; i++) {
            Contact* c = contacts[i];
            if (!c) continue;
            
            // Compute w = A * lambda + b for LCP error
            Eigen::Vector3f w = A[i] * c->lambda + b[i];
            total_lcp_error += computeLCPError(c->lambda, A[i], w, c->mu);
        }
        float avg_lcp_error = numContacts > 0 ? total_lcp_error / numContacts : 0.0f;
        std::cout << "PROX Average LCP Error: " << avg_lcp_error << std::endl;

        // Set the new lambdas to the old contact lambdas
        for (int i = 0; i < lambdasContacts.size(); i++) {
            contacts[i]->lambda = lambdasContacts[i];
        }
    }

    /*// End of the frame
    proxR << "\n";
    proxR.close();

    proxActiveSet << "\n";
    proxActiveSet.close();

    proxInActiveSet << "\n";
    proxInActiveSet.close();

    prox_residual << "\n";
    prox_residual.close();

    (*m_file) << "\n";
    (*m_file).close();*/
}
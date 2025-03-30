// Add body0 contribution to diagonal block
if (!contact->body0->fixed) {
    A.block(idx, idx, dim, dim) += contact->J0Minv * contact->J0.transpose();
    
    // Couple with body0's other contacts
    for (auto otherContact : contact->body0->contacts) {
        if (otherContact != contact) {
            const unsigned int cdim = otherContact->dim;
            const unsigned int cidx = otherContact->idx;
            
            if (otherContact->body0 == contact->body0) {
                A.block(idx, cidx, dim, cdim) += contact->J0Minv * otherContact->J0.transpose();
            } else {
                A.block(idx, cidx, dim, cdim) += contact->J0Minv * otherContact->J1.transpose();
            }
        }
    }
    
    // Couple with body0's joints
    for (auto joint : contact->body0->joints) {
        const unsigned int jdim = joint->dim;
        const unsigned int jidx = joint->idx;
        
        if (joint->body0 == contact->body0) {
            A.block(idx, jidx, dim, jdim) += contact->J0Minv * joint->J0.transpose();
        } else {
            A.block(idx, jidx, dim, jdim) += contact->J0Minv * joint->J1.transpose();
        }
    }
}

// Add body1 contribution to diagonal block
if (!contact->body1->fixed) {
    A.block(idx, idx, dim, dim) += contact->J1Minv * contact->J1.transpose();
    
    // Couple with body1's other contacts
    for (auto otherContact : contact->body1->contacts) {
        if (otherContact != contact) {
            const unsigned int cdim = otherContact->dim;
            const unsigned int cidx = otherContact->idx;
            
            if (otherContact->body0 == contact->body1) {
                A.block(idx, cidx, dim, cdim) += contact->J1Minv * otherContact->J0.transpose();
            } else {
                A.block(idx, cidx, dim, cdim) += contact->J1Minv * otherContact->J1.transpose();
            }
        }
    }
    
    // Couple with body1's joints
    for (auto joint : contact->body1->joints) {
        const unsigned int jdim = joint->dim;
        const unsigned int jidx = joint->idx;
        
        if (joint->body0 == contact->body1) {
            A.block(idx, jidx, dim, jdim) += contact->J1Minv * joint->J0.transpose();
        } else {
            A.block(idx, jidx, dim, jdim) += contact->J1Minv * joint->J1.transpose();
        }
    }
}
}
}

void buildSchurComplementRHS(const std::vector<Joint*>& joints,
                    const std::vector<Contact*>& contacts,
                    float h,
                    Eigen::VectorXf& b,
                    float gamma)
{
// Constants for Baumgarte stabilization
static const float stabilization = 250.0f;
static const float alpha = stabilization * 2.0f;
static const float beta = stabilization * stabilization * 2.0f;

const float hinv = 1.0f / h;
const float baumgarte = (h * beta / (h * beta + alpha));

// Build RHS for joints
for (auto joint : joints) {
const unsigned int dim = joint->dim;
const unsigned int idx = joint->idx;

// Baumgarte stabilization term
b.segment(idx, dim) = -hinv * joint->phi * baumgarte;

// Add force and velocity terms for body0
if (!joint->body0->fixed) {
    for (int i = 0; i < 3; ++i) {
        b.segment(idx, dim) -= joint->J0Minv.col(i) * joint->body0->f(i) * h;
        b.segment(idx, dim) -= joint->J0Minv.col(i+3) * joint->body0->tau(i) * h;
        b.segment(idx, dim) -= joint->J0.col(i) * joint->body0->xdot(i);
        b.segment(idx, dim) -= joint->J0.col(i+3) * joint->body0->omega(i);
    }
}

// Add force and velocity terms for body1
if (!joint->body1->fixed) {
    for (int i = 0; i < 3; ++i) {
        b.segment(idx, dim) -= joint->J1Minv.col(i) * joint->body1->f(i) * h;
        b.segment(idx, dim) -= joint->J1Minv.col(i+3) * joint->body1->tau(i) * h;
        b.segment(idx, dim) -= joint->J1.col(i) * joint->body1->xdot(i);
        b.segment(idx, dim) -= joint->J1.col(i+3) * joint->body1->omega(i);
    }
}
}

// Build RHS for contacts
for (auto contact : contacts) {
const unsigned int dim = contact->dim;
const unsigned int idx = contact->idx;

// Baumgarte stabilization term
b.segment(idx, dim) = -hinv * contact->phi * baumgarte;

// Add force and velocity terms for body0
if (!contact->body0->fixed) {
    for (int i = 0; i < 3; ++i) {
        b.segment(idx, dim) -= contact->J0Minv.col(i) * contact->body0->f(i) * h;
        b.segment(idx, dim) -= contact->J0Minv.col(i+3) * contact->body0->tau(i) * h;
        b.segment(idx, dim) -= contact->J0.col(i) * contact->body0->xdot(i);
        b.segment(idx, dim) -= contact->J0.col(i+3) * contact->body0->omega(i);
    }
}

// Add force and velocity terms for body1
if (!contact->body1->fixed) {
    for (int i = 0; i < 3; ++i) {
        b.segment(idx, dim) -= contact->J1Minv.col(i) * contact->body1->f(i) * h;
        b.segment(idx, dim) -= contact->J1Minv.col(i+3) * contact->body1->tau(i) * h;
        b.segment(idx, dim) -= contact->J1.col(i) * contact->body1->xdot(i);
        b.segment(idx, dim) -= contact->J1.col(i+3) * contact->body1->omega(i);
    }
}
}
}

void updateJointsAndContacts(const Eigen::VectorXf& x,
                    std::vector<Joint*>& joints,
                    std::vector<Contact*>& contacts)
{
// Update joint impulses
for (auto joint : joints) {
joint->lambda = x.segment(joint->idx, joint->dim);
}

// Update contact impulses
for (auto contact : contacts) {
contact->lambda = x.segment(contact->idx, contact->dim);
}
}

} // namespace SolverUtils
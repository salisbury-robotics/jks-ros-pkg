#include "PointShellIsosurface.h"
#include "WilhelmsenProjection.h"
#include <cml/cml.h>
#include <iostream>

using namespace cml;

#ifdef HAVE_CGAL
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

// choose exact integral type (for CGAL's QP solver)
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

void printConstraints();
#endif

// --------------------------------------------------------------------------

class IsosurfaceOracle : public CollisionOracle
{
    PointShellIsosurface   *m_isosurface;
    matrix33d               m_rotation;
    vector3d                m_translation;

public:
    IsosurfaceOracle(PointShellIsosurface *psi, const vector3d &t, const matrix33d &r)
        : m_isosurface(psi), m_rotation(r), m_translation(t)
    { }

    virtual bool inside(const cml::vector3d &p) const
    {
        vector3d q = m_rotation * p + m_translation;
        return m_isosurface->surface(q) < 0.0;
    }
};


// --------------------------------------------------------------------------

PointShellIsosurface::PointShellIsosurface(Sampler * sampler, double isoValue)
    : HapticIsosurface(sampler, isoValue)
{
//    vector3i d = volume->dimensions;
//    int hi = std::max(d[0], std::max(d[1], d[2]));
//    m_featureSize = 1.0 / double(hi);

    // TODO
    m_featureSize = 1/128.;
}

// --------------------------------------------------------------------------

void PointShellIsosurface::computeMassMatrix()
{
    // assume the object has unit mass
    m_mass.set(1.0, 1.0, 1.0);

    // assume mass is equally distributed over all points in the point shell
    double m = 1.0 / m_pointShell.size();
    m_moment.zero();
    for (std::vector<vector3d>::iterator it = m_pointShell.begin();
         it != m_pointShell.end(); ++it)
    {
        for (int j = 0; j < 3; ++j) {
            vector3d p = *it;
            p[j] = 0;
            m_moment[j] += 0.5 * m * p.length_squared();
        }
    }

	// TODO: This works much much better than summation... why?!
    const double r = 0.2;
    const double I = 0.4 * r * r;
	m_moment[0] = I;
	m_moment[1] = I;
	m_moment[2] = I;
}

// --------------------------------------------------------------------------

void PointShellIsosurface::processCut(HapticDisplay *display, const vector3d &p, const vector3d &q)
{
    // displacement vector in world coordinates
    vector3d d = q - p;

    // compute the physical force (N) from the displacement
    double F = d.length() * display->physicalToWorldScale() * display->stiffness();

    // compute the physical tool radius
    double r = m_toolRadius * display->physicalToWorldScale();

    // compute physical displacement (m) based on force and tool radius
    double dp = 1e-7 * F / (r*r);
    double k = std::min(dp / display->physicalToWorldScale(), 0.2);

    // TODO: this is just a hack for a test
    //m_maskCutter->cutFilteredSphere(p + k*d, (0.95 + k) * m_toolRadius);
}

// --------------------------------------------------------------------------

void PointShellIsosurface::update(HapticDisplay *display)
{
    // bail out if we run out of states
    if (display->identifier() >= k_maxStates) return;

    // fetch the state for the correct haptic display
    State &s = m_states[display->identifier()];

    // let q be the new device position
    vector3d q  = display->toolPosition();
    matrix33d R = display->toolOrientation();

    // do a check to ensure the algorithm does not start inside the object
    if (!s.ready)
    {
        display->setProxyPosition(q);
        display->setProxyOrientation(R);

        IsosurfaceOracle oracle(this, q, R);
        IntervalCollisionDetector icd(m_pointShell, zero_3D(), zero_3D(), &oracle, m_featureSize);
        std::vector<CollisionContact> contacts;
        icd.detectCollisions(contacts);

        // if any point is in contact, exit
        if (!contacts.empty()) return;
        else s.ready = true;
    }

    // rotation matrices to go from local to proxy orientation and vice versa
    matrix33d toObject = display->proxyOrientation();
    matrix33d toProxy = transpose(toObject);

    // determine the desired linear acceleration (a) and angular acceleration
    // (alpha) to go from the proxy pose to the tool pose (in the proxy frame)
    vector3d a = toProxy * (display->toolPosition() - display->proxyPosition());

    matrix33d m = toProxy * display->toolOrientation();
    double angle;
    vector3d alpha;
    matrix_to_axis_angle(m, alpha, angle);
    alpha *= angle;

    // prune the active contact set using the current unconstrained accelerations?
    IsosurfaceOracle oracle(this, display->proxyPosition(), display->proxyOrientation());
    IntervalCollisionDetector checker(m_pointShell, a, alpha, &oracle, m_featureSize);

    checker.pruneContacts(m_contacts);

    // put all verified contacts into the contact set for calculating constraints
    m_mutex.lock();
    m_contactPoints.clear();
    m_contactNormals.clear();
    for (std::vector<CollisionContact>::iterator it = m_contacts.begin();
         it != m_contacts.end(); ++it)
    {
        // compute the position in the object frame
        // TODO: decide if it->position or it->contact is the right thing to use
        vector3d q = toObject * it->position + display->proxyPosition();
        m_contactPoints.push_back(it->position);
        m_contactNormals.push_back(toProxy * gradient(q));
    }
    m_mutex.unlock();

//    m_mutex.lock();
//    m_contactPoints.clear();
//    m_contactNormals.clear();
//    for (int i = 0; i < contactPoints.size(); ++i)
//    {
//        // compute the position in the object frame
//        vector3d q = toObject * contactPoints[i] + display->proxyPosition();
//        m_contactPoints.push_back(m_pointShell[m_contactIndices[i]]);
//        m_contactNormals.push_back(toProxy * gradient(q));
//    }
//    m_mutex.unlock();

    // if the current contact set is non-empty, test the desired acceleration
    // between the proxy and device against the constraints (in the proxy frame)
    vector3d ac = a, alphac = alpha;
    if (!m_contactNormals.empty())
        constrainedAcceleration(a, alpha, ac, alphac);

    // limit the constrained acceleration to a maximum displacement for a
    // single time step
    /* the interval collision detector is now robust to this limit
    double d = ac.length() + alphac.length();
    if (d > 0.01) {
        ac = 0.005 * ac / d;
        alphac = 0.005 * alphac / d;
    }
    */

    // run collision detection to compute the next contact set
    IntervalCollisionDetector icd(m_pointShell, ac, alphac, &oracle, m_featureSize);
    double t = icd.detectCollisions(m_contacts);

    // move the proxy according to the constrained acceleration and earliest
    // detected collision time
    display->setProxyPosition(display->proxyPosition() + toObject * (t * ac));
    angle = alphac.length();
    if (angle > 0.0) {
        alphac /= angle;
        matrix_rotation_axis_angle(m, alphac, t * angle);
        matrix33d po = toObject * m;
        matrix_orthogonalize_3x3(po);
        display->setProxyOrientation(po);
    }

    // check to see if any points have penetrated the surface, and attempt
    // to resolve the interpenentrations by pushing the proxy out
    resolvePenetrations(display);

    // mill away part of the mask if the first buttong on the device is down
    if (display->buttonState(0))
        processCut(display, display->proxyPosition(), display->toolPosition());
}

// --------------------------------------------------------------------------

bool PointShellIsosurface::resolvePenetrations(HapticDisplay *display)
{
    // rotation matrices to go from local to proxy orientation and vice versa
    matrix33d toObject = display->proxyOrientation();
    matrix33d toProxy = transpose(toObject);

    IsosurfaceOracle oracle(this, display->proxyPosition(),
                            display->proxyOrientation());

    // add interpenetrating points into a penetrating set
    std::vector<CollisionContact> within;
    for (std::vector<CollisionContact>::iterator it = m_contacts.begin();
         it != m_contacts.end(); ++it)
    {
        // compute the position in the object frame
        vector3d q = toObject * it->position + display->proxyPosition();
        if (surface(q) < 0.0) {
            CollisionContact cc;
            cc.index = it->index;
            cc.position = it->position;
            cc.normal = toProxy * gradient(q);
            within.push_back(cc);
        }
    }

    if (within.empty()) return true;

    // compute a nominal acceleration based on the penetrating set
    vector3d a = zero_3D(), alpha = zero_3D();
    int c = 0;
    for (std::vector<CollisionContact>::iterator it = within.begin();
         it != within.end(); ++it)
    {
        double magnitude = it->normal.length();
        if (magnitude > 1e-10) {
            vector3d n = it->normal / magnitude;
            a += n;
            alpha += cross(it->position, n);
            ++c;
        }
    }

    if (c == 0) return false;
    double k = 100.0; // proxy will move as far as 1/k
    for (int i = 0; i < 3; ++i) {
        a[i] /= c * m_mass[i] * k;
        alpha[i] /= c * m_moment[i] * k;
    }

    // travel along <a,alpha> to see if we can move the point shell out of the surface
    IntervalCollisionDetector resolver(m_pointShell, a, alpha, &oracle, m_featureSize);
    double t = resolver.resolvePenetrations(within);

    if (t == 0.0) return false;

    // move the proxy according to nominal "exit" vector
    display->setProxyPosition(display->proxyPosition() + toObject * (t * a));
    double angle = alpha.length();
    if (angle > 0.0) {
        alpha /= angle;
        matrix33d m;
        matrix_rotation_axis_angle(m, alpha, t * angle);
        matrix33d po = toObject * m;
        matrix_orthogonalize_3x3(po);
        display->setProxyOrientation(po);
    }

    return true;
}

// --------------------------------------------------------------------------

// this is the real version that uses Wilhelmsen's projection algorithm
void PointShellIsosurface::constrainedAcceleration(const vector3d &a, const vector3d &alpha,
                                                    vector3d &ac, vector3d &alphac)
{
    // use the CGAL implementation as an oracle, cuz we know it works
//    constrainedAcceleration6(a, alpha, ac, alphac);

    // set up typing for projection algorithm
    typedef long double number;
    typedef WilhelmsenProjection<number>::vector6 vector6;

    // set up a mass matrix for conversion to Euclidean space
    vector6 M;
    for (int i = 0; i < 3; ++i) {
        M[i]    = sqrt(m_mass[i]);
        M[i+3]  = sqrt(m_moment[i]);
    }

    // set up target point based on desired acceleration
    vector6 b;
    for (int i = 0; i < 3; ++i) {
        b[i]    = M[i] * a[i];
        b[i+3]  = M[i+3] * alpha[i];
    }

    // a global scale for the projection, used for conditioning
    number scale = WilhelmsenProjection<number>::norm(b);
    if (scale > 1e-10) {
        b /= scale;
    }
    else {
        ac.zero();
        alphac.zero();
        return;
    }

    // set up constraint matrix from contact set
    std::vector<vector6> C;
    for (int i = 0; i < m_contactNormals.size(); ++i)
    {
        const vector3d &n = m_contactNormals[i];
        vector3d k = cross(m_contactPoints[i], n);

        vector6 c;
        for (int j = 0; j < 3; ++j) {
            c[j] = -n[j] / M[j];
            c[j+3] = -k[j] / M[j+3];
        }
        C.push_back(c);
    }

    // condition the set of constraints
    WilhelmsenProjection<number>::conditionGenerators(C);

    // solve the projection
    vector6 p = WilhelmsenProjection<number>::projectCone(C, b);
    vector6 c = scale * (b - p);
    vector3d bc, betac;
    for (int i = 0; i < 3; ++i) {
        bc[i] = c[i] / M[i];
        betac[i] = c[i+3] / M[i+3];
    }
/*
    // check the answer
    double dl = length(ac - bc);
    double da = length(alphac - betac);
    if (dl > 1e-6 || da > 1e-6)
    {
        std::cout << "Projection discrepency: dl=" << dl << " da=" << da << " C=" << C.size() << std::endl;
        std::cout << "\ta =" << a  << "\talpha =" << alpha  << std::endl;
        std::cout << "\tac=" << ac << "\talphac=" << alphac << std::endl;
        std::cout << "\tbc=" << bc << "\tbetac =" << betac  << std::endl;
        printConstraints();
        WilhelmsenProjection<number>::printConstraints();
        std::cout << "Culled constraints: " << culled << std::endl;
    }
*/
    ac = bc;
    alphac = betac;
}

// --------------------------------------------------------------------------

#ifdef HAVE_CGAL
// this implementation uses the QP solver in CGAL... works, but is too slow
CGAL::Quadratic_program_solution<ET> solution;

void printConstraints()
{
    std::cout << "Basic constraints: ";
    for (CGAL::Quadratic_program_solution<ET>::Index_iterator
         it = solution.basic_constraint_indices_begin();
         it != solution.basic_constraint_indices_end(); ++it)
                std::cout << *it << "  ";
    std::cout << std::endl;
}

void PointShellIsosurface::constrainedAcceleration6(const vector3d &a, const vector3d &alpha,
                                                   vector3d &ac, vector3d &alphac)
{
    // this method uses the Quadratic Programming solver from CGAL to find the
    // nearest projected point on the cone of permitted accelerations

    // a QP with default constraints Ax >= b, and no bounds on solution variables
    CGAL::Quadratic_program<double> qp(CGAL::LARGER, false, 0, false, 0);

    // add the contact constraints to the QP
    for (int i = 0; i < m_contactNormals.size(); ++i)
    {
        const vector3d &n = m_contactNormals[i];
        vector3d k = cross(m_contactPoints[i], n);
        qp.set_a(0, i, n[0]);
        qp.set_a(1, i, n[1]);
        qp.set_a(2, i, n[2]);
        qp.set_a(3, i, k[0]);
        qp.set_a(4, i, k[1]);
        qp.set_a(5, i, k[2]);
        qp.set_b(i, 0);
    }

    // add the kinetic distance objective to the QP

    // D is mass matrix (currently set to identity)
//    const double m = 1.0, r = 0.1;
//    const double I = 1.0;//0.4 * m * r * r;
    qp.set_d(0, 0, m_mass[0]);
    qp.set_d(1, 1, m_mass[1]);
    qp.set_d(2, 2, m_mass[2]);
    qp.set_d(3, 3, m_moment[0]);
    qp.set_d(4, 4, m_moment[1]);
    qp.set_d(5, 5, m_moment[2]);

    // c is negative of mass matrix times unconstrained acceleration
    qp.set_c(0, -m_mass[0] * a[0]);
    qp.set_c(1, -m_mass[1] * a[1]);
    qp.set_c(2, -m_mass[2] * a[2]);
    qp.set_c(3, -m_moment[0] * alpha[0]);
    qp.set_c(4, -m_moment[1] * alpha[1]);
    qp.set_c(5, -m_moment[2] * alpha[2]);

    // solve the QP, using ET as the exact type
    try {
        solution = CGAL::solve_quadratic_program(qp, ET());

        ac[0] = CGAL::to_double(*(solution.variable_values_begin() + 0));
        ac[1] = CGAL::to_double(*(solution.variable_values_begin() + 1));
        ac[2] = CGAL::to_double(*(solution.variable_values_begin() + 2));
        alphac[0] = CGAL::to_double(*(solution.variable_values_begin() + 3));
        alphac[1] = CGAL::to_double(*(solution.variable_values_begin() + 4));
        alphac[2] = CGAL::to_double(*(solution.variable_values_begin() + 5));
    }
    catch (...) {
        ac = zero_3D();
        alphac = zero_3D();
        std::cout << "CGAL exception caught!" << std::endl;
    }
}

// working 3-dof version...
void PointShellIsosurface::constrainedAcceleration3(const vector3d &a, const vector3d &alpha,
                                                    vector3d &ac, vector3d &alphac)
{
    // this method uses the Quadratic Programming solver from CGAL to find the
    // nearest projected point on the cone of permitted accelerations

    // a QP with default constraints Ax >= b, and no bounds on solution variables
    CGAL::Quadratic_program<double> qp(CGAL::LARGER, false, 0, false, 0);

    // add the contact constraints to the QP
    for (int i = 0; i < m_contactNormals.size(); ++i)
    {
        const vector3d &n = m_contactNormals[i];
        qp.set_a(0, i, n[0]);
        qp.set_a(1, i, n[1]);
        qp.set_a(2, i, n[2]);
        qp.set_b(i, 0);
    }

    // add the kinetic distance objective to the QP

    // D is mass matrix (currently set to identity)
    qp.set_d(0, 0, 1);
    qp.set_d(1, 1, 1);
    qp.set_d(2, 2, 1);

    // c is negative of mass matrix times unconstrained acceleration
    qp.set_c(0, -a[0]);
    qp.set_c(1, -a[1]);
    qp.set_c(2, -a[2]);

    // solve the QP, using ET as the exact type
    try {
        CGAL::Quadratic_program_solution<ET> solution = CGAL::solve_quadratic_program(qp, ET());

        ac[0] = CGAL::to_double(*(solution.variable_values_begin() + 0));
        ac[1] = CGAL::to_double(*(solution.variable_values_begin() + 1));
        ac[2] = CGAL::to_double(*(solution.variable_values_begin() + 2));
    }
    catch (...) {
        ac = zero_3D();
    }
}

#endif

// --------------------------------------------------------------------------

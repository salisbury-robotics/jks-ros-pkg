#include "IntervalCollisionDetector.h"
#include <queue>
#include <cml/cml.h>

// for timing instrumentation
#ifndef _WIN32
#include <sys/time.h>
#endif

using namespace cml;

// --------------------------------------------------------------------------

IntervalCollisionDetector::IntervalCollisionDetector(const std::vector<cml::vector3d> &pointShell,
                                                     const vector3d &a, const vector3d &alpha,
                                                     CollisionOracle *oracle, double featureSize)
    : m_points(pointShell), m_oracle(oracle)
{
    m_linear = a;
    m_displacement = m_linear.length();

    m_angle = alpha.length();
    if (m_angle > 1e-10)    m_axis = alpha / m_angle;
    else                    m_axis.set(1.0, 0.0, 0.0);

    m_feature = featureSize;
    m_epsilon = featureSize / 4.0;
}

// --------------------------------------------------------------------------

vector3d IntervalCollisionDetector::pointAtTime(const vector3d &p, double t)
{
    return rotate_vector(p, m_axis, m_angle * t) + m_linear * t;
}

double IntervalCollisionDetector::arcLength(const cml::vector3d &p)
{
    return m_displacement + (p - p * fabs(dot(p, m_axis))).length() * m_angle;
}

void IntervalCollisionDetector::testPoint(PointInterval &pi)
{
    // if the point is already in contact, see if we can advance t1 to bound it
    if (pi.contact)
    {
        double t = 0.5 * (pi.t1 + pi.t2);
        vector3d p = pointAtTime(m_points[pi.index], t);
        if (m_oracle->inside(p))    pi.t2 = t;
        else                        pi.t1 = t;
    }
    // otherwise see if we can detect the first contact
    else
    {
        vector3d p = pointAtTime(m_points[pi.index], pi.t2);
        if (m_oracle->inside(p))    pi.contact = true;
        else {
            pi.t1 = pi.t2;
            pi.t2 = std::min(pi.t2 + pi.tStep, 1.0);
        }
    }
}

// --------------------------------------------------------------------------

double IntervalCollisionDetector::detectCollisions(std::vector<CollisionContact> &contacts)
{
    std::priority_queue<PointInterval, std::vector<PointInterval>, After> pq;
    std::vector<PointInterval> candidates;
/*
    // a little instrumentation to time this method
    struct timeval start, end;
    gettimeofday(&start, NULL);
*/
    // run through the existing contact set, and mark points already present
    std::vector<int> existing(m_points.size(), -1);
    for (int i = 0; i < contacts.size(); ++i)
    {
        existing[contacts[i].index] = i;
    }

    // intialize points and put into priority queue
    for (int i = 0; i < m_points.size(); ++i)
    {
        PointInterval pi(i);
        pi.marked   = existing[i];
        double al   = std::max(arcLength(m_points[i]), 1e-10);
        pi.tStep    = std::min(m_feature / al, 1.0);
        pi.tLimit   = std::min(m_epsilon / al, 1.0);
        pi.t2       = pi.tStep;
        pq.push(pi);
    }

    // run through the queue and test point intervals for collision
    int evaluations = 0;
    double first = 1.0; // best upper bound on first collision time
    double none = 1.0;  // best lower bound on non-collision time
    while (!pq.empty())
    {
        PointInterval pi = pq.top(); pq.pop();

        // test the point
        testPoint(pi);
        ++evaluations;

        // check for discard conditions
        if (pi.t1 >= first) continue;

        // check for contact and commit conditions
        if (pi.contact)
        {
            first = std::min(first, pi.t2);

            // if the point is within desired accuracy, add to the candidate set
            if (pi.t2 - pi.t1 <= pi.tLimit) {
                candidates.push_back(pi);
                continue;
            }
        }

        // if we get this far, then run the point through again
        pq.push(pi);
    }

    // extract from the candidate set the points that are really contacts
    for (std::vector<PointInterval>::iterator it = candidates.begin();
         it != candidates.end(); ++it)
    {
        if (it->t1 >= first) continue;
        none = std::min(none, it->t1);

        // check if it is a new contact
        if (it->marked < 0) {
            CollisionContact cc;
            cc.index = it->index;
            cc.position = m_points[cc.index];
            cc.contact = pointAtTime(cc.position, it->t2);
            contacts.push_back(cc);
        }
        // otherwise update existing contact
        else {
            vector3d p = contacts[it->marked].position;
            contacts[it->marked].contact = pointAtTime(p, it->t2);
        }
    }
/*
    // timing instrumentation
    gettimeofday(&end, NULL);
    long useconds = (end.tv_sec - start.tv_sec) * 1000000 +
                    (end.tv_usec - start.tv_usec);

    // periodically print out debug info
    static int counter = 0;
    static int high = 0;
    static long longest = 0;
    static double latest = 0.0;
    high = std::max(high, evaluations);
    longest = std::max(longest, useconds);
    latest = std::max(latest, none);
    if (++counter >= 1000) {
        std::cout << "ICD evaluated " << evaluations << " samples (max="
                  << high << ") in " << useconds << " microseconds (max="
                  << longest << "). latest=" << latest << std::endl;
        counter = 0;
        high = 0;
        longest = 0;
        latest = 0.0;
    }
*/
    // TODO: Ideally, we should clamp the motion to prevent it from ever penetrating,
    //       but for now returning the time of first collision seems to work better.
    //       I suspect this is due to the way constraints work on concave surfaces...
    //return none;
    return first;
}

// --------------------------------------------------------------------------

void IntervalCollisionDetector::pruneContacts(std::vector<CollisionContact> &contacts)
{
    std::vector<CollisionContact> confirmed;

    // run through the existing contact set, and mark points already present
    std::vector<bool> used(m_points.size(), false);
    for (std::vector<CollisionContact>::iterator it = contacts.begin();
         it != contacts.end(); ++it)
    {
        double al   = std::max(arcLength(it->position), 1e-10);
        double t    = std::min(m_epsilon / al, 1.0);
        vector3d p  = pointAtTime(it->position, t);
        if (m_oracle->inside(p)) {
            confirmed.push_back(*it);
//            contactPoints.push_back(p);
        }
    }

    // swap the confirmed contact set for the old one if they are different
    if (contacts.size() > confirmed.size())
        contacts.swap(confirmed);
}

// --------------------------------------------------------------------------

double IntervalCollisionDetector::resolvePenetrations(const std::vector<CollisionContact> &contacts)
{
    double earliest = 0.0;

    // perform interval visection on each point to find the earliest time it
    // exits the surface
    for (std::vector<CollisionContact>::const_iterator it = contacts.begin();
         it != contacts.end(); ++it)
    {
        double al   = std::max(arcLength(it->position), 1e-10);
        double t2   = std::min(2.0 * m_epsilon / al, 1.0);
        double t1   = 0.0;
        double dt   = std::min(0.4 * m_epsilon / al, 1.0);

        // if we can't get out, then don't move?
        if (m_oracle->inside(pointAtTime(it->position, t2)))
            return 0.0;

        while (t2 - t1 > dt)
        {
            double t = 0.5 * (t1 + t2);
            vector3d p  = pointAtTime(it->position, t);
            if (m_oracle->inside(p)) t1 = t;
            else                     t2 = t;
        }
        earliest = std::max(earliest, t2);
    }

    return earliest;
}

// --------------------------------------------------------------------------

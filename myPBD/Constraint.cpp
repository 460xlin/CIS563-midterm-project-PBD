//
// Created by sean on 3/2/18.
//

#include "Constraint.h"

#ifndef EPSILON
#define EPSILON 0.00001f
#endif

//----------Constraint Class----------//
Constraint::Constraint() :
        m_vertices(NULL),
        m_stiffness(1.0f)
{
    ;
}

Constraint::Constraint(PointList *verts, float stiff) :
        m_vertices(verts),
        m_stiffness(stiff)
{
    ;
}

Constraint::Constraint(const Constraint& other) :
        m_vertices(other.m_vertices),
        m_stiffness(other.m_stiffness)
{
    ;
}

Constraint::~Constraint()
{
    m_vertices = NULL;
}

bool Constraint::project_constraint()
{
    return true;
}

//----------FixedPointConstraint Class----------//
FixedPointConstraint::FixedPointConstraint() :
        Constraint()
{
    ;
}

FixedPointConstraint::FixedPointConstraint(PointList *verts, unsigned int p0, const glm::vec3& fixedpoint) :
        Constraint(verts, 1.0f),
        m_p0(p0),
        m_fixd_point(fixedpoint)
{
    ;
}

FixedPointConstraint::FixedPointConstraint(const FixedPointConstraint& other) :
        Constraint(other),
        m_p0(other.m_p0),
        m_fixd_point(other.m_fixd_point)
{
    ;
}

FixedPointConstraint::~FixedPointConstraint()
{
    ;
}

bool FixedPointConstraint::project_constraint()
{// TODO: implement the project function for FixedPointConstraint.
    //return true if current position is OK. return false if the position is being projected.
    m_vertices->posLock[m_p0];
    glm::vec3 predictPos = m_vertices->posPredict[m_p0];

    //float value = 0.0f;
    float value = glm::length(predictPos - m_fixd_point);
    if(value < EPSILON)
        return true;

    glm::vec3 dp0 = m_fixd_point - predictPos;
    m_vertices->posPredict[m_p0] += dp0 * m_stiffness;

    //if(std::isnan(m_vertices->posPredict[m_p0].x) || std::isnan(m_vertices->posPredict[m_p0].y) || std::isnan(m_vertices->posPredict[m_p0].z))
        //printf("fixed:nan>>%d\n",m_p0);

    return false;
}

//----------StretchConstraint Class----------//
StretchConstraint::StretchConstraint() :
        Constraint()
{
    ;
}

StretchConstraint::StretchConstraint(PointList *verts, float stiff, unsigned int p1, unsigned int p2, float length) :
        Constraint(verts, stiff),
        m_p1(p1),
        m_p2(p2),
        m_rest_length(length)
{
    ;
}

StretchConstraint::StretchConstraint(const StretchConstraint& other) :
        Constraint(other),
        m_p1(other.m_p1),
        m_p2(other.m_p2),
        m_rest_length(other.m_rest_length)
{
    ;
}

StretchConstraint::~StretchConstraint()
{
    ;
}

bool StretchConstraint::project_constraint()
{// TODO: implement the project function for StretchConstraint.
    //return true if current position is OK. return false if the position is being projected.
    glm::vec3 p1, p2;
    p1 = m_vertices->posPredict[m_p1];
    p2 = m_vertices->posPredict[m_p2];

    float length = glm::length(p1 - p2);
    if(glm::abs(length - m_rest_length) < EPSILON)
        return true;

    glm::vec3 dp1, dp2;
    float w1 = m_vertices->invMass[m_p1];
    float w2 = m_vertices->invMass[m_p2];
    if(length==0) length = 1;
    dp1 = -1 * w1 / (w1 + w2) * (length - m_rest_length) * (p1 - p2) / length;
    dp2 = w2 / (w1 + w2) * (length - m_rest_length) * (p1 - p2) / length;
    m_vertices->posPredict[m_p1] += dp1 * m_stiffness;
    m_vertices->posPredict[m_p2] += dp2 * m_stiffness;

    //if(std::isnan(m_vertices->posPredict[m_p1].x) || std::isnan(m_vertices->posPredict[m_p1].y) || std::isnan(m_vertices->posPredict[m_p1].z) ||
       //std::isnan(m_vertices->posPredict[m_p2].x) || std::isnan(m_vertices->posPredict[m_p2].y) || std::isnan(m_vertices->posPredict[m_p2].z))
        //printf("stretch:nan>>%d,%d\n",m_p1,m_p2);


    return false;
}

//----------BendConstraint Class----------//
BendConstraint::BendConstraint() :
        Constraint()
{
    ;
}

BendConstraint::BendConstraint(PointList *verts, float stiff, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, float phi) :
        Constraint(verts, stiff),
        m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4),
        m_phi(phi)
{
    ;
}

BendConstraint::BendConstraint(const BendConstraint& other) :
        Constraint(other),
        m_p1(other.m_p1), m_p2(other.m_p2), m_p3(other.m_p3), m_p4(other.m_p4),
        m_phi(other.m_phi)
{
    ;
}

BendConstraint::~BendConstraint()
{
    ;
}

bool BendConstraint::project_constraint()
{// TODO: implement the project function for BendConstraint.
    //return true if current position is OK. return false if the position is being projected.
    glm::vec3 p1 = m_vertices->posPredict[m_p1],
            p2 = m_vertices->posPredict[m_p2],
            p3 = m_vertices->posPredict[m_p3],
            p4 = m_vertices->posPredict[m_p4];

    float w1 = m_vertices->invMass[m_p1];
    float w2 = m_vertices->invMass[m_p2];
    float w3 = m_vertices->invMass[m_p3];
    float w4 = m_vertices->invMass[m_p4];

    glm::vec3 newP1(0, 0, 0);
    glm::vec3 newP2 = p2 - p1;
    glm::vec3 newP3 = p3 - p1;
    glm::vec3 newP4 = p4 - p1;

    float lx23 = glm::length(glm::cross(newP2, newP3));
    if(lx23==0) lx23 = 1;

    float lx24 = glm::length(glm::cross(newP2, newP4));
    if(lx24==0) lx24 = 1;

    glm::vec3 n1 = glm::cross(newP2, newP3) / lx23;
    glm::vec3 n2 = glm::cross(newP2, newP4) / lx24;
    float d = glm::clamp(glm::dot(n1, n2), -1.f, 1.f);

    if(glm::abs(acos(d) - m_phi) <EPSILON)
        return true;

    glm::vec3 q3 = (glm::cross(newP2, n2) + glm::cross(n1, newP2) * d) / lx23;
    glm::vec3 q4 = (glm::cross(newP2, n1) + glm::cross(n2, newP2) * d) / lx24;
    glm::vec3 q2 = -(glm::cross(newP3, n2) + glm::cross(n1, newP3) * d) / lx23
              -(glm::cross(newP4, n1) + glm::cross(n2, newP4) * d) / lx24;
    glm::vec3 q1 = -q2 - q3 - q4;

    float denominator = w1 * glm::dot(q1,q1) + w2 * glm::dot(q2,q2) + w3 * glm::dot(q3,q3) + w4 * glm::dot(q4,q4);
    if(denominator==0) denominator = 1;

    glm::vec3 dp1 = -1.f * w1 * glm::sqrt(1-d*d)*(glm::acos(d) - m_phi) * q1 / denominator;
    glm::vec3 dp2 = -1.f * w2 * glm::sqrt(1-d*d)*(glm::acos(d) - m_phi) * q2 / denominator;
    glm::vec3 dp3 = -1.f * w3 * glm::sqrt(1-d*d)*(glm::acos(d) - m_phi) * q3 / denominator;
    glm::vec3 dp4 = -1.f * w4 * glm::sqrt(1-d*d)*(glm::acos(d) - m_phi) * q4 / denominator;

    m_vertices->posPredict[m_p1] += dp1 * m_stiffness;
    m_vertices->posPredict[m_p2] += dp2 * m_stiffness;
    m_vertices->posPredict[m_p3] += dp3 * m_stiffness;
    m_vertices->posPredict[m_p4] += dp4 * m_stiffness;

    //if(std::isnan(m_vertices->posPredict[m_p1].x) || std::isnan(m_vertices->posPredict[m_p1].y) || std::isnan(m_vertices->posPredict[m_p1].z) ||
       //std::isnan(m_vertices->posPredict[m_p2].x) || std::isnan(m_vertices->posPredict[m_p2].y) || std::isnan(m_vertices->posPredict[m_p2].z) ||
       //std::isnan(m_vertices->posPredict[m_p3].x) || std::isnan(m_vertices->posPredict[m_p3].y) || std::isnan(m_vertices->posPredict[m_p3].z) ||
       //std::isnan(m_vertices->posPredict[m_p4].x) || std::isnan(m_vertices->posPredict[m_p4].y) || std::isnan(m_vertices->posPredict[m_p4].z))
        //printf("bend:nan>>%d,%d,%d,%d,%f\n",m_p1,m_p2,m_p3,m_p4,denominator);

    return false;
}

//----------CollisionConstraint Class----------//
CollisionConstraint::CollisionConstraint() :
        Constraint()
{
    ;
}

CollisionConstraint::CollisionConstraint(PointList *verts, unsigned int p0, const glm::vec3& q, const glm::vec3& n) :
        Constraint(verts, 1.0f),
        m_p0(p0),
        m_ref_point(q),//intersection point
        m_normal(n)
{
    ;
}

CollisionConstraint::CollisionConstraint(const CollisionConstraint& other) :
        Constraint(other),
        m_p0(other.m_p0),
        m_ref_point(other.m_ref_point),
        m_normal(other.m_normal)
{
    ;
}

CollisionConstraint::~CollisionConstraint()
{
    ;
}

bool CollisionConstraint::project_constraint()
{// TODO: implement the project function for CollisionConstraint.
    //return true if current position is OK. return false if the position is being projected.
    glm::vec3 p0 = m_vertices->posPredict[m_p0];
    glm::vec3 dir = p0 - m_ref_point;
    float value = glm::dot(dir, m_normal);
    //float value = 0.0f;
    if(value > 0.0f)
        return true;

    glm::vec3 dp0 = m_ref_point - p0;
    m_vertices->posPredict[m_p0] += dp0 * m_stiffness;

    return false;
}

//----------SelfCollisionConstraint Class----------//
SelfCollisionConstraint::SelfCollisionConstraint() :
        Constraint()
{
    ;
}

SelfCollisionConstraint::SelfCollisionConstraint(PointList *verts, unsigned int q, unsigned int p1, unsigned int p2, unsigned int p3, float h) :
        Constraint(verts, 1.0f),
        m_q(q), m_p1(p1), m_p2(p2), m_p3(p3),
        m_h(h)
{
    ;
}
SelfCollisionConstraint::SelfCollisionConstraint(const SelfCollisionConstraint& other) :
        Constraint(other),
        m_q(other.m_q), m_p1(other.m_p1), m_p2(other.m_p2), m_p3(other.m_p3),
        m_h(other.m_h)
{
    ;
}

SelfCollisionConstraint::~SelfCollisionConstraint()
{

}

bool SelfCollisionConstraint::project_constraint()
{
    glm::vec3 q, p1, p2, p3;
    q =  m_vertices->posPredict[m_q];
    p1 = m_vertices->posPredict[m_p1];
    p2 = m_vertices->posPredict[m_p2];
    p3 = m_vertices->posPredict[m_p3];

    q = q - p1;
    p2 = p2 - p1;
    p3 = p3 - p1;
    p1 = glm::vec3(0.0f);

    glm::vec3 normal(glm::cross(p2, p3));
    float c23 = glm::length(normal);
    if(c23==0) c23=1;//me
    normal = glm::normalize(normal);

    float value = glm::dot(q, normal) - m_h;
    if(value > 0.0f)
        return true;

    glm::vec3 dcq, dcp1, dcp2, dcp3;
    dcq = normal;
    dcp2 = (glm::cross(p3, q) + glm::cross(normal, p3) * glm::dot(normal, q)) / c23;
    dcp3 = -(glm::cross(p2, q) + glm::cross(normal, p2) * glm::dot(normal, q)) / c23;
    dcp1 = -dcq - dcp2 - dcp3;

    float wq, w1, w2, w3;
    wq = m_vertices->invMass[m_q];
    w1 = m_vertices->invMass[m_p1];
    w2 = m_vertices->invMass[m_p2];
    w3 = m_vertices->invMass[m_p3];

    float denominator = w1 * glm::dot(dcp1, dcp1) + w2 * glm::dot(dcp2, dcp2) + w3 * glm::dot(dcp3, dcp3) + wq * glm::dot(dcq, dcq);
    assert(denominator < EPSILON);

    glm::vec3 dq, dp1, dp2, dp3;
    if(denominator==0) denominator=1;
    float s = value / denominator;
    dq = -wq * s * dcq;
    dp1 = -w1 * s * dcp1;
    dp2 = -w2 * s * dcp2;
    dp3 = -w3 * s * dcp3;

    m_vertices->posPredict[m_q] += dq * m_stiffness;
    m_vertices->posPredict[m_p1] += dp1 * m_stiffness;
    m_vertices->posPredict[m_p2] += dp2 * m_stiffness;
    m_vertices->posPredict[m_p3] += dp3 * m_stiffness;
    return false;
}
//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_CONSTRAINT_H
#define MYPBD_CONSTRAINT_H

#include "PointList.h"

class Constraint
{
public:
    Constraint();
    Constraint(PointList *verts, float stiff);
    Constraint(const Constraint& other);
    virtual ~Constraint();

    void set_stiffness(float k)
    {
        m_stiffness = k;
    }
    virtual bool project_constraint();
protected:
    // a pointer to all the vertices.
    PointList *m_vertices;
    // stiffness decide how much the vertex would move towards the constrained position.
    float m_stiffness;
};

class FixedPointConstraint : public Constraint
{
public:
    FixedPointConstraint();
    FixedPointConstraint(PointList *verts, unsigned int p0, const glm::vec3& fixedpoint);
    FixedPointConstraint(const FixedPointConstraint& other);
    virtual ~FixedPointConstraint();

    virtual bool project_constraint();
protected:
    // cardinality for stretch constraint is 1.
    unsigned int m_p0;
    glm::vec3 m_fixd_point;
};

class StretchConstraint : public Constraint
{
public:
    StretchConstraint();
    StretchConstraint(PointList *verts, float stiff, unsigned int p1, unsigned int p2, float length);
    StretchConstraint(const StretchConstraint& other);
    virtual ~StretchConstraint();

    virtual bool project_constraint();
protected:
    // cardinality for stretch constraint is 2.
    unsigned int m_p1, m_p2;
    // rest length
    float m_rest_length;
};

class BendConstraint : public Constraint
{
public:
    BendConstraint();
    BendConstraint(PointList *verts, float stiff, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, float phi);
    BendConstraint(const BendConstraint& other);
    virtual ~BendConstraint();

    virtual bool project_constraint();
protected:
    // cardinality for bend constraint is 4.
    unsigned int m_p1, m_p2, m_p3, m_p4;
    // rest dihedral angle.
    float m_phi;
};

class CollisionConstraint : public Constraint
{
public:
    CollisionConstraint();
    CollisionConstraint(PointList *verts, unsigned int p0, const glm::vec3& q, const glm::vec3& n);
    CollisionConstraint(const CollisionConstraint& other);
    virtual ~CollisionConstraint();

    virtual bool project_constraint();

    const glm::vec3& normal() const
    {
        return m_normal;
    }
    unsigned int index() const
    {
        return m_p0;
    }
protected:
    // cardinality for collision constraint is 1.
    unsigned int m_p0;
    glm::vec3 m_ref_point, m_normal;
};

class SelfCollisionConstraint : public Constraint
{
public:
    SelfCollisionConstraint();
    SelfCollisionConstraint(PointList *verts, unsigned int q, unsigned int p1, unsigned int p2, unsigned int p3, float h);
    SelfCollisionConstraint(const SelfCollisionConstraint& other);
    virtual ~SelfCollisionConstraint();

    virtual bool project_constraint();
    glm::vec3 normal() const
    {
        glm::vec3 e1, e2;
        e1 = m_vertices->posPredict[m_p2] - m_vertices->posPredict[m_p1];
        e2 = m_vertices->posPredict[m_p3] - m_vertices->posPredict[m_p1];

        return glm::normalize(glm::cross(e1, e2));
    }
    unsigned int index() const
    {
        return m_q;
    }
protected:
    // cardinality for self collision constraint is 4.
    unsigned int m_q, m_p1, m_p2, m_p3;
    float m_h;
};


#endif //MYPBD_CONSTRAINT_H

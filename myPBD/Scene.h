//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_SCENE_H
#define MYPBD_SCENE_H

#include "glm/glm.hpp"
#include <vector>
#include <string>

class Primitive
{
public:
    Primitive()
    {

    };

    virtual ~Primitive()
    {

    };

    virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const = 0;
    virtual void exportFile(std::string name, int frame) = 0;
};

class Plane : public Primitive
{
public:
    Plane(glm::vec3 normal, float value) : m_normal(normal), m_value(value)
    {

    }

    virtual ~Plane()
    {

    }

    virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;
    virtual void exportFile(std::string name, int frame);
    glm::vec3 m_normal;
    float m_value;//distance between origin and the plane
};

class Sphere : public Primitive
{
public:
    Sphere(const glm::vec3 pos, float radius) : m_center(pos), m_radius(radius)
    {

    }

    virtual ~Sphere()
    {

    }

    virtual bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;
    virtual void exportFile(std::string name, int frame);
    glm::vec3 m_center;
    float m_radius;
};

class Scene
{
public:
    Scene();
    virtual ~Scene();
    void insert_primitive(Primitive* const new_primitive);
    bool line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const;

    std::vector<Primitive*> m_primitives;
};


#endif //MYPBD_SCENE_H

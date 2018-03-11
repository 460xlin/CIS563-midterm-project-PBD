//
// Created by sean on 3/2/18.
//

#include "Scene.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <Partio.h>

//----------Scene Class----------//
Scene::Scene()
{

}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        delete (*iter);
    }
    m_primitives.clear();
}

void Scene::insert_primitive(Primitive* const new_primitive)
{
    m_primitives.push_back(new_primitive);
}

bool Scene::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// assume no intersection between primitives. i.e. a line intersects at most one primitive.
    for(std::vector<Primitive*>::const_iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        if((*iter)->line_intersection(p1, p2, threshold, intersect, normal))
            return true;
    }
    return false;
}

//----------Plane Class----------//
bool Plane::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{
    //p1 is position, p2 is predicted position
    float v1, v2;
    v1 = glm::dot(p1, m_normal) - m_value;
    v2 = glm::dot(p2, m_normal) - m_value;
    if(v2 < threshold)
    {
        normal = m_normal;
        if(v1 >= threshold)
        {// continuous collision handling.
            intersect = ((v1 - threshold) * p2 - (v2 - threshold) * p1) / (v1 - v2);
        }
        else
        {// static collision handling.
            intersect = p2 - (v2 - threshold) * normal;
        }
        return true;
    }
    else
        return false;
}

void Plane::exportFile(std::string name, int frame) {
    //TO DO
}

//----------Sphere Class----------//
bool Sphere::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// TODO: implement line-sphere intersection. you can refer to line-plane intersection.
    float v1, v2;// v1 v2 are distance to sphere for p1 and p2.
    v1 = glm::length(p1 - m_center) - m_radius;
    v2 = glm::length(p2 - m_center) - m_radius;

    if(v2 < threshold)
    {
        if(v1 >= threshold)
        {// continuous collision handling.
            glm::vec3 newV0 = p2 - p1;
            glm::vec3 newP0 = glm::vec3(p1[0], p1[1], p1[2]);

            newV0 = glm::normalize(newV0);

            float temp = glm::dot(newV0, m_center - newP0);
            float t = temp - sqrt(temp * temp - glm::dot(m_center - newP0, m_center - newP0) + (m_radius + threshold) * (m_radius + threshold));
            if(t < -0.001)
                bool error = true;
            intersect = newP0 + t * newV0;
            normal = glm::normalize(intersect - m_center);
        }
        else
        {// static collision handling.
            glm::vec3 newV0 = p2 - m_center;
            float length = glm::length(newV0);
            float t = (m_radius + threshold) / length;

            intersect = m_center + newV0 * t;
            normal = glm::normalize(intersect - m_center);
        }
        return true;
    }
    else
        return false;
}

int Sphere::importFile(std::string name) {
    float diameter = -1;
    glm::vec3 firstV(0.f);
    glm::vec3 secondV(0.f);
    bool gotFirstV = false;

    std::fstream file;
    file.open(name, std::ios::in);
    if (file.is_open())
    {
        std::string str;
        while (std::getline(file, str))
        {
            if (str.substr(0, 2) == "v ")
            {
                std::stringstream ss;
                ss << str.substr(2);
                glm::vec3 v;
                ss >> v.x;
                ss >> v.y;
                ss >> v.z;
                //extra!!
                if(!gotFirstV) {
                    firstV = v;
                    secondV = v;//for clarity
                    gotFirstV = true;
                }
                else {
                    float distance = glm::length(firstV - v);
                    if(distance > diameter)
                    {
                        diameter = distance;
                        secondV = v;
                    }
                }
                //extra!!
                vertices.push_back(v);
            }
            else if (str.substr(0, 3) == "vt ")
            {
                //nothing
            }
            else if (str.substr(0, 3) == "vn ")
            {
                //nothing
            }
            else if (str.substr(0, 2) == "f ")
            {
                //nothing
            }
            else if (str[0] == '#')
            {
                //comment
            }
            else
            {
                //others
            }
        }
    }
    else
    {
        std::cout << "can not open" << name << std::endl;
        return 1;
    }

    radiusScale = m_radius / (diameter / 2.f);
    centerFake = (firstV + secondV)/2.f;

    file.close();
    return 0;
}

void Sphere::exportFile(std::string name, int frame) {
    //TO DO
    glm::vec3 centerOffset =  glm::vec3(0.f) - centerFake;
    int n = vertices.size();

    Partio::ParticlesDataMutable *parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);

    for (int i = 0; i < n; i++) {
        int idx = parts->addParticle();
        float *m = parts->dataWrite<float>(mH, idx);
        float *p = parts->dataWrite<float>(posH, idx);
        float *v = parts->dataWrite<float>(vH, idx);

        m[0] = 1.0;
        p[0] = (vertices[i].x + centerOffset.x) * radiusScale + m_center.x;
        p[1] = (vertices[i].y + centerOffset.y) * radiusScale + m_center.y;
        p[2] = (vertices[i].z + centerOffset.z) * radiusScale + m_center.z;
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
    }

    std::stringstream ss;
    ss << name;
    ss << frame;
    ss << ".bgeo";
    Partio::write(ss.str().c_str(), *parts);
    parts->release();
}
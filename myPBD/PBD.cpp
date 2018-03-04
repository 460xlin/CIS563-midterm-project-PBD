//
// Created by sean on 3/2/18.
//

#include "PBD.h"
#include <Partio.h>
#include <sstream>
#include <fstream>

//#define MYDEBUG_UPDATE //print pos after each step in update
//#define MYDEBUG_CONSTRAINT //print pos in resolveConstraints after projecting each type of constraint
//#define MYDEBUG_PREDICT //print posPredict instead of pos
//#define MYDEBUG_LOGTXT //out put invMass, velocity, position to several txt files

PBD::PBD() {

}

PBD::~PBD(){
    for(int i = 0;i<constraint.size();i++) {
        delete constraint[i];
    }
}

void PBD::initialize(int _dimX, int _dimZ, float _thickness, int _iteration, glm::vec3 clothMin, glm::vec3 clothMax){
    dimX = _dimX;
    dimZ = _dimZ;
    thickness = _thickness;
    iteration = _iteration;
    glm::vec3 delta = clothMax - clothMin;
    delta.x /= (dimX - 1.f);
    //delta.y /= (dimY - 1.f);
    delta.z /= (dimZ - 1.f);
    pointList.resize(dimX * dimZ);
    for(int i = 0;i<dimX;i++){
        for(int k = 0;k<dimZ;k++){
            int index = dimZ * i + k;
            pointList.pos[index] = glm::vec3(delta.x * i + clothMin.x, (clothMin.y + clothMax.y)/2.f,delta.z*k+clothMin.z);
            pointList.velocity[index] = glm::vec3(0.f);
            pointList.invMass[index] = 1.f;
        }
    }

    triangleList.resize((dimX-1) * (dimZ-1) * 2 * 3);
    bool rowFlip = false, columnFlip = false;
    for(int i = 0;i<dimX - 1;i++){
        for(int k = 0;k<dimZ - 1;k++)
        {
            int index = (dimZ-1) * i + k;
            triangleList[6*index+0] = dimZ*i+k;
            triangleList[6*index+1] = dimZ*i+k+1;
            triangleList[6*index+2] = dimZ*(i+1)+(rowFlip^columnFlip ? k+1 : k);

            triangleList[6*index+3] = dimZ*(i+1)+k+1;
            triangleList[6*index+4] = dimZ*(i+1)+k;
            triangleList[6*index+5] = dimZ*i+(rowFlip^columnFlip ? k : k+1);

            rowFlip = !rowFlip;
        }
        columnFlip = !columnFlip;
        rowFlip = false;
    }
    generateEdgeList();
    generateInternalConstraints();
}

void PBD::update(Scene *s, float dt) {
    glm::vec3 gravity(0.0f, -9.8f, 0.0f);
    applyExternalForce(gravity, dt);//!
#ifdef MYDEBUG_UPDATE
    std::cout << ">>>1.applyExternalForce:<<<" << std::endl;
    printDebug();
#endif
    dampVelocity(0.01f);
    computePredictedPostion(dt);//!
#ifdef MYDEBUG_UPDATE
    std::cout << ">>>2.computePredictedPostion:<<<" << std::endl;
    printDebug();
#endif
    collisionDetection(s);//?
#ifdef MYDEBUG_UPDATE
    std::cout << ">>>2.5 collisionDetection:<<<" << std::endl;
    printDebug();
#endif
    resolveConstraints();//?
#ifdef MYDEBUG_UPDATE
    std::cout << ">>>2.8 resolveConstraints:<<<" << std::endl;
    printDebug();
#endif
    integration(dt);//!
#ifdef MYDEBUG_UPDATE
    std::cout << ">>>3.integration:<<<" << std::endl;
    printDebug();
#endif
    updateVelocity(0.98f, 0.4f);
    cleanCollisionConstraints();
}

void PBD::generateEdgeList()
{
    int vert_num = pointList.size;
    int tri_num = triangleList.size() / 3;

    int *first_edge = new int[vert_num + 3 * tri_num];
    int *next_edge = first_edge + vert_num;

    for(int i = 0; i < vert_num; ++i)
        first_edge[i] = -1;

    int edge_count = 0;
    const int* triangle = &triangleList[0];
    int i1, i2;
    for(int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 < i2)
            {
                Edge new_edge;
                new_edge.v1 = i1;
                new_edge.v2 = i2;
                new_edge.tri1 = t;
                new_edge.tri2 = t;
                edgeList.push_back(new_edge);

                int edge_idx = first_edge[i1];
                if(edge_idx == -1)
                {
                    first_edge[i1] = edge_count;
                }
                else
                {
                    while(true)
                    {
                        int idx = next_edge[edge_idx];
                        if(idx == -1)
                        {
                            next_edge[edge_idx] = edge_count;
                            break;
                        }
                        edge_idx = idx;
                    }
                }

                next_edge[edge_count] = -1;
                edge_count++;
            }
            i1 = i2;
        }
        triangle += 3;
    }

    triangle = &triangleList[0];
    for(int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 > i2)
            {
                bool is_new_edge = true;
                for(int edge_idx = first_edge[i2]; edge_idx != -1; edge_idx = next_edge[edge_idx])
                {
                    Edge *edge = &edgeList[edge_idx];
                    if((edge->v2 == i1) && (edge->tri1 == edge->tri2))
                    {
                        edge->tri2 = t;
                        is_new_edge = false;
                        break;
                    }
                }
                if(is_new_edge)
                {
                    Edge new_edge;
                    new_edge.v1 = i1;
                    new_edge.v2 = i2;
                    new_edge.tri1 = t;
                    new_edge.tri2 = t;
                    edgeList.push_back(new_edge);

                    int edge_idx = first_edge[i1];
                    if(edge_idx == -1)
                    {
                        first_edge[i1] = edge_count;
                    }
                    else
                    {
                        while(true)
                        {
                            int idx = next_edge[edge_idx];
                            if(idx == -1)
                            {
                                next_edge[edge_idx] = edge_count;
                                break;
                            }
                            edge_idx = idx;
                        }
                    }

                    next_edge[edge_count] = -1;
                    edge_count++;
                }
            }
            i1 = i2;
        }
        triangle += 3;
    }

    delete[] first_edge;
    //printf("Edge number: %u.\n", edgeList.size());
}

void PBD::generateInternalConstraints() {
    for(int i = 0; i < dimX; ++i)
    {
        for(int k = 0; k < dimZ; ++k)
        {
            int index = dimZ * i + k;

            if(k == 0)
            {
                FixedPointConstraint* fixedConstraint = new FixedPointConstraint(&pointList, index, pointList.pos[index]);
                //constraint.push_back(fixedConstraint);
            }
        }
    }

    // generate stretch constraints. assign a stretch constraint for each edge.
    glm::vec3 p1, p2;
    float stretch_stiff = 0.5f;
    float s_stiff = 1.0f - std::pow((1 - stretch_stiff), 1.0f / iteration);
    for(std::vector<Edge>::iterator e = edgeList.begin(); e != edgeList.end(); ++e)
    {
        int start = e->v1;
        int end = e->v2;

        float restLength = glm::length(pointList.pos[start] - pointList.pos[end]);
        StretchConstraint* stretchConstraint = new StretchConstraint(&pointList, s_stiff, start, end, restLength);
        constraint.push_back(stretchConstraint);

    }

    glm::vec3 bendP1, bendP2, bendP3, bendP4;
    float phi;
    int id1, id2, id3, id4;
    int *tri;

    float bend_stiff = 0.1f;
    float b_stiff = 1.0f - std::pow((1 - bend_stiff), 1.0f / iteration);
    for(std::vector<Edge>::iterator e = edgeList.begin(); e != edgeList.end(); ++e)
    {
        if(e->tri1 == e->tri2)
            continue;
        id1 = e->v1;
        id2 = e->v2;

        tri = &triangleList[3 * e->tri1];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id3 = *tri;

        tri = &triangleList[3 * e->tri2];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id4 = *tri;

        bendP1 = pointList.pos[id1];
        bendP2 = pointList.pos[id2];
        bendP3 = pointList.pos[id3];
        bendP4 = pointList.pos[id4];

        glm::vec3 vecP1P2 = bendP2 - bendP1;
        glm::vec3 vecP1P3 = bendP3 - bendP1;
        glm::vec3 vecP1P4 = bendP4 - bendP1;
        glm::vec3 normal_p1p2p3 = glm::cross(vecP1P2, vecP1P3);
        glm::vec3 normal_p1p2p4 = glm::cross(vecP1P2, vecP1P4);

        glm::vec3 newP1(0, 0, 0);
        glm::vec3 newP2 = bendP2 - bendP1;
        glm::vec3 newP3 = bendP3 - bendP1;
        glm::vec3 newP4 = bendP4 - bendP1;

        glm::vec3 n1 = glm::cross(newP2, newP3) / glm::length(glm::cross(newP2, newP3));
        glm::vec3 n2 = glm::cross(newP2, newP4) / glm::length(glm::cross(newP2, newP4));


        float cosTheta = glm::dot(n1, n2);
        cosTheta = glm::clamp(cosTheta, -1.f, 1.f);
        float theta = glm::acos(cosTheta);

        float cosTheta1 = glm::dot(normal_p1p2p3, normal_p1p2p4) / glm::length(normal_p1p2p3) / glm::length(normal_p1p2p4);
        cosTheta1 = glm::clamp(cosTheta1, -1.f, 1.f);
        float theta1 = glm::acos(cosTheta1);

        if(abs(theta - theta1) > 0.001)
            bool error = true;


        BendConstraint* bendConstraint = new BendConstraint(&pointList, b_stiff, id1, id2, id3, id4, theta);
        constraint.push_back(bendConstraint);
    }
}

void PBD::applyExternalForce(glm::vec3 force, float dt) {
    int size = pointList.size;
    for(unsigned int i = 0; i < size; ++i)
    {
        pointList.velocity[i] += dt * force * pointList.invMass[i];
    }
}

void PBD::dampVelocity(float k_damp)
{
    float totalMass = 0.0f;
    glm::vec3 massPos;
    glm::vec3 massVel;
    for(int i = 0; i < pointList.size; ++i)
    {
        glm::vec3 pos = pointList.pos[i];
        glm::vec3 vel = pointList.velocity[i];
        float mass = 1 / pointList.invMass[i];
        massPos += mass * pos;
        massVel += mass * vel;
        totalMass += mass;
    }

    glm::vec3 centerMassPos = massPos / totalMass;
    glm::vec3 centerMassVel = massVel / totalMass;

    glm::vec3 L;
    glm::mat3x3 I;
    for(int i = 0; i < pointList.size; ++i)
    {
        float mass = 1 / pointList.invMass[i];
        glm::vec3 vel = pointList.velocity[i];
        glm::vec3 r = pointList.pos[i] - centerMassPos;
        L += glm::cross(r, mass * vel);

        glm::mat3x3 skewMatrixR(glm::vec3(0, r[2], -r[1]), glm::vec3(-r[2], 0, r[0]), glm::vec3(r[1], -r[0], 0));
        I += skewMatrixR * glm::transpose(skewMatrixR) * mass;
    }

    glm::vec3 omega = I._inverse() * L;

    for(int i = 0; i < pointList.size; ++i)
    {
        glm::vec3 r = pointList.pos[i] - centerMassPos;
        glm::vec3 vel = pointList.velocity[i];
        glm::vec3 deltaV = centerMassVel + glm::cross(omega, r) - vel;
        pointList.velocity[i] += k_damp * deltaV;
    }

}

void PBD::computePredictedPostion(float dt)
{
    int size = pointList.size;
    for(int i = 0; i < size; ++i)
    {
        pointList.posPredict[i] = pointList.pos[i] + dt * pointList.velocity[i];
    }
}

void PBD::collisionDetection(Scene *s)
{
    int size = pointList.size;
    glm::vec3 x, p, q, n;
    for(int i = 0; i < size; ++i)
    {
        x = pointList.pos[i];
        p = pointList.posPredict[i];
        if(s->line_intersection(x, p, thickness, q, n))
        {
            CollisionConstraint c(&pointList, i, q, n);
            constraintCollision.push_back(c);
        }
    }
    // TODO: implement self collision if you want to.
    // selfCollisionDetection();
}

void PBD::selfCollisionDetection()
{// TODO: implement self collision if you want to.
    ;
}

void PBD::resolveConstraints()
{
    bool all_solved = true;
    bool reverse = false;
    int i, size;
    for(int n = 0; n < iteration; ++n)
    {
        size = constraint.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= constraint[i]->project_constraint();
        }
#ifdef MYDEBUG_CONSTRAINT
        std::cout << ">>>A.internal constraints:<<<" << std::endl;
        printDebug();
#endif
        // solve all the external constraints.
        size = constraintCollision.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= constraintCollision[i].project_constraint();
        }
#ifdef MYDEBUG_CONSTRAINT
        std::cout << ">>>B.external collision constraints:<<<" << std::endl;
        printDebug();
#endif
        // solve all the self collisions.
        size = constraintSelfCollision.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= constraintSelfCollision[i].project_constraint();
        }
#ifdef MYDEBUG_CONSTRAINT
        std::cout << ">>>C.self collision constraints:<<<" << std::endl;
        printDebug();
#endif
        if(all_solved)
            break;
        reverse = !reverse;
    }
    pointList.unlockPosAll();
}

void PBD::integration(float dt)
{
    int size = pointList.size;
    float inv_dt = 1.0f / dt;
    for(unsigned int i = 0; i < size; ++i)
    {
        pointList.velocity[i] = (pointList.posPredict[i] - pointList.pos[i]) * inv_dt;
        pointList.pos[i] = pointList.posPredict[i];
    }
}

void PBD::updateVelocity(float friction, float restitution)
{
    glm::vec3 normal, vn, vt;
    float norm_fraction;
    for(std::vector<CollisionConstraint>::iterator s = constraintCollision.begin(); s != constraintCollision.end(); ++s)
    {
        int pointIndex = s->index();
        glm::vec3 currentVel = pointList.velocity[pointIndex];
        normal = s->normal();
        if(glm::dot(normal, currentVel) < 0)
        {
            vn = glm::dot(normal, currentVel) * normal;
            vt = currentVel - vn;

            glm::vec3 newVn = -1 * restitution * vn;
            glm::vec3 newVt = friction * vt;

            pointList.velocity[pointIndex] = newVn + newVt;
        }
        int a = 0;
    }

    for(std::vector<SelfCollisionConstraint>::iterator s = constraintSelfCollision.begin(); s != constraintSelfCollision.end(); ++s)
    {// TODO: add this part if you added self collisions already.
        ;
    }
}

void PBD::cleanCollisionConstraints()
{
    constraintCollision.clear();
    constraintSelfCollision.clear();
}

void PBD::exportFile(std::string name, int frame) {
    int n = pointList.size;

#ifdef MYDEBUG_LOGTXT
    std::ofstream ofs;
    std::stringstream oss;
    oss << name;
    oss << frame;
    oss << ".txt";
    ofs.open(oss.str().c_str(), std::ios_base::out | std::ios_base::trunc);
#endif

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

        m[0] = 1.0 / pointList.invMass[i];
        p[0] = pointList.pos[i].x;
        p[1] = pointList.pos[i].y;
        p[2] = pointList.pos[i].z;
        v[0] = pointList.velocity[i].x;
        v[1] = pointList.velocity[i].y;
        v[2] = pointList.velocity[i].z;

#ifdef MYDEBUG_LOGTXT
        ofs << "invMass:" << m[0]
            << ", posX:" << p[0]
            << ", posY:" << p[1]
            << ", posZ:" << p[2]
            << ", velX:" << v[0]
            << ", velY:" << v[1]
            << ", velZ:" << v[2] << std::endl;
#endif
    }

#ifdef MYDEBUG_LOGTXT
    ofs.close();
#endif

    std::stringstream ss;
    ss << name;
    ss << frame;
    ss << ".bgeo";
    Partio::write(ss.str().c_str(), *parts);
    parts->release();
}

void PBD::printAll() {
    std::cout << "pos::::::::::::" << std::endl;
    for(int i = 0;i<pointList.size;i++) {
        std::cout << i << ":" << pointList.pos[i].x << "," << pointList.pos[i].y << "," << pointList.pos[i].z << std::endl;
    }
}

void PBD::printAllPredict() {
    std::cout << "posPredict::::::::::::" << std::endl;
    for(int i = 0;i<pointList.size;i++) {
        std::cout << i << ":" << pointList.posPredict[i].x << "," << pointList.posPredict[i].y << "," << pointList.posPredict[i].z << std::endl;
    }
}

void PBD::printDebug() {
#ifdef MYDEBUG_PREDICT
    printAllPredict();
#else
    printAll();
#endif
}
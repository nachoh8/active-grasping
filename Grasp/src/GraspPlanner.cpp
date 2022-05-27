#include "../include/Grasp/GraspPlanner.hpp"

#include <iomanip>
#include <stdexcept>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/MathTools.h>

#include "../include/Grasp/GraspVars.hpp"

namespace Grasp {

/// Init

GraspPlanner::GraspPlanner(const GraspPlannerParams& params)
: params(params)
{
    loadScene();
}

GraspPlanner::GraspPlanner(const std::string& json_file)
{
    if (!load_GraspPlannerParams_json(json_file, this->params)) {
        throw "Errpr creating GraspPlanner from json";
    }

    loadScene();
}

void GraspPlanner::loadScene() {
    /// Load robot
    robot.reset();
    
    robot = VirtualRobot::RobotIO::loadRobot(params.robot_file);

    if (!robot) {
        exit(1);
    }

    VirtualRobot::EndEffectorPtr _eef = robot->getEndEffector(params.eef_name);

    if (!_eef) {
        exit(1);
    }

    if (!params.preshape.empty())
    {
        _eef->setPreshape(params.preshape);
    }

    eefCloned = _eef->createEefRobot("eef", "icub_eef");
    eef = eefCloned->getEndEffector(params.eef_name);

    if (params.has_eef_pose) {
        moveEE(params.eef_position, params.eef_orientation);
    }

    /// Load object
    object = VirtualRobot::ObjectIO::loadManipulationObject(params.object_file);

    if (!object) {
        exit(1);
    }
    /*
    if (params.has_obj_pose) {
        float x[6];
        x[0] = params.obj_position.x();
        x[1] = params.obj_position.y();
        x[2] = params.obj_position.z();
        x[3] = params.obj_orientation.x();
        x[4] = params.obj_orientation.y();
        x[5] = params.obj_orientation.z();

        Eigen::Matrix4f m;
        VirtualRobot::MathTools::posrpy2eigen4f(x, m);

        object->setGlobalPose(m);
    } else {
        // objectToTCP
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        object->setGlobalPose(pos);
    }
    */
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity(4,4);
    if (params.has_obj_pose) {
        float x[6];
        //x[0] = params.obj_position.x();
        //x[1] = params.obj_position.y();
        //x[2] = params.obj_position.z();
        x[3] = params.obj_orientation.x();
        x[4] = params.obj_orientation.y();
        x[5] = params.obj_orientation.z();

        VirtualRobot::MathTools::posrpy2eigen4f(x, m);

        object->setGlobalPose(m);
    } else {
        object->setGlobalPose(m);
    }
    

    /// Set quality measure
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();
    
    // TODO: search about Grasp class
    // std::string name = "Grasp Planner - " + eef->getName();
    // grasps.reset(new VirtualRobot::GraspSet(name, eefCloned->getType(), params.eef_name));
}

/// Public

GraspResult GraspPlanner::executeQueryGrasp(const std::vector<double>& query) {
    if (query.size() != NUM_GRASP_VARS) {
        std::cerr << "Error: query size is different of " << NUM_GRASP_VARS << "!!!\n";
        exit(1);
    }
    objectModel = object->getCollisionModel()->getTriMeshModel();
    Eigen::Matrix4f poseObject = object->getGlobalPose();

    //CARTESIANS:
    /*
    Eigen::Vector3f xyz(query[GRASP_VAR::TRANS_X], query[GRASP_VAR::TRANS_Y], query[GRASP_VAR::TRANS_Z]);
    */

    //Eigen::Matrix4f poseObject = object->getGlobalPose();

    // SPHERICAL:
    /*
    VirtualRobot::MathTools::SphericalCoord scoords;

    scoords.r = query[GRASP_VAR::TRANS_RHO];
    scoords.theta = query[GRASP_VAR::TRANS_THETA];
    scoords.phi = query[GRASP_VAR::TRANS_PHI];
    

    Eigen::Vector3f xyz = VirtualRobot::MathTools::toPosition(scoords);
    */

    //std::cout << "***POSE OBJECT: " << poseObject.block(0,3,3,1) << std::endl;
    //std::cout << "***POSE POINT: " << xyz << std::endl;
    

    // SPHERICAL KNOWING RAY INTERSECTION
    
    VirtualRobot::MathTools::SphericalCoord scoords;
    scoords.theta = query[GRASP_VAR::TRANS_THETA];
    //scoords.theta = 1.8171027536166593;
    scoords.phi = query[GRASP_VAR::TRANS_PHI];
    //scoords.phi = -2.2386673065403975;
    scoords.r = 0;

    Eigen::Vector3f rayOrigin = poseObject.block(0, 3, 3, 1);
    //Eigen::Vector3f outIntersectionPoint;
    float rho = 0;

    float x_ = sin(scoords.theta)*cos(scoords.phi);
    float y_ = sin(scoords.theta)*cos(scoords.phi);
    float z_ = cos(scoords.theta);
    Eigen::Vector3f rayVector = {x_,y_,z_};
    rayVector.normalize();
    float x = 0;
    float y = 0;
    float z = 0;

    bool intersect;
    int limit = objectModel->faces.size();
    for (int faceIndex = 0; faceIndex < limit; faceIndex++){
        std::size_t nVert1 = (objectModel->faces[faceIndex]).id1;
        std::size_t nVert2 = (objectModel->faces[faceIndex]).id2;
        std::size_t nVert3 = (objectModel->faces[faceIndex]).id3;
        //std::cout << "***INDEX: "<< faceIndex << "/" << objectModel->faces.size() << std::endl;
        Eigen::Vector3f Vert1 = objectModel->vertices[nVert1];
        Eigen::Vector3f Vert2 = objectModel->vertices[nVert2];
        Eigen::Vector3f Vert3 = objectModel->vertices[nVert3];
        intersect = RayIntersectsTriangle(rayOrigin, rayVector, Vert1, Vert2, Vert3, rho);
        //std::cout << "***INTERSECT?: " << intersect << std::endl;
        if (intersect == true)
        {
            scoords.r = rho;
        }
    }
    std::cout << "***RHO: " << scoords.r << std::endl;
    std::cout << "***THETA: " << scoords.theta << std::endl;
    std::cout << "***PHI: " << scoords.phi << std::endl;
    
    Eigen::Vector3f xyz = VirtualRobot::MathTools::toPosition(scoords);
    std::cout << "***POSITION: " << xyz << std::endl;
    
    
    //EULER
    /*
    // Euler ZXZ to Matrix conversion
    Eigen::Matrix3f m;
    float psi = query[GRASP_VAR::ROT_ROLL];
    float theta_ = query[GRASP_VAR::ROT_PITCH];
    float phi_ = query[GRASP_VAR::ROT_YAW];
    float sa, ca, sb, cb, sc, cc;

    sa = sin(psi);
    ca = cos(psi);
    sb = sin(theta_);
    cb = cos(theta_);
    sc = sin(phi_);
    cc = cos(phi_);

    m(1, 1) = ca * cc - sa * cb * sc;
    m(1, 2) = -ca * sc - sa * cb * cc;
    m(1, 3) = sa * sb;

    m(2, 1) = sa * cc + ca * cb * sc;
    m(2, 2) = -sa * sc + ca * cb * cc;
    m(2, 3) = -ca * sb;

    m(3, 1) = sb * sc;
    m(3, 2) = sb * cc;
    m(3, 3) = cb;
    
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen3f2rpy(m);
    */

    //rpy
    Eigen::Vector3f rpy(query[GRASP_VAR::ROT_ROLL], query[GRASP_VAR::ROT_PITCH], query[GRASP_VAR::ROT_YAW]);

    // 2. Execute grasp

    return executeGrasp(xyz, rpy);
}

GraspResult GraspPlanner::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {

    // 1. Open and Move 
    
    openEE();
    moveEE(xyz, rpy);

    // 2. Check Collisions
    
    GraspData grasp;
    grasp.pos = xyz;
    grasp.ori = rpy;
    if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
        std::cout << "Error: Collision detected!" << std::endl;
        grasp.result = GraspResult();
        if (save_grasp) grasps.push_back(grasp);
        return grasp.result;
    }

    // 3. Close EE
    closeEE();

    // 4. Evaluate grasp
    grasp.result = graspQuality();

    if (save_grasp) {
        grasps.push_back(grasp);
        std::cout << "Grasp " << grasps.size() << ":\n";
    }

    std::cout << "Grasp Quality (epsilon measure):" << grasp.result.measure << std::endl;
    std::cout << "v measure:" << grasp.result.volume << std::endl;
    //std::cout << "Force closure: " << (grasp.result.force_closure ? "yes" : "no") << std::endl;

    return grasp.result;
}

/// Grasping

void GraspPlanner::moveEE(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
    float x[6];
    x[0] = xyz.x();
    x[1] = xyz.y();
    x[2] = xyz.z();
    x[3] = rpy.x();
    x[4] = rpy.y();
    x[5] = rpy.z();

    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);

    // m = eefCloned->getGlobalPose() * m;
    eefCloned->setGlobalPose(m);
}

GraspResult GraspPlanner::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return GraspResult(epsilon, volume, fc);;
    }

    std::cout << "GraspQuality: not contacts!!!\n";

    return GraspResult();
}

void GraspPlanner::closeEE()
{
    contacts.clear();
    
    contacts = eef->closeActors(object);
}

void GraspPlanner::openEE()
{
    contacts.clear();

    if (!params.preshape.empty())
    {
        eef->setPreshape(params.preshape);
    }
    else
    {
        eef->openActors();
    }
}

bool GraspPlanner::RayIntersectsTriangle(Eigen::Vector3f rayOrigin, Eigen::Vector3f rayVector, 
                                                            Eigen::Vector3f vertex0, Eigen::Vector3f vertex1, Eigen::Vector3f vertex2, 
                                                            float& rho)
    {
        const float EPSILON = 0.0000001;
        Eigen::Vector3f edge1, edge2, h, s, q;
        float a,f,u,v;
        edge1 = vertex1 - vertex0;
        edge2 = vertex2 - vertex0;
        h = rayVector.cross(edge2);
        a = edge1.dot(h);
        if (a > -EPSILON && a < EPSILON)
        {
            return false;    // This ray is parallel to this triangle.
        }
        f = 1.0/a;
        s = rayOrigin - vertex0;
        u = f * s.dot(h);
        if (u < 0.0 || u > 1.0)
        {
            return false;
        }
        q = s.cross(edge1);
        v = f * rayVector.dot(q);
        float t = f * edge2.dot(q);
        if (v < 0.0 || u + v > 1.0)
        {
            return false;
        }
        // At this stage we can compute t to find out where the intersection point is on the line.
        if (t > EPSILON) // ray intersection
        {
            //outIntersectionPoint = rayOrigin + rayVector * t;
            rho = t;
            return true;
        }
        else // This means that there is a line intersection but not a ray intersection.
        {
            return false;
        }    
    }

}

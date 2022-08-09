#include "../include/Grasp/GraspPlannerS.hpp"

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

#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>
#include <GraspPlanning/GraspPlanner/GenericGraspPlanner.h>
#include <GraspPlanning/ApproachMovementSurfaceNormal.h>

#include "../include/Grasp/GraspVars.hpp"

namespace Grasp {

/// Init

GraspPlannerS::GraspPlannerS(const GraspPlannerSParams& params)
: params(params)
{
    loadScene();
}

GraspPlannerS::GraspPlanner(const std::string& json_file)
{
    if (!load_GraspPlannerParams_json(json_file, this->params)) {
        throw "Errpr creating GraspPlanner from json";
    }

    loadScene();
}

void GraspPlannerS::loadScene() {
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
    TCP = eef->getTcp();

    /*
    if (params.has_eef_pose) {
        moveEE(params.eef_position, params.eef_orientation);
    }
    */
    Eigen::Vector3f pos_ini = {0,0,0};
    Eigen::Vector3f or_ini = {0,0,0};
    moveEE(pos_ini, or_ini);
    wOrigin = eef->getTcp()->getGlobalPose();

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
        //x[1] = 100;
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

GraspResult GraspPlannerS::executeQueryGrasp(const std::vector<double>& query) {
    if (query.size() != NUM_GRASP_VARS) {
        std::cerr << "Error: query size is different of " << NUM_GRASP_VARS << "!!!\n";
        exit(1);
    }
    objectModel = object->getCollisionModel()->getTriMeshModel();
    
    // SPHERICAL:

    VirtualRobot::MathTools::SphericalCoord scoords;

    scoords.r = query[GRASP_VAR::TRANS_RHO];
    scoords.theta = query[GRASP_VAR::TRANS_THETA];
    scoords.phi = query[GRASP_VAR::TRANS_PHI];
    

    Eigen::Vector3f xyz = VirtualRobot::MathTools::toPosition(scoords); 

    //std::cout << "***POSE OBJECT: " << poseObject.block(0,3,3,1) << std::endl;
    //std::cout << "***POSE POINT: " << xyz << std::endl;

    //rpy
    Eigen::Vector3f rpy(query[GRASP_VAR::ROT_ROLL], query[GRASP_VAR::ROT_PITCH], query[GRASP_VAR::ROT_YAW]);

    // 2. Execute grasp
    /*
    openEE();
    moveEE(xyz, rpy);
    std::cout << "*** Moving away *** " << std::endl;
    GraspPlanning::ApproachMovementSurfaceNormal::moveEEFAway(rayVector, 1.0f, 50);
    */

    return executeGrasp(xyz, rpy);
}

GraspResult GraspPlannerS::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {

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

void GraspPlannerS::moveEE(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
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
    //eefCloned->setGlobalPose(m);
    //TCP->setGlobalPose(m);
    eefCloned->setGlobalPoseForRobotNode(TCP, m);
}

GraspResult GraspPlannerS::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return GraspResult(epsilon, volume, fc, comp_rho);;
    }

    std::cout << "GraspQuality: not contacts!!!\n";

    return GraspResult();
}

void GraspPlannerS::closeEE()
{
    contacts.clear();
    
    contacts = eef->closeActors(object);
}

void GraspPlannerS::openEE()
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

bool GraspPlannerS::RayIntersectsTriangle(Eigen::Vector3f rayOrigin, Eigen::Vector3f rayVector, 
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

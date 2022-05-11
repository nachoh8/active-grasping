#include "GraspPlannerIK.hpp"

#include <vector>

#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>

#include <MotionPlanning/Planner/GraspIkRrt.h>
#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <MotionPlanning/PostProcessing/ShortcutProcessor.h>

/// INIT

GraspPlannerIK::GraspPlannerIK(const std::string& sceneFile, const std::string& reachFile, const std::string& rns,
                const std::string& eef, const std::string& colModel, const std::string& colModelRob)
{

    this->eefName = eef;
    this->rnsName = rns;
    this->colModelName = colModel;
    this->colModelNameRob = colModelRob;

    loadScene(sceneFile);

    loadReach(reachFile);

    /// Set quality measure

    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();
}

void GraspPlannerIK::loadScene(const std::string& sceneFile)
{
    VirtualRobot::ScenePtr scene = VirtualRobot::SceneIO::loadScene(sceneFile);

    if (!scene)
    {
        VR_ERROR << " no scene ..." << std::endl;
        exit(1);
    }

    std::vector< VirtualRobot::RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "Need exactly 1 robot" << std::endl;
        exit(1);
    }

    robot = robots[0];


    std::vector< VirtualRobot::ManipulationObjectPtr > objects = scene->getManipulationObjects();

    if (objects.size() != 1)
    {
        VR_ERROR << "Need exactly 1 manipulation object" << std::endl;
        exit(1);
    }

    object = objects[0];
    VR_INFO << "using first manipulation object: " << object->getName() << std::endl;


    obstacles = scene->getObstacles();
    
    eef = robot->getEndEffector(eefName);

    if (!eef)
    {
        VR_ERROR << "Need a correct EEF in robot" << std::endl;
        exit(1);
    }

    // graspSet = object->getGraspSet(eef);
    graspSet.reset(new VirtualRobot::GraspSet(eefName, robot->getType(), eefName));

    rns = robot->getRobotNodeSet(rnsName);

    if (!rns)
    {
        VR_ERROR << "Need a correct robot node set in robot" << std::endl;
        exit(1);
    }

    rns->getJointValues(startConfig);
}

void GraspPlannerIK::loadReach(const std::string& reachFile)
{
    std::cout << "Loading Reachability from " << reachFile << std::endl;
    reachSpace.reset(new VirtualRobot::Reachability(robot));

    try
    {
        reachSpace->load(reachFile);
        // VirtualRobot::GraspSetPtr rg = reachSpace->getReachableGrasps(object->getGraspSet(eef), object);
        // graspSet->removeAllGrasps();
        // graspSet->addGrasp(rg->getGrasps()[0]);
        
    }
    catch (VirtualRobot::VirtualRobotException& e)
    {
        std::cout << " ERROR while loading reach space" << std::endl;
        std::cout << e.what();
        reachSpace.reset();
        return;
    }

    reachSpace->print();
}

/// Public

Grasp::GraspResult GraspPlannerIK::executeQueryGrasp(const std::vector<double>& query) {
    return Grasp::GraspResult();
}

Grasp::GraspResult GraspPlannerIK::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp) {
    /// 1. pos, rpy -> pose matrix
    float x[6];
    x[0] = xyz.x();
    x[1] = xyz.y();
    x[2] = xyz.z();
    x[3] = rpy.x();
    x[4] = rpy.y();
    x[5] = rpy.z();

    Eigen::Matrix4f wTq; // wTq
    VirtualRobot::MathTools::posrpy2eigen4f(x, wTq);
    // VirtualRobot::RobotNodePtr tcp = eef->getTcp();
    // m = tcp->getGlobalPose() * m; // if m was relative traslationm, rotation

    /// 2. Open and Move EEF
    reset();
    
    VirtualRobot::RobotNodePtr tcp = eef->getTcp();
    Eigen::Matrix4f wTtcp_o = tcp->getGlobalPose();
    robot->setGlobalPoseForRobotNode(tcp, wTq);

    Eigen::Matrix4f wTobj = object->getGlobalPose();
    Eigen::Matrix4f tcpTobj = tcp->toLocalCoordinateSystem(wTobj);

    VirtualRobot::GraspPtr queryGrasp(new VirtualRobot::Grasp("Query Grasp", robot->getType(), eef->getName(), tcpTobj));
    
    robot->setGlobalPoseForRobotNode(tcp, wTtcp_o); // set orig pose

    if (plan(queryGrasp)) { // measure grasp quality
        // closeEEF();

        Grasp::GraspResult result = graspQuality();

        std::cout << "Grasp Quality (epsilon measure):" << result.measure << std::endl;
        std::cout << "v measure:" << result.volume << std::endl;
        std::cout << "Force closure: " << (result.force_closure ? "yes" : "no") << std::endl;

        return result;
    }

    return Grasp::GraspResult();
}

/// Grasping

bool GraspPlannerIK::plan(VirtualRobot::GraspPtr targetGrasp) {
    /// 1. IKSolver setup
    VirtualRobot::GenericIKSolverPtr ikSolver(new VirtualRobot::GenericIKSolver(rns));

    // set reachability
    if (reachSpace) {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // set collision detection
    VirtualRobot::CDManagerPtr cdm;
    cdm.reset(new VirtualRobot::CDManager());
    /*
    VirtualRobot::SceneObjectSetPtr colModelSet = robot->getRobotNodeSet(colModelName);
    VirtualRobot::SceneObjectSetPtr colModelSet2;

    if (!colModelNameRob.empty())
    {
        colModelSet2 = robot->getRobotNodeSet(colModelNameRob);
    }

    if (colModelSet)
    {
        cdm->addCollisionModel(object);
        cdm->addCollisionModel(colModelSet);

        if (colModelSet2)
        {
            cdm->addCollisionModel(colModelSet2);
        }

        ikSolver->collisionDetection(cdm);
    }
    */
    // set params
    ikSolver->setMaximumError(5.0f, 0.04f);
    ikSolver->setupJacobian(0.9f, 20);

    /// 2. Cspace setup
    cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns));

    /// 3. GraspIKRRT setup
    
    // create Grasp set
    VirtualRobot::GraspSetPtr _graspSet (new VirtualRobot::GraspSet(eefName, robot->getType(), eefName));
    // graspSet.reset(new VirtualRobot::GraspSet(eefName, robot->getType(), eefName));
    _graspSet->addGrasp(targetGrasp);

    // set grasp planner
    // std::cout << "Grasp reachable:" << std::endl;
    // VirtualRobot::GraspPtr rg = graspSet->getGrasps()[0];
    // std::cout << rg->getTransformation() << std::endl;
    // std::cout << "Grasp target:" << std::endl;
    // std::cout << targetGrasp->getTransformation() << std::endl;

    Saba::GraspIkRrtPtr ikRrt(new Saba::GraspIkRrt(cspace, object, ikSolver, _graspSet, 0.9999f));
    ikRrt->setStart(startConfig);
    
    bool planOK = ikRrt->plan();
    VR_INFO << " Planning success: " << planOK << std::endl;
    if (planOK) { // optimize path
        solution = ikRrt->getSolution();
        Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = ikRrt->getTree();
        tree2 = ikRrt->getTree2();

        graspSet->addGrasp(targetGrasp);
    } else {
        solution.reset();
        solutionOptimized.reset();
        tree.reset();
        tree2.reset();
    }

    return planOK;
}

void GraspPlannerIK::closeEEF()
{
    contacts.clear();

    contacts = eef->closeActors(object);
}

void GraspPlannerIK::openEEF()
{
    contacts.clear();

    eef->openActors();
}

Grasp::GraspResult GraspPlannerIK::graspQuality() {
    if (contacts.size() > 0) {
        qualityMeasure->setContactPoints(contacts);

        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();

        return Grasp::GraspResult(epsilon, volume, fc);;
    }

    std::cout << "GraspQuality: not contacts!!!\n";

    return Grasp::GraspResult();
}

/// OTHERS

void GraspPlannerIK::reset() {
    openEEF();
    robot->setJointValues(rns, startConfig);
}

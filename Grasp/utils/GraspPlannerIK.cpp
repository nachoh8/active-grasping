#include "GraspPlannerIK.hpp"

#include <vector>

#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>

#include <MotionPlanning/Planner/GraspIkRrt.h>
#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <MotionPlanning/PostProcessing/ShortcutProcessor.h>

inline void poseError(const Eigen::Matrix4f& currentPose, const Eigen::Matrix4f& targetPose, float& posError, float& oriError) {
    posError = (currentPose.block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
    
    VirtualRobot::MathTools::Quaternion q1 = VirtualRobot::MathTools::eigen4f2quat(currentPose);
    VirtualRobot::MathTools::Quaternion q2 = VirtualRobot::MathTools::eigen4f2quat(targetPose);
    VirtualRobot::MathTools::Quaternion d = getDelta(q1, q2);
    oriError = fabs(180.0f - (d.w + 1.0f) * 90.0f);
}


/// INIT

GraspPlannerIK::GraspPlannerIK(const std::string& sceneFile, const std::string& reachFile, const std::string& rns,
                const std::string& eef, const std::string& colModel, const std::string& colModelRob)
{

    this->eefName = eef;
    this->rnsName = rns;
    this->colModelName = colModel;
    this->colModelNameRob = colModelRob;

    useCollision = true;
    useReachability = false;
    useOnlyPosition = false;

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

Grasp::GraspResult GraspPlannerIK::executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy) {
    /// pos, rpy -> pose matrix
    float x[6];
    x[0] = xyz.x();
    x[1] = xyz.y();
    x[2] = xyz.z();
    x[3] = rpy.x();
    x[4] = rpy.y();
    x[5] = rpy.z();

    Eigen::Matrix4f targetPose;
    VirtualRobot::MathTools::posrpy2eigen4f(x, targetPose);

    return executeGrasp(targetPose);
}

Grasp::GraspResult GraspPlannerIK::executeGrasp(const Eigen::Matrix4f& targetPose) {
    /// 1. To start config
    reset();

    /// 2. Plan and execute
    if (plan(targetPose)) {
        /// 3. Set final config
        Eigen::VectorXf finalPos;
        birrtSolOptimized->interpolate(1, finalPos);
        robot->setJointValues(rns, finalPos);

        /*float posError, oriError;
        poseError(eef->getTcp()->getGlobalPose(), targetPose, posError, oriError);

        std::cout << "BiRRT Error pos: " << posError << std::endl;
        std::cout << "BiRRT Error ori: " << oriError << std::endl;*/

        /// 4. Measure grasp quality
        if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), eef->createSceneObjectSet())) {
            std::cout << "Error: EEF Collision detected!" << std::endl;
            reset();
            return Grasp::GraspResult();
        }

        closeEEF();
        Grasp::GraspResult result = graspQuality();

        std::cout << "Grasp Quality (epsilon measure):" << result.measure << std::endl;
        std::cout << "Volume measure:" << result.volume << std::endl;
        std::cout << "Force closure: " << (result.force_closure ? "yes" : "no") << std::endl;

        return result;
    }

    reset();

    return Grasp::GraspResult();
}

/// Grasping

bool GraspPlannerIK::plan(Eigen::Matrix4f targetPose) {
    /// 1. IKSolver setup
    VirtualRobot::GenericIKSolverPtr ikSolver(new VirtualRobot::GenericIKSolver(rns));

    // set reachability
    if (useReachability && reachSpace) {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // set collision detection
    VirtualRobot::CDManagerPtr cdm;
    cdm.reset(new VirtualRobot::CDManager());
    
    if (useCollision) {
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
    }
    
    // set params
    ikSolver->setMaximumError(ikMaxErrorPos, ikMaxErrorOri);
    ikSolver->setupJacobian(ikJacobianStepSize, ikJacobianMaxLoops);

    VirtualRobot::IKSolver::CartesianSelection selection = useOnlyPosition ? VirtualRobot::IKSolver::Position : VirtualRobot::IKSolver::All;

    /// 2. Solve IK
    bool planOK = ikSolver->solve(targetPose, selection, ikMaxLoops);
    std::cout << "IK Solver success: " << planOK << std::endl;
    if (!planOK) {
        return false;
    }

    Eigen::Matrix4f actPose = eef->getTcp()->getGlobalPose();
    float posError, oriError;
    poseError(actPose, targetPose, posError, oriError);

    std::cout << "IK Solver Error pos: " << posError << std::endl;
    std::cout << "IK Solver Error ori: " << oriError << std::endl;

    /// 3. Get Goal config and reset
    Eigen::VectorXf goalConfig;
    rns->getJointValues(goalConfig);
    reset();

    /// 4. BiRRT setup
    cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns, 1000000));
    cspace->setSamplingSize(cspacePathStepSize);
    cspace->setSamplingSizeDCD(cspaceColStepSize);

    Saba::BiRrtPtr rrt(new Saba::BiRrt(cspace, Saba::Rrt::eExtend, Saba::Rrt::eConnect));
    rrt->setStart(startConfig);
    rrt->setGoal(goalConfig);

    /// 5. Execute and postprocessing
    planOK = rrt->plan(true);
    std::cout << "BiRRT success: " << planOK << std::endl;
    std::cout << "BiRRT time: " << rrt->getPlanningTimeMS() << " ms" << std::endl;
    if (!planOK) {
        return false;
    }

    birrtSolution = rrt->getSolution();
    Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(birrtSolution, cspace, false));
    birrtSolOptimized = postProcessing->optimize(optOptimzeStep);

    return true;
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

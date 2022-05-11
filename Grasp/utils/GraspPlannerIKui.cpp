
#include "GraspPlannerIKui.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "MotionPlanning/Planner/GraspIkRrt.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <cmath>
#include <QImage>
#include <QGLWidget>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

/// INIT

GraspPlannerIKui::GraspPlannerIKui(std::string& sceneFile, std::string& reachFile, std::string& rns, std::string& eef, std::string& colModel, std::string& colModelRob)
    : QMainWindow(nullptr), GraspPlannerIK(sceneFile, reachFile, rns, eef, colModel, colModelRob)
{

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    graspsSep = new SoSeparator;
    reachableGraspsSep = new SoSeparator;
    reachabilitySep = new SoSeparator;
    obstaclesSep = new SoSeparator;
    rrtSep = new SoSeparator;

    playbackMode = false;

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(graspsSep);
    sceneSep->addChild(reachableGraspsSep);
    sceneSep->addChild(reachabilitySep);
    sceneSep->addChild(obstaclesSep);
    sceneSep->addChild(rrtSep);

    setupUI();

    buildVisu();

    targetPoseBox = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    targetPoseBox->showCoordinateSystem(true);
    targetPoseBoxSep = new SoSeparator();
    targetPoseBoxSep->addChild(CoinVisualization(targetPoseBox->getVisualization()).getCoinVisualization());
    sceneSep->addChild(targetPoseBoxSep);
    box2TCP();

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}

/// EEF

void GraspPlannerIKui::closeEEF()
{
    GraspPlannerIK::closeEEF();

    redraw();

}

void GraspPlannerIKui::openEEF()
{
    GraspPlannerIK::openEEF();

    redraw();

}

void GraspPlannerIKui::closeEEFbtn()
{
    closeEEF();

    Grasp::GraspResult result = graspQuality();

    std::cout << "Grasp Quality (epsilon measure):" << result.measure << std::endl;
    std::cout << "v measure:" << result.volume << std::endl;
    std::cout << "Force closure: " << (result.force_closure ? "yes" : "no") << std::endl;
}

void GraspPlannerIKui::openEEFbtn()
{
    openEEF();
}

/// UI

GraspPlannerIKui::~GraspPlannerIKui()
{
    sceneSep->unref();
}


void GraspPlannerIKui::timerCB(void* data, SoSensor* /*sensor*/)
{
    GraspPlannerIKui* ikWindow = static_cast<GraspPlannerIKui*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3] = (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        ikWindow->updateObject(x);
        ikWindow->redraw();
    }

    int maxSlider = 200;

    if (ikWindow->playbackMode && ikWindow->playCounter <= maxSlider)
    {
        if (ikWindow->playCounter == 0)
        {
            ikWindow->openEEF();
            ikWindow->sliderSolution(0);
            ikWindow->playCounter++;
        }
        else if (ikWindow->playCounter == maxSlider)
        {
            ikWindow->sliderSolution(1000);
            ikWindow->closeEEF();
            std::cout << "Stopping playback" << std::endl;
            ikWindow->playbackMode = false;
        }
        else
        {
            ikWindow->playCounter++;
            float pos = (float)ikWindow->playCounter / (float)maxSlider;
            ikWindow->sliderSolution((int)(pos * 1000.0f));
        }

        ikWindow->redraw();
        ikWindow->saveScreenshot();
    }
}


void GraspPlannerIKui::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(false);
    viewer->setAntialiasing(true, 4);
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(false);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEFbtn()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEFbtn()));

    connect(UI.checkBoxSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachableGrasps, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachabilitySpace, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.pushButtonIKRRT, SIGNAL(clicked()), this, SLOT(planIKRRT()));
    connect(UI.pushButtonPlay, SIGNAL(clicked()), this, SLOT(playAndSave()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
    connect(UI.horizontalSliderSolution, SIGNAL(valueChanged(int)), this, SLOT(sliderSolution(int)));

    UI.checkBoxColCheckIK->setChecked(false);
    UI.checkBoxReachabilitySpaceIK->setChecked(false);

}

void GraspPlannerIKui::playAndSave()
{
    if (playbackMode)
    {
        playbackMode = false;
    }
    else
    {
        playCounter = 0;
        playbackMode = true;
    }
}

QString GraspPlannerIKui::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void GraspPlannerIKui::resetSceneryAll()
{
    GraspPlannerIK::reset();
}


void GraspPlannerIKui::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void GraspPlannerIKui::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("renderFrame_%06d.png", counter);
    counter++;
    redraw();
    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "BMP");

    if (bRes)
    {
        std::cout << "wrote image " << counter << std::endl;
    }
    else
    {
        std::cout << "failed writing image " << counter << std::endl;
    }

}

void GraspPlannerIKui::buildVisu()
{
    showCoordSystem();

    robotSep->removeAllChildren();
    //bool colModel = (UI.checkBoxColModel->isChecked());
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }

    objectSep->removeAllChildren();

    if (object)
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }
    }

    obstaclesSep->removeAllChildren();

    if (obstacles.size() > 0)
    {
        for (const auto & obstacle : obstacles)
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(obstacle, colModel);

            if (visualisationNode)
            {
                obstaclesSep->addChild(visualisationNode);
            }
        }
    }

    buildGraspSetVisu();

    buildRRTVisu();

    redraw();
}

int GraspPlannerIKui::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GraspPlannerIKui::quit()
{
    std::cout << "IKRRTWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}


void GraspPlannerIKui::updateObject(float x[6])
{
    /*if (object)
    {
        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << std::endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << std::endl;
        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
        std::cout << "object " << std::endl;
        std::cout << m << std::endl;

    }*/

    if (targetPoseBox)
    {
        /*std::cout << "TCP" << std::endl;
        std::cout << eef->getTcp()->getGlobalPose() << std::endl;

        std::cout << "Grasp reachable:" << std::endl;
        VirtualRobot::GraspPtr rg = graspSet->getGrasps()[0];
        std::cout << rg->getTransformation() << std::endl;

        Eigen::Matrix4f xx = rg->getTcpPoseGlobal(object->getGlobalPose());
        
        Eigen::Vector3f xyz = xx.block<3,1>(0,3);
        Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(xx);
        std::cout << xyz.transpose() << std::endl;
        std::cout << rpy.transpose() << std::endl;*/

        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << std::endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << std::endl;
        //std::cout << "Grasp target:" << std::endl;

        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = targetPoseBox->getGlobalPose() * m;
        targetPoseBox->setGlobalPose(m);

        /*std::cout << "global pose" << std::endl;
        xyz = m.block<3,1>(0,3);
        rpy = VirtualRobot::MathTools::eigen4f2rpy(m);
        std::cout << xyz.transpose() << std::endl;
        std::cout << rpy.transpose() << std::endl;

        VirtualRobot::RobotNodePtr tcp = eef->getTcp();
        Eigen::Matrix4f wTtcp_o = tcp->getGlobalPose();
        robot->setGlobalPoseForRobotNode(tcp, m);

        Eigen::Matrix4f wTobj = object->getGlobalPose();
        Eigen::Matrix4f tcpTobj = tcp->toLocalCoordinateSystem(wTobj);

        VirtualRobot::GraspPtr targetGrasp(new VirtualRobot::Grasp("Query Grasp", robot->getType(), eef->getName(), tcpTobj));

        std::cout << "grasp target transformation" << std::endl;
        std::cout << targetGrasp->getTransformation() << std::endl;
        
        xx = targetGrasp->getTcpPoseGlobal(object->getGlobalPose());
        xyz = xx.block<3,1>(0,3);
        rpy = VirtualRobot::MathTools::eigen4f2rpy(xx);
        std::cout << xyz.transpose() << std::endl;
        std::cout << rpy.transpose() << std::endl;

        robot->setGlobalPoseForRobotNode(tcp, wTtcp_o);*/
    }

    redraw();

}

void GraspPlannerIKui::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
    buildVisu();
}

void GraspPlannerIKui::showCoordSystem()
{
    if (eef)
    {
        RobotNodePtr tcp = eef->getTcp();

        if (!tcp)
        {
            return;
        }

        tcp->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }

    if (object)
    {
        object->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }
}

void GraspPlannerIKui::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!UI.checkBoxSolution->isChecked())
    {
        return;
    }

    if (!solution)
    {
        return;
    }

    std::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, eef->getTcpName()));

    if (tree)
    {
        w->addTree(tree);
    }

    if (tree2)
    {
        w->addTree(tree2);
    }

    //w->addCSpacePath(solution);
    if (solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    //w->addConfiguration(startConfig,Saba::CoinRrtWorkspaceVisualization::eGreen,3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

void GraspPlannerIKui::buildGraspSetVisu()
{
    graspsSep->removeAllChildren();

    if (UI.checkBoxGraspSet->isChecked() && eef && graspSet && object)
    {
        SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(graspSet, eef, object->getGlobalPose());

        if (visu)
        {
            graspsSep->addChild(visu);
        }
    }

    // show reachable graps
    reachableGraspsSep->removeAllChildren();

    if (UI.checkBoxReachableGrasps->isChecked() && eef && graspSet && object && reachSpace)
    {
        GraspSetPtr rg = reachSpace->getReachableGrasps(graspSet, object);

        if (rg->getSize() > 0)
        {
            SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(rg, eef, object->getGlobalPose());

            if (visu)
            {
                reachableGraspsSep->addChild(visu);
            }
        }
    }
}


void GraspPlannerIKui::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    reachabilitySep->removeAllChildren();

    if (UI.checkBoxReachabilitySpace->checkState() == Qt::Checked)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace, VirtualRobot::ColorMap::eRed, true);

        if (visualisationNode)
        {
            reachabilitySep->addChild(visualisationNode);
        }

        /*
            VirtualRobot::CoinVisualizationPtr visualization = reachSpace->getVisualization<CoinVisualization>();
            SoNode* visualisationNode = NULL;
            if (visualization)
                visualisationNode = visualization->getCoinVisualization();

            if (visualisationNode)
                reachabilitySep->addChild(visualisationNode);
        */
    }
}

void GraspPlannerIKui::planIKRRT()
{
    Eigen::Matrix4f m = targetPoseBox->getGlobalPose();
    Eigen::Vector3f xyz = m.block<3,1>(0,3);
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(m);
    
    std::cout << "Execute grasp\n";
    std::cout << xyz.transpose() << std::endl;
    std::cout << rpy.transpose() << std::endl;

    // Eigen::Vector3f xyz = targetPoseBox->getGlobalPosition();
    // Eigen::Vector3f rpy = targetPoseBox->getGlobalOrientation().eulerAngles(0, 1, 2);

    executeGrasp(xyz, rpy);

    sliderSolution(1000);

    buildVisu();
}

void GraspPlannerIKui::colModel()
{
    buildVisu();
}

void GraspPlannerIKui::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    Saba::CSpacePathPtr s = solution;

    if (solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);

    redraw();
    //saveScreenshot();
}

void GraspPlannerIKui::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}

void GraspPlannerIKui::box2TCP()
{
    if (!eef) return;

    RobotNodePtr tcp = eef->getTcp();
    if (!tcp || !targetPoseBox)
    {
        return;
    }

    // Eigen::Matrix4f m = tcp->getGlobalPose();
    float x[6];
    x[0] = -246;
    x[1] = -93;
    x[2] = 664;
    x[3] = -1.53;
    x[4] = -0.034;
    x[5] = 2.35;

    Eigen::Matrix4f m;
    VirtualRobot::MathTools::posrpy2eigen4f(x, m);
    targetPoseBox->setGlobalPose(m);
    viewer->render();
}

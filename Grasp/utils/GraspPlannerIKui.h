
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>

#include "MotionPlanning/Saba.h"
#include "MotionPlanning/CSpace/CSpacePath.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "GraspPlannerIK.hpp"
#include "GraspPlannerIKParams.hpp"

#include "ui_IKRRT.h"

class GraspPlannerIKui : public QMainWindow, public GraspPlannerIK
{
    Q_OBJECT
public:
    GraspPlannerIKui(const GraspPlannerIKParams& params);
    ~GraspPlannerIKui() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();

    void colModel();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();
    void sliderReleased_ObjectA();
    void sliderReleased_ObjectB();
    void sliderReleased_ObjectG();
    void sliderSolution(int pos);

    void buildVisu();

    void showCoordSystem();
    void reachVisu();

    void planIKRRT();

    void closeEEFbtn();

    void openEEFbtn();

    void box2TCP();

protected:

    /// GRASP

    void closeEEF();

    void openEEF();

    /// UI

    void setupUI();
    QString formatString(const char* s, float f);

    void buildGraspSetVisu();

    void buildRRTVisu();

    void updateObject(float x[6]);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();

    Ui::MainWindowIKRRT UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* graspsSep;
    SoSeparator* reachableGraspsSep;
    SoSeparator* reachabilitySep;
    SoSeparator* obstaclesSep;
    SoSeparator* rrtSep;
    SoSeparator* targetPoseBoxSep;

    bool playbackMode;
    int playCounter;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

    VirtualRobot::ObstaclePtr targetPoseBox;
};


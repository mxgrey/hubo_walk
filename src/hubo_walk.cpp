/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 15, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#include "hubo_walk.h"



namespace hubo_walk_space
{

HuboWalkPanel::HuboWalkPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new HuboWalkWidget;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

HuboWalkWidget::~HuboWalkWidget()
{
    refreshManager->alive = false;
    refreshManager->terminate();
}

void HuboWalkPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config); 
}

void HuboWalkPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

HuboWalkWidget::HuboWalkWidget(QWidget *parent)
    : QTabWidget(parent)
{

    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";

    //initializeAchStructs();

    initializeCommandTab();
    std::cerr << "Command Tab loaded" << std::endl;
    
    initializeZmpParamTab();
    std::cerr << "ZMP Parameters Tab loaded" << std::endl;
    
    addTab(commandTab, "Command");
    addTab(zmpParamTab, "ZMP Parameters");

//    initializeAchConnections();

//    refreshManager = new HuboRefreshManager;
//    refreshManager->parentWidget = this;
//    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
//    refreshManager->start();
}

void HuboRefreshManager::run()
{
    alive = true;
    waitTime = 1000;
    connect(this, SIGNAL(signalRefresh()), parentWidget, SLOT(refreshState()));
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

void HuboRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}

void HuboWalkWidget::achCreateCatch(QProcess::ProcessError err)
{
    ROS_INFO("Creating Ach Channel Failed: Error Code %d", (int)err);
}

void HuboWalkWidget::initializeCommandTab()
{
    QSizePolicy pbsize(QSizePolicy::Maximum, QSizePolicy::Maximum);
    
    // Set up the networking box
    QVBoxLayout* achdLayout = new QVBoxLayout;

    achdConnect = new QPushButton;
    achdConnect->setSizePolicy(pbsize);
    achdConnect->setText("Connect");
    achdConnect->setToolTip("Connect to Hubo's on board computer");
    achdLayout->addWidget(achdConnect, 0, Qt::AlignHCenter | Qt::AlignBottom);
    connect(achdConnect, SIGNAL(clicked()), this, SLOT(achdConnectSlot()));
    
    achdDisconnect = new QPushButton;
    achdDisconnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdDisconnect->setText("Disconnect");
    achdDisconnect->setToolTip("Disconnect from Hubo's on board computer");
    achdLayout->addWidget(achdDisconnect, 0, Qt::AlignHCenter | Qt::AlignTop);
    connect(achdDisconnect, SIGNAL(clicked()), this, SLOT(achdDisconnectSlot()));

    QHBoxLayout* statusLayout = new QHBoxLayout;
    QLabel* staticLabel = new QLabel;
    staticLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    staticLabel->setText("Status: ");
    statusLayout->addWidget(staticLabel);
    statusLabel = new QLabel;
    statusLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    statusLabel->setText("Not Connected");
    statusLayout->addWidget(statusLabel);
    achdLayout->addLayout(statusLayout);

    QHBoxLayout* networkLayout = new QHBoxLayout;
    networkLayout->addLayout(achdLayout);

    QHBoxLayout* ipLayout = new QHBoxLayout;
    ipLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ipAddrAEdit = new QLineEdit;
    ipAddrAEdit->setMaxLength(3);
    ipAddrAEdit->setMaximumWidth(50);
    ipAddrAEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrAEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrAEdit);
    connect(ipAddrAEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot1 = new QLabel;
    dot1->setText(".");
    ipLayout->addWidget(dot1);
    ipAddrBEdit = new QLineEdit;
    ipAddrBEdit->setMaxLength(3);
    ipAddrBEdit->setMaximumWidth(50);
    ipAddrBEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrBEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrBEdit);
    connect(ipAddrBEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot2 = new QLabel;
    dot2->setText(".");
    ipLayout->addWidget(dot2);
    ipAddrCEdit = new QLineEdit;
    ipAddrCEdit->setMaxLength(3);
    ipAddrCEdit->setMaximumWidth(50);
    ipAddrCEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrCEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrCEdit);
    connect(ipAddrCEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot3 = new QLabel;
    dot3->setText(".");
    ipLayout->addWidget(dot3);
    ipAddrDEdit = new QLineEdit;
    ipAddrDEdit->setMaxLength(3);
    ipAddrDEdit->setMaximumWidth(50);
    ipAddrDEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrDEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrDEdit);
    connect(ipAddrDEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));

    QLabel* ipTitle = new QLabel;
    ipTitle->setText("IP Address");
    ipTitle->setToolTip("IP Address for Hubo's on board computer");

    QVBoxLayout* ipUpperLayout = new QVBoxLayout;
    ipUpperLayout->addWidget(ipTitle, 0, Qt::AlignLeft | Qt::AlignBottom);
    ipUpperLayout->addLayout(ipLayout);

    networkLayout->addLayout(ipUpperLayout);



    QGroupBox* networkBox = new QGroupBox;
    networkBox->setStyleSheet(groupStyleSheet);
    networkBox->setTitle("Ach Networking");
    networkBox->setLayout(networkLayout);

    setIPAddress(192, 168, 1, 0);
    ////////////////////
    
    QHBoxLayout* controlSelectLayout = new QHBoxLayout;
    radioSelectGroup = new QButtonGroup;
    radioSelectGroup->setExclusive(true);
    
    guiSelect = new QRadioButton;
    guiSelect->setText("GUI");
    guiSelect->setToolTip("Use the GUI below to control Hubo's walking");
    radioSelectGroup->addButton(guiSelect);
    controlSelectLayout->addWidget(guiSelect);
    guiSelect->setChecked(true);
    
    joySelect = new QRadioButton;
    joySelect->setText("Joystick");
    joySelect->setCheckable(false);
    joySelect->setToolTip("Use a handheld controller to control Hubo's walking\n"
                          "(The controller must be plugged into this computer)");
    radioSelectGroup->addButton(joySelect);
    controlSelectLayout->addWidget(joySelect);
    
    QVBoxLayout* joyLayout = new QVBoxLayout;
    joyLaunch = new QPushButton;
    joyLaunch->setSizePolicy(pbsize);
    joyLaunch->setText("Launch Joystick");
    joyLaunch->setToolTip("Begin a program which will read for joystick commands");
    joyLayout->addWidget(joyLaunch, 0, Qt::AlignBottom);
    connect(joyLaunch, SIGNAL(clicked()), this, SLOT(handleJoyLaunch()));
    
    QHBoxLayout* joyStatusLayout = new QHBoxLayout;
    QLabel* statlab = new QLabel;
    statlab->setText("Status:");
    joyStatusLayout->addWidget(statlab);
    joyStatus = new QLabel;
    joyStatus->setText("Off");
    joyStatusLayout->addWidget(joyStatus);
    std::cerr << "Up to addLayout" << std::endl;
    joyStatusLayout->setAlignment(Qt::AlignTop);
    joyLayout->addLayout(joyStatusLayout);
    
    controlSelectLayout->addLayout(joyLayout);
    
    QGroupBox* controlSelectBox = new QGroupBox;
    controlSelectBox->setTitle("Control Method");
    controlSelectBox->setStyleSheet(groupStyleSheet);
    controlSelectBox->setLayout(controlSelectLayout);
    
    
    QVBoxLayout* controlLayout = new QVBoxLayout;
    
    QHBoxLayout* paramLayout = new QHBoxLayout;
    QLabel* walkLab = new QLabel;
    walkLab->setText("Walk Distance:");
    walkLab->setToolTip("Distance to walk (m) after a click");
    paramLayout->addWidget(walkLab, 0, Qt::AlignRight);
    walkDistanceBox = new QDoubleSpinBox;
    walkDistanceBox->setSingleStep(0.5);
    walkDistanceBox->setToolTip(walkLab->toolTip());
    paramLayout->addWidget(walkDistanceBox, 0, Qt::AlignLeft);
    
    continuousBox = new QCheckBox;
    continuousBox->setChecked(false);
    continuousBox->setText("Continuous");
    continuousBox->setToolTip("Ignore the walk distance, and walk until Stop is selected");
    paramLayout->addWidget(continuousBox, 0, Qt::AlignLeft);
    connect(continuousBox, SIGNAL(toggled(bool)), this, SLOT(handleContinuous()));
    
    QLabel* turnLab = new QLabel;
    turnLab->setText("Turn Radius:");
    turnLab->setToolTip("Radius of curvature (m) for turning");
    paramLayout->addWidget(turnLab, 0, Qt::AlignRight);
    radiusBox = new QDoubleSpinBox;
    radiusBox->setMinimum(0);
    radiusBox->setSingleStep(0.1);
    radiusBox->setMaximum(10000);
    radiusBox->setToolTip(turnLab->toolTip());
    paramLayout->addWidget(radiusBox, 0, Qt::AlignLeft);
    
    controlLayout->addLayout(paramLayout);
    
    QGridLayout* wasdLayout = new QGridLayout;
    wasdLayout->setAlignment(Qt::AlignCenter);
    
    turnLeftButton = new QPushButton;
    turnLeftButton->setText("  Turn  ");
    turnLeftButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(turnLeftButton, 0, 0, 1, 1, Qt::AlignCenter);
    connect(turnLeftButton, SIGNAL(clicked()), this, SLOT(handleTurnLeft()));
    
    forwardButton = new QPushButton;
    forwardButton->setText("Forward ");
    forwardButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(forwardButton, 0, 1, 1, 1, Qt::AlignCenter);
    connect(forwardButton, SIGNAL(clicked()), this, SLOT(handleForward()));
    
    turnRightButton = new QPushButton;
    turnRightButton->setText("  Turn  ");
    turnRightButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(turnRightButton, 0, 2, 1, 1, Qt::AlignCenter);
    connect(turnRightButton, SIGNAL(clicked()), this, SLOT(handleTurnRight()));
    
    leftButton = new QPushButton;
    leftButton->setText("  Left  ");
    leftButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(leftButton, 1, 0, 1, 1, Qt::AlignCenter);
    connect(leftButton, SIGNAL(clicked()), this, SLOT(handleLeft()));
    
    stopButton = new QPushButton;
    stopButton->setText("  Stop  ");
    stopButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(stopButton, 1, 1, 1, 1, Qt::AlignCenter);
    connect(stopButton, SIGNAL(clicked()), this, SLOT(handleStop()));
    
    rightButton = new QPushButton;
    rightButton->setText(" Right  ");
    rightButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(rightButton, 1, 2, 1, 1, Qt::AlignCenter);
    connect(rightButton, SIGNAL(clicked()), this, SLOT(handleright()));
    
    
    backButton = new QPushButton;
    backButton->setText("Backward");
    backButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(backButton, 2, 1, 1, 1, Qt::AlignCenter);
    connect(backButton, SIGNAL(clicked()), this, SLOT(handleback()));
    
    controlLayout->addLayout(wasdLayout);
    
    
    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(networkBox);
    masterCTLayout->addWidget(controlSelectBox);
    masterCTLayout->addLayout(controlLayout);

    commandTab = new QWidget;
    commandTab->setLayout(masterCTLayout);
}

void HuboWalkWidget::initializeZmpParamTab()
{
    QSizePolicy pbsize(QSizePolicy::Maximum, QSizePolicy::Maximum);
    
    QHBoxLayout* profileLayoutTop = new QHBoxLayout;
    QLabel* profileLab = new QLabel;
    profileLab->setText("Profile:");
    profileLayoutTop->addWidget(profileLab, 0, Qt::AlignVCenter | Qt::AlignRight);
    
    profileSelect = new QComboBox;
    fillProfileSelect();
    profileLayoutTop->addWidget(profileSelect);
    
    saveProfile = new QPushButton;
    saveProfile->setSizePolicy(pbsize);
    saveProfile->setText("Save");
    saveProfile->setToolTip("Save the values below into the currently selected profile");
    profileLayoutTop->addWidget(saveProfile);
    connect(saveProfile, SIGNAL(clicked()), this, SLOT(handleProfileSave()));
    
    deleteProfile = new QPushButton;
    deleteProfile->setSizePolicy(pbsize);
    deleteProfile->setText("Delete");
    deleteProfile->setToolTip("Remove the current profile from the list\n"
                              "WARNING: This is permanent!");
    profileLayoutTop->addWidget(deleteProfile);
    connect(deleteProfile, SIGNAL(clicked()), this, SLOT(handleProfileDelete()));
    
    QHBoxLayout* profileLayoutBottom = new QHBoxLayout;
    saveAsProfile = new QPushButton;
    saveAsProfile->setSizePolicy(pbsize);
    saveAsProfile->setText("Save As...");
    saveAsProfile->setToolTip("Save the values below as a new profile with the following name:");
    profileLayoutBottom->addWidget(saveAsProfile);
    connect(saveAsProfile, SIGNAL(clicked()), this, SLOT(handleProfileSaveAs()));
    
    saveAsEdit = new QLineEdit;
    saveAsEdit->setToolTip("Enter a name for a new profile");
    profileLayoutBottom->addWidget(saveAsEdit);
    
    QVBoxLayout* profileLayoutMaster = new QVBoxLayout;
    profileLayoutMaster->addLayout(profileLayoutTop);
    profileLayoutMaster->addLayout(profileLayoutBottom);
    
    
    QVBoxLayout* leftColumn = new QVBoxLayout;
    
    QVBoxLayout* zmpSettingsLayout = new QVBoxLayout;
    zmpSettingsLayout->setAlignment(Qt::AlignCenter);
    
    QHBoxLayout* xoffsetLay = new QHBoxLayout;
    QLabel* xoffsetLab = new QLabel;
    xoffsetLab->setText("X-Offset:");
    xoffsetLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    xoffsetLab->setToolTip("How far in x should the ZMP be offset from the ankle joint?");
    xoffsetLay->addWidget(xoffsetLab);
    
    xOffsetBox = new QDoubleSpinBox;
    xOffsetBox->setSizePolicy(pbsize);
    xOffsetBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    xOffsetBox->setToolTip(xoffsetLab->toolTip());
    xOffsetBox->setSingleStep(0.01);
    xOffsetBox->setMinimum(-10);
    xOffsetBox->setMaximum(10);
    xoffsetLay->addWidget(xOffsetBox);
    
    zmpSettingsLayout->addLayout(xoffsetLay);
    
    QHBoxLayout* yoffsetLay = new QHBoxLayout;
    QLabel* yoffsetLab = new QLabel;
    yoffsetLab->setText("Y-Offset:");
    yoffsetLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    yoffsetLab->setToolTip("How far in y should the ZMP be offset from the ankle joint?");
    yoffsetLay->addWidget(yoffsetLab);
    
    yOffsetBox = new QDoubleSpinBox;
    yOffsetBox->setSizePolicy(pbsize);
    yOffsetBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    yOffsetBox->setToolTip(yoffsetLab->toolTip());
    yOffsetBox->setSingleStep(0.01);
    yOffsetBox->setMinimum(-10);
    yOffsetBox->setMaximum(10);
    yoffsetLay->addWidget(yOffsetBox);
    
    zmpSettingsLayout->addLayout(yoffsetLay);
    
    QHBoxLayout* jerkLayout = new QHBoxLayout;
    QLabel* jerkPenaltyLab = new QLabel;
    jerkPenaltyLab->setText("Jerk Penalty:");
    jerkPenaltyLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    jerkPenaltyLab->setToolTip("How much should jerk be penalized by the Preview Controller?");
    jerkLayout->addWidget(jerkPenaltyLab);
    
    jerkPenalBox = new QDoubleSpinBox;
    jerkPenalBox->setSizePolicy(pbsize);
    jerkPenalBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    jerkPenalBox->setToolTip(jerkPenaltyLab->toolTip());
    jerkPenalBox->setSingleStep(0.01);
    jerkPenalBox->setMinimum(-1000);
    jerkPenalBox->setMaximum(1000);
    jerkLayout->addWidget(jerkPenalBox);
    
    zmpSettingsLayout->addLayout(jerkLayout);
    
    QHBoxLayout* lookAheadLayout = new QHBoxLayout;
    QLabel* lookAheadLab = new QLabel;
    lookAheadLab->setText("Look Ahead Time:");
    lookAheadLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    lookAheadLab->setToolTip("How far ahead (sec) should the Preview Controller consider?");
    lookAheadLayout->addWidget(lookAheadLab);
    
    lookAheadBox = new QDoubleSpinBox;
    lookAheadBox->setSizePolicy(pbsize);
    lookAheadBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    lookAheadBox->setToolTip(lookAheadLab->toolTip());
    lookAheadBox->setSingleStep(0.01);
    lookAheadBox->setMinimum(-1000);
    lookAheadBox->setMaximum(1000);
    lookAheadLayout->addWidget(lookAheadBox);
    
    zmpSettingsLayout->addLayout(lookAheadLayout);
    
    
    QGroupBox* zmpSettingsBox = new QGroupBox;
    zmpSettingsBox->setTitle("ZMP Settings");
    zmpSettingsBox->setStyleSheet(groupStyleSheet);
    zmpSettingsBox->setLayout(zmpSettingsLayout);
    leftColumn->addWidget(zmpSettingsBox);
    
    
    QVBoxLayout* timeSettingsLayout = new QVBoxLayout;
    timeSettingsLayout->setAlignment(Qt::AlignCenter);
    
    QHBoxLayout* startupTimeLay = new QHBoxLayout;
    QLabel* startupTimeLab = new QLabel;
    startupTimeLab->setText("Startup Time:");
    startupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    startupTimeLab->setToolTip("How much time (s) should be spent starting up?");
    startupTimeLay->addWidget(startupTimeLab);
    
    startupTimeBox = new QDoubleSpinBox;
    startupTimeBox->setSizePolicy(pbsize);
    startupTimeBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    startupTimeBox->setToolTip(startupTimeLab->toolTip());
    startupTimeBox->setSingleStep(0.01);
    startupTimeBox->setMinimum(0);
    startupTimeBox->setMaximum(1000);
    startupTimeLay->addWidget(startupTimeBox);
    
    timeSettingsLayout->addLayout(startupTimeLay);
    
    QHBoxLayout* shutdownTimeLay = new QHBoxLayout;
    QLabel* shutdownTimeLab = new QLabel;
    shutdownTimeLab->setText("Shutdown Time:");
    shutdownTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    shutdownTimeLab->setToolTip("How much time (s) should be spent stopping?");
    shutdownTimeLay->addWidget(shutdownTimeLab);
    
    shutdownTimeBox = new QDoubleSpinBox;
    shutdownTimeBox->setSizePolicy(pbsize);
    shutdownTimeBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    shutdownTimeBox->setToolTip(shutdownTimeLab->toolTip());
    shutdownTimeBox->setSingleStep(0.01);
    shutdownTimeBox->setMinimum(0);
    shutdownTimeBox->setMaximum(1000);
    shutdownTimeLay->addWidget(shutdownTimeBox);
    
    timeSettingsLayout->addLayout(shutdownTimeLay);

    QHBoxLayout* doubleSupTimeLay = new QHBoxLayout;
    QLabel* doubleSupTimeLab = new QLabel;
    doubleSupTimeLab->setText("Double Support Time:");
    doubleSupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    doubleSupTimeLab->setToolTip("How much time (s) should be spent in double support?\n"
                                 "i.e. Having two feet on the ground");
    doubleSupTimeLay->addWidget(doubleSupTimeLab);
    
    doubleSupportBox = new QDoubleSpinBox;
    doubleSupportBox->setSizePolicy(pbsize);
    doubleSupportBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    doubleSupportBox->setToolTip(doubleSupTimeLab->toolTip());
    doubleSupportBox->setSingleStep(0.01);
    doubleSupportBox->setMinimum(0);
    doubleSupportBox->setMaximum(1000);
    doubleSupTimeLay->addWidget(doubleSupportBox);
    
    timeSettingsLayout->addLayout(doubleSupTimeLay);
    
    QHBoxLayout* singleSupTimeLay = new QHBoxLayout;
    QLabel* singleSupTimeLab = new QLabel;
    singleSupTimeLab->setText("Single Support Time:");
    singleSupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    singleSupTimeLab->setToolTip("How much time (s) should be spent in single support?\n"
                                 "i.e. Having one foot on the ground");
    singleSupTimeLay->addWidget(singleSupTimeLab);
    
    singleSupportBox = new QDoubleSpinBox;
    singleSupportBox->setSizePolicy(pbsize);
    singleSupportBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    singleSupportBox->setToolTip(singleSupTimeLab->toolTip());
    singleSupportBox->setSingleStep(0.01);
    singleSupportBox->setMinimum(0);
    singleSupportBox->setMaximum(1000);
    singleSupTimeLay->addWidget(singleSupportBox);
    
    timeSettingsLayout->addLayout(singleSupTimeLay);
    
    QGroupBox* timeSettingsBox = new QGroupBox;
    timeSettingsBox->setTitle("Time Settings");
    timeSettingsBox->setStyleSheet(groupStyleSheet);
    timeSettingsBox->setLayout(timeSettingsLayout);
    leftColumn->addWidget(timeSettingsBox);
    
    
    
    QVBoxLayout* rightColumn = new QVBoxLayout;
    
    QHBoxLayout* ikSenseLayout = new QHBoxLayout;
    QLabel* ikSenseLab = new QLabel;
    ikSenseLab->setText("IK Sensitivity:");
    ikSenseLayout->addWidget(ikSenseLab, 0, Qt::AlignVCenter | Qt::AlignRight);
    ikSenseSelect = new QComboBox;
    ikSenseLayout->addWidget(ikSenseSelect);//, 0, Qt::AlignVCenter | Qt::AlignLeft);
    
    rightColumn->addLayout(ikSenseLayout);
    
    
    QVBoxLayout* swingSettingsLayout = new QVBoxLayout;
    swingSettingsLayout->setAlignment(Qt::AlignCenter);
    
    QHBoxLayout* footLiftLay = new QHBoxLayout;
    QLabel* footliftLab = new QLabel;
    footliftLab->setText("Foot Liftoff Height:");
    footliftLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    footliftLab->setToolTip("How heigh (m) should the swing foot lift off the ground?");
    footLiftLay->addWidget(footliftLab);

    liftoffHeightBox = new QDoubleSpinBox;
    liftoffHeightBox->setSizePolicy(pbsize);
    liftoffHeightBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    liftoffHeightBox->setToolTip(footliftLab->toolTip());
    liftoffHeightBox->setSingleStep(0.01);
    liftoffHeightBox->setMinimum(0);
    liftoffHeightBox->setMaximum(2);
    footLiftLay->addWidget(liftoffHeightBox);
    
    swingSettingsLayout->addLayout(footLiftLay);
    
    QHBoxLayout* stepDistanceLay = new QHBoxLayout;
    QLabel* stepDistanceLab = new QLabel;
    stepDistanceLab->setText("Step Distance:");
    stepDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    stepDistanceLab->setToolTip("How far forward (m) should the swing foot step?");
    stepDistanceLay->addWidget(stepDistanceLab);

    stepDistanceBox = new QDoubleSpinBox;
    stepDistanceBox->setSizePolicy(pbsize);
    stepDistanceBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    stepDistanceBox->setToolTip(stepDistanceLab->toolTip());
    stepDistanceBox->setSingleStep(0.01);
    stepDistanceBox->setMinimum(0);
    stepDistanceBox->setMaximum(5);
    stepDistanceLay->addWidget(stepDistanceBox);
    
    swingSettingsLayout->addLayout(stepDistanceLay);

    QHBoxLayout* sideStepDistanceLay = new QHBoxLayout;
    QLabel* sideStepDistanceLab = new QLabel;
    sideStepDistanceLab->setText("Side Step Distance:");
    sideStepDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    sideStepDistanceLab->setToolTip("How far sideways (m) should the swing foot step?");
    sideStepDistanceLay->addWidget(sideStepDistanceLab);

    sideStepDistanceBox = new QDoubleSpinBox;
    sideStepDistanceBox->setSizePolicy(pbsize);
    sideStepDistanceBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    sideStepDistanceBox->setToolTip(sideStepDistanceLab->toolTip());
    sideStepDistanceBox->setSingleStep(0.01);
    sideStepDistanceBox->setMinimum(0);
    sideStepDistanceBox->setMaximum(5);
    sideStepDistanceLay->addWidget(sideStepDistanceBox);

    swingSettingsLayout->addLayout(sideStepDistanceLay);

    
    QHBoxLayout* lateralDistanceLay = new QHBoxLayout;
    QLabel* lateralDistanceLab = new QLabel;
    lateralDistanceLab->setText("Lateral Distance:");
    lateralDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    lateralDistanceLab->setToolTip("How far from the center (m) should the swing foot be placed?");
    lateralDistanceLay->addWidget(lateralDistanceLab);

    lateralDistanceBox = new QDoubleSpinBox;
    lateralDistanceBox->setSizePolicy(pbsize);
    lateralDistanceBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    lateralDistanceBox->setToolTip(lateralDistanceLab->toolTip());
    lateralDistanceBox->setSingleStep(0.01);
    lateralDistanceBox->setMinimum(0);
    lateralDistanceBox->setMaximum(5);
    lateralDistanceLay->addWidget(lateralDistanceBox);
    
    swingSettingsLayout->addLayout(lateralDistanceLay);
    
    QGroupBox* swingSettingsBox = new QGroupBox;
    swingSettingsBox->setTitle("Swing Foot Settings");
    swingSettingsBox->setStyleSheet(groupStyleSheet);
    swingSettingsBox->setLayout(swingSettingsLayout);
    rightColumn->addWidget(swingSettingsBox);
    
    
    
    QVBoxLayout* comSettingsLayout = new QVBoxLayout;
    comSettingsLayout->setAlignment(Qt::AlignCenter);
    
    QHBoxLayout* comHeightLay = new QHBoxLayout;
    QLabel* comHeightLab = new QLabel;
    comHeightLab->setText("Center of Mass Height:");
    comHeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comHeightLab->setToolTip("How heigh (m) should the center of mass be from the ground?");
    comHeightLay->addWidget(comHeightLab);

    comHeightBox = new QDoubleSpinBox;
    comHeightBox->setSizePolicy(pbsize);
    comHeightBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comHeightBox->setToolTip(comHeightLab->toolTip());
    comHeightBox->setSingleStep(0.01);
    comHeightBox->setMinimum(0);
    comHeightBox->setMaximum(5);
    comHeightLay->addWidget(comHeightBox);
    
    comSettingsLayout->addLayout(comHeightLay);
    
    QHBoxLayout* comIKAngleWeightLay = new QHBoxLayout;
    QLabel* comIKAngleWeightLab = new QLabel;
    comIKAngleWeightLab->setText("Center of Mass Height:");
    comIKAngleWeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comIKAngleWeightLab->setToolTip("How heigh (m) should the center of mass be from the ground?");
    comIKAngleWeightLay->addWidget(comIKAngleWeightLab);

    comIKAngleWeightBox = new QDoubleSpinBox;
    comIKAngleWeightBox->setSizePolicy(pbsize);
    comIKAngleWeightBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comIKAngleWeightBox->setToolTip(comIKAngleWeightLab->toolTip());
    comIKAngleWeightBox->setSingleStep(0.01);
    comIKAngleWeightBox->setMinimum(0);
    comIKAngleWeightBox->setMaximum(5);
    comIKAngleWeightLay->addWidget(comIKAngleWeightBox);
    
    comSettingsLayout->addLayout(comIKAngleWeightLay);
    
    QGroupBox* comSettingsBox = new QGroupBox;
    comSettingsBox->setTitle("Center of Mass Settings");
    comSettingsBox->setStyleSheet(groupStyleSheet);
    comSettingsBox->setLayout(comSettingsLayout);
    rightColumn->addWidget(comSettingsBox);
    
    
    QHBoxLayout* bottomLayout = new QHBoxLayout;
    bottomLayout->addLayout(leftColumn);
    bottomLayout->addLayout(rightColumn);
    
    QVBoxLayout* masterZMPLayout = new QVBoxLayout;
    masterZMPLayout->addLayout(profileLayoutMaster);
    masterZMPLayout->addLayout(bottomLayout);
    
    zmpParamTab = new QWidget;
    zmpParamTab->setLayout(masterZMPLayout);
}

void HuboWalkWidget::fillProfileSelect()
{
    
}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_walk_space::HuboWalkPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_walk_space::HuboWalkWidget, QTabWidget )

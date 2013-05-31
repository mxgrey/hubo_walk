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



    rviz::Config ip_config = config.mapGetChild("HuboIP");
    QVariant a, b, c, d;
    if( !ip_config.mapGetValue("ipAddrA", &a) || !ip_config.mapGetValue("ipAddrB", &b)
     || !ip_config.mapGetValue("ipAddrC", &c) || !ip_config.mapGetValue("ipAddrD", &d))
        ROS_INFO("Loading the IP Address Failed");
    else
        content->setIPAddress(a.toInt(), b.toInt(), c.toInt(), d.toInt());

    
    rviz::Config p_config = config.mapGetChild("ZmpProfiles");
    QVariant pNum;
    
    if( p_config.mapGetValue("ZmpProfileNum", &pNum) )
    {
        QVariant selectedProfile;
        config.mapGetValue("SelectedZmpProfile", &selectedProfile);
        content->zmpProfiles.resize(size_t(pNum.toInt()));
        
        for(int i=0; i < int(content->zmpProfiles.size()); i++)
        {
            QVariant temp;
            p_config.mapGetValue("ZmpProfileName"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].name = temp.toString();
            p_config.mapGetValue("max_step_count"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.max_step_count = size_t(temp.toDouble());
            p_config.mapGetValue("step_length"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.step_length = temp.toDouble();
            p_config.mapGetValue("halfStanceWidth"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.halfStanceWidth = temp.toDouble();
            p_config.mapGetValue("foot_liftoff_z"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.foot_liftoff_z = temp.toDouble();
            p_config.mapGetValue("sidestep_length"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.sidestep_length = temp.toDouble();
            p_config.mapGetValue("com_height"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.com_height = temp.toDouble();
            p_config.mapGetValue("com_ik_ascl"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.com_ik_ascl = temp.toDouble();
            p_config.mapGetValue("zmpoff_y"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.zmpoff_y = temp.toDouble();
            p_config.mapGetValue("zmpoff_x"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.zmpoff_x = temp.toDouble();
            p_config.mapGetValue("lookahead_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.lookahead_time = temp.toDouble();
            p_config.mapGetValue("startup_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.startup_time = temp.toDouble();
            p_config.mapGetValue("shutdown_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.shutdown_time = temp.toDouble();
            p_config.mapGetValue("double_support_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.double_support_time = temp.toDouble();
            p_config.mapGetValue("single_support_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.single_support_time = temp.toDouble();
            p_config.mapGetValue("zmp_jerk_penalty"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.zmp_jerk_penalty = temp.toDouble();
            p_config.mapGetValue("ik_sense"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.ik_sense = ik_error_sensitivity(temp.toInt());
        }
        
        content->updateProfileBox();
        content->profileSelect->setCurrentIndex(selectedProfile.toInt());
    }
    else
        ROS_INFO("No zmp profiles found");


    rviz::Config pb_config = config.mapGetChild("BalProfiles");
    QVariant pbNum;

    if( pb_config.mapGetValue("BalProfileNum", &pbNum) )
    {
        QVariant selectedProfile;
        config.mapGetValue("SelectedBalProfile", &selectedProfile);
        content->balProfiles.resize(size_t(pbNum.toInt()));

        for(int i=0; i < int(content->balProfiles.size()); i++)
        {
            QVariant temp;
            pb_config.mapGetValue("BalProfileName"+QString::number(i),
                                 &temp);
            content->balProfiles[i].name = temp.toString();
            pb_config.mapGetValue("flatten_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.flattening_gain[LEFT] = temp.toDouble();
            pb_config.mapGetValue("flatten_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.flattening_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("decay_gain"+QString::number(i),
                                  &temp);
            content->balProfiles[i].vals.decay_gain[LEFT] = temp.toDouble();
            content->balProfiles[i].vals.decay_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("thresh_min_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.force_min_threshold[LEFT] = temp.toDouble();
            pb_config.mapGetValue("thresh_min_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.force_min_threshold[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("thresh_max_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.force_max_threshold[LEFT] = temp.toDouble();
            pb_config.mapGetValue("thresh_max_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.force_max_threshold[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("straightenP_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.straightening_pitch_gain[LEFT] = temp.toDouble();
            pb_config.mapGetValue("straightenP_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.straightening_pitch_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("straightenR_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.straightening_roll_gain[LEFT] = temp.toDouble();
            pb_config.mapGetValue("straightenR_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.straightening_roll_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("spring_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.spring_gain[LEFT] = temp.toDouble();
            pb_config.mapGetValue("spring_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.spring_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("damp_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.damping_gain[LEFT] = temp.toDouble();
            pb_config.mapGetValue("damp_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.damping_gain[RIGHT] = temp.toDouble();
            pb_config.mapGetValue("response_l"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.fz_response[LEFT] = temp.toDouble();
            pb_config.mapGetValue("response_r"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.fz_response[RIGHT] = temp.toDouble();
        }

        content->updatebalProfileBox();
        content->balProfileSelect->setCurrentIndex(selectedProfile.toInt());
    }
    else
        ROS_INFO("No balance profiles found");

    
}

void HuboWalkPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("Class", getClassId());

    rviz::Config ip_config = config.mapMakeChild("HuboIP");

    QVariant a = QVariant(content->getIPAddress(0));
    QVariant b = QVariant(content->getIPAddress(1));
    QVariant c = QVariant(content->getIPAddress(2));
    QVariant d = QVariant(content->getIPAddress(3));

    ip_config.mapSetValue("ipAddrA", a);
    ip_config.mapSetValue("ipAddrB", b);
    ip_config.mapSetValue("ipAddrC", c);
    ip_config.mapSetValue("ipAddrD", d);
    
    QVariant selectedProfile = QVariant(content->profileSelect->currentIndex());
    config.mapSetValue("SelectedZmpProfile", selectedProfile);
    
    rviz::Config p_config = config.mapMakeChild("ZmpProfiles");
    
    QVariant pNum = QVariant(int(content->zmpProfiles.size()));
    p_config.mapSetValue("ZmpProfileNum", pNum);
    
    for(int i=0; i < int(content->zmpProfiles.size()); i++)
    {
        content->zmpProfiles[i].name.replace(" ","_");
        p_config.mapSetValue("ZmpProfileName"+QString::number(i),
                             QVariant(content->zmpProfiles[i].name));
        p_config.mapSetValue("max_step_count"+QString::number(i),
                             QVariant(int(content->zmpProfiles[i].vals.max_step_count)));
        p_config.mapSetValue("step_length"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.step_length));
        p_config.mapSetValue("halfStanceWidth"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.halfStanceWidth));
        p_config.mapSetValue("foot_liftoff_z"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.foot_liftoff_z));
        p_config.mapSetValue("sidestep_length"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.sidestep_length));
        p_config.mapSetValue("com_height"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.com_height));
        p_config.mapSetValue("com_ik_ascl"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.com_ik_ascl));
        p_config.mapSetValue("zmpoff_y"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.zmpoff_y));
        p_config.mapSetValue("zmpoff_x"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.zmpoff_x));
        p_config.mapSetValue("lookahead_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.lookahead_time));
        p_config.mapSetValue("startup_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.startup_time));
        p_config.mapSetValue("shutdown_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.shutdown_time));
        p_config.mapSetValue("double_support_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.double_support_time));
        p_config.mapSetValue("single_support_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.single_support_time));
        p_config.mapSetValue("zmp_jerk_penalty"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.zmp_jerk_penalty));
        p_config.mapSetValue("ik_sense"+QString::number(i),
                             QVariant(int(content->zmpProfiles[i].vals.ik_sense)));
    }


    QVariant selectedbalProfile = QVariant(content->balProfileSelect->currentIndex());
    config.mapSetValue("SelectedBalProfile", selectedbalProfile);

    rviz::Config pb_config = config.mapMakeChild("BalProfiles");

    QVariant pbNum = QVariant(int(content->balProfiles.size()));
    pb_config.mapSetValue("BalProfileNum", pbNum);

    for(int i=0; i < int(content->balProfiles.size()); i++)
    {
        content->balProfiles[i].name.replace(" ","_");
        pb_config.mapSetValue("BalProfileName"+QString::number(i),
                             QVariant(content->balProfiles[i].name));
        pb_config.mapSetValue("flatten_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.flattening_gain[LEFT]));
        pb_config.mapSetValue("flatten_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.flattening_gain[RIGHT]));
        pb_config.mapSetValue("decay_gain"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.decay_gain[LEFT]));
        pb_config.mapSetValue("thresh_min_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.force_min_threshold[LEFT]));
        pb_config.mapSetValue("thresh_min_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.force_min_threshold[RIGHT]));
        pb_config.mapSetValue("thresh_max_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.force_max_threshold[LEFT]));
        pb_config.mapSetValue("thresh_max_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.force_max_threshold[RIGHT]));
        pb_config.mapSetValue("straightenP_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.straightening_pitch_gain[LEFT]));
        pb_config.mapSetValue("straightenP_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.straightening_pitch_gain[RIGHT]));
        pb_config.mapSetValue("straightenR_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.straightening_roll_gain[LEFT]));
        pb_config.mapSetValue("straightenR_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.straightening_roll_gain[RIGHT]));
        pb_config.mapSetValue("spring_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.spring_gain[LEFT]));
        pb_config.mapSetValue("spring_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.spring_gain[RIGHT]));
        pb_config.mapSetValue("damp_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.damping_gain[LEFT]));
        pb_config.mapSetValue("damp_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.damping_gain[RIGHT]));
        pb_config.mapSetValue("response_l"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.fz_response[LEFT]));
        pb_config.mapSetValue("response_r"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.fz_response[RIGHT]));
    }
    
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

    memset(&cmd, 0, sizeof(cmd));

    initializeCommandTab();
    std::cerr << "Command Tab loaded" << std::endl;
    
    initializeZmpParamTab();
    std::cerr << "ZMP Parameters Tab loaded" << std::endl;

    initializeBalParamTab();
    std::cerr << "Balance Parameters Tab loaded" << std::endl;
    
    addTab(commandTab, "Command");
    addTab(zmpParamTab, "ZMP Parameters");
    addTab(balParamTab, "Balance Parameters");

    initializeAchConnections();

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
    joyStatusLayout->setAlignment(Qt::AlignTop);
    joyLayout->addLayout(joyStatusLayout);
    
    controlSelectLayout->addLayout(joyLayout);
    
    QGroupBox* controlSelectBox = new QGroupBox;
    controlSelectBox->setTitle("Control Method");
    controlSelectBox->setStyleSheet(groupStyleSheet);
    controlSelectBox->setLayout(controlSelectLayout);
    
    
    QVBoxLayout* controlLayout = new QVBoxLayout;
    
    QHBoxLayout* paramLayout = new QHBoxLayout;
    
    QVBoxLayout* distanceLayout = new QVBoxLayout;
    QHBoxLayout* walkLayout = new QHBoxLayout;
    QLabel* walkLab = new QLabel;
    walkLab->setText("Walk Distance:");
    walkLab->setToolTip("Distance to walk (m) after a click");
    walkLayout->addWidget(walkLab, 0, Qt::AlignRight);
    walkDistanceBox = new QDoubleSpinBox;
    walkDistanceBox->setSingleStep(0.5);
    walkDistanceBox->setValue(0.2);
    walkDistanceBox->setToolTip(walkLab->toolTip());
    walkLayout->addWidget(walkDistanceBox, 0, Qt::AlignLeft);
    distanceLayout->addLayout(walkLayout);

    QHBoxLayout* rotateAngleLayout = new QHBoxLayout;
    QLabel* rotateAngleLab = new QLabel;
    rotateAngleLab->setText("Turn-in-place Angle:");
    rotateAngleLab->setToolTip("Angle (rad) after a click");
    rotateAngleLayout->addWidget(rotateAngleLab, 0, Qt::AlignRight);
    rotateAngleBox = new QDoubleSpinBox;
    rotateAngleBox->setSingleStep(0.1);
    rotateAngleBox->setValue(0.5);
    rotateAngleBox->setToolTip(rotateAngleLab->toolTip());
    rotateAngleLayout->addWidget(rotateAngleBox, 0, Qt::AlignLeft);
    distanceLayout->addLayout(rotateAngleLayout);
    
    continuousBox = new QCheckBox;
    continuousBox->setChecked(false);
    continuousBox->setText("Continuous");
    continuousBox->setToolTip("Ignore the walk distance, and walk until Stop is selected");
    distanceLayout->addWidget(continuousBox, 0, Qt::AlignLeft);
    
    paramLayout->addLayout(distanceLayout);
    
    
    QLabel* maxStepLab = new QLabel;
    maxStepLab->setText("Max Step Count:");
    maxStepLab->setToolTip("Cut-off for number of steps to take");
    paramLayout->addWidget(maxStepLab, 0, Qt::AlignRight | Qt::AlignVCenter);
    maxStepBox = new QSpinBox;
    maxStepBox->setSingleStep(1);
    maxStepBox->setToolTip(maxStepLab->toolTip());
    maxStepBox->setValue(10);
    maxStepBox->setMaximum(10);
    paramLayout->addWidget(maxStepBox, 0, Qt::AlignLeft | Qt::AlignVCenter);
    
    
    QLabel* turnLab = new QLabel;
    turnLab->setText("Turn Radius:");
    turnLab->setToolTip("Radius of curvature (m) for turning");
    paramLayout->addWidget(turnLab, 0, Qt::AlignRight | Qt::AlignVCenter);
    radiusBox = new QDoubleSpinBox;
    radiusBox->setValue(5.0);
    radiusBox->setMinimum(0);
    radiusBox->setSingleStep(0.1);
    radiusBox->setMaximum(10000);
    radiusBox->setToolTip(turnLab->toolTip());
    paramLayout->addWidget(radiusBox, 0, Qt::AlignLeft | Qt::AlignVCenter);
    
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
    connect(rightButton, SIGNAL(clicked()), this, SLOT(handleRight()));
    
    
    backButton = new QPushButton;
    backButton->setText("Backward");
    backButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(backButton, 2, 1, 1, 1, Qt::AlignCenter);
    connect(backButton, SIGNAL(clicked()), this, SLOT(handleBackward()));
    
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
    profileLayoutTop->addWidget(profileSelect);
    connect(profileSelect, SIGNAL(currentIndexChanged(int)), this, SLOT(handleProfileSelect(int)));
    
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
    xOffsetBox->setDecimals(4);
    xOffsetBox->setValue(0.038);
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
    yOffsetBox->setDecimals(4);
    yOffsetBox->setValue(0);
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
    jerkPenalBox->setValue(1);
    jerkPenalBox->setSingleStep(0.1);
    jerkPenalBox->setMinimum(0);
    jerkPenalBox->setMaximum(1000);
    jerkLayout->addWidget(jerkPenalBox);

    penalFactor = 1e-8;
    QLabel* penalFactorLab = new QLabel;
    penalFactorLab->setText(" x "+QString::number(penalFactor,'g'));
    jerkLayout->addWidget(penalFactorLab);
    
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
    lookAheadBox->setValue(2.5);
    lookAheadBox->setSingleStep(0.1);
    lookAheadBox->setMinimum(0);
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
    startupTimeBox->setDecimals(3);
    startupTimeBox->setValue(0.5);
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
    shutdownTimeBox->setDecimals(3);
    shutdownTimeBox->setValue(0.5);
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
    doubleSupportBox->setDecimals(3);
    doubleSupportBox->setValue(0.01);
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
    singleSupportBox->setDecimals(3);
    singleSupportBox->setValue(0.50);
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
    
    ikSenseSelect->addItem("Strict");
    ikSenseSelect->addItem("Permissive");
    ikSenseSelect->addItem("Sloppy (DANGEROUS)");
    ikSenseSelect->setCurrentIndex(0);
    
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
    liftoffHeightBox->setDecimals(4);
    liftoffHeightBox->setValue(0.06);
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
    stepDistanceBox->setDecimals(4);
    stepDistanceBox->setValue(0.1);
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
    sideStepDistanceBox->setDecimals(4);
    sideStepDistanceBox->setValue(0.01);
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
    lateralDistanceBox->setDecimals(4);
    lateralDistanceBox->setValue(0.0885);
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
    comHeightBox->setDecimals(3);
    comHeightBox->setValue(0.5);
    comHeightBox->setSingleStep(0.01);
    comHeightBox->setMinimum(0);
    comHeightBox->setMaximum(5);
    comHeightLay->addWidget(comHeightBox);
    
    comSettingsLayout->addLayout(comHeightLay);
    
    QHBoxLayout* comIKAngleWeightLay = new QHBoxLayout;
    QLabel* comIKAngleWeightLab = new QLabel;
    comIKAngleWeightLab->setText("IK Angle Weight:");
    comIKAngleWeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comIKAngleWeightLab->setToolTip("I don't really know what this does.");
    comIKAngleWeightLay->addWidget(comIKAngleWeightLab);

    comIKAngleWeightBox = new QDoubleSpinBox;
    comIKAngleWeightBox->setSizePolicy(pbsize);
    comIKAngleWeightBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comIKAngleWeightBox->setToolTip(comIKAngleWeightLab->toolTip());
    comIKAngleWeightBox->setDecimals(3);
    comIKAngleWeightBox->setValue(0);
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
    
    saveAsEdit->setText("Default");
    handleProfileSaveAs();
    saveAsEdit->setText("Default-Backup");
    handleProfileSaveAs();
    saveAsEdit->clear();
    
    profileSelect->setCurrentIndex(0);
}


void HuboWalkWidget::initializeBalParamTab()
{
    QSizePolicy pbsize(QSizePolicy::Maximum, QSizePolicy::Maximum);

    QHBoxLayout* balProfileLayoutTop = new QHBoxLayout;
    QLabel* balProfileLab = new QLabel;
    balProfileLab->setText("Profile:");
    balProfileLayoutTop->addWidget(balProfileLab, 0, Qt::AlignVCenter | Qt::AlignRight);

    balProfileSelect = new QComboBox;
    balProfileLayoutTop->addWidget(balProfileSelect);
    connect(balProfileSelect, SIGNAL(currentIndexChanged(int)), this, SLOT(handlebalProfileSelect(int)));

    savebalProfile = new QPushButton;
    savebalProfile->setSizePolicy(pbsize);
    savebalProfile->setText("Save");
    savebalProfile->setToolTip("Save the values below into the currently selected profile");
    balProfileLayoutTop->addWidget(savebalProfile);
    connect(savebalProfile, SIGNAL(clicked()), this, SLOT(handlebalProfileSave()));

    deletebalProfile = new QPushButton;
    deletebalProfile->setSizePolicy(pbsize);
    deletebalProfile->setText("Delete");
    deletebalProfile->setToolTip("Remove the current profile from the list\n"
                                 "WARNING: This is permanent!");
    balProfileLayoutTop->addWidget(deletebalProfile);
    connect(deletebalProfile, SIGNAL(clicked()), this, SLOT(handlebalProfileDelete()));

    QHBoxLayout* balProfileLayoutBottom = new QHBoxLayout;
    saveAsbalProfile = new QPushButton;
    saveAsbalProfile->setSizePolicy(pbsize);
    saveAsbalProfile->setText("Save As...");
    saveAsbalProfile->setToolTip("Save the values below as a new profile with the following name:");
    balProfileLayoutBottom->addWidget(saveAsbalProfile);
    connect(saveAsbalProfile, SIGNAL(clicked()), this, SLOT(handlebalProfileSaveAs()));

    balSaveAsEdit = new QLineEdit;
    balSaveAsEdit->setToolTip("Enter a name for a new profile");
    balProfileLayoutBottom->addWidget(balSaveAsEdit);

    QVBoxLayout* balProfileLayoutMaster = new QVBoxLayout;
    balProfileLayoutMaster->addLayout(balProfileLayoutTop);
    balProfileLayoutMaster->addLayout(balProfileLayoutBottom);


    QVBoxLayout* bottomLayout = new QVBoxLayout;

    QHBoxLayout* flatLayout = new QHBoxLayout;
    QLabel* flatLab = new QLabel;
    flatLab->setText("Flattening Gains:");
    flatLab->setToolTip("Gains for foot flattening (Left/Right)");
    flatLayout->addWidget(flatLab);
    flattenBoxL = new QDoubleSpinBox;
    flattenBoxL->setDecimals(4);
    flattenBoxL->setSingleStep(0.005);
    flattenBoxL->setMinimum(0);
    flattenBoxL->setMaximum(99999);
    flattenBoxL->setValue(0.001);
    flatLayout->addWidget(flattenBoxL);
    flattenBoxR = new QDoubleSpinBox;
    flattenBoxR->setDecimals(4);
    flattenBoxR->setSingleStep(0.005);
    flattenBoxR->setMinimum(0);
    flattenBoxR->setMaximum(99999);
    flattenBoxR->setValue(0.001);
    flatLayout->addWidget(flattenBoxR);

    bottomLayout->addLayout(flatLayout);

    QHBoxLayout* decayLayout = new QHBoxLayout;
    QLabel* decayLab = new QLabel;
    decayLab->setText("Decay Gain:");
    decayLayout->addWidget(decayLab);
    decayBox = new QDoubleSpinBox;
    decayBox->setDecimals(4);
    decayBox->setSingleStep(0.005);
    decayBox->setMinimum(0);
    decayBox->setMaximum(1);
    decayBox->setValue(0.001);
    decayLayout->addWidget(decayBox);

    bottomLayout->addLayout(decayLayout);

    QHBoxLayout* threshMinLayout = new QHBoxLayout;
    QLabel* threshMinLab = new QLabel;
    threshMinLab->setText("Minimum Force Threshold:");
    threshMinLab->setToolTip("Minimum force cut-off for flattening foot (Left/Right)");
    threshMinLayout->addWidget(threshMinLab);
    threshMinBoxL = new QDoubleSpinBox;
    threshMinBoxL->setDecimals(4);
    threshMinBoxL->setSingleStep(1);
    threshMinBoxL->setMinimum(-99999);
    threshMinBoxL->setMaximum(99999);
    threshMinBoxL->setValue(10);
    threshMinLayout->addWidget(threshMinBoxL);
    threshMinBoxR = new QDoubleSpinBox;
    threshMinBoxR->setDecimals(4);
    threshMinBoxR->setSingleStep(1);
    threshMinBoxR->setMinimum(-99999);
    threshMinBoxR->setMaximum(99999);
    threshMinBoxR->setValue(10);
    threshMinLayout->addWidget(threshMinBoxR);

    bottomLayout->addLayout(threshMinLayout);

    QHBoxLayout* threshMaxLayout = new QHBoxLayout;
    QLabel* threshMaxLab = new QLabel;
    threshMaxLab->setText("Maximum Force Threshold:");
    threshMaxLab->setToolTip("Minimum force cut-off for flattening foot (Left/Right)");
    threshMaxLayout->addWidget(threshMaxLab);
    threshMaxBoxL = new QDoubleSpinBox;
    threshMaxBoxL->setDecimals(4);
    threshMaxBoxL->setSingleStep(1);
    threshMaxBoxL->setMinimum(-99999);
    threshMaxBoxL->setMaximum(99999);
    threshMaxBoxL->setValue(50);
    threshMaxLayout->addWidget(threshMaxBoxL);
    threshMaxBoxR = new QDoubleSpinBox;
    threshMaxBoxR->setDecimals(4);
    threshMaxBoxR->setSingleStep(1);
    threshMaxBoxR->setMinimum(-99999);
    threshMaxBoxR->setMaximum(99999);
    threshMaxBoxR->setValue(50);
    threshMaxLayout->addWidget(threshMaxBoxR);

    bottomLayout->addLayout(threshMaxLayout);

    QHBoxLayout* straightenPLayout = new QHBoxLayout;
    QLabel* straightenPLab = new QLabel;
    straightenPLab->setText("Torso pitch-straightening Gain:");
    straightenPLab->setToolTip("Gain for keeping torso upright (Left/Right)");
    straightenPLayout->addWidget(straightenPLab);
    straightenPBoxL = new QDoubleSpinBox;
    straightenPBoxL->setDecimals(4);
    straightenPBoxL->setSingleStep(1);
    straightenPBoxL->setMinimum(-99999);
    straightenPBoxL->setMaximum(99999);
    straightenPBoxL->setValue(10);
    straightenPLayout->addWidget(straightenPBoxL);
    straightenPBoxR = new QDoubleSpinBox;
    straightenPBoxR->setDecimals(4);
    straightenPBoxR->setSingleStep(1);
    straightenPBoxR->setMinimum(-99999);
    straightenPBoxR->setMaximum(99999);
    straightenPBoxR->setValue(10);
    straightenPLayout->addWidget(straightenPBoxR);

    bottomLayout->addLayout(straightenPLayout);

    QHBoxLayout* straightenRLayout = new QHBoxLayout;
    QLabel* straightenRLab = new QLabel;
    straightenRLab->setText("Torso roll-straightening Gain:");
    straightenRLab->setToolTip("Gain for keeping torso upright (Left/Right)");
    straightenRLayout->addWidget(straightenRLab);
    straightenRBoxL = new QDoubleSpinBox;
    straightenRBoxL->setDecimals(4);
    straightenRBoxL->setSingleStep(1);
    straightenRBoxL->setMinimum(-99999);
    straightenRBoxL->setMaximum(99999);
    straightenRBoxL->setValue(10);
    straightenRLayout->addWidget(straightenRBoxL);
    straightenRBoxR = new QDoubleSpinBox;
    straightenRBoxR->setDecimals(4);
    straightenRBoxR->setSingleStep(1);
    straightenRBoxR->setMinimum(-99999);
    straightenRBoxR->setMaximum(99999);
    straightenRBoxR->setValue(10);
    straightenRLayout->addWidget(straightenRBoxR);

    bottomLayout->addLayout(straightenRLayout);

    QHBoxLayout* springLayout = new QHBoxLayout;
    QLabel* springLab = new QLabel;
    springLab->setText("Leg Stiffness Gain:");
    springLab->setToolTip("Gain for keeping the legs stiff (Left/Right)");
    springLayout->addWidget(springLab);
    springBoxL = new QDoubleSpinBox;
    springBoxL->setDecimals(4);
    springBoxL->setSingleStep(1);
    springBoxL->setMinimum(-99999);
    springBoxL->setMaximum(99999);
    springBoxL->setValue(10);
    springLayout->addWidget(springBoxL);
    springBoxR = new QDoubleSpinBox;
    springBoxR->setDecimals(4);
    springBoxR->setSingleStep(1);
    springBoxR->setMinimum(-99999);
    springBoxR->setMaximum(99999);
    springBoxR->setValue(10);
    springLayout->addWidget(springBoxR);

    bottomLayout->addLayout(springLayout);

    QHBoxLayout* dampLayout = new QHBoxLayout;
    QLabel* dampLab = new QLabel;
    dampLab->setText("Leg Damping Gain:");
    dampLab->setToolTip("Gain for damping leg movement (Left/Right)");
    dampLayout->addWidget(dampLab);
    dampBoxL = new QDoubleSpinBox;
    dampBoxL->setDecimals(4);
    dampBoxL->setSingleStep(1);
    dampBoxL->setMinimum(-99999);
    dampBoxL->setMaximum(99999);
    dampBoxL->setValue(10);
    dampLayout->addWidget(dampBoxL);
    dampBoxR = new QDoubleSpinBox;
    dampBoxR->setDecimals(4);
    dampBoxR->setSingleStep(1);
    dampBoxR->setMinimum(-99999);
    dampBoxR->setMaximum(99999);
    dampBoxR->setValue(10);
    dampLayout->addWidget(dampBoxR);

    bottomLayout->addLayout(dampLayout);

    QHBoxLayout* responseLayout = new QHBoxLayout;
    QLabel* responseLab = new QLabel;
    responseLab->setText("Leg Response Gain:");
    responseLab->setToolTip("Gain for how much legs should respond to force (Left/Right)");
    responseLayout->addWidget(responseLab);
    responseBoxL = new QDoubleSpinBox;
    responseBoxL->setDecimals(4);
    responseBoxL->setSingleStep(1);
    responseBoxL->setMinimum(-99999);
    responseBoxL->setMaximum(99999);
    responseBoxL->setValue(10);
    responseLayout->addWidget(responseBoxL);
    responseBoxR = new QDoubleSpinBox;
    responseBoxR->setDecimals(4);
    responseBoxR->setSingleStep(1);
    responseBoxR->setMinimum(-99999);
    responseBoxR->setMaximum(99999);
    responseBoxR->setValue(10);
    responseLayout->addWidget(responseBoxR);

    bottomLayout->addLayout(responseLayout);

    updateBalParams = new QPushButton;
    updateBalParams->setText("Send");
    updateBalParams->setToolTip("Send this set of parameters to the balancing daemon");
    connect(updateBalParams, SIGNAL(clicked()), this, SLOT(sendBalParams()));
    bottomLayout->addWidget(updateBalParams);

    balSaveAsEdit->setText("Default");
    handlebalProfileSaveAs();
    balSaveAsEdit->setText("Default-Backup");
    handlebalProfileSaveAs();
    balSaveAsEdit->clear();

    balProfileSelect->setCurrentIndex(0);

    QVBoxLayout* masterBalLayout = new QVBoxLayout;
    masterBalLayout->addLayout(balProfileLayoutMaster);
    masterBalLayout->addLayout(bottomLayout);

    balParamTab = new QWidget;
    balParamTab->setLayout(masterBalLayout);
}






}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_walk_space::HuboWalkPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_walk_space::HuboWalkWidget, QTabWidget )

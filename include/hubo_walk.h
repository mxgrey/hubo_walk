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

#ifndef HUBO_WALK_H
#define HUBO_WALK_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QCheckBox>
#include <QComboBox>
#include <QVector>

#include <vector>

#include <rviz/panel.h>

#include <hubo.h>
#include <hubo-jointparams.h>

//#include <hubo_motion_ros/AchNetworkWidget.h>
#ifdef HAVE_HUBOMZ
#include <zmp-daemon.h>
#endif
#include <balance-daemon.h>
//#include <hubo_motion_ros/include/hubo_motion_ros/AchNetworkWidget.h>


namespace hubo_walk_space
{

class HuboWalkWidget;


class HuboRefreshManager : public QThread
{
Q_OBJECT
public:
    HuboWalkWidget* parentWidget;
    bool alive;
    int waitTime;

protected:
    virtual void run();

protected slots:
    void getWaitTime(int t);

signals:
    void signalRefresh();

};

#ifdef HAVE_HUBOMZ
class ZmpProfile
{
public:
    QString name;
    zmp_cmd_t vals;
};
#endif // HAVE_HUBOMZ

class BalProfile
{
public:
    QString name;
    balance_params_t vals;
};

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class HuboWalkWidget: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  HuboWalkWidget( QWidget* parent = 0 );
  ~HuboWalkWidget();

  QString groupStyleSheet;


  // Handler for the nested ach daemon process
  QProcess achChannelZmp;
  QProcess achChannelZmpState;
  QProcess achChannelBal;
  QProcess achdZmp;
  QProcess achdZmpState;
  QProcess achdBal;
  QProcess achdBalCmd;
  bool zmpConnected;
  bool zmpStateConnected;
  bool balConnected;

  // Update timer
  HuboRefreshManager* refreshManager;
  int getRefreshTime();

  // Ach Channels for sending and receiving data
  ach_channel_t zmpStateChan;
  ach_channel_t zmpCmdChan;
  ach_channel_t balanceParamChan;
  ach_channel_t balanceCmdChan;
  walkMode_t tempWalkMode;

  void initializeAchConnections();
//  void initializeAchStructs();
  void sendCommand();
  void sendBalCommand();

  void setIPAddress(int a, int b, int c, int d);
  int getIPAddress(int index);

  // Structs for storing data to transmit
  // TODO: Make the correct structs
#ifdef HAVE_HUBOMZ
  struct zmp_cmd cmd;
  struct zmp_state zmpState;
  walkMode_t walkMode;
#endif // HAVE_HUBOMZ
  struct balance_params balParams;
  struct balance_cmd balCmd;
  
  
  // Handling profiles TODO
  //std::vector<zmp_params> profiles;
#ifdef HAVE_HUBOMZ
  QVector<ZmpProfile> zmpProfiles;
  QVector<ZmpProfile> zmpQuadProfiles;
  bool fillProfile(zmp_cmd_t &vals, const walkMode_t walkMode);
#endif // HAVE_HUBOMZ
  void updateProfileBox();
  void updateQuadrupedProfileBox();
  QVector<BalProfile> balProfiles;
  void fillbalProfile(balance_params_t &vals);
  void updatebalProfileBox();
  
  QWidget* commandTab;

    QPushButton* achdConnect;
    QPushButton* achdDisconnect;
    QLabel* statusLabel;
    QLineEdit* ipAddrAEdit;
    QLineEdit* ipAddrBEdit;
    QLineEdit* ipAddrCEdit;
    QLineEdit* ipAddrDEdit;
    
    
    QButtonGroup* radioSelectGroup;
    QRadioButton* guiSelect;
    QRadioButton* joySelect;
    QPushButton* joyLaunch;
    QLabel* joyStatus;
    
    
    QDoubleSpinBox* strideBox;
    QDoubleSpinBox* walkDistanceBox;
    QDoubleSpinBox* rotateAngleBox;
    QSpinBox* maxStepBox;
    QDoubleSpinBox* radiusBox;
    QCheckBox* continuousBox;
    QPushButton* forwardButton;
    QPushButton* leftButton;
    QPushButton* rightButton;
    QPushButton* backButton;
    QPushButton* stopButton;
    QPushButton* turnLeftButton;
    QPushButton* turnRightButton;
    QPushButton* quadrupedButton;
    QPushButton* bipedButton;

    // ZMP State
    QLineEdit* zmpResultEdit;
    QLineEdit* walkModeEdit;


    double heightScale;
    QSlider* heightSlide;
    QDoubleSpinBox* comXOffsetBox;
    QPushButton* staticButton;
    QPushButton* balOffButton;
  ///////////////
    

    //----------------------
    // ZMP BIPED PARAM TAB
    //----------------------
    QWidget* zmpBipedParamTab;
    
      QComboBox* profileSelect;
      QPushButton* saveProfile;
      QPushButton* deleteProfile;
      QPushButton* saveAsProfile;
      QLineEdit* saveAsEdit;
      
      QDoubleSpinBox* xOffsetBox;
      QDoubleSpinBox* yOffsetBox;
      QDoubleSpinBox* jerkPenalBox;
      double penalFactor;
      QDoubleSpinBox* lookAheadBox;
      
      QDoubleSpinBox* startupTimeBox;
      QDoubleSpinBox* shutdownTimeBox;
      QDoubleSpinBox* doubleSupportBox;
      QDoubleSpinBox* singleSupportBox;
      QDoubleSpinBox* pauseTimeBox;
      QDoubleSpinBox* transitionToQuadTimeBox;

      QComboBox* walkTypeSelect;
      QComboBox* ikSenseSelect;
      
      QDoubleSpinBox* liftoffHeightBox;
      QDoubleSpinBox* stepDistanceBox;
      QDoubleSpinBox* sideStepDistanceBox;
      QDoubleSpinBox* lateralDistanceBox;
      
      QDoubleSpinBox* comHeightBox;
	  QDoubleSpinBox* torsoPitchBox;
      QDoubleSpinBox* comIKAngleWeightBox;
      QCheckBox* constantBodyZBox;

    ///////////////

    //----------------------
    // ZMP QUADRUPED PARAM TAB
    //----------------------
    QWidget* zmpQuadrupedParamTab;
    
      QComboBox* profileSelectQuad;
      QPushButton* saveProfileQuad;
      QPushButton* deleteProfileQuad;
      QPushButton* saveAsProfileQuad;
      QLineEdit* saveAsEditQuad;
      
      QDoubleSpinBox* xOffsetBoxQuad;
      QDoubleSpinBox* yOffsetBoxQuad;
      QDoubleSpinBox* jerkPenalBoxQuad;
      double penalFactorQuad;
      QDoubleSpinBox* lookAheadBoxQuad;
      
      QDoubleSpinBox* startupTimeBoxQuad;
      QDoubleSpinBox* shutdownTimeBoxQuad;
      QDoubleSpinBox* doubleSupportBoxQuad;
      QDoubleSpinBox* singleSupportBoxQuad;
      QDoubleSpinBox* pauseTimeBoxQuad;
      QDoubleSpinBox* transitionToBipedTimeBoxQuad;

      QComboBox* walkTypeSelectQuad;
      QComboBox* ikSenseSelectQuad;
      
      QDoubleSpinBox* liftoffHeightBoxQuad;
      QDoubleSpinBox* stepDistanceBoxQuad;
      QDoubleSpinBox* sideStepDistanceBoxQuad;
      QDoubleSpinBox* lateralDistanceBoxQuad;
      QDoubleSpinBox* quadStanceLengthBoxQuad;
      QDoubleSpinBox* quadStabilityMarginBoxQuad;
      QDoubleSpinBox* halfPegWidthBoxQuad;

      QDoubleSpinBox* comHeightBoxQuad;
	  QDoubleSpinBox* torsoPitchBoxQuad;
      QDoubleSpinBox* comIKAngleWeightBoxQuad;
      QCheckBox* constantBodyZBoxQuad;
    //////////

#ifdef HAVE_HUBOMZ
      ik_error_sensitivity int2ikSense(int index);
      int ikSense2int(ik_error_sensitivity ik_sense);
#endif // HAVE_HUBOMZ

    QWidget* balParamTab;

      QComboBox* balProfileSelect;
      QPushButton* savebalProfile;
      QPushButton* deletebalProfile;
      QPushButton* saveAsbalProfile;
      QLineEdit* balSaveAsEdit;

      QDoubleSpinBox* flattenBoxBal;
      QDoubleSpinBox* flattenBoxWalk;
      QDoubleSpinBox* decayBoxBal;
      QDoubleSpinBox* decayBoxWalk;
      QDoubleSpinBox* threshMinBoxBal;
      QDoubleSpinBox* threshMinBoxWalk;
      QDoubleSpinBox* threshMaxBoxBal;
      QDoubleSpinBox* threshMaxBoxWalk;

      QDoubleSpinBox* straightenPBoxBal;
      QDoubleSpinBox* straightenPBoxWalk;

      QDoubleSpinBox* straightenRBoxBal;
      QDoubleSpinBox* straightenRBoxWalk;

      QDoubleSpinBox* springBoxBal;
      QDoubleSpinBox* springBoxWalk;
      QDoubleSpinBox* dampBoxBal;
      QDoubleSpinBox* dampBoxWalk;
      QDoubleSpinBox* responseBoxBal;
      QDoubleSpinBox* responseBoxWalk;

      // Gains for nudging the hips using the F/T sensors.
      // Different gains for single and double support.
      QDoubleSpinBox* singleSupportHipNudgeGainBoxP;
      QDoubleSpinBox* singleSupportHipNudgeGainBoxD;
      QDoubleSpinBox* doubleSupportHipNudgeGainBoxP;
      QDoubleSpinBox* doubleSupportHipNudgeGainBoxD;

      QPushButton* updateBalParams;


protected:
  int ipAddrA;
  int ipAddrB;
  int ipAddrC;
  int ipAddrD;

signals:
  void sendWaitTime(int t);

  // Slots will be "connected" to signals in order to respond to user events
protected Q_SLOTS:

  void sendBalParams();

  void handleStaticButton();
  void handleBalOffButton();

  // Handle button events
  void handleProfileSave();
  void handleProfileDelete();
  void handleProfileSaveAs();
  void handleProfileSelect(int index);

  void handleQuadrupedProfileSave();
  void handleQuadrupedProfileDelete();
  void handleQuadrupedProfileSaveAs();
  void handleQuadrupedProfileSelect(int index);

  void handlebalProfileSave();
  void handlebalProfileDelete();
  void handlebalProfileSaveAs();
  void handlebalProfileSelect(int index);

  void handleJoyLaunch();
  
  void handleForward();
  void handleLeft();
  void handleTurnLeft();
  void handleRight();
  void handleTurnRight();
  void handleBackward();
  void handleStop();
  void handleGoQuadruped();
  void handleGoBiped();
  void printNotWalkingMessage();

  // Update all state information
  void refreshState();

  // Deal with achd crashes/failures
  void achdExitError(QProcess::ProcessError err);
  void achdExitFinished(int num);
  void achdConnectSlot();
  void achdDisconnectSlot();
  
  void ipEditHandle(const QString &text);

private:

  ///////////////
  void initializeCommandTab();
  void initializeZmpBipedParamTab();
  void initializeZmpQuadrupedParamTab();
  void initializeBalParamTab();
  



};

class HuboWalkPanel: public rviz::Panel
{
Q_OBJECT
public:
    HuboWalkPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:

    HuboWalkWidget* content;

};

} // end namespace rviz_plugin_tutorials


#endif // TELEOP_PANEL_H

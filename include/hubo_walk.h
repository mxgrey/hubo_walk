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
#include <zmp-daemon.h>
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

class ZmpProfile
{
public:
    QString name;
    zmp_cmd_t vals;
};

class BalProfile
{
public:
    QString name;
    balance_gains_t vals;
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
  QProcess achChannelBal;
  QProcess achdZmp;
  bool zmpConnected;
  QProcess achdBal;
  bool balConnected;
  
  // Update timer
  HuboRefreshManager* refreshManager;
  int getRefreshTime();

  // Ach Channels for sending and receiving data
  ach_channel_t stateChan;
  bool stateOpen;
  ach_channel_t zmpCmdChan;
  ach_channel_t balanceParamChan;
  bool zmpCmdOpen;

  void initializeAchConnections();
  void initializeAchStructs();
  void sendCommand();

  void setIPAddress(int a, int b, int c, int d);
  int getIPAddress(int index);

  // Structs for storing data to transmit
  // TODO: Make the correct structs
  struct zmp_cmd cmd;
  struct balance_gains balParams;
  
  
  // Handling profiles TODO
  //std::vector<zmp_params> profiles;
  QVector<ZmpProfile> zmpProfiles;
  void fillProfile(zmp_cmd_t &vals);
  void updateProfileBox();
  QVector<BalProfile> balProfiles;
  void fillbalProfile(balance_gains_t &vals);
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
  ///////////////
    
    
    // TODO: Update all the following
    ///////////////
    QWidget* zmpParamTab;
    
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
      
      QComboBox* walkTypeSelect;
      QComboBox* ikSenseSelect;
      
      QDoubleSpinBox* liftoffHeightBox;
      QDoubleSpinBox* stepDistanceBox;
      QDoubleSpinBox* sideStepDistanceBox;
      QDoubleSpinBox* lateralDistanceBox;
      
      QDoubleSpinBox* comHeightBox;
      QDoubleSpinBox* comIKAngleWeightBox;
    ///////////////

      
      ik_error_sensitivity int2ikSense(int index);
      int ikSense2int(ik_error_sensitivity ik_sense);


    QWidget* balParamTab;

      QComboBox* balProfileSelect;
      QPushButton* savebalProfile;
      QPushButton* deletebalProfile;
      QPushButton* saveAsbalProfile;
      QLineEdit* balSaveAsEdit;

      QDoubleSpinBox* flattenBoxL;
      QDoubleSpinBox* flattenBoxR;
      QDoubleSpinBox* decayBox;
      QDoubleSpinBox* threshMinBoxL;
      QDoubleSpinBox* threshMinBoxR;
      QDoubleSpinBox* threshMaxBoxL;
      QDoubleSpinBox* threshMaxBoxR;

      QDoubleSpinBox* straightenPBoxL;
      QDoubleSpinBox* straightenPBoxR;

      QDoubleSpinBox* straightenRBoxL;
      QDoubleSpinBox* straightenRBoxR;

      QDoubleSpinBox* springBoxL;
      QDoubleSpinBox* springBoxR;
      QDoubleSpinBox* dampBoxL;
      QDoubleSpinBox* dampBoxR;
      QDoubleSpinBox* responseBoxL;
      QDoubleSpinBox* responseBoxR;

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

  // Handle button events
  void handleProfileSave();
  void handleProfileDelete();
  void handleProfileSaveAs();
  void handleProfileSelect(int index);

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
  void initializeZmpParamTab();
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

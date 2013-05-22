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


void HuboWalkWidget::handleForward()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = WALKING_FORWARD;
    sendCommand();
}

void HuboWalkWidget::handleBackward()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = WALKING_BACKWARD;
}

void HuboWalkWidget::handleLeft()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = SIDESTEPPING_LEFT;
}

void HuboWalkWidget::handleRight()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = SIDESTEPPING_RIGHT;
}

void HuboWalkWidget::handleTurnLeft()
{
    cmd.walk_type = walk_circle;
    cmd.cmd_state = WALKING_FORWARD;
}

void HuboWalkWidget::handleTurnRight()
{
    cmd.walk_type = walk_circle;
    cmd.cmd_state = WALKING_FORWARD;
}

void HuboWalkWidget::handleStop()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = STOP;
}

void HuboWalkWidget::sendCommand()
{
    fillProfile(cmd);
    cmd.walk_dist = walkDistanceBox->value();
    cmd.walk_circle_radius = radiusBox->value();
    cmd.walk_continuous = continuousBox->isChecked();
    ach_put(&zmpCmdChan, &cmd, sizeof(cmd));
}

void HuboWalkWidget::fillProfile(zmp_cmd_t &vals)
{
    vals.max_step_count = maxStepBox->value();
    vals.step_length = stepDistanceBox->value() ;
    vals.footstep_y = lateralDistanceBox->value() ;
    vals.foot_liftoff_z = liftoffHeightBox->value() ;
    vals.sidestep_length = sideStepDistanceBox->value() ;
    vals.com_height = comHeightBox->value() ;
    vals.com_ik_ascl = comIKAngleWeightBox->value() ;
    vals.zmpoff_y = yOffsetBox->value() ;
    vals.zmpoff_x = xOffsetBox->value() ;
    vals.lookahead_time = lookAheadBox->value() ;
    vals.startup_time = startupTimeBox->value() ;
    vals.shutdown_time = shutdownTimeBox->value() ;
    vals.double_support_time = doubleSupportBox->value() ;
    vals.single_support_time = singleSupportBox->value() ;
    vals.zmp_jerk_penalty = jerkPenalBox->value() ;
    vals.ik_sense = int2ikSense(ikSenseSelect->currentIndex()) ;
}

void HuboWalkWidget::handleProfileSave()
{
    int index = profileSelect->currentIndex();
    fillProfile(zmpProfiles[index].vals);
    
    saveAsEdit->setText("Remember to save your RViz session!");
}

void HuboWalkWidget::handleProfileSelect(int index)
{
    maxStepBox->setValue(zmpProfiles[index].vals.max_step_count );
    stepDistanceBox->setValue(zmpProfiles[index].vals.step_length ) ;
    lateralDistanceBox->setValue(zmpProfiles[index].vals.footstep_y ) ;
    liftoffHeightBox->setValue(zmpProfiles[index].vals.foot_liftoff_z ) ;
    sideStepDistanceBox->setValue(zmpProfiles[index].vals.sidestep_length ) ;
    comHeightBox->setValue(zmpProfiles[index].vals.com_height ) ;
    comIKAngleWeightBox->setValue(zmpProfiles[index].vals.com_ik_ascl ) ;
    yOffsetBox->setValue(zmpProfiles[index].vals.zmpoff_y ) ;
    xOffsetBox->setValue(zmpProfiles[index].vals.zmpoff_x ) ;
    lookAheadBox->setValue(zmpProfiles[index].vals.lookahead_time ) ;
    startupTimeBox->setValue(zmpProfiles[index].vals.startup_time ) ;
    shutdownTimeBox->setValue(zmpProfiles[index].vals.shutdown_time ) ;
    doubleSupportBox->setValue(zmpProfiles[index].vals.double_support_time ) ;
    singleSupportBox->setValue(zmpProfiles[index].vals.single_support_time ) ;
    jerkPenalBox->setValue(zmpProfiles[index].vals.zmp_jerk_penalty ) ;
    ikSenseSelect->setCurrentIndex(ikSense2int(zmpProfiles[index].vals.ik_sense));
    
    saveAsEdit->clear();
}

void HuboWalkWidget::handleProfileDelete()
{
    zmpProfiles.remove(profileSelect->currentIndex());
    updateProfileBox();
}

void HuboWalkWidget::handleProfileSaveAs()
{
    ZmpProfile tempProf;
    tempProf.name = saveAsEdit->text();
    fillProfile(tempProf.vals);
    zmpProfiles.append(tempProf);
    updateProfileBox();
    profileSelect->setCurrentIndex(profileSelect->findText(tempProf.name));
    
    saveAsEdit->setText("Remember to save your RViz session!");
}

void HuboWalkWidget::updateProfileBox()
{
    profileSelect->clear();
    for(int i=0; i < zmpProfiles.size(); i++)
        profileSelect->addItem(zmpProfiles[i].name);
}

void HuboWalkWidget::handleJoyLaunch()
{
    
}

ik_error_sensitivity HuboWalkWidget::int2ikSense(int index)
{
    switch(index)
    {
    case 0:
        return ik_strict; break;
    case 1:
        return ik_swing_permissive; break;
    case 2:
        return ik_sloppy; break;
    }
}

int HuboWalkWidget::ikSense2int(ik_error_sensitivity ik_sense)
{
    switch(ik_sense)
    {
    case ik_strict:
        return 0; break;
    case ik_swing_permissive:
        return 1; break;
    case ik_sloppy:
        return 2; break;
    }
}


void HuboWalkWidget::setIPAddress(int a, int b, int c, int d)
{
    ipAddrA = a;
    ipAddrB = b;
    ipAddrC = c;
    ipAddrD = d;

    ipAddrAEdit->setText(QString::number(ipAddrA));
    ipAddrBEdit->setText(QString::number(ipAddrB));
    ipAddrCEdit->setText(QString::number(ipAddrC));
    ipAddrDEdit->setText(QString::number(ipAddrD));
}

int HuboWalkWidget::getIPAddress(int index)
{
    switch(index)
    {
    case 0:
        return ipAddrA; break;
    case 1:
        return ipAddrB; break;
    case 2:
        return ipAddrC; break;
    case 3:
        return ipAddrD; break;
    }
}


void HuboWalkWidget::ipEditHandle(const QString &text)
{
    ipAddrA = ipAddrAEdit->text().toInt();
    ipAddrB = ipAddrBEdit->text().toInt();
    ipAddrC = ipAddrCEdit->text().toInt();
    ipAddrD = ipAddrDEdit->text().toInt();
}

void HuboWalkWidget::initializeAchConnections()
{
    achChannelZmp.start("ach mk " + QString::fromLocal8Bit(CHAN_ZMP_CMD_NAME)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
}

void HuboWalkWidget::achdConnectSlot()
{
    achdZmp.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(CHAN_ZMP_CMD_NAME));
}

}

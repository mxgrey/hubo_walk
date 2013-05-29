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
    sendCommand();
}

void HuboWalkWidget::handleLeft()
{
    cmd.walk_type = walk_sidestep;
    cmd.cmd_state = SIDESTEPPING_LEFT;
    sendCommand();
}

void HuboWalkWidget::handleRight()
{
    cmd.walk_type = walk_sidestep;
    cmd.cmd_state = SIDESTEPPING_RIGHT;
    sendCommand();
}

void HuboWalkWidget::handleTurnLeft()
{
    cmd.walk_type = walk_circle;
    cmd.cmd_state = WALKING_FORWARD;
    sendCommand();
}

void HuboWalkWidget::handleTurnRight()
{
    cmd.walk_type = walk_circle;
    cmd.cmd_state = WALKING_FORWARD;
    sendCommand();
}

void HuboWalkWidget::handleStop()
{
    cmd.walk_type = walk_line;
    cmd.cmd_state = STOP;
    sendCommand();
}

void HuboWalkWidget::sendCommand()
{
    fillProfile(cmd);
    cmd.walk_dist = walkDistanceBox->value();
    cmd.sidewalk_dist = walkDistanceBox->value();
    cmd.walk_circle_radius = radiusBox->value();
    cmd.walk_continuous = continuousBox->isChecked();
    ach_status_t r = ach_put(&zmpCmdChan, &cmd, sizeof(cmd));
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;
    sendBalParams();
}

void HuboWalkWidget::sendBalParams()
{
    fillbalProfile(balParams);
    ach_status_t r = ach_put(&balanceParamChan, &balParams, sizeof(balParams));
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;
    else
        std::cout << "Parameters sent" << std::endl;
}


void HuboWalkWidget::fillProfile(zmp_cmd_t &vals)
{
    vals.max_step_count = maxStepBox->value();
    vals.step_length = stepDistanceBox->value() ;
    vals.halfStanceWidth = lateralDistanceBox->value() ;
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
    vals.zmp_jerk_penalty = jerkPenalBox->value()*penalFactor ;
    vals.ik_sense = int2ikSense(ikSenseSelect->currentIndex()) ;
}

void HuboWalkWidget::handleProfileSave()
{
    int index = profileSelect->currentIndex();
    fillProfile(zmpProfiles[index].vals);
    
    saveAsEdit->clear();
    saveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::handleProfileSelect(int index)
{
    maxStepBox->setValue(zmpProfiles[index].vals.max_step_count );
    stepDistanceBox->setValue(zmpProfiles[index].vals.step_length ) ;
    lateralDistanceBox->setValue(zmpProfiles[index].vals.halfStanceWidth ) ;
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
    jerkPenalBox->setValue(zmpProfiles[index].vals.zmp_jerk_penalty/penalFactor ) ;
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

    saveAsEdit->clear();
    saveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::updateProfileBox()
{
    profileSelect->clear();
    for(int i=0; i < zmpProfiles.size(); i++)
        profileSelect->addItem(zmpProfiles[i].name);
}


///////////// BALANCE

void HuboWalkWidget::fillbalProfile(balance_gains_t &vals)
{
    vals.flattening_gain[LEFT] = flattenBoxL->value();
    vals.flattening_gain[RIGHT] = flattenBoxR->value() ;
    vals.decay_gain[LEFT] = decayBox->value() ;
    vals.decay_gain[RIGHT] = decayBox->value() ;
    vals.force_min_threshold[LEFT] = threshMinBoxL->value();
    vals.force_min_threshold[RIGHT] = threshMinBoxR->value();
    vals.force_max_threshold[LEFT] = threshMaxBoxL->value();
    vals.force_max_threshold[RIGHT] = threshMaxBoxR->value();
    vals.straightening_pitch_gain[LEFT] = straightenPBoxL->value() ;
    vals.straightening_pitch_gain[RIGHT] = straightenPBoxR->value() ;
    vals.straightening_roll_gain[LEFT] = straightenRBoxL->value() ;
    vals.straightening_roll_gain[RIGHT] = straightenRBoxR->value() ;
    vals.spring_gain[LEFT] = springBoxL->value() ;
    vals.spring_gain[RIGHT] = springBoxR->value() ;
    vals.damping_gain[LEFT] = dampBoxL->value() ;
    vals.damping_gain[RIGHT] = dampBoxR->value() ;
    vals.fz_response[LEFT] = responseBoxL->value() ;
    vals.fz_response[RIGHT] = responseBoxR->value() ;
}

void HuboWalkWidget::handlebalProfileSave()
{
    int index = balProfileSelect->currentIndex();
    fillbalProfile(balProfiles[index].vals);

    balSaveAsEdit->clear();
    balSaveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::handlebalProfileSelect(int index)
{
    flattenBoxL->setValue( balProfiles[index].vals.flattening_gain[LEFT] );
    flattenBoxR->setValue( balProfiles[index].vals.flattening_gain[RIGHT] );
    decayBox->setValue( balProfiles[index].vals.decay_gain[LEFT] );
    threshMinBoxL->setValue( balProfiles[index].vals.force_min_threshold[LEFT] );
    threshMinBoxR->setValue( balProfiles[index].vals.force_min_threshold[RIGHT] );
    threshMaxBoxL->setValue( balProfiles[index].vals.force_max_threshold[LEFT] );
    threshMaxBoxR->setValue( balProfiles[index].vals.force_max_threshold[RIGHT] );
    straightenPBoxL->setValue( balProfiles[index].vals.straightening_pitch_gain[LEFT] );
    straightenPBoxR->setValue( balProfiles[index].vals.straightening_pitch_gain[RIGHT] );
    straightenRBoxL->setValue( balProfiles[index].vals.straightening_roll_gain[LEFT] );
    straightenRBoxR->setValue( balProfiles[index].vals.straightening_roll_gain[RIGHT] );
    springBoxL->setValue( balProfiles[index].vals.spring_gain[LEFT] );
    springBoxR->setValue( balProfiles[index].vals.spring_gain[RIGHT] );
    dampBoxL->setValue( balProfiles[index].vals.damping_gain[LEFT] );
    dampBoxR->setValue( balProfiles[index].vals.damping_gain[RIGHT] );
    responseBoxL->setValue( balProfiles[index].vals.fz_response[LEFT] );
    responseBoxR->setValue( balProfiles[index].vals.fz_response[RIGHT] );

    balSaveAsEdit->clear();
}

void HuboWalkWidget::handlebalProfileDelete()
{
    balProfiles.remove(balProfileSelect->currentIndex());
    updatebalProfileBox();
}

void HuboWalkWidget::handlebalProfileSaveAs()
{
    BalProfile tempProf;
    tempProf.name = balSaveAsEdit->text();
    fillbalProfile(tempProf.vals);
    balProfiles.append(tempProf);
    updatebalProfileBox();
    balProfileSelect->setCurrentIndex(balProfileSelect->findText(tempProf.name));

    balSaveAsEdit->clear();
    balSaveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::updatebalProfileBox()
{
    balProfileSelect->clear();
    for(int i=0; i < balProfiles.size(); i++)
        balProfileSelect->addItem(balProfiles[i].name);
}


///////////// END: BALANCE




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
    achChannelZmp.waitForFinished();
    ach_status_t r = ach_open(&zmpCmdChan, CHAN_ZMP_CMD_NAME, NULL);
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;

    achChannelBal.start("ach mk " + QString::fromLocal8Bit(BALANCE_PARAM_CHAN)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    achChannelBal.waitForFinished();
    r = ach_open(&balanceParamChan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;
}

void HuboWalkWidget::achdConnectSlot()
{
    achdZmp.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(CHAN_ZMP_CMD_NAME));
    connect(&achdZmp, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdZmp, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));
    achdBal.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(BALANCE_PARAM_CHAN));
    connect(&achdBal, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdBal, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));
    statusLabel->setText("Connected");
}

void HuboWalkWidget::achdDisconnectSlot()
{
    achdZmp.kill();
    achdBal.kill();
    statusLabel->setText("Disconnected");
}

void HuboWalkWidget::achdExitError(QProcess::ProcessError err)
{
    statusLabel->setText("Disconnected");
}

void HuboWalkWidget::achdExitFinished(int num)
{
    statusLabel->setText("Disconnected");
}

}

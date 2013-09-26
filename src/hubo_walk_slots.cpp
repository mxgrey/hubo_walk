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

void HuboWalkWidget::refreshState()
{
#ifdef HAVE_HUBOMZ
    size_t fs;
    ach_status_t r = ach_get( &zmpStateChan, &zmpState, sizeof(zmpState), &fs, NULL, ACH_O_LAST );
    if(!ACH_OK && !ACH_MISSED_FRAME && !ACH_STALE_FRAMES)
        std::cout << "ZMP State ach_get result: " << ach_result_to_string(r) << "\n";
    zmpResultEdit->setText(QString::fromStdString(zmp_result_to_string(zmpState.result)));
    walkModeEdit->setText(QString::fromStdString(walkMode_to_string(zmpState.walkMode)));
    tempWalkMode = zmpState.walkMode;
#endif

}

#ifdef HAVE_HUBOMZ
void HuboWalkWidget::handleForward()
{
    cmd.walk_type = WALK_FORWARD;
    sendCommand();
}

void HuboWalkWidget::handleBackward()
{
    cmd.walk_type = WALK_BACKWARD;
    sendCommand();
}

void HuboWalkWidget::handleLeft()
{
    cmd.walk_type = SIDESTEP_LEFT;
    sendCommand();
}

void HuboWalkWidget::handleRight()
{
    cmd.walk_type = SIDESTEP_RIGHT;
    sendCommand();
}

void HuboWalkWidget::handleTurnLeft()
{
    cmd.walk_type = ROTATE_LEFT;
    sendCommand();
}

void HuboWalkWidget::handleTurnRight()
{
    cmd.walk_type = ROTATE_RIGHT;
    sendCommand();
}

void HuboWalkWidget::handleStop()
{
    cmd.walk_type = STOP_WALKING;
    sendCommand();
}

void HuboWalkWidget::handleGoQuadruped()
{
    zmpState.walkMode = QUADRUPED_MODE;
    cmd.walk_type = GOTO_QUADRUPED;
    sendCommand();
    zmpState.walkMode = tempWalkMode;
}

void HuboWalkWidget::handleGoBiped()
{
    zmpState.walkMode = BIPED_MODE;
    cmd.walk_type = GOTO_BIPED;
    sendCommand();
    zmpState.walkMode = tempWalkMode;
}

#else
void HuboWalkWidget::handleForward()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleBackward()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleLeft()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleRight()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleTurnLeft()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleTurnRight()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleStop()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleGoQuadruped()
{
    printNotWalkingMessage();
}

void HuboWalkWidget::handleGoBiped()
{
    printNotWalkingMessage();
}
void HuboWalkWidget::printNotWalkingMessage()
{
    std::cout << "\n滚开！この野郎! 당신은나쁜놈! Get lost."
              << "There's no zmp-daemon to send commands to"
              << std::endl;
}


#endif // HAVE_HUBOMZ

void HuboWalkWidget::sendCommand()
{
#ifdef HAVE_HUBOMZ
    bool fillSuccess = false;
    fillSuccess = fillProfile(cmd, zmpState.walkMode);
    if(fillSuccess)
    {
        cmd.params.walk_dist = walkDistanceBox->value();
        cmd.params.sidewalk_dist = walkDistanceBox->value();
        cmd.params.turn_in_place_angle = rotateAngleBox->value();
        cmd.params.walk_circle_radius = radiusBox->value();
        cmd.walk_continuous = continuousBox->isChecked();
        ach_status_t r = ach_put(&zmpCmdChan, &cmd, sizeof(cmd));
        if( r != ACH_OK )
            std::cout << "ZMP Command Ach Error: " << ach_result_to_string(r) << std::endl;
    }
    else
        std::cout << "Invalid walk mode requested. Check headers. Command not sent" << std::endl;
#endif // HAVE_HUBOMZ

    balCmd.cmd_request = BAL_ZMP_WALKING;
    sendBalCommand();
    sendBalParams();
}

void HuboWalkWidget::handleStaticButton()
{
    sendBalParams();
    balCmd.cmd_request = BAL_LEGS_ONLY;
    sendBalCommand();
}

void HuboWalkWidget::handleBalOffButton()
{
    balCmd.cmd_request = BAL_READY;
    sendBalCommand();
}

void HuboWalkWidget::sendBalCommand()
{

    balCmd.height = heightSlide->value()/heightScale;
    balCmd.com_x_offset = comXOffsetBox->value();
    ach_status_t r = ach_put( &balanceCmdChan, &balCmd, sizeof(balCmd) );
    if( r != ACH_OK )
        std::cout << "Balance Command Ach Error: " << ach_result_to_string(r) << std::endl;

}

void HuboWalkWidget::sendBalParams()
{
    fillbalProfile(balParams);
    ach_status_t r = ach_put(&balanceParamChan, &balParams, sizeof(balParams));
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;
}

#ifdef HAVE_HUBOMZ
bool HuboWalkWidget::fillProfile(zmp_cmd_t &vals, const walkMode_t walkMode)
{
    if(BIPED_MODE == walkMode)
    {
        vals.params.max_step_count = maxStepBox->value();
        vals.params.step_length = stepDistanceBox->value() ;
        vals.params.half_stance_width = lateralDistanceBox->value() ;
        vals.params.step_height = liftoffHeightBox->value() ;
        vals.params.sidestep_length = sideStepDistanceBox->value() ;
        vals.params.com_height = comHeightBox->value() ;
        vals.params.torso_pitch = torsoPitchBox->value() ;
        vals.params.com_ik_angle_weight = comIKAngleWeightBox->value() ;
        vals.params.constant_body_z = constantBodyZBox->isChecked() ;
        vals.params.zmpoff_y = yOffsetBox->value() ;
        vals.params.zmpoff_x = xOffsetBox->value() ;
        vals.params.lookahead_time = lookAheadBox->value() ;
        vals.params.walk_startup_time = startupTimeBox->value() ;
        vals.params.walk_shutdown_time = shutdownTimeBox->value() ;
        vals.params.min_double_support_time = doubleSupportBox->value() ;
        vals.params.min_single_support_time = singleSupportBox->value() ;
        vals.params.min_pause_time = pauseTimeBox->value() ;
        vals.params.quad_transition_time = transitionToQuadTimeBox->value() ;
        vals.params.quad_stance_length = quadStanceLengthBoxQuad->value() ;
        vals.params.quad_stability_margin = quadStabilityMarginBoxQuad->value() ;
        vals.params.half_peg_width = halfPegWidthBoxQuad->value() ;
        vals.params.zmp_R = jerkPenalBox->value()*penalFactor ;
        vals.params.ik_sense = int2ikSense(ikSenseSelect->currentIndex()) ;
        return true;
    }
    else if(QUADRUPED_MODE == walkMode)
    {
        vals.params.max_step_count = maxStepBox->value();
        vals.params.step_length = stepDistanceBoxQuad->value() ;
        vals.params.half_stance_width = lateralDistanceBoxQuad->value() ;
        vals.params.step_height = liftoffHeightBoxQuad->value() ;
        vals.params.sidestep_length = sideStepDistanceBoxQuad->value() ;
        vals.params.com_height = comHeightBoxQuad->value() ;
        vals.params.torso_pitch = torsoPitchBoxQuad->value() ;
        vals.params.com_ik_angle_weight = comIKAngleWeightBoxQuad->value() ;
        vals.params.constant_body_z = constantBodyZBoxQuad->isChecked() ;
        vals.params.zmpoff_y = yOffsetBoxQuad->value() ;
        vals.params.zmpoff_x = xOffsetBoxQuad->value() ;
        vals.params.lookahead_time = lookAheadBoxQuad->value() ;
        vals.params.walk_startup_time = startupTimeBoxQuad->value() ;
        vals.params.walk_shutdown_time = shutdownTimeBoxQuad->value() ;
        vals.params.min_double_support_time = doubleSupportBoxQuad->value() ;
        vals.params.min_single_support_time = singleSupportBoxQuad->value() ;
        vals.params.min_pause_time = pauseTimeBoxQuad->value() ;
        vals.params.quad_transition_time = transitionToBipedTimeBoxQuad->value() ;
        vals.params.quad_stance_length = quadStanceLengthBoxQuad->value() ;
        vals.params.quad_stability_margin = quadStabilityMarginBoxQuad->value() ;
        vals.params.half_peg_width = halfPegWidthBoxQuad->value() ;
        vals.params.zmp_R = jerkPenalBoxQuad->value()*penalFactor ;
        vals.params.ik_sense = int2ikSense(ikSenseSelectQuad->currentIndex()) ;
        return true;
    }
    else
    {
        std::cout << "Invalid walk mode in \"fillProfile\" function" << std::endl;
        return false;
    }
}
#endif // HAVE_HUBOMZ

#ifdef HAVE_HUBOMZ
void HuboWalkWidget::handleProfileSave()
{
    int index = profileSelect->currentIndex();
    fillProfile(zmpProfiles[index].vals, BIPED_MODE);
    
    saveAsEdit->clear();
    saveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::handleQuadrupedProfileSave()
{
    int index = profileSelectQuad->currentIndex();
    fillProfile(zmpQuadProfiles[index].vals, QUADRUPED_MODE);
    
    saveAsEditQuad->clear();
    saveAsEditQuad->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::handleProfileSelect(int index)
{
    maxStepBox->setValue(zmpProfiles[index].vals.params.max_step_count );
    stepDistanceBox->setValue(zmpProfiles[index].vals.params.step_length ) ;
    lateralDistanceBox->setValue(zmpProfiles[index].vals.params.half_stance_width ) ;
    liftoffHeightBox->setValue(zmpProfiles[index].vals.params.step_height ) ;
    sideStepDistanceBox->setValue(zmpProfiles[index].vals.params.sidestep_length ) ;
    comHeightBox->setValue(zmpProfiles[index].vals.params.com_height ) ;
    torsoPitchBox->setValue(zmpProfiles[index].vals.params.torso_pitch ) ;
    comIKAngleWeightBox->setValue(zmpProfiles[index].vals.params.com_ik_angle_weight ) ;
    constantBodyZBox->setChecked(zmpProfiles[index].vals.params.constant_body_z ) ;
    yOffsetBox->setValue(zmpProfiles[index].vals.params.zmpoff_y ) ;
    xOffsetBox->setValue(zmpProfiles[index].vals.params.zmpoff_x ) ;
    lookAheadBox->setValue(zmpProfiles[index].vals.params.lookahead_time ) ;
    startupTimeBox->setValue(zmpProfiles[index].vals.params.walk_startup_time ) ;
    shutdownTimeBox->setValue(zmpProfiles[index].vals.params.walk_shutdown_time ) ;
    doubleSupportBox->setValue(zmpProfiles[index].vals.params.min_double_support_time ) ;
    singleSupportBox->setValue(zmpProfiles[index].vals.params.min_single_support_time ) ;
    pauseTimeBox->setValue(zmpProfiles[index].vals.params.min_pause_time ) ;
    transitionToQuadTimeBox->setValue(zmpProfiles[index].vals.params.quad_transition_time ) ;
    jerkPenalBox->setValue(zmpProfiles[index].vals.params.zmp_R/penalFactor ) ;
    ikSenseSelect->setCurrentIndex(ikSense2int(zmpProfiles[index].vals.params.ik_sense));
    
    saveAsEdit->clear();
}

void HuboWalkWidget::handleQuadrupedProfileSelect(int index)
{
    maxStepBox->setValue(zmpProfiles[index].vals.params.max_step_count );
    stepDistanceBoxQuad->setValue(zmpQuadProfiles[index].vals.params.step_length ) ;
    lateralDistanceBoxQuad->setValue(zmpQuadProfiles[index].vals.params.half_stance_width ) ;
    liftoffHeightBoxQuad->setValue(zmpQuadProfiles[index].vals.params.step_height ) ;
    sideStepDistanceBoxQuad->setValue(zmpQuadProfiles[index].vals.params.sidestep_length ) ;
    comHeightBoxQuad->setValue(zmpQuadProfiles[index].vals.params.com_height ) ;
    torsoPitchBoxQuad->setValue(zmpQuadProfiles[index].vals.params.torso_pitch ) ;
    comIKAngleWeightBoxQuad->setValue(zmpQuadProfiles[index].vals.params.com_ik_angle_weight ) ;
    constantBodyZBoxQuad->setChecked(zmpQuadProfiles[index].vals.params.constant_body_z ) ;
    yOffsetBoxQuad->setValue(zmpQuadProfiles[index].vals.params.zmpoff_y ) ;
    xOffsetBoxQuad->setValue(zmpQuadProfiles[index].vals.params.zmpoff_x ) ;
    lookAheadBoxQuad->setValue(zmpQuadProfiles[index].vals.params.lookahead_time ) ;
    startupTimeBoxQuad->setValue(zmpQuadProfiles[index].vals.params.walk_startup_time ) ;
    shutdownTimeBoxQuad->setValue(zmpQuadProfiles[index].vals.params.walk_shutdown_time ) ;
    doubleSupportBoxQuad->setValue(zmpQuadProfiles[index].vals.params.min_double_support_time ) ;
    singleSupportBoxQuad->setValue(zmpQuadProfiles[index].vals.params.min_single_support_time ) ;
    pauseTimeBoxQuad->setValue(zmpQuadProfiles[index].vals.params.min_pause_time ) ;
    transitionToBipedTimeBoxQuad->setValue(zmpQuadProfiles[index].vals.params.quad_transition_time ) ;
    quadStanceLengthBoxQuad->setValue(zmpQuadProfiles[index].vals.params.quad_stance_length ) ;
    quadStabilityMarginBoxQuad->setValue(zmpQuadProfiles[index].vals.params.quad_stability_margin ) ;
    jerkPenalBoxQuad->setValue(zmpQuadProfiles[index].vals.params.zmp_R/penalFactor ) ;
    ikSenseSelectQuad->setCurrentIndex(ikSense2int(zmpQuadProfiles[index].vals.params.ik_sense));
    
    saveAsEditQuad->clear();
}

void HuboWalkWidget::handleProfileDelete()
{
    zmpProfiles.remove(profileSelect->currentIndex());
    updateProfileBox();
}

void HuboWalkWidget::handleQuadrupedProfileDelete()
{
    zmpQuadProfiles.remove(profileSelectQuad->currentIndex());
    updateQuadrupedProfileBox();
}

void HuboWalkWidget::handleProfileSaveAs()
{
    ZmpProfile tempProf;
    tempProf.name = saveAsEdit->text();
    fillProfile(tempProf.vals, BIPED_MODE);
    zmpProfiles.append(tempProf);
    updateProfileBox();
    profileSelect->setCurrentIndex(profileSelect->findText(tempProf.name));

    saveAsEdit->clear();
    saveAsEdit->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::handleQuadrupedProfileSaveAs()
{
    ZmpProfile tempProf;
    tempProf.name = saveAsEditQuad->text();
    fillProfile(tempProf.vals, QUADRUPED_MODE);
    zmpQuadProfiles.append(tempProf);
    updateQuadrupedProfileBox();
    profileSelectQuad->setCurrentIndex(profileSelectQuad->findText(tempProf.name));

    saveAsEditQuad->clear();
    saveAsEditQuad->setPlaceholderText("Remember to save your RViz session! (Ctrl-S)");
}

void HuboWalkWidget::updateProfileBox()
{
    profileSelect->clear();
    for(int i=0; i < zmpProfiles.size(); i++)
        profileSelect->addItem(zmpProfiles[i].name);
}

void HuboWalkWidget::updateQuadrupedProfileBox()
{
    profileSelectQuad->clear();
    for(int i=0; i < zmpQuadProfiles.size(); i++)
        profileSelectQuad->addItem(zmpQuadProfiles[i].name);
}
#endif // HAVE_HUBOMZ

///////////// BALANCE

void HuboWalkWidget::fillbalProfile(balance_params_t &vals)
{
    // Fill in balance gains
    vals.balance_gains.flattening_gain = flattenBoxBal->value();
    vals.balance_gains.decay_gain = decayBoxBal->value() ;
    vals.balance_gains.force_min_threshold = threshMinBoxBal->value();
    vals.balance_gains.force_max_threshold = threshMaxBoxBal->value();
    vals.balance_gains.straightening_pitch_gain = straightenPBoxBal->value() ;
    vals.balance_gains.straightening_roll_gain = straightenRBoxBal->value() ;
    vals.balance_gains.spring_gain = springBoxBal->value() ;
    vals.balance_gains.damping_gain = dampBoxBal->value() ;
    vals.balance_gains.fz_response = responseBoxBal->value() ;
    vals.balance_gains.single_support_hip_nudge_kp = singleSupportHipNudgeGainBoxP->value();
    vals.balance_gains.single_support_hip_nudge_kd = singleSupportHipNudgeGainBoxD->value();
    vals.balance_gains.double_support_hip_nudge_kp = doubleSupportHipNudgeGainBoxP->value();
    vals.balance_gains.double_support_hip_nudge_kd = doubleSupportHipNudgeGainBoxD->value();

    // Fill in walking gains
    vals.walking_gains.flattening_gain = flattenBoxWalk->value() ;
    vals.walking_gains.decay_gain = decayBoxWalk->value() ;
    vals.walking_gains.force_min_threshold = threshMinBoxWalk->value();
    vals.walking_gains.force_max_threshold = threshMaxBoxWalk->value();
    vals.walking_gains.straightening_pitch_gain = straightenPBoxWalk->value() ;
    vals.walking_gains.straightening_roll_gain = straightenRBoxWalk->value() ;
    vals.walking_gains.spring_gain = springBoxWalk->value() ;
    vals.walking_gains.damping_gain = dampBoxWalk->value() ;
    vals.walking_gains.fz_response = responseBoxWalk->value() ;

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
    // Set balance gains
    flattenBoxBal->setValue( balProfiles[index].vals.balance_gains.flattening_gain );
    decayBoxBal->setValue( balProfiles[index].vals.balance_gains.decay_gain );
    threshMinBoxBal->setValue( balProfiles[index].vals.balance_gains.force_min_threshold );
    threshMaxBoxBal->setValue( balProfiles[index].vals.balance_gains.force_max_threshold );
    straightenPBoxBal->setValue( balProfiles[index].vals.balance_gains.straightening_pitch_gain );
    straightenRBoxBal->setValue( balProfiles[index].vals.balance_gains.straightening_roll_gain );
    springBoxBal->setValue( balProfiles[index].vals.balance_gains.spring_gain );
    dampBoxBal->setValue( balProfiles[index].vals.balance_gains.damping_gain );
    responseBoxBal->setValue( balProfiles[index].vals.balance_gains.fz_response );
    singleSupportHipNudgeGainBoxP->setValue( balProfiles[index].vals.balance_gains.single_support_hip_nudge_kp );
    singleSupportHipNudgeGainBoxD->setValue( balProfiles[index].vals.balance_gains.single_support_hip_nudge_kd );
    doubleSupportHipNudgeGainBoxP->setValue( balProfiles[index].vals.balance_gains.double_support_hip_nudge_kp );
    doubleSupportHipNudgeGainBoxD->setValue( balProfiles[index].vals.balance_gains.double_support_hip_nudge_kd );

    // Set walking gains
    flattenBoxWalk->setValue( balProfiles[index].vals.walking_gains.flattening_gain );
    threshMinBoxWalk->setValue( balProfiles[index].vals.walking_gains.force_min_threshold );
    threshMaxBoxWalk->setValue( balProfiles[index].vals.walking_gains.force_max_threshold );
    straightenPBoxWalk->setValue( balProfiles[index].vals.walking_gains.straightening_pitch_gain );
    straightenRBoxWalk->setValue( balProfiles[index].vals.walking_gains.straightening_roll_gain );
    springBoxWalk->setValue( balProfiles[index].vals.walking_gains.spring_gain );
    dampBoxWalk->setValue( balProfiles[index].vals.walking_gains.damping_gain );
    responseBoxWalk->setValue( balProfiles[index].vals.walking_gains.fz_response );

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

#ifdef HAVE_HUBOMZ
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
#endif // HAVE_HUBOMZ

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
    ach_status_t r;
#ifdef HAVE_HUBOMZ
    achChannelZmp.start("ach mk " + QString::fromLocal8Bit(CHAN_ZMP_CMD_NAME)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    achChannelZmp.waitForFinished();
    r = ach_open(&zmpCmdChan, CHAN_ZMP_CMD_NAME, NULL);
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << "for channel "
                  << CHAN_ZMP_CMD_NAME << std::endl;

    achChannelZmpState.start("ach mk " + QString::fromLocal8Bit(HUBO_CHAN_ZMP_STATE_NAME)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    achChannelZmpState.waitForFinished();
    r = ach_open(&zmpStateChan, HUBO_CHAN_ZMP_STATE_NAME, NULL);
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << "for channel " 
                  << HUBO_CHAN_ZMP_STATE_NAME << std::endl;

#endif // HAVE_HUBOMZ

    achChannelBal.start("ach mk " + QString::fromLocal8Bit(BALANCE_PARAM_CHAN)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    achChannelBal.waitForFinished();
    r = ach_open(&balanceParamChan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;

    achChannelBal.start("ach mk " + QString::fromLocal8Bit(BALANCE_CMD_CHAN)
                        + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    achChannelBal.waitForFinished();
    r = ach_open(&balanceCmdChan, BALANCE_CMD_CHAN, NULL );
    if( r != ACH_OK )
        std::cout << "Ach Error: " << ach_result_to_string(r) << std::endl;
}

void HuboWalkWidget::achdConnectSlot()
{
#ifdef HAVE_HUBOMZ
    // For sending zmp commands to hubo
    achdZmp.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(CHAN_ZMP_CMD_NAME));
    connect(&achdZmp, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdZmp, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));

    // For receiving zmp state info from hubo
    achdZmpState.start("achd pull " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(HUBO_CHAN_ZMP_STATE_NAME));
    connect(&achdZmpState, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdZmpState, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));
#endif // HAVE_HUBOMZ
    // For sending balance parameters to hubo
    achdBal.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(BALANCE_PARAM_CHAN));
    connect(&achdBal, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdBal, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));

    // For sending balancing commands to hubo
    achdBalCmd.start("achd push " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(BALANCE_CMD_CHAN));
    connect(&achdBalCmd, SIGNAL(finished(int)), this, SLOT(achdExitFinished(int)));
    connect(&achdBalCmd, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdExitError(QProcess::ProcessError)));

    statusLabel->setText("Connected");
}

void HuboWalkWidget::achdDisconnectSlot()
{
    achdZmp.kill();
    achdZmpState.kill();
    achdBal.kill();
    achdBalCmd.kill();
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

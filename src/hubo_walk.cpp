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
    achdDisconnectSlot();
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
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

#ifdef HAVE_HUBOMZ
    // Biped ZMP Profiles
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
            content->zmpProfiles[i].vals.params.max_step_count = temp.toInt();
            p_config.mapGetValue("step_length"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.step_length = temp.toDouble();
            p_config.mapGetValue("half_stance_width"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.half_stance_width = temp.toDouble();
            p_config.mapGetValue("half_peg_width"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.half_peg_width = temp.toDouble();
            p_config.mapGetValue("step_height"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.step_height = temp.toDouble();
            p_config.mapGetValue("sidestep_length"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.sidestep_length = temp.toDouble();
            p_config.mapGetValue("com_height"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.com_height = temp.toDouble();
            p_config.mapGetValue("torso_pitch"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.torso_pitch = temp.toDouble();
            p_config.mapGetValue("com_ik_angle_weight"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.com_ik_angle_weight = temp.toDouble();
            p_config.mapGetValue("fixed_com_offset_x"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.fixed_com_offset_x = temp.toDouble();
            p_config.mapGetValue("fixed_com_offset_y"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.fixed_com_offset_y = temp.toDouble();
            p_config.mapGetValue("fixed_com_offset_z"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.fixed_com_offset_z = temp.toDouble();
            p_config.mapGetValue("zmpoff_y"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.zmpoff_y = temp.toDouble();
            p_config.mapGetValue("zmpoff_x"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.zmpoff_x = temp.toDouble();
            p_config.mapGetValue("zmp_dist_gain"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.zmp_dist_gain = temp.toDouble();
            p_config.mapGetValue("lookahead_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.lookahead_time = temp.toDouble();
            p_config.mapGetValue("startup_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.walk_startup_time = temp.toDouble();
            p_config.mapGetValue("shutdown_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.walk_shutdown_time = temp.toDouble();
            p_config.mapGetValue("min_double_support_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.min_double_support_time = temp.toDouble();
            p_config.mapGetValue("min_single_support_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.min_single_support_time = temp.toDouble();
            p_config.mapGetValue("min_pause_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.min_pause_time = temp.toDouble();
            p_config.mapGetValue("quad_transition_time"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.quad_transition_time = temp.toDouble();
            p_config.mapGetValue("quad_stance_length"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.quad_stance_length = temp.toDouble();
            p_config.mapGetValue("quad_stability_margin"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.quad_stability_margin = temp.toDouble();
            p_config.mapGetValue("zmp_jerk_penalty"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.zmp_R = temp.toDouble();
            p_config.mapGetValue("ik_sense"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.ik_sense = ik_error_sensitivity(temp.toInt());
            p_config.mapGetValue("zmp_jerk_penalty"+QString::number(i),
                                 &temp);
            content->zmpProfiles[i].vals.params.zmp_R = temp.toDouble();
        }
        
        content->updateProfileBox();
        content->profileSelect->setCurrentIndex(selectedProfile.toInt());
    }
    else
        ROS_INFO("No zmp profiles found");

    // Quadruped ZMP Profiles
    rviz::Config pQuad_config = config.mapGetChild("zmpQuadProfiles");
    QVariant pNumQuad;
    
    if( pQuad_config.mapGetValue("ZmpQuadProfileNum", &pNumQuad) )
    {
        QVariant selectedProfileQuad;
        config.mapGetValue("SelectedZmpQuadProfile", &selectedProfileQuad);
        content->zmpQuadProfiles.resize(size_t(pNumQuad.toInt()));
        
        for(int i=0; i < int(content->zmpQuadProfiles.size()); i++)
        {
            QVariant temp;
            pQuad_config.mapGetValue("ZmpQuadProfileName"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].name = temp.toString();
            pQuad_config.mapGetValue("max_step_count"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.max_step_count = temp.toInt();
            pQuad_config.mapGetValue("step_length"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.step_length = temp.toDouble();
            pQuad_config.mapGetValue("half_stance_width"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.half_stance_width = temp.toDouble();
            pQuad_config.mapGetValue("half_peg_width"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.half_peg_width = temp.toDouble();
            pQuad_config.mapGetValue("step_height"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.step_height = temp.toDouble();
            pQuad_config.mapGetValue("sidestep_length"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.sidestep_length = temp.toDouble();
            pQuad_config.mapGetValue("com_height"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.com_height = temp.toDouble();
            pQuad_config.mapGetValue("torso_pitch"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.torso_pitch = temp.toDouble();
            pQuad_config.mapGetValue("com_ik_angle_weight"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.com_ik_angle_weight = temp.toDouble();
            pQuad_config.mapGetValue("fixed_com_offset_x"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.fixed_com_offset_x = temp.toDouble();
            pQuad_config.mapGetValue("fixed_com_offset_y"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.fixed_com_offset_y = temp.toDouble();
            pQuad_config.mapGetValue("fixed_com_offset_z"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.fixed_com_offset_z = temp.toDouble();
            pQuad_config.mapGetValue("zmpoff_y"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.zmpoff_y = temp.toDouble();
            pQuad_config.mapGetValue("zmpoff_x"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.zmpoff_x = temp.toDouble();
            pQuad_config.mapGetValue("zmp_dist_gain"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.zmp_dist_gain = temp.toDouble();
            pQuad_config.mapGetValue("lookahead_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.lookahead_time = temp.toDouble();
            pQuad_config.mapGetValue("startup_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.walk_startup_time = temp.toDouble();
            pQuad_config.mapGetValue("shutdown_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.walk_shutdown_time = temp.toDouble();
            pQuad_config.mapGetValue("min_double_support_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.min_double_support_time = temp.toDouble();
            pQuad_config.mapGetValue("min_single_support_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.min_single_support_time = temp.toDouble();
            pQuad_config.mapGetValue("min_pause_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.min_pause_time = temp.toDouble();
            pQuad_config.mapGetValue("biped_transition_time"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.quad_transition_time = temp.toDouble();
            pQuad_config.mapGetValue("quad_stance_length"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.quad_stance_length = temp.toDouble();
            pQuad_config.mapGetValue("quad_stability_margin"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.quad_stability_margin = temp.toDouble();
            pQuad_config.mapGetValue("zmp_jerk_penalty"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.zmp_R = temp.toDouble();
            pQuad_config.mapGetValue("ik_sense"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.ik_sense = ik_error_sensitivity(temp.toInt());
            pQuad_config.mapGetValue("zmp_jerk_penalty"+QString::number(i),
                                 &temp);
            content->zmpQuadProfiles[i].vals.params.zmp_R = temp.toDouble();
        }
        
        content->updateQuadrupedProfileBox();
        content->profileSelectQuad->setCurrentIndex(selectedProfileQuad.toInt());
    }
    else
        ROS_INFO("No quad zmp profiles found");

#endif // HAVE_HUBOMZ

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
            pb_config.mapGetValue("flatten_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.flattening_gain = temp.toDouble();
            pb_config.mapGetValue("flatten_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.flattening_gain = temp.toDouble();
            pb_config.mapGetValue("decay_gain_bal"+QString::number(i),
                                  &temp);
            content->balProfiles[i].vals.balance_gains.decay_gain = temp.toDouble();
            pb_config.mapGetValue("decay_gain_walk"+QString::number(i),
                                  &temp);
            content->balProfiles[i].vals.walking_gains.decay_gain = temp.toDouble();
            pb_config.mapGetValue("thresh_min_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.force_min_threshold = temp.toDouble();
            pb_config.mapGetValue("thresh_min_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.force_min_threshold = temp.toDouble();
            pb_config.mapGetValue("thresh_max_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.force_max_threshold = temp.toDouble();
            pb_config.mapGetValue("thresh_max_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.force_max_threshold = temp.toDouble();
            pb_config.mapGetValue("straightenP_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.straightening_pitch_gain = temp.toDouble();
            pb_config.mapGetValue("straightenP_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.straightening_pitch_gain = temp.toDouble();
            pb_config.mapGetValue("straightenR_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.straightening_roll_gain = temp.toDouble();
            pb_config.mapGetValue("straightenR_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.straightening_roll_gain = temp.toDouble();
            pb_config.mapGetValue("spring_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.spring_gain = temp.toDouble();
            pb_config.mapGetValue("spring_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.spring_gain = temp.toDouble();
            pb_config.mapGetValue("damp_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.damping_gain = temp.toDouble();
            pb_config.mapGetValue("damp_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.damping_gain = temp.toDouble();
            pb_config.mapGetValue("response_bal"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.fz_response = temp.toDouble();
            pb_config.mapGetValue("response_walk"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.walking_gains.fz_response = temp.toDouble();
            pb_config.mapGetValue("single_support_hip_nudge_kp"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.single_support_hip_nudge_kp = temp.toDouble();
            pb_config.mapGetValue("single_support_hip_nudge_kd"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.single_support_hip_nudge_kd = temp.toDouble();
            pb_config.mapGetValue("double_support_hip_nudge_kp"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.double_support_hip_nudge_kp = temp.toDouble();
            pb_config.mapGetValue("double_support_hip_nudge_kd"+QString::number(i),
                                 &temp);
            content->balProfiles[i].vals.balance_gains.double_support_hip_nudge_kd = temp.toDouble();
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
    QVariant selectedProfileQuad = QVariant(content->profileSelectQuad->currentIndex());

    config.mapSetValue("SelectedZmpProfile", selectedProfile);
    config.mapSetValue("SelectedZmpQuadProfile", selectedProfileQuad);

    rviz::Config p_config = config.mapMakeChild("ZmpProfiles");
    rviz::Config pQuad_config = config.mapMakeChild("zmpQuadProfiles");

#ifdef HAVE_HUBOMZ
    // Biped zmp params tab 
    QVariant pNum = QVariant(int(content->zmpProfiles.size()));
    p_config.mapSetValue("ZmpProfileNum", pNum);
    
    for(int i=0; i < int(content->zmpProfiles.size()); i++)
    {
        content->zmpProfiles[i].name.replace(" ","_");
        p_config.mapSetValue("ZmpProfileName"+QString::number(i),
                             QVariant(content->zmpProfiles[i].name));
        p_config.mapSetValue("max_step_count"+QString::number(i),
                             QVariant(int(content->zmpProfiles[i].vals.params.max_step_count)));
        p_config.mapSetValue("step_length"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.step_length));
        p_config.mapSetValue("half_stance_width"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.half_stance_width));
        p_config.mapSetValue("half_peg_width"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.half_peg_width));
        p_config.mapSetValue("step_height"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.step_height));
        p_config.mapSetValue("sidestep_length"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.sidestep_length));
        p_config.mapSetValue("com_height"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.com_height));
        p_config.mapSetValue("torso_pitch"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.torso_pitch));
        p_config.mapSetValue("com_ik_angle_weight"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.com_ik_angle_weight));
        p_config.mapSetValue("fixed_com_offset_x"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.fixed_com_offset_x));
        p_config.mapSetValue("fixed_com_offset_y"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.fixed_com_offset_y));
        p_config.mapSetValue("fixed_com_offset_z"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.fixed_com_offset_z));
        p_config.mapSetValue("zmpoff_y"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.zmpoff_y));
        p_config.mapSetValue("zmpoff_x"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.zmpoff_x));
        p_config.mapSetValue("zmp_dist_gain"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.zmp_dist_gain));
        p_config.mapSetValue("lookahead_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.lookahead_time));
        p_config.mapSetValue("startup_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.walk_startup_time));
        p_config.mapSetValue("shutdown_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.walk_shutdown_time));
        p_config.mapSetValue("min_double_support_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.min_double_support_time));
        p_config.mapSetValue("min_single_support_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.min_single_support_time));
        p_config.mapSetValue("min_pause_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.min_pause_time));
        p_config.mapSetValue("quad_transition_time"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.quad_transition_time));
        p_config.mapSetValue("quad_stance_length"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.quad_stance_length));
        p_config.mapSetValue("quad_stability_margin"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.quad_stability_margin));
        p_config.mapSetValue("zmp_jerk_penalty"+QString::number(i),
                             QVariant(content->zmpProfiles[i].vals.params.zmp_R));
        p_config.mapSetValue("ik_sense"+QString::number(i),
                             QVariant(int(content->zmpProfiles[i].vals.params.ik_sense)));
    }

    // Quadruped zmp params tab 
    QVariant pNumQuad = QVariant(int(content->zmpQuadProfiles.size()));
    pQuad_config.mapSetValue("ZmpQuadProfileNum", pNumQuad);
    
    for(int i=0; i < int(content->zmpQuadProfiles.size()); i++)
    {
        content->zmpQuadProfiles[i].name.replace(" ","_");
        pQuad_config.mapSetValue("ZmpQuadProfileName"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].name));
        pQuad_config.mapSetValue("max_step_count"+QString::number(i),
                             QVariant(int(content->zmpQuadProfiles[i].vals.params.max_step_count)));
        pQuad_config.mapSetValue("step_length"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.step_length));
        pQuad_config.mapSetValue("half_stance_width"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.half_stance_width));
        pQuad_config.mapSetValue("half_peg_width"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.half_peg_width));
        pQuad_config.mapSetValue("step_height"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.step_height));
        pQuad_config.mapSetValue("sidestep_length"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.sidestep_length));
        pQuad_config.mapSetValue("com_height"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.com_height));
        pQuad_config.mapSetValue("torso_pitch"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.torso_pitch));
        pQuad_config.mapSetValue("com_ik_angle_weight"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.com_ik_angle_weight));
        pQuad_config.mapSetValue("fixed_com_offset_x"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.fixed_com_offset_x));
        pQuad_config.mapSetValue("fixed_com_offset_y"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.fixed_com_offset_y));
        pQuad_config.mapSetValue("fixed_com_offset_z"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.fixed_com_offset_z));
        pQuad_config.mapSetValue("zmpoff_y"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.zmpoff_y));
        pQuad_config.mapSetValue("zmpoff_x"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.zmpoff_x));
        pQuad_config.mapSetValue("zmp_dist_gain"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.zmp_dist_gain));
        pQuad_config.mapSetValue("lookahead_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.lookahead_time));
        pQuad_config.mapSetValue("startup_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.walk_startup_time));
        pQuad_config.mapSetValue("shutdown_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.walk_shutdown_time));
        pQuad_config.mapSetValue("min_double_support_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.min_double_support_time));
        pQuad_config.mapSetValue("min_single_support_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.min_single_support_time));
        pQuad_config.mapSetValue("min_pause_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.min_pause_time));
        pQuad_config.mapSetValue("biped_transition_time"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.quad_transition_time));
        pQuad_config.mapSetValue("quad_stance_length"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.quad_stance_length));
        pQuad_config.mapSetValue("quad_stability_margin"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.quad_stability_margin));
        pQuad_config.mapSetValue("zmp_jerk_penalty"+QString::number(i),
                             QVariant(content->zmpQuadProfiles[i].vals.params.zmp_R));
        p_config.mapSetValue("ik_sense"+QString::number(i),
                             QVariant(int(content->zmpQuadProfiles[i].vals.params.ik_sense)));
    }


#endif // HAVE_HUBOMZ

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
        pb_config.mapSetValue("flatten_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.flattening_gain));
        pb_config.mapSetValue("flatten_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.flattening_gain));
        pb_config.mapSetValue("decay_gain_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.decay_gain));
        pb_config.mapSetValue("decay_gain_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.decay_gain));
        pb_config.mapSetValue("thresh_min_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.force_min_threshold));
        pb_config.mapSetValue("thresh_min_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.force_min_threshold));
        pb_config.mapSetValue("thresh_max_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.force_max_threshold));
        pb_config.mapSetValue("thresh_max_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.force_max_threshold));
        pb_config.mapSetValue("straightenP_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.straightening_pitch_gain));
        pb_config.mapSetValue("straightenP_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.straightening_pitch_gain));
        pb_config.mapSetValue("straightenR_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.straightening_roll_gain));
        pb_config.mapSetValue("straightenR_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.straightening_roll_gain));
        pb_config.mapSetValue("spring_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.spring_gain));
        pb_config.mapSetValue("spring_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.spring_gain));
        pb_config.mapSetValue("damp_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.damping_gain));
        pb_config.mapSetValue("damp_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.damping_gain));
        pb_config.mapSetValue("response_bal"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.fz_response));
        pb_config.mapSetValue("response_walk"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.walking_gains.fz_response));
        pb_config.mapSetValue("single_support_hip_nudge_kp"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.single_support_hip_nudge_kp));
        pb_config.mapSetValue("single_support_hip_nudge_kd"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.single_support_hip_nudge_kd));
        pb_config.mapSetValue("double_support_hip_nudge_kp"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.double_support_hip_nudge_kp));
        pb_config.mapSetValue("double_support_hip_nudge_kd"+QString::number(i),
                             QVariant(content->balProfiles[i].vals.balance_gains.double_support_hip_nudge_kd));
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


#ifdef HAVE_HUBOMZ
    memset(&cmd, 0, sizeof(cmd));
    memset(&zmpState, 0, sizeof(zmpState));
    tempWalkMode = MODELESS;
#endif // HAVE_HUBOMZ
    memset(&balParams, 0, sizeof(balParams));
    memset(&balCmd, 0, sizeof(balCmd));

    initializeCommandTab();
    std::cerr << "Command Tab loaded" << std::endl;
    addTab(commandTab, "Command");

#ifdef HAVE_HUBOMZ
    std::cerr << "Initializing Biped Param Tab\n";
    initializeZmpBipedParamTab();
    std::cerr << "ZMP Biped Parameters Tab loaded" << std::endl;

    std::cerr << "Initializing Quadruped Param Tab\n";
    initializeZmpQuadrupedParamTab();
    std::cerr << "ZMP Quadruped Parameters Tab loaded" << std::endl;

    // Biped
    saveAsEdit->setText("Default");
    handleProfileSaveAs();
    saveAsEdit->setText("Default-Backup");
    handleProfileSaveAs();
    saveAsEdit->clear();

    profileSelect->setCurrentIndex(0);

    // Quadruped
    saveAsEditQuad->setText("Default");
    handleQuadrupedProfileSaveAs();
    saveAsEditQuad->setText("Default-Backup");
    handleQuadrupedProfileSaveAs();
    saveAsEditQuad->clear();

    profileSelectQuad->setCurrentIndex(0);

    // Add tabs
    addTab(zmpBipedParamTab, "Biped Params");
    addTab(zmpQuadrupedParamTab, "Quad Params");

#else
    std::cerr << "ZMP Parameters Tabs will NOT be loaded because hubomz is not installed" << std::endl;
#endif // HAVE_HUBOMZ

    initializeBalParamTab();
    std::cerr << "Balance Parameters Tab loaded" << std::endl;
    addTab(balParamTab, "Balance Parameters");

    initializeAchConnections();

    refreshManager = new HuboRefreshManager;
    refreshManager->parentWidget = this;
    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();

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
//    waitTime = t;
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
    joySelect->setDisabled(true);
    joySelect->setToolTip("Use a handheld controller to control Hubo's walking\n"
                          "(The controller must be plugged into this computer)");
    radioSelectGroup->addButton(joySelect);
    controlSelectLayout->addWidget(joySelect);
    
    QVBoxLayout* joyLayout = new QVBoxLayout;
    joyLaunch = new QPushButton;
    joyLaunch->setSizePolicy(pbsize);
    joyLaunch->setText("Launch Joystick");
    joyLaunch->setToolTip("Begin a program which will read for joystick commands");
    joyLaunch->setDisabled(true);
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
    
    
    QHBoxLayout* controlLayout = new QHBoxLayout;

    QGroupBox* zmpCtrlGroup = new QGroupBox;
    zmpCtrlGroup->setTitle("ZMP Commands");
    zmpCtrlGroup->setStyleSheet(groupStyleSheet);
    QHBoxLayout* zmpCtrlLayout = new QHBoxLayout;
    QVBoxLayout* paramLayout = new QVBoxLayout;

    continuousBox = new QCheckBox;
    continuousBox->setText("Walk Continuously");
    continuousBox->setToolTip("Ignore the walk distance, and walk until Stop is selected");
    continuousBox->setChecked(false);
    continuousBox->setDisabled(false);
    paramLayout->addWidget(continuousBox, 0, Qt::AlignLeft);

    QLabel* walkLab = new QLabel;
    walkLab->setText("Walk Distance:");
    walkLab->setToolTip("Distance to walk (m) after a click");
    paramLayout->addWidget(walkLab, 0, Qt::AlignLeft);
    walkDistanceBox = new QDoubleSpinBox;
    walkDistanceBox->setSingleStep(0.5);
    walkDistanceBox->setValue(0.2);
    walkDistanceBox->setToolTip(walkLab->toolTip());
    paramLayout->addWidget(walkDistanceBox, 0, Qt::AlignLeft);

    QLabel* rotateAngleLab = new QLabel;
    rotateAngleLab->setText("Turn-in-place\nAngle:");
    rotateAngleLab->setToolTip("Angle (rad) after a click");
    paramLayout->addWidget(rotateAngleLab, 0, Qt::AlignLeft | Qt::AlignBottom);
    rotateAngleBox = new QDoubleSpinBox;
    rotateAngleBox->setSingleStep(0.1);
    rotateAngleBox->setValue(0.3);
    rotateAngleBox->setToolTip(rotateAngleLab->toolTip());
    paramLayout->addWidget(rotateAngleBox, 0, Qt::AlignLeft | Qt::AlignTop);

    QLabel* maxStepLab = new QLabel;
    maxStepLab->setText("Max Steps:");
    maxStepLab->setToolTip("Cut-off for number of steps to take");
    paramLayout->addWidget(maxStepLab, 0, Qt::AlignLeft | Qt::AlignBottom);
    maxStepBox = new QSpinBox;
    maxStepBox->setSingleStep(1);
    maxStepBox->setToolTip(maxStepLab->toolTip());
    maxStepBox->setValue(4);
    paramLayout->addWidget(maxStepBox, 0, Qt::AlignLeft | Qt::AlignTop);
    
    QLabel* turnLab = new QLabel;
    turnLab->setText("Turn Radius:");
    turnLab->setToolTip("Radius of curvature (m) for turning");
    paramLayout->addWidget(turnLab, 0, Qt::AlignLeft | Qt::AlignBottom);
    radiusBox = new QDoubleSpinBox;
    radiusBox->setValue(5.0);
    radiusBox->setMinimum(0);
    radiusBox->setSingleStep(0.1);
    radiusBox->setMaximum(10000);
    radiusBox->setToolTip(turnLab->toolTip());
    paramLayout->addWidget(radiusBox, 0, Qt::AlignLeft | Qt::AlignTop);

    zmpCtrlLayout->addLayout(paramLayout);
    
    QGridLayout* wasdLayout = new QGridLayout;
    wasdLayout->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    
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

    quadrupedButton = new QPushButton;
    quadrupedButton->setText("Go Quad");
    quadrupedButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(quadrupedButton, 4, 0, 1, 1, Qt::AlignCenter);
    connect(quadrupedButton, SIGNAL(clicked()), this, SLOT(handleGoQuadruped()));
    
    bipedButton = new QPushButton;
    bipedButton->setText(" Go Biped ");
    bipedButton->setSizePolicy(pbsize);
    wasdLayout->addWidget(bipedButton, 4, 2, 1, 1, Qt::AlignCenter);
    connect(bipedButton, SIGNAL(clicked()), this, SLOT(handleGoBiped()));
    
    zmpCtrlLayout->addLayout(wasdLayout);

    zmpCtrlGroup->setLayout(zmpCtrlLayout);
    controlLayout->addWidget(zmpCtrlGroup, 0, Qt::AlignLeft);

    // ZMP State Display
    QGroupBox* zmpStateGroup = new QGroupBox;
    zmpStateGroup->setTitle("ZMP State");
    zmpStateGroup->setStyleSheet(groupStyleSheet);

    QHBoxLayout* zmpStateLayout = new QHBoxLayout;
    zmpStateLayout->setAlignment(Qt::AlignLeft);

    QHBoxLayout* zmpResultLayout = new QHBoxLayout;
    zmpResultLayout->setAlignment(Qt::AlignLeft);

    QLabel* zmpResultLabel = new QLabel;
    zmpResultLabel->setText("Result:");
    zmpResultLabel->setToolTip("Result the zmp-daemon is in");
    zmpResultLayout->addWidget(zmpResultLabel);

    zmpResultEdit = new QLineEdit;
    zmpResultEdit->setReadOnly(true);
    zmpResultEdit->setMaxLength(20);
    zmpResultEdit->setMaximumWidth(250);
    zmpResultEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    zmpResultEdit->setToolTip("Result of zmp-daemon command");
    zmpResultLayout->addWidget(zmpResultEdit, 0, Qt::AlignLeft);

    QHBoxLayout* walkModeLayout = new QHBoxLayout;
    walkModeLayout->setAlignment(Qt::AlignLeft);

    QLabel* walkModeLabel = new QLabel;
    walkModeLabel->setText("Walk Mode:");
    walkModeLabel->setToolTip("Walk mode the zmp-daemon is in");
    walkModeLayout->addWidget(walkModeLabel);

    walkModeEdit = new QLineEdit;
    walkModeEdit->setReadOnly(true);
    walkModeEdit->setMaxLength(20);
    walkModeEdit->setMaximumWidth(250);
    walkModeEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    walkModeEdit->setToolTip("Walk mode the zmp-daemon is in");
    walkModeLayout->addWidget(walkModeEdit, 0, Qt::AlignLeft);

    zmpStateLayout->addLayout(zmpResultLayout);
    zmpStateLayout->addLayout(walkModeLayout);
    zmpStateGroup->setLayout(zmpStateLayout);


    // Balance Control Group
    QGroupBox* balCtrlGroup = new QGroupBox;
    balCtrlGroup->setTitle("Static Commands");
    balCtrlGroup->setStyleSheet(groupStyleSheet);
    QVBoxLayout* balLayout = new QVBoxLayout;

    QHBoxLayout* staticCmdsLayout = new QHBoxLayout;

    QVBoxLayout* heightLayout = new QVBoxLayout;
    heightScale = 1000;
    QLabel* heightLabel = new QLabel;
    heightLabel->setText("Height");
    heightLayout->addWidget(heightLabel);
    heightSlide = new QSlider(Qt::Vertical);
    // TODO: Put the following values in a header
    heightSlide->setMaximum((int)((0.33008 + 0.32995 + 0.28947 + 0.0795 - 0.02)*heightScale));
    // ^ Taken from hubo-motion-rt/src/balance-daemon.cpp
    heightSlide->setMinimum((int)((0.25+0.28947+0.0795)*heightScale));
    // ^ Taken from hubo-motion-rt/src/balance-daemon.cpp
    heightSlide->setValue(heightSlide->maximum());
    heightLayout->addWidget(heightSlide);
    connect( heightSlide, SIGNAL(valueChanged(int)), this, SLOT(handleStaticButton()) );

    QVBoxLayout* comXOffsetLayout = new QVBoxLayout;
    QLabel* comXOffsetLab = new QLabel;
    comXOffsetLab->setText("COM-X Offset (m)");
    comXOffsetLayout->addWidget(comXOffsetLab, 0, Qt::AlignCenter);
    comXOffsetBox = new QDoubleSpinBox;
    comXOffsetBox->setDecimals(4);
    comXOffsetBox->setSingleStep(0.001);
    comXOffsetBox->setMinimum(-0.05);
    comXOffsetBox->setMaximum(0.05);
    comXOffsetBox->setValue(0.0);
    comXOffsetLayout->addWidget(comXOffsetBox, 0, Qt::AlignTop);

    staticCmdsLayout->addLayout(heightLayout, 0);
    staticCmdsLayout->addLayout(comXOffsetLayout, 1);
    staticCmdsLayout->setAlignment(Qt::AlignTop);

    balLayout->addLayout(staticCmdsLayout, 0);

    staticButton = new QPushButton;
    staticButton->setText("Balance");
    staticButton->setToolTip("Enter a static balance mode which allows Hubo to squat up and down");
    balLayout->addWidget(staticButton);
    connect( staticButton, SIGNAL(clicked()), this, SLOT(handleStaticButton()) );

    balOffButton = new QPushButton;
    balOffButton->setText(" Off  ");
    balOffButton->setToolTip("Turn off all forms of balance control");
    balLayout->addWidget(balOffButton);
    connect( balOffButton, SIGNAL(clicked()), this, SLOT(handleBalOffButton()) );

    balCtrlGroup->setLayout(balLayout);
    controlLayout->addWidget(balCtrlGroup);
    
    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(networkBox);
    masterCTLayout->addWidget(controlSelectBox);
    masterCTLayout->addLayout(controlLayout);
    masterCTLayout->addWidget(zmpStateGroup);

    commandTab = new QWidget;
    commandTab->setLayout(masterCTLayout);
}

#ifdef HAVE_HUBOMZ

// BIPED PARAM TAB INITIALIZATION
void HuboWalkWidget::initializeZmpBipedParamTab()
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
    yOffsetBox->setValue(0.010);
    yOffsetBox->setSingleStep(0.01);
    yOffsetBox->setMinimum(-10);
    yOffsetBox->setMaximum(10);
    yoffsetLay->addWidget(yOffsetBox);
    
    zmpSettingsLayout->addLayout(yoffsetLay);

    QHBoxLayout* zmpDistGainLay = new QHBoxLayout;
    QLabel* zmpDistGainLab = new QLabel;
    zmpDistGainLab->setText("Distribution Gain:");
    zmpDistGainLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    zmpDistGainLab->setToolTip("Additional seconds of double support per meter of ZMP motion");
    zmpDistGainLay->addWidget(zmpDistGainLab);

    zmpDistGainBox = new QDoubleSpinBox;
    zmpDistGainBox->setSizePolicy(pbsize);
    zmpDistGainBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    zmpDistGainBox->setToolTip(zmpDistGainLab->toolTip());
    zmpDistGainBox->setDecimals(4);
    zmpDistGainBox->setValue(0.00);
    zmpDistGainBox->setSingleStep(0.5);
    zmpDistGainBox->setMinimum(0);
    zmpDistGainBox->setMaximum(20);
    zmpDistGainLay->addWidget(zmpDistGainBox);

    zmpSettingsLayout->addLayout(zmpDistGainLay);
    
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

    // Startup time
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
    startupTimeBox->setValue(1.0);
    startupTimeBox->setSingleStep(0.01);
    startupTimeBox->setMinimum(0);
    startupTimeBox->setMaximum(1000);
    startupTimeLay->addWidget(startupTimeBox);
    
    timeSettingsLayout->addLayout(startupTimeLay);

    // Shutdown time
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
    shutdownTimeBox->setValue(1.0);
    shutdownTimeBox->setSingleStep(0.01);
    shutdownTimeBox->setMinimum(0);
    shutdownTimeBox->setMaximum(1000);
    shutdownTimeLay->addWidget(shutdownTimeBox);
    
    timeSettingsLayout->addLayout(shutdownTimeLay);

    // Min double support time
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
    doubleSupportBox->setValue(0.1);
    doubleSupportBox->setSingleStep(0.01);
    doubleSupportBox->setMinimum(0);
    doubleSupportBox->setMaximum(1000);
    doubleSupTimeLay->addWidget(doubleSupportBox);
    
    timeSettingsLayout->addLayout(doubleSupTimeLay);

    // Min single support time
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
    singleSupportBox->setValue(0.8);
    singleSupportBox->setSingleStep(0.01);
    singleSupportBox->setMinimum(0);
    singleSupportBox->setMaximum(1000);
    singleSupTimeLay->addWidget(singleSupportBox);
    
    timeSettingsLayout->addLayout(singleSupTimeLay);

    // Min pause time
    QHBoxLayout* pauseTimeLay = new QHBoxLayout;
    QLabel* pauseTimeLab = new QLabel;
    pauseTimeLab->setText("Pause Time:");
    pauseTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    pauseTimeLab->setToolTip("How much time (s) should be spent stationary in double support?");
    pauseTimeLay->addWidget(pauseTimeLab);
    
    pauseTimeBox = new QDoubleSpinBox;
    pauseTimeBox->setSizePolicy(pbsize);
    pauseTimeBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    pauseTimeBox->setToolTip(pauseTimeLab->toolTip());
    pauseTimeBox->setDecimals(2);
    pauseTimeBox->setValue(0.0);
    pauseTimeBox->setSingleStep(0.1);
    pauseTimeBox->setMinimum(0);
    pauseTimeBox->setMaximum(50);
    pauseTimeLay->addWidget(pauseTimeBox);
    
    timeSettingsLayout->addLayout(pauseTimeLay);

    // Transition to quadruped time
    QHBoxLayout* transitionToQuadTimeLay = new QHBoxLayout;
    QLabel* transitionToQuadTimeLab = new QLabel;
    transitionToQuadTimeLab->setText("Quadruped Transition Time:");
    transitionToQuadTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    transitionToQuadTimeLab->setToolTip("How much time (s) should be spent transitioning to Quadruped stance?");
    transitionToQuadTimeLay->addWidget(transitionToQuadTimeLab);

    transitionToQuadTimeBox = new QDoubleSpinBox;
    transitionToQuadTimeBox->setSizePolicy(pbsize);
    transitionToQuadTimeBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    transitionToQuadTimeBox->setToolTip(transitionToQuadTimeLab->toolTip());
    transitionToQuadTimeBox->setDecimals(2);
    transitionToQuadTimeBox->setValue(8.00);
    transitionToQuadTimeBox->setSingleStep(0.5);
    transitionToQuadTimeBox->setMinimum(0);
    transitionToQuadTimeBox->setMaximum(50);
    transitionToQuadTimeLay->addWidget(transitionToQuadTimeBox);

    timeSettingsLayout->addLayout(transitionToQuadTimeLay);

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
    liftoffHeightBox->setValue(0.04);
    liftoffHeightBox->setSingleStep(0.01);
    liftoffHeightBox->setMinimum(0);
    liftoffHeightBox->setMaximum(2);
    footLiftLay->addWidget(liftoffHeightBox);
    
    swingSettingsLayout->addLayout(footLiftLay);
    
    QHBoxLayout* stepDistanceLay = new QHBoxLayout;
    QLabel* stepDistanceLab = new QLabel;
    stepDistanceLab->setText("Step Length:");
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
    sideStepDistanceBox->setValue(0.03);
    sideStepDistanceBox->setSingleStep(0.01);
    sideStepDistanceBox->setMinimum(0);
    sideStepDistanceBox->setMaximum(5);
    sideStepDistanceLay->addWidget(sideStepDistanceBox);

    swingSettingsLayout->addLayout(sideStepDistanceLay);

    
    QHBoxLayout* lateralDistanceLay = new QHBoxLayout;
    QLabel* lateralDistanceLab = new QLabel;
    lateralDistanceLab->setText("Half Stance Width:");
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
    
    
    // CENTER OF MASS SETTINGS LAYOUT
    QVBoxLayout* comSettingsLayout = new QVBoxLayout;
    comSettingsLayout->setAlignment(Qt::AlignCenter);

    // CENTER OF MASS HEIGHT
    QHBoxLayout* comHeightLay = new QHBoxLayout;
    QLabel* comHeightLab = new QLabel;
    comHeightLab->setText("Biped COM Height:");
    comHeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comHeightLab->setToolTip("How heigh (m) should the center of mass be from the ground in biped stance?");
    comHeightLay->addWidget(comHeightLab);

    comHeightBox = new QDoubleSpinBox;
    comHeightBox->setSizePolicy(pbsize);
    comHeightBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comHeightBox->setToolTip(comHeightLab->toolTip());
    comHeightBox->setDecimals(3);
    comHeightBox->setValue(0.84);
    comHeightBox->setSingleStep(0.01);
    comHeightBox->setMinimum(0);
    comHeightBox->setMaximum(5);
    comHeightLay->addWidget(comHeightBox);
    
    comSettingsLayout->addLayout(comHeightLay);

    // TORSO PITCH
    QHBoxLayout* torsoPitchLay = new QHBoxLayout;
    QLabel* torsoPitchLab = new QLabel;
    torsoPitchLab->setText("Torso Pitch:");
    torsoPitchLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    torsoPitchLab->setToolTip("How much the torso should be pitched in radians? (Positive is forward)");
    torsoPitchLay->addWidget(torsoPitchLab);

    torsoPitchBox = new QDoubleSpinBox;
    torsoPitchBox->setSizePolicy(pbsize);
    torsoPitchBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    torsoPitchBox->setToolTip(torsoPitchLab->toolTip());
    torsoPitchBox->setDecimals(3);
    torsoPitchBox->setValue(0);
    torsoPitchBox->setSingleStep(0.01);
    torsoPitchBox->setMinimum(-3);
    torsoPitchBox->setMaximum(3);
    torsoPitchLay->addWidget(torsoPitchBox);
    
    comSettingsLayout->addLayout(torsoPitchLay);

    
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

    QHBoxLayout* fixed_com_offset_xLay = new QHBoxLayout;
    QLabel* fixed_com_offset_xLab = new QLabel;
    fixed_com_offset_xLab->setText("Fixed COM Offset:");
    fixed_com_offset_xLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    fixed_com_offset_xLab->setToolTip("Offset of fixed COM from trunk (m).");
    fixed_com_offset_xLay->addWidget(fixed_com_offset_xLab);

    fixed_com_offset_xBox = new QDoubleSpinBox;
    fixed_com_offset_xBox->setSizePolicy(pbsize);
    fixed_com_offset_xBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_xBox->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_xBox->setDecimals(3);
    fixed_com_offset_xBox->setValue(0);
    fixed_com_offset_xBox->setSingleStep(0.01);
    fixed_com_offset_xBox->setMinimum(0);
    fixed_com_offset_xBox->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_xBox);

    fixed_com_offset_yBox = new QDoubleSpinBox;
    fixed_com_offset_yBox->setSizePolicy(pbsize);
    fixed_com_offset_yBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_yBox->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_yBox->setDecimals(3);
    fixed_com_offset_yBox->setValue(0);
    fixed_com_offset_yBox->setSingleStep(0.01);
    fixed_com_offset_yBox->setMinimum(0);
    fixed_com_offset_yBox->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_yBox);

    fixed_com_offset_zBox = new QDoubleSpinBox;
    fixed_com_offset_zBox->setSizePolicy(pbsize);
    fixed_com_offset_zBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_zBox->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_zBox->setDecimals(3);
    fixed_com_offset_zBox->setValue(0);
    fixed_com_offset_zBox->setSingleStep(0.01);
    fixed_com_offset_zBox->setMinimum(0);
    fixed_com_offset_zBox->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_zBox);

    comSettingsLayout->addLayout(fixed_com_offset_xLay);
    
    // Constant Body Z
    constantBodyZBox = new QCheckBox;
    constantBodyZBox->setText("Constant Body Z");
    constantBodyZBox->setToolTip("Ignore the walk distance, and walk until Stop is selected");
    constantBodyZBox->setChecked(true);
    constantBodyZBox->setDisabled(false);
    comSettingsLayout->addWidget(constantBodyZBox, 0, Qt::AlignRight);

    comSettingsLayout->addWidget(constantBodyZBox);

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
    
    zmpBipedParamTab = new QWidget;
    zmpBipedParamTab->setLayout(masterZMPLayout);
    
//    saveAsEdit->setText("Default");
//    handleProfileSaveAs();
//    saveAsEdit->setText("Default-Backup");
//    handleProfileSaveAs();
//    saveAsEdit->clear();
    
//    profileSelect->setCurrentIndex(0);
}
// end BIPED PARAM TAB INITIALIZATION

// QUADRUPED PARAM TAB INITIALIZATION
void HuboWalkWidget::initializeZmpQuadrupedParamTab()
{
    QSizePolicy pbsize(QSizePolicy::Maximum, QSizePolicy::Maximum);
    
    QHBoxLayout* profileLayoutTop = new QHBoxLayout;
    QLabel* profileLab = new QLabel;
    profileLab->setText("Profile:");
    profileLayoutTop->addWidget(profileLab, 0, Qt::AlignVCenter | Qt::AlignRight);
    
    profileSelectQuad = new QComboBox;
    profileLayoutTop->addWidget(profileSelectQuad);
    connect(profileSelectQuad, SIGNAL(currentIndexChanged(int)), this, SLOT(handleQuadrupedProfileSelect(int)));
    
    saveProfileQuad = new QPushButton;
    saveProfileQuad->setSizePolicy(pbsize);
    saveProfileQuad->setText("Save");
    saveProfileQuad->setToolTip("Save the values below into the currently selected profile");
    profileLayoutTop->addWidget(saveProfileQuad);
    connect(saveProfileQuad, SIGNAL(clicked()), this, SLOT(handleQuadrupedProfileSave()));
    
    deleteProfileQuad = new QPushButton;
    deleteProfileQuad->setSizePolicy(pbsize);
    deleteProfileQuad->setText("Delete");
    deleteProfileQuad->setToolTip("Remove the current profile from the list\n"
                              "WARNING: This is permanent!");
    profileLayoutTop->addWidget(deleteProfileQuad);
    connect(deleteProfileQuad, SIGNAL(clicked()), this, SLOT(handleQuadrupedProfileDelete()));
    
    QHBoxLayout* profileLayoutBottom = new QHBoxLayout;
    saveAsProfileQuad = new QPushButton;
    saveAsProfileQuad->setSizePolicy(pbsize);
    saveAsProfileQuad->setText("Save As...");
    saveAsProfileQuad->setToolTip("Save the values below as a new profile with the following name:");
    profileLayoutBottom->addWidget(saveAsProfileQuad);
    connect(saveAsProfileQuad, SIGNAL(clicked()), this, SLOT(handleQuadrupedProfileSaveAs()));
    
    saveAsEditQuad = new QLineEdit;
    saveAsEditQuad->setToolTip("Enter a name for a new profile");
    profileLayoutBottom->addWidget(saveAsEditQuad);
    
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
    
    xOffsetBoxQuad = new QDoubleSpinBox;
    xOffsetBoxQuad->setSizePolicy(pbsize);
    xOffsetBox->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    xOffsetBoxQuad->setToolTip(xoffsetLab->toolTip());
    xOffsetBoxQuad->setDecimals(4);
    xOffsetBoxQuad->setValue(0.038);
    xOffsetBoxQuad->setSingleStep(0.01);
    xOffsetBoxQuad->setMinimum(-10);
    xOffsetBoxQuad->setMaximum(10);
    xoffsetLay->addWidget(xOffsetBoxQuad);
    
    zmpSettingsLayout->addLayout(xoffsetLay);
    
    QHBoxLayout* yoffsetLay = new QHBoxLayout;
    QLabel* yoffsetLab = new QLabel;
    yoffsetLab->setText("Y-Offset:");
    yoffsetLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    yoffsetLab->setToolTip("How far in y should the ZMP be offset from the ankle joint?");
    yoffsetLay->addWidget(yoffsetLab);
    
    yOffsetBoxQuad = new QDoubleSpinBox;
    yOffsetBoxQuad->setSizePolicy(pbsize);
    yOffsetBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    yOffsetBoxQuad->setToolTip(yoffsetLab->toolTip());
    yOffsetBoxQuad->setDecimals(4);
    yOffsetBoxQuad->setValue(0);
    yOffsetBoxQuad->setSingleStep(0.01);
    yOffsetBoxQuad->setMinimum(-10);
    yOffsetBoxQuad->setMaximum(10);
    yoffsetLay->addWidget(yOffsetBoxQuad);
    
    zmpSettingsLayout->addLayout(yoffsetLay);

    QHBoxLayout* zmpDistGainLay = new QHBoxLayout;
    QLabel* zmpDistGainLab = new QLabel;
    zmpDistGainLab->setText("Distribution Gain:");
    zmpDistGainLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    zmpDistGainLab->setToolTip("How far in y should the ZMP be offset from the ankle joint?");
    zmpDistGainLay->addWidget(zmpDistGainLab);

    zmpDistGainBoxQuad = new QDoubleSpinBox;
    zmpDistGainBoxQuad->setSizePolicy(pbsize);
    zmpDistGainBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    zmpDistGainBoxQuad->setToolTip(zmpDistGainLab->toolTip());
    zmpDistGainBoxQuad->setDecimals(4);
    zmpDistGainBoxQuad->setValue(0);
    zmpDistGainBoxQuad->setSingleStep(0.5);
    zmpDistGainBoxQuad->setMinimum(0);
    zmpDistGainBoxQuad->setMaximum(20);
    zmpDistGainLay->addWidget(zmpDistGainBoxQuad);

    zmpSettingsLayout->addLayout(zmpDistGainLay);

    QHBoxLayout* jerkLayout = new QHBoxLayout;
    QLabel* jerkPenaltyLab = new QLabel;
    jerkPenaltyLab->setText("Jerk Penalty:");
    jerkPenaltyLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    jerkPenaltyLab->setToolTip("How much should jerk be penalized by the Preview Controller?");
    jerkLayout->addWidget(jerkPenaltyLab);
    
    jerkPenalBoxQuad = new QDoubleSpinBox;
    jerkPenalBoxQuad->setSizePolicy(pbsize);
    jerkPenalBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    jerkPenalBoxQuad->setToolTip(jerkPenaltyLab->toolTip());
    jerkPenalBoxQuad->setValue(1);
    jerkPenalBoxQuad->setSingleStep(0.1);
    jerkPenalBoxQuad->setMinimum(0);
    jerkPenalBoxQuad->setMaximum(1000);
    jerkLayout->addWidget(jerkPenalBoxQuad);

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
    
    lookAheadBoxQuad = new QDoubleSpinBox;
    lookAheadBoxQuad->setSizePolicy(pbsize);
    lookAheadBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    lookAheadBoxQuad->setToolTip(lookAheadLab->toolTip());
    lookAheadBoxQuad->setValue(2.5);
    lookAheadBoxQuad->setSingleStep(0.1);
    lookAheadBoxQuad->setMinimum(0);
    lookAheadBoxQuad->setMaximum(1000);
    lookAheadLayout->addWidget(lookAheadBoxQuad);
    
    zmpSettingsLayout->addLayout(lookAheadLayout);
    
    
    QGroupBox* zmpSettingsBoxQuad = new QGroupBox;
    zmpSettingsBoxQuad->setTitle("ZMP Settings");
    zmpSettingsBoxQuad->setStyleSheet(groupStyleSheet);
    zmpSettingsBoxQuad->setLayout(zmpSettingsLayout);
    leftColumn->addWidget(zmpSettingsBoxQuad);
    
    
    QVBoxLayout* timeSettingsLayout = new QVBoxLayout;
    timeSettingsLayout->setAlignment(Qt::AlignCenter);

    // Startup time
    QHBoxLayout* startupTimeLay = new QHBoxLayout;
    QLabel* startupTimeLab = new QLabel;
    startupTimeLab->setText("Startup Time:");
    startupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    startupTimeLab->setToolTip("How much time (s) should be spent starting up?");
    startupTimeLay->addWidget(startupTimeLab);
    
    startupTimeBoxQuad = new QDoubleSpinBox;
    startupTimeBoxQuad->setSizePolicy(pbsize);
    startupTimeBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    startupTimeBoxQuad->setToolTip(startupTimeLab->toolTip());
    startupTimeBoxQuad->setDecimals(3);
    startupTimeBoxQuad->setValue(1.0);
    startupTimeBoxQuad->setSingleStep(0.01);
    startupTimeBoxQuad->setMinimum(0);
    startupTimeBoxQuad->setMaximum(1000);
    startupTimeLay->addWidget(startupTimeBoxQuad);
    
    timeSettingsLayout->addLayout(startupTimeLay);

    // Shutdown time
    QHBoxLayout* shutdownTimeLay = new QHBoxLayout;
    QLabel* shutdownTimeLab = new QLabel;
    shutdownTimeLab->setText("Shutdown Time:");
    shutdownTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    shutdownTimeLab->setToolTip("How much time (s) should be spent stopping?");
    shutdownTimeLay->addWidget(shutdownTimeLab);
    
    shutdownTimeBoxQuad = new QDoubleSpinBox;
    shutdownTimeBoxQuad->setSizePolicy(pbsize);
    shutdownTimeBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    shutdownTimeBoxQuad->setToolTip(shutdownTimeLab->toolTip());
    shutdownTimeBoxQuad->setDecimals(3);
    shutdownTimeBoxQuad->setValue(1.0);
    shutdownTimeBoxQuad->setSingleStep(0.01);
    shutdownTimeBoxQuad->setMinimum(0);
    shutdownTimeBoxQuad->setMaximum(1000);
    shutdownTimeLay->addWidget(shutdownTimeBoxQuad);
    
    timeSettingsLayout->addLayout(shutdownTimeLay);

    // Min double support time
    QHBoxLayout* doubleSupTimeLay = new QHBoxLayout;
    QLabel* doubleSupTimeLab = new QLabel;
    doubleSupTimeLab->setText("Double Support Time:");
    doubleSupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    doubleSupTimeLab->setToolTip("How much time (s) should be spent in double support?\n"
                                 "i.e. Having two feet on the ground");
    doubleSupTimeLay->addWidget(doubleSupTimeLab);
    
    doubleSupportBoxQuad = new QDoubleSpinBox;
    doubleSupportBoxQuad->setSizePolicy(pbsize);
    doubleSupportBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    doubleSupportBoxQuad->setToolTip(doubleSupTimeLab->toolTip());
    doubleSupportBoxQuad->setDecimals(2);
    doubleSupportBoxQuad->setValue(1.0);
    doubleSupportBoxQuad->setSingleStep(0.1);
    doubleSupportBoxQuad->setMinimum(0);
    doubleSupportBoxQuad->setMaximum(50);
    doubleSupTimeLay->addWidget(doubleSupportBoxQuad);
    
    timeSettingsLayout->addLayout(doubleSupTimeLay);

    // Min single support time
    QHBoxLayout* singleSupTimeLay = new QHBoxLayout;
    QLabel* singleSupTimeLab = new QLabel;
    singleSupTimeLab->setText("Single Support Time:");
    singleSupTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    singleSupTimeLab->setToolTip("How much time (s) should be spent in single support?\n"
                                 "i.e. Having one foot on the ground");
    singleSupTimeLay->addWidget(singleSupTimeLab);
    
    singleSupportBoxQuad = new QDoubleSpinBox;
    singleSupportBoxQuad->setSizePolicy(pbsize);
    singleSupportBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    singleSupportBoxQuad->setToolTip(singleSupTimeLab->toolTip());
    singleSupportBoxQuad->setDecimals(2);
    singleSupportBoxQuad->setValue(1.00);
    singleSupportBoxQuad->setSingleStep(0.1);
    singleSupportBoxQuad->setMinimum(0);
    singleSupportBoxQuad->setMaximum(50);
    singleSupTimeLay->addWidget(singleSupportBoxQuad);
    
    timeSettingsLayout->addLayout(singleSupTimeLay);

    // Min pause time
    QHBoxLayout* pauseTimeLay = new QHBoxLayout;
    QLabel* pauseTimeLab = new QLabel;
    pauseTimeLab->setText("Pause Time:");
    pauseTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    pauseTimeLab->setToolTip("How much time (s) should be spent stationary in double support?");
    pauseTimeLay->addWidget(pauseTimeLab);
    
    pauseTimeBoxQuad = new QDoubleSpinBox;
    pauseTimeBoxQuad->setSizePolicy(pbsize);
    pauseTimeBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    pauseTimeBoxQuad->setToolTip(pauseTimeLab->toolTip());
    pauseTimeBoxQuad->setDecimals(2);
    pauseTimeBoxQuad->setValue(0.25);
    pauseTimeBoxQuad->setSingleStep(0.01);
    pauseTimeBoxQuad->setMinimum(0);
    pauseTimeBoxQuad->setMaximum(5);
    pauseTimeLay->addWidget(pauseTimeBoxQuad);
    
    timeSettingsLayout->addLayout(pauseTimeLay);

    // Transition to biped time
    QHBoxLayout* transitionToBipedTimeLay = new QHBoxLayout;
    QLabel* transitionToBipedTimeLab = new QLabel;
    transitionToBipedTimeLab->setText("Biped Transition Time:");
    transitionToBipedTimeLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    transitionToBipedTimeLab->setToolTip("How much time (s) should be spent transitioning to Biped stance?");
    transitionToBipedTimeLay->addWidget(transitionToBipedTimeLab);

    transitionToBipedTimeBoxQuad = new QDoubleSpinBox;
    transitionToBipedTimeBoxQuad->setSizePolicy(pbsize);
    transitionToBipedTimeBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    transitionToBipedTimeBoxQuad->setToolTip(transitionToBipedTimeLab->toolTip());
    transitionToBipedTimeBoxQuad->setDecimals(2);
    transitionToBipedTimeBoxQuad->setValue(8.00);
    transitionToBipedTimeBoxQuad->setSingleStep(0.1);
    transitionToBipedTimeBoxQuad->setMinimum(0);
    transitionToBipedTimeBoxQuad->setMaximum(50);
    transitionToBipedTimeLay->addWidget(transitionToBipedTimeBoxQuad);

    timeSettingsLayout->addLayout(transitionToBipedTimeLay);

    QGroupBox* timeSettingsBoxQuad = new QGroupBox;
    timeSettingsBoxQuad->setTitle("Time Settings");
    timeSettingsBoxQuad->setStyleSheet(groupStyleSheet);
    timeSettingsBoxQuad->setLayout(timeSettingsLayout);
    leftColumn->addWidget(timeSettingsBoxQuad);
    
    
    
    QVBoxLayout* rightColumn = new QVBoxLayout;
    
    QHBoxLayout* ikSenseLayout = new QHBoxLayout;
    QLabel* ikSenseLab = new QLabel;
    ikSenseLab->setText("IK Sensitivity:");
    ikSenseLayout->addWidget(ikSenseLab, 0, Qt::AlignVCenter | Qt::AlignRight);
    ikSenseSelectQuad = new QComboBox;
    ikSenseLayout->addWidget(ikSenseSelectQuad);//, 0, Qt::AlignVCenter | Qt::AlignLeft);
    
    ikSenseSelectQuad->addItem("Strict");
    ikSenseSelectQuad->addItem("Permissive");
    ikSenseSelectQuad->addItem("Sloppy (DANGEROUS)");
    ikSenseSelectQuad->setCurrentIndex(0);
    
    rightColumn->addLayout(ikSenseLayout);
    
    
    QVBoxLayout* swingSettingsLayout = new QVBoxLayout;
    swingSettingsLayout->setAlignment(Qt::AlignCenter);
    
    QHBoxLayout* footLiftLay = new QHBoxLayout;
    QLabel* footliftLab = new QLabel;
    footliftLab->setText("Foot Liftoff Height:");
    footliftLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    footliftLab->setToolTip("How heigh (m) should the swing foot lift off the ground?");
    footLiftLay->addWidget(footliftLab);

    liftoffHeightBoxQuad = new QDoubleSpinBox;
    liftoffHeightBoxQuad->setSizePolicy(pbsize);
    liftoffHeightBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    liftoffHeightBoxQuad->setToolTip(footliftLab->toolTip());
    liftoffHeightBoxQuad->setDecimals(4);
    liftoffHeightBoxQuad->setValue(0.04);
    liftoffHeightBoxQuad->setSingleStep(0.01);
    liftoffHeightBoxQuad->setMinimum(0);
    liftoffHeightBoxQuad->setMaximum(2);
    footLiftLay->addWidget(liftoffHeightBoxQuad);
    
    swingSettingsLayout->addLayout(footLiftLay);
    
    QHBoxLayout* stepDistanceLay = new QHBoxLayout;
    QLabel* stepDistanceLab = new QLabel;
    stepDistanceLab->setText("Step Length:");
    stepDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    stepDistanceLab->setToolTip("How far forward (m) should the swing foot step?");
    stepDistanceLay->addWidget(stepDistanceLab);

    stepDistanceBoxQuad = new QDoubleSpinBox;
    stepDistanceBoxQuad->setSizePolicy(pbsize);
    stepDistanceBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    stepDistanceBoxQuad->setToolTip(stepDistanceLab->toolTip());
    stepDistanceBoxQuad->setDecimals(4);
    stepDistanceBoxQuad->setValue(0.2);
    stepDistanceBoxQuad->setSingleStep(0.01);
    stepDistanceBoxQuad->setMinimum(0);
    stepDistanceBoxQuad->setMaximum(5);
    stepDistanceLay->addWidget(stepDistanceBoxQuad);
    
    swingSettingsLayout->addLayout(stepDistanceLay);

    QHBoxLayout* sideStepDistanceLay = new QHBoxLayout;
    QLabel* sideStepDistanceLab = new QLabel;
    sideStepDistanceLab->setText("Side Step Length:");
    sideStepDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    sideStepDistanceLab->setToolTip("How far sideways (m) should the swing foot step in quadruped mode?");
    sideStepDistanceLay->addWidget(sideStepDistanceLab);

    sideStepDistanceBoxQuad = new QDoubleSpinBox;
    sideStepDistanceBoxQuad->setSizePolicy(pbsize);
    sideStepDistanceBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    sideStepDistanceBoxQuad->setToolTip(sideStepDistanceLab->toolTip());
    sideStepDistanceBoxQuad->setDecimals(3);
    sideStepDistanceBoxQuad->setValue(0.05);
    sideStepDistanceBoxQuad->setSingleStep(0.01);
    sideStepDistanceBoxQuad->setMinimum(0);
    sideStepDistanceBoxQuad->setMaximum(5);
    sideStepDistanceLay->addWidget(sideStepDistanceBoxQuad);

    swingSettingsLayout->addLayout(sideStepDistanceLay);

    
    QHBoxLayout* lateralDistanceLay = new QHBoxLayout;
    QLabel* lateralDistanceLab = new QLabel;
    lateralDistanceLab->setText("Half Stance Width:");
    lateralDistanceLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    lateralDistanceLab->setToolTip("How far from the center (m) should the swing foot be placed?");
    lateralDistanceLay->addWidget(lateralDistanceLab);

    lateralDistanceBoxQuad = new QDoubleSpinBox;
    lateralDistanceBoxQuad->setSizePolicy(pbsize);
    lateralDistanceBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    lateralDistanceBoxQuad->setToolTip(lateralDistanceLab->toolTip());
    lateralDistanceBoxQuad->setDecimals(4);
    lateralDistanceBoxQuad->setValue(0.0885);
    lateralDistanceBoxQuad->setSingleStep(0.01);
    lateralDistanceBoxQuad->setMinimum(0);
    lateralDistanceBoxQuad->setMaximum(5);
    lateralDistanceLay->addWidget(lateralDistanceBoxQuad);
    
    swingSettingsLayout->addLayout(lateralDistanceLay);

    // Quad stance length
    QHBoxLayout* quadStanceLengthLay = new QHBoxLayout;
    QLabel* quadStanceLengthLab = new QLabel;
    quadStanceLengthLab->setText("Quad Stance Length:");
    quadStanceLengthLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    quadStanceLengthLab->setToolTip("How far from the feet (m) should the pegs be placed?");
    quadStanceLengthLay->addWidget(quadStanceLengthLab);

    quadStanceLengthBoxQuad = new QDoubleSpinBox;
    quadStanceLengthBoxQuad->setSizePolicy(pbsize);
    quadStanceLengthBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    quadStanceLengthBoxQuad->setToolTip(quadStanceLengthLab->toolTip());
    quadStanceLengthBoxQuad->setDecimals(4);
    quadStanceLengthBoxQuad->setValue(0.25);
    quadStanceLengthBoxQuad->setSingleStep(0.01);
    quadStanceLengthBoxQuad->setMinimum(0);
    quadStanceLengthBoxQuad->setMaximum(5);
    quadStanceLengthLay->addWidget(quadStanceLengthBoxQuad);

    swingSettingsLayout->addLayout(quadStanceLengthLay);
    
    // Quad stability margin
    QHBoxLayout* quadStabilityMarginLay = new QHBoxLayout;
    QLabel* quadStabilityMarginLab = new QLabel;
    quadStabilityMarginLab->setText("Quad Stability Margin:");
    quadStabilityMarginLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    quadStabilityMarginLab->setToolTip("What is the margin of stability (m)?");
    quadStabilityMarginLay->addWidget(quadStabilityMarginLab);

    quadStabilityMarginBoxQuad = new QDoubleSpinBox;
    quadStabilityMarginBoxQuad->setSizePolicy(pbsize);
    quadStabilityMarginBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    quadStabilityMarginBoxQuad->setToolTip(quadStabilityMarginLab->toolTip());
    quadStabilityMarginBoxQuad->setDecimals(4);
    quadStabilityMarginBoxQuad->setValue(0.035);
    quadStabilityMarginBoxQuad->setSingleStep(0.01);
    quadStabilityMarginBoxQuad->setMinimum(0);
    quadStabilityMarginBoxQuad->setMaximum(5);
    quadStabilityMarginLay->addWidget(quadStabilityMarginBoxQuad);

    swingSettingsLayout->addLayout(quadStabilityMarginLay);

    // Half Peg Width
    QHBoxLayout* halfPegWidthLay = new QHBoxLayout;
    QLabel* halfPegWidthLab = new QLabel;
    halfPegWidthLab->setText("Half Peg Width:");
    halfPegWidthLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    halfPegWidthLab->setToolTip("Half the distance between the pegs in the y-direction (m)");
    halfPegWidthLay->addWidget(halfPegWidthLab);

    halfPegWidthBoxQuad = new QDoubleSpinBox;
    halfPegWidthBoxQuad->setSizePolicy(pbsize);
    halfPegWidthBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    halfPegWidthBoxQuad->setToolTip(halfPegWidthLab->toolTip());
    halfPegWidthBoxQuad->setDecimals(4);
    halfPegWidthBoxQuad->setValue(0.3);
    halfPegWidthBoxQuad->setSingleStep(0.01);
    halfPegWidthBoxQuad->setMinimum(0);
    halfPegWidthBoxQuad->setMaximum(5);
    halfPegWidthLay->addWidget(halfPegWidthBoxQuad);

    swingSettingsLayout->addLayout(halfPegWidthLay);

    QGroupBox* swingSettingsBoxQuad = new QGroupBox;
    swingSettingsBoxQuad->setTitle("Swing Foot Settings");
    swingSettingsBoxQuad->setStyleSheet(groupStyleSheet);
    swingSettingsBoxQuad->setLayout(swingSettingsLayout);
    rightColumn->addWidget(swingSettingsBoxQuad);
    
    
    // CENTER OF MASS SETTINGS LAYOUT
    QVBoxLayout* comSettingsLayout = new QVBoxLayout;
    comSettingsLayout->setAlignment(Qt::AlignCenter);

    // CENTER OF MASS HEIGHT
    QHBoxLayout* comHeightLay = new QHBoxLayout;
    QLabel* comHeightLab = new QLabel;
    comHeightLab->setText("Quadruped COM Height:");
    comHeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comHeightLab->setToolTip("How heigh (m) should the center of mass be from the ground in quadruped stance?");
    comHeightLay->addWidget(comHeightLab);

    comHeightBoxQuad = new QDoubleSpinBox;
    comHeightBoxQuad->setSizePolicy(pbsize);
    comHeightBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comHeightBoxQuad->setToolTip(comHeightLab->toolTip());
    comHeightBoxQuad->setDecimals(3);
    comHeightBoxQuad->setValue(0.25);
    comHeightBoxQuad->setSingleStep(0.01);
    comHeightBoxQuad->setMinimum(0);
    comHeightBoxQuad->setMaximum(5);
    comHeightLay->addWidget(comHeightBoxQuad);
    
    comSettingsLayout->addLayout(comHeightLay);

    // TORSO PITCH
    QHBoxLayout* torsoPitchLay = new QHBoxLayout;
    QLabel* torsoPitchLab = new QLabel;
    torsoPitchLab->setText("Torso Pitch:");
    torsoPitchLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    torsoPitchLab->setToolTip("How much the torso should be pitched in radians? (Positive is forward)");
    torsoPitchLay->addWidget(torsoPitchLab);

    torsoPitchBoxQuad = new QDoubleSpinBox;
    torsoPitchBoxQuad->setSizePolicy(pbsize);
    torsoPitchBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    torsoPitchBoxQuad->setToolTip(torsoPitchLab->toolTip());
    torsoPitchBoxQuad->setDecimals(3);
    torsoPitchBoxQuad->setMinimum(-3);
    torsoPitchBoxQuad->setMaximum(3);
    torsoPitchBoxQuad->setValue(-1.22);
    torsoPitchBoxQuad->setSingleStep(0.01);
    torsoPitchLay->addWidget(torsoPitchBoxQuad);
    
    comSettingsLayout->addLayout(torsoPitchLay);

    
    QHBoxLayout* comIKAngleWeightLay = new QHBoxLayout;
    QLabel* comIKAngleWeightLab = new QLabel;
    comIKAngleWeightLab->setText("IK Angle Weight:");
    comIKAngleWeightLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    comIKAngleWeightLab->setToolTip("I don't really know what this does.");
    comIKAngleWeightLay->addWidget(comIKAngleWeightLab);

    comIKAngleWeightBoxQuad = new QDoubleSpinBox;
    comIKAngleWeightBoxQuad->setSizePolicy(pbsize);
    comIKAngleWeightBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    comIKAngleWeightBoxQuad->setToolTip(comIKAngleWeightLab->toolTip());
    comIKAngleWeightBoxQuad->setDecimals(3);
    comIKAngleWeightBoxQuad->setValue(0);
    comIKAngleWeightBoxQuad->setSingleStep(0.01);
    comIKAngleWeightBoxQuad->setMinimum(0);
    comIKAngleWeightBoxQuad->setMaximum(5);
    comIKAngleWeightLay->addWidget(comIKAngleWeightBoxQuad);
    
    comSettingsLayout->addLayout(comIKAngleWeightLay);

    QHBoxLayout* fixed_com_offset_xLay = new QHBoxLayout;
    QLabel* fixed_com_offset_xLab = new QLabel;
    fixed_com_offset_xLab->setText("Fixed COM Offset X:");
    fixed_com_offset_xLab->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    fixed_com_offset_xLab->setToolTip("Offset of fixed COM from trunk (m).");
    fixed_com_offset_xLay->addWidget(fixed_com_offset_xLab);

    fixed_com_offset_xBoxQuad = new QDoubleSpinBox;
    fixed_com_offset_xBoxQuad->setSizePolicy(pbsize);
    fixed_com_offset_xBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_xBoxQuad->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_xBoxQuad->setDecimals(3);
    fixed_com_offset_xBoxQuad->setValue(0);
    fixed_com_offset_xBoxQuad->setSingleStep(0.01);
    fixed_com_offset_xBoxQuad->setMinimum(0);
    fixed_com_offset_xBoxQuad->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_xBoxQuad);

    fixed_com_offset_yBoxQuad = new QDoubleSpinBox;
    fixed_com_offset_yBoxQuad->setSizePolicy(pbsize);
    fixed_com_offset_yBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_yBoxQuad->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_yBoxQuad->setDecimals(3);
    fixed_com_offset_yBoxQuad->setValue(0);
    fixed_com_offset_yBoxQuad->setSingleStep(0.01);
    fixed_com_offset_yBoxQuad->setMinimum(0);
    fixed_com_offset_yBoxQuad->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_yBoxQuad);

    fixed_com_offset_zBoxQuad = new QDoubleSpinBox;
    fixed_com_offset_zBoxQuad->setSizePolicy(pbsize);
    fixed_com_offset_zBoxQuad->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    fixed_com_offset_zBoxQuad->setToolTip(fixed_com_offset_xLab->toolTip());
    fixed_com_offset_zBoxQuad->setDecimals(3);
    fixed_com_offset_zBoxQuad->setValue(0);
    fixed_com_offset_zBoxQuad->setSingleStep(0.01);
    fixed_com_offset_zBoxQuad->setMinimum(0);
    fixed_com_offset_zBoxQuad->setMaximum(5);
    fixed_com_offset_xLay->addWidget(fixed_com_offset_zBoxQuad);

    comSettingsLayout->addLayout(fixed_com_offset_xLay);

    // Constant Body Z
    constantBodyZBoxQuad = new QCheckBox;
    constantBodyZBoxQuad->setText("Constant Body Z");
    constantBodyZBoxQuad->setToolTip("Ignore the walk distance, and walk until Stop is selected");
    constantBodyZBoxQuad->setChecked(true);
    constantBodyZBoxQuad->setDisabled(false);
    comSettingsLayout->addWidget(constantBodyZBoxQuad, 0, Qt::AlignRight);
    
    QGroupBox* comSettingsBoxQuad = new QGroupBox;
    comSettingsBoxQuad->setTitle("Center of Mass Settings");
    comSettingsBoxQuad->setStyleSheet(groupStyleSheet);
    comSettingsBoxQuad->setLayout(comSettingsLayout);
    rightColumn->addWidget(comSettingsBoxQuad);
    
    
    QHBoxLayout* bottomLayout = new QHBoxLayout;
    bottomLayout->addLayout(leftColumn);
    bottomLayout->addLayout(rightColumn);
    
    QVBoxLayout* masterZMPLayout = new QVBoxLayout;
    masterZMPLayout->addLayout(profileLayoutMaster);
    masterZMPLayout->addLayout(bottomLayout);
    
    zmpQuadrupedParamTab = new QWidget;
    zmpQuadrupedParamTab->setLayout(masterZMPLayout);
    
//    saveAsEditQuad->setText("Default");
//    handleQuadrupedProfileSaveAs();
//    saveAsEditQuad->setText("Default-Backup");
//    handleQuadrupedProfileSaveAs();
//    saveAsEditQuad->clear();
    
//    profileSelectQuad->setCurrentIndex(0);
    // end QUADRUPED PARAM TAB INITIALIZATION
}

#endif // HAVE_HUBOMZ

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

    QHBoxLayout* gainsLabelLayout = new QHBoxLayout;
    QLabel* gainTypeLab = new QLabel;
    gainTypeLab->setText("<u><b>Gain Type</b></u>");
    gainsLabelLayout->addWidget(gainTypeLab);
    QLabel* balGainsLab = new QLabel;
    balGainsLab->setText("<u><b>Balance Gains<\b></u>");
    gainsLabelLayout->addWidget(balGainsLab);
    QLabel* walkGainsLab = new QLabel;
    walkGainsLab->setText("<u><b>Walking Gains</b></u>");
    gainsLabelLayout->addWidget(walkGainsLab);
    bottomLayout->addLayout(gainsLabelLayout);

    QHBoxLayout* flatLayout = new QHBoxLayout;
    QLabel* flatLab = new QLabel;
    flatLab->setText("Flattening Gains:");
    flatLab->setToolTip("Gains for foot flattening (Left/Right)");
    flatLayout->addWidget(flatLab);
    flattenBoxBal = new QDoubleSpinBox;
    flattenBoxBal->setDecimals(4);
    flattenBoxBal->setSingleStep(0.003);
    flattenBoxBal->setMinimum(0);
    flattenBoxBal->setMaximum(99999);
    flattenBoxBal->setValue(0.0005);
    flatLayout->addWidget(flattenBoxBal);
    flattenBoxWalk = new QDoubleSpinBox;
    flattenBoxWalk->setDecimals(4);
    flattenBoxWalk->setSingleStep(0.003);
    flattenBoxWalk->setMinimum(0);
    flattenBoxWalk->setMaximum(99999);
    flattenBoxWalk->setValue(0.0005);
    flatLayout->addWidget(flattenBoxWalk);

    bottomLayout->addLayout(flatLayout);

    QHBoxLayout* decayLayout = new QHBoxLayout;
    QLabel* decayLab = new QLabel;
    decayLab->setText("Decay Gain:");
    decayLayout->addWidget(decayLab);
    decayBoxBal = new QDoubleSpinBox;
    decayBoxBal->setDecimals(4);
    decayBoxBal->setSingleStep(0.05);
    decayBoxBal->setMinimum(0);
    decayBoxBal->setMaximum(1);
    decayBoxBal->setValue(0.5);
    decayLayout->addWidget(decayBoxBal);
    decayBoxWalk = new QDoubleSpinBox;
    decayBoxWalk->setDecimals(4);
    decayBoxWalk->setSingleStep(0.05);
    decayBoxWalk->setMinimum(0);
    decayBoxWalk->setMaximum(1);
    decayBoxWalk->setValue(0.5);
    decayLayout->addWidget(decayBoxWalk);

    bottomLayout->addLayout(decayLayout);

    QHBoxLayout* threshMinLayout = new QHBoxLayout;
    QLabel* threshMinLab = new QLabel;
    threshMinLab->setText("Minimum Force Threshold:");
    threshMinLab->setToolTip("Minimum force cut-off for flattening foot (Left/Right)");
    threshMinLayout->addWidget(threshMinLab);
    threshMinBoxBal = new QDoubleSpinBox;
    threshMinBoxBal->setDecimals(4);
    threshMinBoxBal->setSingleStep(1);
    threshMinBoxBal->setMinimum(-99999);
    threshMinBoxBal->setMaximum(99999);
    threshMinBoxBal->setValue(12);
    threshMinLayout->addWidget(threshMinBoxBal);
    threshMinBoxWalk = new QDoubleSpinBox;
    threshMinBoxWalk->setDecimals(4);
    threshMinBoxWalk->setSingleStep(1);
    threshMinBoxWalk->setMinimum(-99999);
    threshMinBoxWalk->setMaximum(99999);
    threshMinBoxWalk->setValue(12);
    threshMinLayout->addWidget(threshMinBoxWalk);

    bottomLayout->addLayout(threshMinLayout);

    QHBoxLayout* threshMaxLayout = new QHBoxLayout;
    QLabel* threshMaxLab = new QLabel;
    threshMaxLab->setText("Maximum Force Threshold:");
    threshMaxLab->setToolTip("Minimum force cut-off for flattening foot (Left/Right)");
    threshMaxLayout->addWidget(threshMaxLab);
    threshMaxBoxBal = new QDoubleSpinBox;
    threshMaxBoxBal->setDecimals(4);
    threshMaxBoxBal->setSingleStep(1);
    threshMaxBoxBal->setMinimum(-99999);
    threshMaxBoxBal->setMaximum(99999);
    threshMaxBoxBal->setValue(55);
    threshMaxLayout->addWidget(threshMaxBoxBal);
    threshMaxBoxWalk = new QDoubleSpinBox;
    threshMaxBoxWalk->setDecimals(4);
    threshMaxBoxWalk->setSingleStep(1);
    threshMaxBoxWalk->setMinimum(-99999);
    threshMaxBoxWalk->setMaximum(99999);
    threshMaxBoxWalk->setValue(55);
    threshMaxLayout->addWidget(threshMaxBoxWalk);

    bottomLayout->addLayout(threshMaxLayout);

    QHBoxLayout* straightenPLayout = new QHBoxLayout;
    QLabel* straightenPLab = new QLabel;
    straightenPLab->setText("IMU Offset Gain P:");
    straightenPLab->setToolTip("Gain for keeping torso upright (Front/Back)");
    straightenPLayout->addWidget(straightenPLab);
    straightenPBoxBal = new QDoubleSpinBox;
    straightenPBoxBal->setDecimals(4);
    straightenPBoxBal->setSingleStep(1);
    straightenPBoxBal->setMinimum(-99999);
    straightenPBoxBal->setMaximum(99999);
    straightenPBoxBal->setValue(0.055);
    straightenPLayout->addWidget(straightenPBoxBal);
    straightenPBoxWalk = new QDoubleSpinBox;
    straightenPBoxWalk->setDecimals(4);
    straightenPBoxWalk->setSingleStep(1);
    straightenPBoxWalk->setMinimum(-99999);
    straightenPBoxWalk->setMaximum(99999);
    straightenPBoxWalk->setValue(0.0);
    straightenPLayout->addWidget(straightenPBoxWalk);

    bottomLayout->addLayout(straightenPLayout);

    QHBoxLayout* straightenRLayout = new QHBoxLayout;
    QLabel* straightenRLab = new QLabel;
    straightenRLab->setText("IMU Offset Gain R:");
    straightenRLab->setToolTip("Gain for keeping torso upright (Left/Right)");
    straightenRLayout->addWidget(straightenRLab);
    straightenRBoxBal = new QDoubleSpinBox;
    straightenRBoxBal->setDecimals(4);
    straightenRBoxBal->setSingleStep(1);
    straightenRBoxBal->setMinimum(-99999);
    straightenRBoxBal->setMaximum(99999);
    straightenRBoxBal->setValue(0);
    straightenRLayout->addWidget(straightenRBoxBal);
    straightenRBoxWalk = new QDoubleSpinBox;
    straightenRBoxWalk->setDecimals(4);
    straightenRBoxWalk->setSingleStep(1);
    straightenRBoxWalk->setMinimum(-99999);
    straightenRBoxWalk->setMaximum(99999);
    straightenRBoxWalk->setValue(0);
    straightenRLayout->addWidget(straightenRBoxWalk);

    bottomLayout->addLayout(straightenRLayout);

    QHBoxLayout* springLayout = new QHBoxLayout;
    QLabel* springLab = new QLabel;
    springLab->setText("Spring Gain:");
    springLab->setToolTip("Gain for the speed of squatting up and down | Spring constant");
    springLayout->addWidget(springLab);
    springBoxBal = new QDoubleSpinBox;
    springBoxBal->setDecimals(4);
    springBoxBal->setSingleStep(1);
    springBoxBal->setMinimum(-99999);
    springBoxBal->setMaximum(99999);
    springBoxBal->setValue(0.2);
    springLayout->addWidget(springBoxBal);
    springBoxWalk = new QDoubleSpinBox;
    springBoxWalk->setDecimals(4);
    springBoxWalk->setSingleStep(1);
    springBoxWalk->setMinimum(-99999);
    springBoxWalk->setMaximum(99999);
    springBoxWalk->setValue(0.0);
    springLayout->addWidget(springBoxWalk);

    bottomLayout->addLayout(springLayout);

    QHBoxLayout* dampLayout = new QHBoxLayout;
    QLabel* dampLab = new QLabel;
    dampLab->setText("Damping Gain:");
    dampLab->setToolTip("Damping gain for complying the knees | Damping constant");
    dampLayout->addWidget(dampLab);
    dampBoxBal = new QDoubleSpinBox;
    dampBoxBal->setDecimals(4);
    dampBoxBal->setSingleStep(1);
    dampBoxBal->setMinimum(-99999);
    dampBoxBal->setMaximum(99999);
    dampBoxBal->setValue(0);
    dampLayout->addWidget(dampBoxBal);
    dampBoxWalk = new QDoubleSpinBox;
    dampBoxWalk->setDecimals(4);
    dampBoxWalk->setSingleStep(1);
    dampBoxWalk->setMinimum(-99999);
    dampBoxWalk->setMaximum(99999);
    dampBoxWalk->setValue(0);
    dampLayout->addWidget(dampBoxWalk);

    bottomLayout->addLayout(dampLayout);

    QHBoxLayout* responseLayout = new QHBoxLayout;
    QLabel* responseLab = new QLabel;
    responseLab->setText("Mass Gain:");
    responseLab->setToolTip("Knee response based on Fz in feet | Mass constant");
    responseLayout->addWidget(responseLab);
    responseBoxBal = new QDoubleSpinBox;
    responseBoxBal->setDecimals(4);
    responseBoxBal->setSingleStep(1);
    responseBoxBal->setMinimum(-99999);
    responseBoxBal->setMaximum(99999);
    responseBoxBal->setValue(0);
    responseLayout->addWidget(responseBoxBal);
    responseBoxWalk = new QDoubleSpinBox;
    responseBoxWalk->setDecimals(4);
    responseBoxWalk->setSingleStep(1);
    responseBoxWalk->setMinimum(-99999);
    responseBoxWalk->setMaximum(99999);
    responseBoxWalk->setValue(0);
    responseLayout->addWidget(responseBoxWalk);

    bottomLayout->addLayout(responseLayout);

    // Single-support hip nudge proportional and derivative gains based on ankle torque readings
    QHBoxLayout* singleSupportHipNudgeGainLayout = new QHBoxLayout;
    QLabel* singleSupportHipNudgeGainLab = new QLabel;
    singleSupportHipNudgeGainLab->setText("Single-Support Hip Nudge (Kp, Kd):");
    singleSupportHipNudgeGainLab->setToolTip("Gains for nudging the hips in single-support using feedback from the F/T sensors");
    singleSupportHipNudgeGainLayout->addWidget(singleSupportHipNudgeGainLab);
    singleSupportHipNudgeGainBoxP = new QDoubleSpinBox;
    singleSupportHipNudgeGainBoxP->setDecimals(4);
    singleSupportHipNudgeGainBoxP->setSingleStep(0.01);
    singleSupportHipNudgeGainBoxP->setMinimum(0);
    singleSupportHipNudgeGainBoxP->setMaximum(10);
    singleSupportHipNudgeGainBoxP->setValue(0);
    singleSupportHipNudgeGainLayout->addWidget(singleSupportHipNudgeGainBoxP);
    singleSupportHipNudgeGainBoxD = new QDoubleSpinBox;
    singleSupportHipNudgeGainBoxD->setDecimals(4);
    singleSupportHipNudgeGainBoxD->setSingleStep(0.01);
    singleSupportHipNudgeGainBoxD->setMinimum(0);
    singleSupportHipNudgeGainBoxD->setMaximum(10);
    singleSupportHipNudgeGainBoxD->setValue(0);
    singleSupportHipNudgeGainLayout->addWidget(singleSupportHipNudgeGainBoxD);

    bottomLayout->addLayout(singleSupportHipNudgeGainLayout);

    // Double-support hip nudge proportional and derivative gains based on ankle torque readings.
    QHBoxLayout* doubleSupportHipNudgeGainLayout = new QHBoxLayout;
    QLabel* doubleSupportHipNudgeGainLab = new QLabel;
    doubleSupportHipNudgeGainLab->setText("Double-Support Hip Nudge (Kp, Kd):");
    doubleSupportHipNudgeGainLab->setToolTip("Gains for nudging the hips in double-support using feedback from the F/T sensors");
    doubleSupportHipNudgeGainLayout->addWidget(doubleSupportHipNudgeGainLab);
    doubleSupportHipNudgeGainBoxP = new QDoubleSpinBox;
    doubleSupportHipNudgeGainBoxP->setDecimals(4);
    doubleSupportHipNudgeGainBoxP->setSingleStep(0.01);
    doubleSupportHipNudgeGainBoxP->setMinimum(0);
    doubleSupportHipNudgeGainBoxP->setMaximum(10);
    doubleSupportHipNudgeGainBoxP->setValue(0.0005);
    doubleSupportHipNudgeGainLayout->addWidget(doubleSupportHipNudgeGainBoxP);
    doubleSupportHipNudgeGainBoxD = new QDoubleSpinBox;
    doubleSupportHipNudgeGainBoxD->setDecimals(4);
    doubleSupportHipNudgeGainBoxD->setSingleStep(0.01);
    doubleSupportHipNudgeGainBoxD->setMinimum(0);
    doubleSupportHipNudgeGainBoxD->setMaximum(10);
    doubleSupportHipNudgeGainBoxD->setValue(0);
    doubleSupportHipNudgeGainLayout->addWidget(doubleSupportHipNudgeGainBoxD);

    bottomLayout->addLayout(doubleSupportHipNudgeGainLayout);

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

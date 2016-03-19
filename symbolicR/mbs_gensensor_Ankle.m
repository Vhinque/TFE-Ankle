%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Fri Mar 11 09:08:37 2016
%
%	==> Project name : Ankle
%	==> using XML input file 
%
%	==> Number of joints : 7
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 182
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,7);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C1 = cos(q(1));
  S1 = sin(q(1));
  C2 = cos(q(2));
  S2 = sin(q(2));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C3 = cos(q(3));
  S3 = sin(q(3));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_2_0_1 = = 
 
% Trigonometric Variables  

  S1p2 = C1*S2+S1*C2;
  C1p2 = C1*C2-S1*S2;

% = = Block_0_0_0_4_0_3 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;

% = = Block_0_0_0_6_0_4 = = 
 
% Trigonometric Variables  

  S3p6 = C3*S6+S3*C6;
  C3p6 = C3*C6-S3*S6;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.R(1,1) = C1;
    sens.R(1,3) = -S1;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S1;
    sens.R(3,3) = C1;
    sens.OM(2) = qd(1);
    sens.OMP(2) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_0_1 = = 
 
% Sensor Kinematics 


    RLcp1_12 = s.dpt(1,2)*C1;
    RLcp1_32 = -s.dpt(1,2)*S1;
    OMcp1_22 = qd(1)+qd(2);
    ORcp1_12 = RLcp1_32*qd(1);
    ORcp1_32 = -RLcp1_12*qd(1);
    OPcp1_22 = qdd(1)+qdd(2);
    ACcp1_12 = ORcp1_32*qd(1)+RLcp1_32*qdd(1);
    ACcp1_32 = -(ORcp1_12*qd(1)+RLcp1_12*qdd(1));

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = RLcp1_12;
    sens.P(3) = RLcp1_32;
    sens.R(1,1) = C1p2;
    sens.R(1,3) = -S1p2;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S1p2;
    sens.R(3,3) = C1p2;
    sens.V(1) = ORcp1_12;
    sens.V(3) = ORcp1_32;
    sens.OM(2) = OMcp1_22;
    sens.A(1) = ACcp1_12;
    sens.A(3) = ACcp1_32;
    sens.OMP(2) = OPcp1_22;
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.R(1,1) = C3;
    sens.R(1,3) = -S3;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S3;
    sens.R(3,3) = C3;
    sens.OM(2) = qd(3);
    sens.OMP(2) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_0_3 = = 
 
% Sensor Kinematics 


    RLcp3_14 = s.dpt(1,4)*C3+s.dpt(3,4)*S3;
    RLcp3_34 = -(s.dpt(1,4)*S3-s.dpt(3,4)*C3);
    OMcp3_24 = qd(3)+qd(4);
    ORcp3_14 = RLcp3_34*qd(3);
    ORcp3_34 = -RLcp3_14*qd(3);
    OPcp3_24 = qdd(3)+qdd(4);
    ACcp3_14 = ORcp3_34*qd(3)+RLcp3_34*qdd(3);
    ACcp3_34 = -(ORcp3_14*qd(3)+RLcp3_14*qdd(3));

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = RLcp3_14;
    sens.P(3) = RLcp3_34;
    sens.R(1,1) = C3p4;
    sens.R(1,3) = -S3p4;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S3p4;
    sens.R(3,3) = C3p4;
    sens.V(1) = ORcp3_14;
    sens.V(3) = ORcp3_34;
    sens.OM(2) = OMcp3_24;
    sens.A(1) = ACcp3_14;
    sens.A(3) = ACcp3_34;
    sens.OMP(2) = OPcp3_24;
 
% 
case 5, 


% = = Block_1_0_0_5_0_3 = = 
 
% Sensor Kinematics 


    RLcp4_14 = s.dpt(1,4)*C3+s.dpt(3,4)*S3;
    RLcp4_34 = -(s.dpt(1,4)*S3-s.dpt(3,4)*C3);
    OMcp4_24 = qd(3)+qd(4);
    ORcp4_14 = RLcp4_34*qd(3);
    ORcp4_34 = -RLcp4_14*qd(3);
    OPcp4_24 = qdd(3)+qdd(4);
    RLcp4_15 = q(5)*S3p4;
    RLcp4_35 = q(5)*C3p4;
    POcp4_15 = RLcp4_14+RLcp4_15;
    POcp4_35 = RLcp4_34+RLcp4_35;
    ORcp4_15 = OMcp4_24*RLcp4_35;
    ORcp4_35 = -OMcp4_24*RLcp4_15;
    VIcp4_15 = ORcp4_14+ORcp4_15+qd(5)*S3p4;
    VIcp4_35 = ORcp4_34+ORcp4_35+qd(5)*C3p4;
    ACcp4_15 = OMcp4_24*(ORcp4_35+(2.0)*qd(5)*C3p4)+OPcp4_24*RLcp4_35+ORcp4_34*qd(3)+RLcp4_34*qdd(3)+qdd(5)*S3p4;
    ACcp4_35 = -(OMcp4_24*(ORcp4_15+(2.0)*qd(5)*S3p4)+OPcp4_24*RLcp4_15+ORcp4_14*qd(3)+RLcp4_14*qdd(3)-qdd(5)*C3p4);

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp4_15;
    sens.P(3) = POcp4_35;
    sens.R(1,1) = C3p4;
    sens.R(1,3) = -S3p4;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S3p4;
    sens.R(3,3) = C3p4;
    sens.V(1) = VIcp4_15;
    sens.V(3) = VIcp4_35;
    sens.OM(2) = OMcp4_24;
    sens.A(1) = ACcp4_15;
    sens.A(3) = ACcp4_35;
    sens.OMP(2) = OPcp4_24;
 
% 
case 6, 


% = = Block_1_0_0_6_0_4 = = 
 
% Sensor Kinematics 


    RLcp5_16 = s.dpt(1,5)*C3+s.dpt(3,5)*S3;
    RLcp5_36 = -(s.dpt(1,5)*S3-s.dpt(3,5)*C3);
    OMcp5_26 = qd(3)+qd(6);
    ORcp5_16 = RLcp5_36*qd(3);
    ORcp5_36 = -RLcp5_16*qd(3);
    OPcp5_26 = qdd(3)+qdd(6);
    ACcp5_16 = ORcp5_36*qd(3)+RLcp5_36*qdd(3);
    ACcp5_36 = -(ORcp5_16*qd(3)+RLcp5_16*qdd(3));

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = RLcp5_16;
    sens.P(3) = RLcp5_36;
    sens.R(1,1) = C3p6;
    sens.R(1,3) = -S3p6;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S3p6;
    sens.R(3,3) = C3p6;
    sens.V(1) = ORcp5_16;
    sens.V(3) = ORcp5_36;
    sens.OM(2) = OMcp5_26;
    sens.A(1) = ACcp5_16;
    sens.A(3) = ACcp5_36;
    sens.OMP(2) = OPcp5_26;
 
% 
case 7, 


% = = Block_1_0_0_7_0_4 = = 
 
% Sensor Kinematics 


    RLcp6_16 = s.dpt(1,5)*C3+s.dpt(3,5)*S3;
    RLcp6_36 = -(s.dpt(1,5)*S3-s.dpt(3,5)*C3);
    OMcp6_26 = qd(3)+qd(6);
    ORcp6_16 = RLcp6_36*qd(3);
    ORcp6_36 = -RLcp6_16*qd(3);
    OPcp6_26 = qdd(3)+qdd(6);
    RLcp6_17 = q(7)*C3p6;
    RLcp6_37 = -q(7)*S3p6;
    POcp6_17 = RLcp6_16+RLcp6_17;
    POcp6_37 = RLcp6_36+RLcp6_37;
    ORcp6_17 = OMcp6_26*RLcp6_37;
    ORcp6_37 = -OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_16+ORcp6_17+qd(7)*C3p6;
    VIcp6_37 = ORcp6_36+ORcp6_37-qd(7)*S3p6;
    ACcp6_17 = OMcp6_26*(ORcp6_37-(2.0)*qd(7)*S3p6)+OPcp6_26*RLcp6_37+ORcp6_36*qd(3)+RLcp6_36*qdd(3)+qdd(7)*C3p6;
    ACcp6_37 = -(OMcp6_26*(ORcp6_17+(2.0)*qd(7)*C3p6)+OPcp6_26*RLcp6_17+ORcp6_16*qd(3)+RLcp6_16*qdd(3)+qdd(7)*S3p6);

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_17;
    sens.P(3) = POcp6_37;
    sens.R(1,1) = C3p6;
    sens.R(1,3) = -S3p6;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S3p6;
    sens.R(3,3) = C3p6;
    sens.V(1) = VIcp6_17;
    sens.V(3) = VIcp6_37;
    sens.OM(2) = OMcp6_26;
    sens.A(1) = ACcp6_17;
    sens.A(3) = ACcp6_37;
    sens.OMP(2) = OPcp6_26;

end


% ====== END Task 1 ====== 

  


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
%	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
%	==> Flops complexity : 117
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [Jdqd] = cons_jdqd(s,tsim,usrfun)

 Jdqd = zeros(3,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_0_1 = = 
 
% Trigonometric Variables  

%
  S1p2 = C1*S2+S1*C2;
  C1p2 = C1*C2-S1*S2;

% = = Block_0_1_0_0_0_3 = = 
 
% Trigonometric Variables  

%
  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;

% = = Block_0_1_0_0_0_4 = = 
 
% Trigonometric Variables  

%
  S3p6 = C3*S6+S3*C6;
  C3p6 = C3*C6-S3*S6;
 
% Constraints and Constraints Jacobian 

  RL_2_16 = s.dpt(1,5)*C3+s.dpt(3,5)*S3;
  RL_2_36 = -(s.dpt(1,5)*S3-s.dpt(3,5)*C3);
  RL_2_17 = q(7)*C3p6;
  RL_2_37 = -q(7)*S3p6;

% = = Block_0_2_0_0_0_0 = = 
 
% Constraints Quadratic Terms 

%
  OM_0_24 = qd(3)+qd(4);
%
  OM_1_22 = qd(1)+qd(2);
%
  OM_2_26 = qd(3)+qd(6);
  OR_2_16 = RL_2_36*qd(3);
  OR_2_36 = -RL_2_16*qd(3);
  OR_2_17 = OM_2_26*RL_2_37;
  OR_2_37 = -OM_2_26*RL_2_17;
  VI_2_17 = OR_2_16+OR_2_17+qd(7)*C3p6;
  VI_2_37 = OR_2_36+OR_2_37-qd(7)*S3p6;

% = = Block_0_2_0_0_0_1 = = 
 
% Constraints Quadratic Terms 

%
  jdqd1 = -(OM_0_24*(OM_0_24*q(5)*S3p4-(2.0)*qd(5)*C3p4)-OM_1_22*OM_1_22*s.dpt(1,3)*C1p2-qd(1)*qd(1)*s.dpt(1,2)*C1+qd(3)*qd(3)*(s.dpt(1,4)*C3+...
 s.dpt(3,4)*S3));
  jdqd3 = qd(3)*qd(3)*(s.dpt(1,4)*S3-s.dpt(3,4)*C3)-OM_0_24*(OM_0_24*q(5)*C3p4+(2.0)*qd(5)*S3p4)-(OM_1_22*OM_1_22*s.dpt(1,3)*S1p2+qd(1)*qd(1)*...
 s.dpt(1,2)*S1);
%
  jdqd4 = VI_2_17*VI_2_17+VI_2_37*VI_2_37-(OM_2_26*(OR_2_17+(2.0)*qd(7)*C3p6)+OR_2_16*qd(3))*(RL_2_36+RL_2_37-s.dpt(3,1))+(OM_2_26*(OR_2_37-(2.0)*qd(7)*...
 S3p6)+OR_2_36*qd(3))*(RL_2_16+RL_2_17-s.dpt(1,1));

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  Jdqd(1) = jdqd1;
  Jdqd(2) = jdqd3;
  Jdqd(3) = jdqd4;

% ====== END Task 0 ====== 

  


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
%	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
%	==> Flops complexity : 86
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [h,Jac] = cons_hJ(s,tsim,usrfun)

 h = zeros(3,1);
 Jac = zeros(3,7);

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
 
% Constraints and Constraints Jacobian 

  RL_1_12 = s.dpt(1,2)*C1;
  RL_1_32 = -s.dpt(1,2)*S1;
  RL_1_19 = s.dpt(1,3)*C1p2;
  RL_1_39 = -s.dpt(1,3)*S1p2;
  JT_1_19_1 = RL_1_32+RL_1_39;
  JT_1_39_1 = -(RL_1_12+RL_1_19);

% = = Block_0_1_0_0_0_3 = = 
 
% Trigonometric Variables  

%
  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
 
% Constraints and Constraints Jacobian 

  RL_0_14 = s.dpt(1,4)*C3+s.dpt(3,4)*S3;
  RL_0_34 = -(s.dpt(1,4)*S3-s.dpt(3,4)*C3);
  RL_0_15 = q(5)*S3p4;
  RL_0_35 = q(5)*C3p4;
  JT_0_15_3 = RL_0_34+RL_0_35;
  JT_0_35_3 = -(RL_0_14+RL_0_15);

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

% = = Block_0_1_0_0_1_0 = = 
 
% Constraints and Constraints Jacobian 

%
  h_1 = RL_0_14+RL_0_15-RL_1_12-RL_1_19;
  h_3 = RL_0_34+RL_0_35-RL_1_32-RL_1_39;
%
  Plp11 = RL_2_16+RL_2_17-s.dpt(1,1);
  Plp31 = RL_2_36+RL_2_37-s.dpt(3,1);
  h_4 = (0.50)*(Plp11*Plp11+Plp31*Plp31-s.lrod(1)*s.lrod(1));
%
  Jacu_4_3 = Plp11*(RL_2_36+RL_2_37)-Plp31*(RL_2_16+RL_2_17);
  Jacu_4_6 = Plp11*RL_2_37-Plp31*RL_2_17;
  Jac_4_7 = Plp11*C3p6-Plp31*S3p6;

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  h(1) = h_1;
  h(2) = h_3;
  h(3) = h_4;
  Jac(1,1) = -JT_1_19_1;
  Jac(1,2) = -RL_1_39;
  Jac(1,3) = JT_0_15_3;
  Jac(1,4) = RL_0_35;
  Jac(1,5) = S3p4;
  Jac(2,1) = -JT_1_39_1;
  Jac(2,2) = RL_1_19;
  Jac(2,3) = JT_0_35_3;
  Jac(2,4) = -RL_0_15;
  Jac(2,5) = C3p4;
  Jac(3,3) = Jacu_4_3;
  Jac(3,6) = Jacu_4_6;
  Jac(3,7) = Jac_4_7;

% ====== END Task 0 ====== 

  


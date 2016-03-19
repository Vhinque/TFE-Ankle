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
%	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
%	==> Flops complexity : 197
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [M,c] = dirdyna(s,tsim,usrfun)

 M = zeros(7,7);
 c = zeros(7,1);

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

  C2 = cos(q(2));
  S2 = sin(q(2));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_1_0_0_0_1 = = 
 
% Forward Kinematics 

  BS11 = -qd(1)*qd(1);
  OM22 = qd(1)+qd(2);

% = = Block_0_1_0_0_0_2 = = 
 
% Forward Kinematics 

  BS13 = -qd(3)*qd(3);
  BS93 = -qd(3)*qd(3);

% = = Block_0_1_0_1_0_3 = = 
 
% Forward Kinematics 

  OM24 = qd(3)+qd(4);

% = = Block_0_1_0_1_0_4 = = 
 
% Forward Kinematics 

  OM26 = qd(3)+qd(6);

% = = Block_0_2_0_1_0_3 = = 
 
% Backward Dynamics 

  FA15 = -(s.frc(1,5)-s.m(5)*((2.0)*qd(5)*OM24+BS13*s.dpt(1,4)*C4-BS93*s.dpt(3,4)*S4));
  FA35 = -(s.frc(3,5)+s.m(5)*(q(5)*OM24*OM24-BS13*s.dpt(1,4)*S4-BS93*s.dpt(3,4)*C4));
  FB15_3 = s.m(5)*(q(5)+s.dpt(1,4)*S4+s.dpt(3,4)*C4);
  FB35_3 = -s.m(5)*(s.dpt(1,4)*C4-s.dpt(3,4)*S4);
  CF4_25 = -(s.trq(2,5)-q(5)*FA15);
  CM43_25 = q(5)*FB15_3;
  CM44_25 = q(5)*q(5)*s.m(5);

% = = Block_0_2_0_1_0_4 = = 
 
% Backward Dynamics 

  FA17 = -(s.frc(1,7)+s.m(7)*(q(7)*OM26*OM26-BS13*s.dpt(1,5)*C6+BS93*s.dpt(3,5)*S6));
  FA37 = -(s.frc(3,7)+s.m(7)*((2.0)*qd(7)*OM26-BS13*s.dpt(1,5)*S6-BS93*s.dpt(3,5)*C6));
  FB17_3 = s.m(7)*(s.dpt(1,5)*S6+s.dpt(3,5)*C6);
  FB37_3 = -s.m(7)*(q(7)+s.dpt(1,5)*C6-s.dpt(3,5)*S6);
  CF6_27 = -(s.trq(2,7)+q(7)*FA37);
  CM63_27 = -q(7)*FB37_3;
  CM66_27 = q(7)*q(7)*s.m(7);

% = = Block_0_2_0_2_0_1 = = 
 
% Backward Dynamics 

  FA32 = -(s.frc(3,2)-s.m(2)*BS11*s.dpt(1,2)*S2);
  CF22 = -(s.trq(2,2)+FA32*s.l(1,2));
  FB32_1 = -s.m(2)*(s.l(1,2)+s.dpt(1,2)*C2);
  CM22_1 = s.In(5,2)-FB32_1*s.l(1,2);
  CM22_2 = s.In(5,2)+s.m(2)*s.l(1,2)*s.l(1,2);
  CF21 = -(s.trq(2,1)-CF22-s.frc(3,1)*s.l(1,1)+s.dpt(1,2)*(FA32*C2+S2*(s.frc(1,2)-s.m(2)*(BS11*s.dpt(1,2)*C2-OM22*OM22*s.l(1,2)))));
  CM21_1 = s.In(5,1)+CM22_1+s.m(1)*s.l(1,1)*s.l(1,1)+s.dpt(1,2)*(s.m(2)*s.dpt(1,2)*S2*S2-FB32_1*C2);

% = = Block_0_2_0_2_0_2 = = 
 
% Backward Dynamics 

  CF23 = -(s.trq(2,3)-CF4_25-CF6_27-s.dpt(1,4)*(FA15*S4-FA35*C4)-s.dpt(1,5)*(FA17*S6-FA37*C6)-s.dpt(3,4)*(FA15*C4+FA35*S4)-s.dpt(3,5)*(FA17*C6+...
 FA37*S6)-s.l(1,3)*(s.frc(3,3)-s.m(3)*BS93*s.l(3,3))+s.l(3,3)*(s.frc(1,3)-s.m(3)*BS13*s.l(1,3)));
  CM23_3 = s.In(5,3)+CM43_25+CM63_27+s.m(3)*s.l(1,3)*s.l(1,3)+s.m(3)*s.l(3,3)*s.l(3,3)+s.dpt(1,4)*(FB15_3*S4-FB35_3*C4)+s.dpt(1,5)*(FB17_3*S6-...
 FB37_3*C6)+s.dpt(3,4)*(FB15_3*C4+FB35_3*S4)+s.dpt(3,5)*(FB17_3*C6+FB37_3*S6);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  M(1,1) = CM21_1;
  M(1,2) = CM22_1;
  M(2,1) = CM22_1;
  M(2,2) = CM22_2;
  M(3,3) = CM23_3;
  M(3,4) = CM43_25;
  M(3,5) = FB35_3;
  M(3,6) = CM63_27;
  M(3,7) = FB17_3;
  M(4,3) = CM43_25;
  M(4,4) = CM44_25;
  M(5,3) = FB35_3;
  M(5,5) = s.m(5);
  M(6,3) = CM63_27;
  M(6,6) = CM66_27;
  M(7,3) = FB17_3;
  M(7,7) = s.m(7);
  c(1) = CF21;
  c(2) = CF22;
  c(3) = CF23;
  c(4) = CF4_25;
  c(5) = FA35;
  c(6) = CF6_27;
  c(7) = FA17;

% ====== END Task 0 ====== 

  


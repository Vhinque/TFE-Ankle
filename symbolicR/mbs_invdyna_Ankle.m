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
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 166
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(7,1);

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  BS11 = -qd(1)*qd(1);
  OM22 = qd(1)+qd(2);
  OMp22 = qdd(1)+qdd(2);
  BS13 = -qd(3)*qd(3);
  BS93 = -qd(3)*qd(3);
  OM24 = qd(3)+qd(4);
  OM26 = qd(3)+qd(6);
 
% Backward Dynamics 

  Fs17 = -(s.frc(1,7)-s.m(7)*(qdd(7)-q(7)*OM26*OM26+C6*(qdd(3)*s.dpt(3,5)+BS13*s.dpt(1,5))+S6*(qdd(3)*s.dpt(1,5)-BS93*s.dpt(3,5))));
  Fs37 = -(s.frc(3,7)+s.m(7)*(q(7)*(qdd(3)+qdd(6))+(2.0)*qd(7)*OM26+C6*(qdd(3)*s.dpt(1,5)-BS93*s.dpt(3,5))-S6*(qdd(3)*s.dpt(3,5)+BS13*s.dpt(1,5))));
  Cq26 = -(s.trq(2,7)+q(7)*Fs37);
  Fs15 = -(s.frc(1,5)-s.m(5)*(q(5)*(qdd(3)+qdd(4))+(2.0)*qd(5)*OM24+C4*(qdd(3)*s.dpt(3,4)+BS13*s.dpt(1,4))+S4*(qdd(3)*s.dpt(1,4)-BS93*s.dpt(3,4))));
  Fs35 = -(s.frc(3,5)-s.m(5)*(qdd(5)-q(5)*OM24*OM24-C4*(qdd(3)*s.dpt(1,4)-BS93*s.dpt(3,4))+S4*(qdd(3)*s.dpt(3,4)+BS13*s.dpt(1,4))));
  Cq24 = -(s.trq(2,5)-q(5)*Fs15);
  Cq23 = -(s.trq(2,3)-Cq24-Cq26-qdd(3)*s.In(5,3)-s.dpt(1,4)*(Fs15*S4-Fs35*C4)-s.dpt(1,5)*(Fs17*S6-Fs37*C6)-s.dpt(3,4)*(Fs15*C4+Fs35*S4)-...
 s.dpt(3,5)*(Fs17*C6+Fs37*S6)-s.l(1,3)*(s.frc(3,3)+s.m(3)*(qdd(3)*s.l(1,3)-BS93*s.l(3,3)))+s.l(3,3)*(s.frc(1,3)-s.m(3)*(qdd(3)*s.l(3,3)+BS13*s.l(1,3))...
 ));
  Fs32 = -(s.frc(3,2)+s.m(2)*(OMp22*s.l(1,2)+s.dpt(1,2)*(qdd(1)*C2-BS11*S2)));
  Cq22 = -(s.trq(2,2)-s.In(5,2)*OMp22+Fs32*s.l(1,2));
  Cq21 = -(s.trq(2,1)-Cq22-qdd(1)*s.In(5,1)+s.dpt(1,2)*(Fs32*C2+S2*(s.frc(1,2)+s.m(2)*(OM22*OM22*s.l(1,2)-s.dpt(1,2)*(qdd(1)*S2+BS11*C2))))-...
 s.l(1,1)*(s.frc(3,1)+qdd(1)*s.m(1)*s.l(1,1)));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Cq21;
  Qq(2) = Cq22;
  Qq(3) = Cq23;
  Qq(4) = Cq24;
  Qq(5) = Fs35;
  Qq(6) = Cq26;
  Qq(7) = Fs17;

% ====== END Task 0 ====== 

  


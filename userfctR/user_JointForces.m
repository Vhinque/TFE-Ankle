function [Qq] = user_JointForces(mbs_data,tsim);
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[Qq] = user_JointForces(mbs_data,tsim);
%
% mbs_data : multibody data structure
% tsim : current time
%
% Qq : joint generalized force/torque (for all joints)
% Qq(i) : joint force/torque in joint (i) along its joint axis
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Qq = mbs_data.Qq;

%/*-- Begin of user code --*/

MBS_user.index_stride_dirdyn  = round(tsim * 500 / MBS_user.T_STRIDE)+1;
load('desired_torque.mat')


 % Ankle spring force
 id = mbs_get_joint_id(MBS_info,'Spring_joint');
 K_maccepa = 130000;
 B_maccepa = 200000;
 
 L0_maccepa = -0.007;
 
 Qq(id) =  ( K_maccepa * (mbs_data.q(id)-L0_maccepa) ) - B_maccepa *(mbs_data.qd(id));
 
 
 
%  id = mbs_get_joint_id(MBS_info,'Maccepa_joint');
% 
% if (MBS_user.step == 3)
%     
%     Qq(id) =  mbs_invdyn.Qq(MBS_user.index_stride_equil,1);
%     
% elseif (MBS_user.step == 4)
%     
%     Qq(id) =  mbs_invdyn.Qq(MBS_user.index_stride_dirdyn,1);
%     
% else
%     
%     Qq(id) =  0;
%     
% end

  
 %Ankle secondary spring force
 id = mbs_get_joint_id(MBS_info,'Parallel_Spring');

 K_second = 103000;
 B_second = 0;%1000;

 L0_second = 0.0;% 0.025;
 
 Qq(id) = - ( K_second*(mbs_data.q(id)-L0_second)) - B_second *(mbs_data.qd(id));
    
% %  
%  % Ankle joint torque
%  id = mbs_get_joint_id(MBS_info,'Ankle_joint');
% 
%  CGA_Ankle;
%  
%  Qq(id) = CGA_Ankle_Data ((floor(tsim*10) + 1), 4);
% % 
    
%/*-- End of user code --*/

return

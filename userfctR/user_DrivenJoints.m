function [q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim : current time
%
% q, qd, qdd : updated column vectors of generalized coordinates
%
%
% mbs_data.q : generalized coordinates [column vector]
% mbs_data.qd : generalized velocities [column vector]
% mbs_data.qdd : generalized accelerations [column vector]
% mbs_data.nqc : number of driven variables
% mbs_data.qc : indices of driven variables [column vector]

global MBS_user MBS_info

q   = mbs_data.q;
qd  = mbs_data.qd;
qdd = mbs_data.qdd;

MBS_user.index_stride_dirdyn  = round(tsim * 500 / MBS_user.T_STRIDE)+1;
load('maccepa_angle.mat');


%/*-- Begin of user code --*/
% get the joint id
id = mbs_get_joint_id(MBS_info,'Slider_joint');
angle = 0.9273;

% impose the position, velocity and acceleration
q(id) = angle;
qd(id) = 0;
qdd(id) = 0;





%   index_stride = round(tsim * 500 / 5.0)+1;

%
id = mbs_get_joint_id(MBS_info,'Ankle_joint');

if (MBS_user.step == 3)
    
    q(id) = MBS_user.ankle.theta(MBS_user.index_stride_equil) * 2 * pi / 360;
    qd(id) = 0;
    qdd(id) = 0;
    
elseif (MBS_user.step == 4)
    
    q(id) = -MBS_user.ankle.theta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qd(id) = -MBS_user.ankle.dtheta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qdd(id) = 0;
    
        
elseif (MBS_user.step == 5)
    
    q(id) = -MBS_user.ankle.theta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qd(id) = -MBS_user.ankle.dtheta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qdd(id) = 0;
    
    
else
    
    q(id) = 0;
    qd(id) = 0;
    qdd(id) = 0;
    
end


id = mbs_get_joint_id(MBS_info,'Maccepa_joint');

q(id) = -( MBS_user.ankle.theta(MBS_user.index_stride_dirdyn) * 2 * pi / 360)-0.6435-(maccepa_angle(MBS_user.index_stride_dirdyn)* 2 * pi / 360);
qd(id) = 0;
qdd(id) = 0;


% /*-- End of user code --*/

return

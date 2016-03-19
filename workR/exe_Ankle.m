%--------------------------------------------------------------------------
%   Université catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be  
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   MBsysLab main script template:
%      - featuring default options
%      - to be adapted by the user
%
%   Project : Pendulum Spring
%   Author :  Nicolas Docquier
%   Date :    07/10/2011
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"
MBS_user.step = 1;

% Project loading
prjname = 'Ankle';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure
                                                                            % Have a look at the content of the mbs_data structure on www.robotran.be
KTADataRead_new;                                                            % Load all Winter data
MBS_user.T_STRIDE = 1.0;

                                                                            
%% 2. Coordinate partitioning [mbs_exe_part]                                % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';
MBS_user.step = 2;

%mbs_data = mbs_set_qu(mbs_data,[1:mbs_data.Njoint]);
% all variables are set to "independent"

%mbs_data = mbs_set_qv(mbs_data, [1]);
% variables 3 and 4 are set to "dependent"




opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
% Options for the coordinate partitioning.

[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);
% Run the coordinate partitioning process


% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);

                                                                            


%% 3. Equilibrium [mbs_exe_equil]
%--------------------------------------------------------------------------
% MBS_user.process = 'equil';
% MBS_user.step = 3;
% 
% 
% opt.equil = {'solvemethod','fsolvepk',...
%     'relax',1.0,'itermax',30,'verbose','no'};
% % other options : 'smooth', 'xeqchoice', 'visualize', 'clearmbsglobal'      % Help about options on www.robotran.be
%    %            'senstol', 'equitol', 'static'
% 
% MBS_user.index_stride_equil = 1;
% 
% 
% for i = 1:501
%     
%     [mbs_equil,mbs_data] = mbs_exe_equil(mbs_data,opt.equil);                   % Equilibrium process
%     
%     MBS_user.equil.alpha_maccepa(i) = mbs_data.q(1);
%     MBS_user.equil.angle_AB(i) = mbs_data.q(2);
%     MBS_user.equil.angle_pied(i) = mbs_data.q(3);
%     MBS_user.equil.x_slider(i) = mbs_data.q(5);
%     
%     MBS_user.equil.T_maccepa(i) = mbs_data.Qq(1);
%     MBS_user.equil.T_AB(i) = mbs_data.Qq(2);
%     MBS_user.equil.T_pied(i) = mbs_data.Qq(3);
%     MBS_user.equil.F_ressort(i) = mbs_data.Qq(5);
%     
%     MBS_user.index_stride_equil =  MBS_user.index_stride_equil + 1;
%     
%     
% end
% 
% % Equilibrium results
% disp('Equilibrium results');
% disp(' ');
% 
% disp(['Angle Maccepa = ', num2str(mbs_data.q(1))]);                                  % Units : translation [m]
% disp(['Angle AB = ', num2str(mbs_data.q(2))]);                                  %         rotation    [rad]
% disp(['Angle Pied = ', num2str(mbs_data.q(3))]);
% disp(['Position slider = ', num2str(mbs_data.q(5))]);
% disp(' ');
% 
% disp(['Couple Maccepa = ', num2str(mbs_data.Qq(1))]);                                  % Units : translation [m]
% disp(['Couple AB = ', num2str(mbs_data.Qq(2))]);                                  %         rotation    [rad]
% disp(['Couple Pied = ', num2str(mbs_data.Qq(3))]);
% disp(['Force ressort = ', num2str(mbs_data.Qq(5))]);
% disp(' ');

                                                                            
%% 4. direct dynamics [mbs_exe_dirdyn]
% %--------------------------------------------------------------------------
% MBS_user.process = 'dirdyn';
% MBS_user.step = 4;
% 
% % change inital conditions
% %mbs_data.q(1) = pi/3;
% %mbs_data.q(2) = 0.1;
% 
% % set options
% opt.dirdyn = {'time',0:0.002:1,'motion','simulation',...
%     'odemethod','ode45','save2file','yes','framerate',1000,...
%     'renamefile','no','verbose','yes'};
% % other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
% %                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be
% 
% % launch simulation
% [mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)
% 


% %% 6. Inverse dynamics [mbs_exe_invdyn]
% % % %--------------------------------------------------------------------------
MBS_user.process = 'invdyn';
MBS_user.step = 5;


% Joint_id = mbs_get_joint_id(mbs_info,{'joint1' 'joint2' 'joint3'});
% mbs_data = mbs_set_qa(mbs_data,Joint_id);                                   % Set variables [Joint_id] as actuated 

opt.invdyn = {'motion','trajectory','time',0:(MBS_user.T_STRIDE/50000):MBS_user.T_STRIDE,'framerate',1000,...
    'save2file','yes','renamefile','no','verbose','yes'};
%other options : 'time', 'visualize', 'clearmbsglobal'                     % Help about options on www.robotran.be

[mbs_invdyn,mbs_data] = mbs_exe_invdyn(mbs_data,opt.invdyn);                % Inverse dynamics process

% 
%% 6. Graphical Results
%--------------------------------------------------------------------------


figure(1)
plot(linspace(0,100,501),MBS_user.ankle.T);
hold on
%plot(linspace(0,100,501),-MBS_user.equil.T_pied,'r');
plot(mbs_invdyn.tsim*100 / MBS_user.T_STRIDE,mbs_invdyn.Qq(:,1),'g');
% plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,1),'g');
% plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,2),'g');
% plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,3),'g');
% plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,4),'g');

% plot(mbs_invdyn.tsim*100 / MBS_user.T_STRIDE,-mbs_invdyn.Qq(:,3),'r');

xlabel('Stride [%]');
ylabel('Couple cheville [Nm]');
grid on

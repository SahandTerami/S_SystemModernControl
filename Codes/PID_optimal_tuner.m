function [C_PID, T_PID, ST_opt, OS_opt] = PID_optimal_tuner(G)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% PID Controller Gain Tunining %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Sahand Tangerami %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs:
%   G - Plant transfer function (SISO)
%
% Outputs:
%   C_PID  - Tuned PID controller (pid object)
%   T_PID  - Closed-loop transfer function (feedback(C_PID*G,1))
%   ST_opt - Settling time of the tuned system
%   OS_opt - Overshoot of the tuned system

    try
        C_init = pid(1,1,0.1);
        [C_PID, ~] = pidtune(G, 'PID');
    catch ME
        error('PID Tuning failed: %s', ME.message);
    end

    T_PID = feedback(C_PID*G, 1);

    if isstable(T_PID)
        info_CL = stepinfo(T_PID);
        ST_opt = info_CL.SettlingTime;
        OS_opt = info_CL.Overshoot;
    else
        error('Tuned PID controller is unstable!');
    end
end

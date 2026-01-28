function [Kp_vec, y_final_P, ss_error_perc_P, STime_P, Oshoot_P] = P_controller_sweep(G, N, Kp_scale)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% P Controller Design %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Sahand Tangerami %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs:
%   G         : Plant transfer function
%   N         : Number of gains to test
%   Kp_scale  : Scaling factor for Kp (e.g., 4 → Kp = 4*i)
%
% Outputs:
%   Kp_vec         : Vector of P gains tested
%   y_final_P      : Final value of step response (dcgain)
%   ss_error_perc_P : Steady-state error in %
%   STime_P        : Settling time for each gain
%   Oshoot_P       : Overshoot for each gain

    Kp_vec = zeros(1,N);
    y_final_P = zeros(1,N);
    ss_error_perc_P = zeros(1,N);
    STime_P = zeros(1,N);
    Oshoot_P = zeros(1,N);

    for i = 1:N
        Kp_vec(i) = Kp_scale * i;
        C_P = Kp_vec(i); 
        T_P = feedback(C_P*G, 1);

        if isstable(T_P)
            info_P = stepinfo(T_P);
            y_final_P(i) = dcgain(T_P);
            ss_error_perc_P(i) = 100*(1 - y_final_P(i));
            STime_P(i) = info_P.SettlingTime;
            Oshoot_P(i) = info_P.Overshoot;
        else
            y_final_P(i) = NaN;
            ss_error_perc_P(i) = NaN;
            STime_P(i) = NaN;
            Oshoot_P(i) = NaN;
        end
    end
end

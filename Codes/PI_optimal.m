function [Kp_opt, Ki_opt, T_PI_opt, ST_opt, OS_opt] = PI_optimal(G, Kp_PI, Ki_PI, STime_PI, Oshoot_PI)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Choose Optimal PI Gains %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Sahand Tangerami %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs:
%   G         : Plant transfer function
%   Kp_PI     : Matrix of Kp gains
%   Ki_PI     : Matrix of Ki gains
%   STime_PI  : Matrix of settling times
%   Oshoot_PI : Matrix of overshoot values
%
% Outputs:
%   Kp_opt    : Optimal proportional gain (NaN if none found)
%   Ki_opt    : Optimal integral gain (NaN if none found)
%   T_PI_opt  : Closed-loop transfer function for optimal PI (empty if none)
%   ST_opt    : Settling time of optimal controller (NaN if none)
%   OS_opt    : Overshoot of optimal controller (NaN if none)

    s = tf('s'); 

    [i_idx, j_idx] = find(STime_PI < 5);
    [i_O, j_O] = find(Oshoot_PI < 10);

    ST_pairs = [i_idx, j_idx];      % settling time < 5
    O_pairs  = [i_O, j_O];          % overshoot < 10

    % Find common pairs
    [common_pairs, ~, ~] = intersect(ST_pairs, O_pairs, 'rows');

    if isempty(common_pairs)
        error('No feasible PI controller found: Settling Time < 5 s and Overshoot < 10%% cannot be satisfied.');
    end

    i_common = common_pairs(:,1);
    j_common = common_pairs(:,2);

    % Compute cost = SettlingTime + Overshoot
    cost_common = STime_PI(sub2ind(size(STime_PI), i_common, j_common)) + ...
                  Oshoot_PI(sub2ind(size(Oshoot_PI), i_common, j_common));

    [~, min_idx] = min(cost_common);

    i_opt = i_common(min_idx);
    j_opt = j_common(min_idx);

    Kp_opt = Kp_PI(i_opt, j_opt);
    Ki_opt = Ki_PI(i_opt, j_opt);

    ST_opt = STime_PI(i_opt, j_opt);
    OS_opt = Oshoot_PI(i_opt, j_opt);

    C_PI_opt = Kp_opt + Ki_opt/s;

    T_PI_opt = feedback(C_PI_opt*G, 1);
end

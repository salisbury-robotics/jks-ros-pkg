% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(xdd - J_goal*qdd_c, eye(6)))
%   subject to
%     normal_0'*Jac_0*qdd_c >= 0
%     normal_1'*Jac_1*qdd_c >= 0
%     normal_2'*Jac_2*qdd_c >= 0
%     normal_3'*Jac_3*qdd_c >= 0
%     normal_4'*Jac_4*qdd_c >= 0
%     normal_5'*Jac_5*qdd_c >= 0
%     normal_6'*Jac_6*qdd_c >= 0
%     normal_7'*Jac_7*qdd_c >= 0
%     normal_8'*Jac_8*qdd_c >= 0
%     normal_9'*Jac_9*qdd_c >= 0
%     normal_10'*Jac_10*qdd_c >= 0
%     normal_11'*Jac_11*qdd_c >= 0
%     normal_12'*Jac_12*qdd_c >= 0
%     normal_13'*Jac_13*qdd_c >= 0
%     normal_14'*Jac_14*qdd_c >= 0
%     normal_15'*Jac_15*qdd_c >= 0
%     normal_16'*Jac_16*qdd_c >= 0
%     normal_17'*Jac_17*qdd_c >= 0
%     normal_18'*Jac_18*qdd_c >= 0
%     normal_19'*Jac_19*qdd_c >= 0
%     normal_20'*Jac_20*qdd_c >= 0
%     normal_21'*Jac_21*qdd_c >= 0
%     normal_22'*Jac_22*qdd_c >= 0
%     normal_23'*Jac_23*qdd_c >= 0
%     normal_24'*Jac_24*qdd_c >= 0
%     normal_25'*Jac_25*qdd_c >= 0
%     normal_26'*Jac_26*qdd_c >= 0
%     normal_27'*Jac_27*qdd_c >= 0
%     normal_28'*Jac_28*qdd_c >= 0
%     normal_29'*Jac_29*qdd_c >= 0
%     normal_30'*Jac_30*qdd_c >= 0
%     normal_31'*Jac_31*qdd_c >= 0
%     normal_32'*Jac_32*qdd_c >= 0
%     normal_33'*Jac_33*qdd_c >= 0
%     normal_34'*Jac_34*qdd_c >= 0
%     normal_35'*Jac_35*qdd_c >= 0
%     normal_36'*Jac_36*qdd_c >= 0
%     normal_37'*Jac_37*qdd_c >= 0
%     normal_38'*Jac_38*qdd_c >= 0
%     normal_39'*Jac_39*qdd_c >= 0
%     normal_40'*Jac_40*qdd_c >= 0
%     normal_41'*Jac_41*qdd_c >= 0
%     normal_42'*Jac_42*qdd_c >= 0
%     normal_43'*Jac_43*qdd_c >= 0
%     normal_44'*Jac_44*qdd_c >= 0
%     normal_45'*Jac_45*qdd_c >= 0
%     normal_46'*Jac_46*qdd_c >= 0
%     normal_47'*Jac_47*qdd_c >= 0
%     normal_48'*Jac_48*qdd_c >= 0
%     normal_49'*Jac_49*qdd_c >= 0
%     normal_50'*Jac_50*qdd_c >= 0
%     q_min <= q + qdd_c
%     q + qdd_c <= q_max
%
% with variables
%    qdd_c   7 x 1
%
% and parameters
%   J_goal   6 x 7
%    Jac_0   3 x 7
%    Jac_1   3 x 7
%    Jac_2   3 x 7
%    Jac_3   3 x 7
%    Jac_4   3 x 7
%    Jac_5   3 x 7
%    Jac_6   3 x 7
%    Jac_7   3 x 7
%    Jac_8   3 x 7
%    Jac_9   3 x 7
%   Jac_10   3 x 7
%   Jac_11   3 x 7
%   Jac_12   3 x 7
%   Jac_13   3 x 7
%   Jac_14   3 x 7
%   Jac_15   3 x 7
%   Jac_16   3 x 7
%   Jac_17   3 x 7
%   Jac_18   3 x 7
%   Jac_19   3 x 7
%   Jac_20   3 x 7
%   Jac_21   3 x 7
%   Jac_22   3 x 7
%   Jac_23   3 x 7
%   Jac_24   3 x 7
%   Jac_25   3 x 7
%   Jac_26   3 x 7
%   Jac_27   3 x 7
%   Jac_28   3 x 7
%   Jac_29   3 x 7
%   Jac_30   3 x 7
%   Jac_31   3 x 7
%   Jac_32   3 x 7
%   Jac_33   3 x 7
%   Jac_34   3 x 7
%   Jac_35   3 x 7
%   Jac_36   3 x 7
%   Jac_37   3 x 7
%   Jac_38   3 x 7
%   Jac_39   3 x 7
%   Jac_40   3 x 7
%   Jac_41   3 x 7
%   Jac_42   3 x 7
%   Jac_43   3 x 7
%   Jac_44   3 x 7
%   Jac_45   3 x 7
%   Jac_46   3 x 7
%   Jac_47   3 x 7
%   Jac_48   3 x 7
%   Jac_49   3 x 7
%   Jac_50   3 x 7
% normal_0   3 x 1
% normal_1   3 x 1
% normal_2   3 x 1
% normal_3   3 x 1
% normal_4   3 x 1
% normal_5   3 x 1
% normal_6   3 x 1
% normal_7   3 x 1
% normal_8   3 x 1
% normal_9   3 x 1
% normal_10   3 x 1
% normal_11   3 x 1
% normal_12   3 x 1
% normal_13   3 x 1
% normal_14   3 x 1
% normal_15   3 x 1
% normal_16   3 x 1
% normal_17   3 x 1
% normal_18   3 x 1
% normal_19   3 x 1
% normal_20   3 x 1
% normal_21   3 x 1
% normal_22   3 x 1
% normal_23   3 x 1
% normal_24   3 x 1
% normal_25   3 x 1
% normal_26   3 x 1
% normal_27   3 x 1
% normal_28   3 x 1
% normal_29   3 x 1
% normal_30   3 x 1
% normal_31   3 x 1
% normal_32   3 x 1
% normal_33   3 x 1
% normal_34   3 x 1
% normal_35   3 x 1
% normal_36   3 x 1
% normal_37   3 x 1
% normal_38   3 x 1
% normal_39   3 x 1
% normal_40   3 x 1
% normal_41   3 x 1
% normal_42   3 x 1
% normal_43   3 x 1
% normal_44   3 x 1
% normal_45   3 x 1
% normal_46   3 x 1
% normal_47   3 x 1
% normal_48   3 x 1
% normal_49   3 x 1
% normal_50   3 x 1
%        q   7 x 1
%    q_max   7 x 1
%    q_min   7 x 1
%      xdd   6 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.J_goal, ..., params.xdd, then run
%   [vars, status] = csolve(params, settings)


% Produced by CVXGEN, 2012-08-24 19:52:43 -0700.
% CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2011 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.

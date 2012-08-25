% Produced by CVXGEN, 2012-08-24 19:52:43 -0700.
% CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2011 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.

function [vars, status] = cvxsolve(params, settings)

J_goal = params.J_goal;
Jac_0 = params.Jac_0;
if isfield(params, 'Jac_1')
  Jac_1 = params.Jac_1;
elseif isfield(params, 'Jac')
  Jac_1 = params.Jac{1};
else
  error 'could not find Jac_1'
end
if isfield(params, 'Jac_2')
  Jac_2 = params.Jac_2;
elseif isfield(params, 'Jac')
  Jac_2 = params.Jac{2};
else
  error 'could not find Jac_2'
end
if isfield(params, 'Jac_3')
  Jac_3 = params.Jac_3;
elseif isfield(params, 'Jac')
  Jac_3 = params.Jac{3};
else
  error 'could not find Jac_3'
end
if isfield(params, 'Jac_4')
  Jac_4 = params.Jac_4;
elseif isfield(params, 'Jac')
  Jac_4 = params.Jac{4};
else
  error 'could not find Jac_4'
end
if isfield(params, 'Jac_5')
  Jac_5 = params.Jac_5;
elseif isfield(params, 'Jac')
  Jac_5 = params.Jac{5};
else
  error 'could not find Jac_5'
end
if isfield(params, 'Jac_6')
  Jac_6 = params.Jac_6;
elseif isfield(params, 'Jac')
  Jac_6 = params.Jac{6};
else
  error 'could not find Jac_6'
end
if isfield(params, 'Jac_7')
  Jac_7 = params.Jac_7;
elseif isfield(params, 'Jac')
  Jac_7 = params.Jac{7};
else
  error 'could not find Jac_7'
end
if isfield(params, 'Jac_8')
  Jac_8 = params.Jac_8;
elseif isfield(params, 'Jac')
  Jac_8 = params.Jac{8};
else
  error 'could not find Jac_8'
end
if isfield(params, 'Jac_9')
  Jac_9 = params.Jac_9;
elseif isfield(params, 'Jac')
  Jac_9 = params.Jac{9};
else
  error 'could not find Jac_9'
end
if isfield(params, 'Jac_10')
  Jac_10 = params.Jac_10;
elseif isfield(params, 'Jac')
  Jac_10 = params.Jac{10};
else
  error 'could not find Jac_10'
end
if isfield(params, 'Jac_11')
  Jac_11 = params.Jac_11;
elseif isfield(params, 'Jac')
  Jac_11 = params.Jac{11};
else
  error 'could not find Jac_11'
end
if isfield(params, 'Jac_12')
  Jac_12 = params.Jac_12;
elseif isfield(params, 'Jac')
  Jac_12 = params.Jac{12};
else
  error 'could not find Jac_12'
end
if isfield(params, 'Jac_13')
  Jac_13 = params.Jac_13;
elseif isfield(params, 'Jac')
  Jac_13 = params.Jac{13};
else
  error 'could not find Jac_13'
end
if isfield(params, 'Jac_14')
  Jac_14 = params.Jac_14;
elseif isfield(params, 'Jac')
  Jac_14 = params.Jac{14};
else
  error 'could not find Jac_14'
end
if isfield(params, 'Jac_15')
  Jac_15 = params.Jac_15;
elseif isfield(params, 'Jac')
  Jac_15 = params.Jac{15};
else
  error 'could not find Jac_15'
end
if isfield(params, 'Jac_16')
  Jac_16 = params.Jac_16;
elseif isfield(params, 'Jac')
  Jac_16 = params.Jac{16};
else
  error 'could not find Jac_16'
end
if isfield(params, 'Jac_17')
  Jac_17 = params.Jac_17;
elseif isfield(params, 'Jac')
  Jac_17 = params.Jac{17};
else
  error 'could not find Jac_17'
end
if isfield(params, 'Jac_18')
  Jac_18 = params.Jac_18;
elseif isfield(params, 'Jac')
  Jac_18 = params.Jac{18};
else
  error 'could not find Jac_18'
end
if isfield(params, 'Jac_19')
  Jac_19 = params.Jac_19;
elseif isfield(params, 'Jac')
  Jac_19 = params.Jac{19};
else
  error 'could not find Jac_19'
end
if isfield(params, 'Jac_20')
  Jac_20 = params.Jac_20;
elseif isfield(params, 'Jac')
  Jac_20 = params.Jac{20};
else
  error 'could not find Jac_20'
end
if isfield(params, 'Jac_21')
  Jac_21 = params.Jac_21;
elseif isfield(params, 'Jac')
  Jac_21 = params.Jac{21};
else
  error 'could not find Jac_21'
end
if isfield(params, 'Jac_22')
  Jac_22 = params.Jac_22;
elseif isfield(params, 'Jac')
  Jac_22 = params.Jac{22};
else
  error 'could not find Jac_22'
end
if isfield(params, 'Jac_23')
  Jac_23 = params.Jac_23;
elseif isfield(params, 'Jac')
  Jac_23 = params.Jac{23};
else
  error 'could not find Jac_23'
end
if isfield(params, 'Jac_24')
  Jac_24 = params.Jac_24;
elseif isfield(params, 'Jac')
  Jac_24 = params.Jac{24};
else
  error 'could not find Jac_24'
end
if isfield(params, 'Jac_25')
  Jac_25 = params.Jac_25;
elseif isfield(params, 'Jac')
  Jac_25 = params.Jac{25};
else
  error 'could not find Jac_25'
end
if isfield(params, 'Jac_26')
  Jac_26 = params.Jac_26;
elseif isfield(params, 'Jac')
  Jac_26 = params.Jac{26};
else
  error 'could not find Jac_26'
end
if isfield(params, 'Jac_27')
  Jac_27 = params.Jac_27;
elseif isfield(params, 'Jac')
  Jac_27 = params.Jac{27};
else
  error 'could not find Jac_27'
end
if isfield(params, 'Jac_28')
  Jac_28 = params.Jac_28;
elseif isfield(params, 'Jac')
  Jac_28 = params.Jac{28};
else
  error 'could not find Jac_28'
end
if isfield(params, 'Jac_29')
  Jac_29 = params.Jac_29;
elseif isfield(params, 'Jac')
  Jac_29 = params.Jac{29};
else
  error 'could not find Jac_29'
end
if isfield(params, 'Jac_30')
  Jac_30 = params.Jac_30;
elseif isfield(params, 'Jac')
  Jac_30 = params.Jac{30};
else
  error 'could not find Jac_30'
end
if isfield(params, 'Jac_31')
  Jac_31 = params.Jac_31;
elseif isfield(params, 'Jac')
  Jac_31 = params.Jac{31};
else
  error 'could not find Jac_31'
end
if isfield(params, 'Jac_32')
  Jac_32 = params.Jac_32;
elseif isfield(params, 'Jac')
  Jac_32 = params.Jac{32};
else
  error 'could not find Jac_32'
end
if isfield(params, 'Jac_33')
  Jac_33 = params.Jac_33;
elseif isfield(params, 'Jac')
  Jac_33 = params.Jac{33};
else
  error 'could not find Jac_33'
end
if isfield(params, 'Jac_34')
  Jac_34 = params.Jac_34;
elseif isfield(params, 'Jac')
  Jac_34 = params.Jac{34};
else
  error 'could not find Jac_34'
end
if isfield(params, 'Jac_35')
  Jac_35 = params.Jac_35;
elseif isfield(params, 'Jac')
  Jac_35 = params.Jac{35};
else
  error 'could not find Jac_35'
end
if isfield(params, 'Jac_36')
  Jac_36 = params.Jac_36;
elseif isfield(params, 'Jac')
  Jac_36 = params.Jac{36};
else
  error 'could not find Jac_36'
end
if isfield(params, 'Jac_37')
  Jac_37 = params.Jac_37;
elseif isfield(params, 'Jac')
  Jac_37 = params.Jac{37};
else
  error 'could not find Jac_37'
end
if isfield(params, 'Jac_38')
  Jac_38 = params.Jac_38;
elseif isfield(params, 'Jac')
  Jac_38 = params.Jac{38};
else
  error 'could not find Jac_38'
end
if isfield(params, 'Jac_39')
  Jac_39 = params.Jac_39;
elseif isfield(params, 'Jac')
  Jac_39 = params.Jac{39};
else
  error 'could not find Jac_39'
end
if isfield(params, 'Jac_40')
  Jac_40 = params.Jac_40;
elseif isfield(params, 'Jac')
  Jac_40 = params.Jac{40};
else
  error 'could not find Jac_40'
end
if isfield(params, 'Jac_41')
  Jac_41 = params.Jac_41;
elseif isfield(params, 'Jac')
  Jac_41 = params.Jac{41};
else
  error 'could not find Jac_41'
end
if isfield(params, 'Jac_42')
  Jac_42 = params.Jac_42;
elseif isfield(params, 'Jac')
  Jac_42 = params.Jac{42};
else
  error 'could not find Jac_42'
end
if isfield(params, 'Jac_43')
  Jac_43 = params.Jac_43;
elseif isfield(params, 'Jac')
  Jac_43 = params.Jac{43};
else
  error 'could not find Jac_43'
end
if isfield(params, 'Jac_44')
  Jac_44 = params.Jac_44;
elseif isfield(params, 'Jac')
  Jac_44 = params.Jac{44};
else
  error 'could not find Jac_44'
end
if isfield(params, 'Jac_45')
  Jac_45 = params.Jac_45;
elseif isfield(params, 'Jac')
  Jac_45 = params.Jac{45};
else
  error 'could not find Jac_45'
end
if isfield(params, 'Jac_46')
  Jac_46 = params.Jac_46;
elseif isfield(params, 'Jac')
  Jac_46 = params.Jac{46};
else
  error 'could not find Jac_46'
end
if isfield(params, 'Jac_47')
  Jac_47 = params.Jac_47;
elseif isfield(params, 'Jac')
  Jac_47 = params.Jac{47};
else
  error 'could not find Jac_47'
end
if isfield(params, 'Jac_48')
  Jac_48 = params.Jac_48;
elseif isfield(params, 'Jac')
  Jac_48 = params.Jac{48};
else
  error 'could not find Jac_48'
end
if isfield(params, 'Jac_49')
  Jac_49 = params.Jac_49;
elseif isfield(params, 'Jac')
  Jac_49 = params.Jac{49};
else
  error 'could not find Jac_49'
end
if isfield(params, 'Jac_50')
  Jac_50 = params.Jac_50;
elseif isfield(params, 'Jac')
  Jac_50 = params.Jac{50};
else
  error 'could not find Jac_50'
end
normal_0 = params.normal_0;
if isfield(params, 'normal_1')
  normal_1 = params.normal_1;
elseif isfield(params, 'normal')
  normal_1 = params.normal{1};
else
  error 'could not find normal_1'
end
if isfield(params, 'normal_2')
  normal_2 = params.normal_2;
elseif isfield(params, 'normal')
  normal_2 = params.normal{2};
else
  error 'could not find normal_2'
end
if isfield(params, 'normal_3')
  normal_3 = params.normal_3;
elseif isfield(params, 'normal')
  normal_3 = params.normal{3};
else
  error 'could not find normal_3'
end
if isfield(params, 'normal_4')
  normal_4 = params.normal_4;
elseif isfield(params, 'normal')
  normal_4 = params.normal{4};
else
  error 'could not find normal_4'
end
if isfield(params, 'normal_5')
  normal_5 = params.normal_5;
elseif isfield(params, 'normal')
  normal_5 = params.normal{5};
else
  error 'could not find normal_5'
end
if isfield(params, 'normal_6')
  normal_6 = params.normal_6;
elseif isfield(params, 'normal')
  normal_6 = params.normal{6};
else
  error 'could not find normal_6'
end
if isfield(params, 'normal_7')
  normal_7 = params.normal_7;
elseif isfield(params, 'normal')
  normal_7 = params.normal{7};
else
  error 'could not find normal_7'
end
if isfield(params, 'normal_8')
  normal_8 = params.normal_8;
elseif isfield(params, 'normal')
  normal_8 = params.normal{8};
else
  error 'could not find normal_8'
end
if isfield(params, 'normal_9')
  normal_9 = params.normal_9;
elseif isfield(params, 'normal')
  normal_9 = params.normal{9};
else
  error 'could not find normal_9'
end
if isfield(params, 'normal_10')
  normal_10 = params.normal_10;
elseif isfield(params, 'normal')
  normal_10 = params.normal{10};
else
  error 'could not find normal_10'
end
if isfield(params, 'normal_11')
  normal_11 = params.normal_11;
elseif isfield(params, 'normal')
  normal_11 = params.normal{11};
else
  error 'could not find normal_11'
end
if isfield(params, 'normal_12')
  normal_12 = params.normal_12;
elseif isfield(params, 'normal')
  normal_12 = params.normal{12};
else
  error 'could not find normal_12'
end
if isfield(params, 'normal_13')
  normal_13 = params.normal_13;
elseif isfield(params, 'normal')
  normal_13 = params.normal{13};
else
  error 'could not find normal_13'
end
if isfield(params, 'normal_14')
  normal_14 = params.normal_14;
elseif isfield(params, 'normal')
  normal_14 = params.normal{14};
else
  error 'could not find normal_14'
end
if isfield(params, 'normal_15')
  normal_15 = params.normal_15;
elseif isfield(params, 'normal')
  normal_15 = params.normal{15};
else
  error 'could not find normal_15'
end
if isfield(params, 'normal_16')
  normal_16 = params.normal_16;
elseif isfield(params, 'normal')
  normal_16 = params.normal{16};
else
  error 'could not find normal_16'
end
if isfield(params, 'normal_17')
  normal_17 = params.normal_17;
elseif isfield(params, 'normal')
  normal_17 = params.normal{17};
else
  error 'could not find normal_17'
end
if isfield(params, 'normal_18')
  normal_18 = params.normal_18;
elseif isfield(params, 'normal')
  normal_18 = params.normal{18};
else
  error 'could not find normal_18'
end
if isfield(params, 'normal_19')
  normal_19 = params.normal_19;
elseif isfield(params, 'normal')
  normal_19 = params.normal{19};
else
  error 'could not find normal_19'
end
if isfield(params, 'normal_20')
  normal_20 = params.normal_20;
elseif isfield(params, 'normal')
  normal_20 = params.normal{20};
else
  error 'could not find normal_20'
end
if isfield(params, 'normal_21')
  normal_21 = params.normal_21;
elseif isfield(params, 'normal')
  normal_21 = params.normal{21};
else
  error 'could not find normal_21'
end
if isfield(params, 'normal_22')
  normal_22 = params.normal_22;
elseif isfield(params, 'normal')
  normal_22 = params.normal{22};
else
  error 'could not find normal_22'
end
if isfield(params, 'normal_23')
  normal_23 = params.normal_23;
elseif isfield(params, 'normal')
  normal_23 = params.normal{23};
else
  error 'could not find normal_23'
end
if isfield(params, 'normal_24')
  normal_24 = params.normal_24;
elseif isfield(params, 'normal')
  normal_24 = params.normal{24};
else
  error 'could not find normal_24'
end
if isfield(params, 'normal_25')
  normal_25 = params.normal_25;
elseif isfield(params, 'normal')
  normal_25 = params.normal{25};
else
  error 'could not find normal_25'
end
if isfield(params, 'normal_26')
  normal_26 = params.normal_26;
elseif isfield(params, 'normal')
  normal_26 = params.normal{26};
else
  error 'could not find normal_26'
end
if isfield(params, 'normal_27')
  normal_27 = params.normal_27;
elseif isfield(params, 'normal')
  normal_27 = params.normal{27};
else
  error 'could not find normal_27'
end
if isfield(params, 'normal_28')
  normal_28 = params.normal_28;
elseif isfield(params, 'normal')
  normal_28 = params.normal{28};
else
  error 'could not find normal_28'
end
if isfield(params, 'normal_29')
  normal_29 = params.normal_29;
elseif isfield(params, 'normal')
  normal_29 = params.normal{29};
else
  error 'could not find normal_29'
end
if isfield(params, 'normal_30')
  normal_30 = params.normal_30;
elseif isfield(params, 'normal')
  normal_30 = params.normal{30};
else
  error 'could not find normal_30'
end
if isfield(params, 'normal_31')
  normal_31 = params.normal_31;
elseif isfield(params, 'normal')
  normal_31 = params.normal{31};
else
  error 'could not find normal_31'
end
if isfield(params, 'normal_32')
  normal_32 = params.normal_32;
elseif isfield(params, 'normal')
  normal_32 = params.normal{32};
else
  error 'could not find normal_32'
end
if isfield(params, 'normal_33')
  normal_33 = params.normal_33;
elseif isfield(params, 'normal')
  normal_33 = params.normal{33};
else
  error 'could not find normal_33'
end
if isfield(params, 'normal_34')
  normal_34 = params.normal_34;
elseif isfield(params, 'normal')
  normal_34 = params.normal{34};
else
  error 'could not find normal_34'
end
if isfield(params, 'normal_35')
  normal_35 = params.normal_35;
elseif isfield(params, 'normal')
  normal_35 = params.normal{35};
else
  error 'could not find normal_35'
end
if isfield(params, 'normal_36')
  normal_36 = params.normal_36;
elseif isfield(params, 'normal')
  normal_36 = params.normal{36};
else
  error 'could not find normal_36'
end
if isfield(params, 'normal_37')
  normal_37 = params.normal_37;
elseif isfield(params, 'normal')
  normal_37 = params.normal{37};
else
  error 'could not find normal_37'
end
if isfield(params, 'normal_38')
  normal_38 = params.normal_38;
elseif isfield(params, 'normal')
  normal_38 = params.normal{38};
else
  error 'could not find normal_38'
end
if isfield(params, 'normal_39')
  normal_39 = params.normal_39;
elseif isfield(params, 'normal')
  normal_39 = params.normal{39};
else
  error 'could not find normal_39'
end
if isfield(params, 'normal_40')
  normal_40 = params.normal_40;
elseif isfield(params, 'normal')
  normal_40 = params.normal{40};
else
  error 'could not find normal_40'
end
if isfield(params, 'normal_41')
  normal_41 = params.normal_41;
elseif isfield(params, 'normal')
  normal_41 = params.normal{41};
else
  error 'could not find normal_41'
end
if isfield(params, 'normal_42')
  normal_42 = params.normal_42;
elseif isfield(params, 'normal')
  normal_42 = params.normal{42};
else
  error 'could not find normal_42'
end
if isfield(params, 'normal_43')
  normal_43 = params.normal_43;
elseif isfield(params, 'normal')
  normal_43 = params.normal{43};
else
  error 'could not find normal_43'
end
if isfield(params, 'normal_44')
  normal_44 = params.normal_44;
elseif isfield(params, 'normal')
  normal_44 = params.normal{44};
else
  error 'could not find normal_44'
end
if isfield(params, 'normal_45')
  normal_45 = params.normal_45;
elseif isfield(params, 'normal')
  normal_45 = params.normal{45};
else
  error 'could not find normal_45'
end
if isfield(params, 'normal_46')
  normal_46 = params.normal_46;
elseif isfield(params, 'normal')
  normal_46 = params.normal{46};
else
  error 'could not find normal_46'
end
if isfield(params, 'normal_47')
  normal_47 = params.normal_47;
elseif isfield(params, 'normal')
  normal_47 = params.normal{47};
else
  error 'could not find normal_47'
end
if isfield(params, 'normal_48')
  normal_48 = params.normal_48;
elseif isfield(params, 'normal')
  normal_48 = params.normal{48};
else
  error 'could not find normal_48'
end
if isfield(params, 'normal_49')
  normal_49 = params.normal_49;
elseif isfield(params, 'normal')
  normal_49 = params.normal{49};
else
  error 'could not find normal_49'
end
if isfield(params, 'normal_50')
  normal_50 = params.normal_50;
elseif isfield(params, 'normal')
  normal_50 = params.normal{50};
else
  error 'could not find normal_50'
end
q = params.q;
q_max = params.q_max;
q_min = params.q_min;
xdd = params.xdd;

cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable qdd_c(7, 1);

  minimize(quad_form(xdd - J_goal*qdd_c, eye(6)));
  subject to
    normal_0'*Jac_0*qdd_c >= 0;
    normal_1'*Jac_1*qdd_c >= 0;
    normal_2'*Jac_2*qdd_c >= 0;
    normal_3'*Jac_3*qdd_c >= 0;
    normal_4'*Jac_4*qdd_c >= 0;
    normal_5'*Jac_5*qdd_c >= 0;
    normal_6'*Jac_6*qdd_c >= 0;
    normal_7'*Jac_7*qdd_c >= 0;
    normal_8'*Jac_8*qdd_c >= 0;
    normal_9'*Jac_9*qdd_c >= 0;
    normal_10'*Jac_10*qdd_c >= 0;
    normal_11'*Jac_11*qdd_c >= 0;
    normal_12'*Jac_12*qdd_c >= 0;
    normal_13'*Jac_13*qdd_c >= 0;
    normal_14'*Jac_14*qdd_c >= 0;
    normal_15'*Jac_15*qdd_c >= 0;
    normal_16'*Jac_16*qdd_c >= 0;
    normal_17'*Jac_17*qdd_c >= 0;
    normal_18'*Jac_18*qdd_c >= 0;
    normal_19'*Jac_19*qdd_c >= 0;
    normal_20'*Jac_20*qdd_c >= 0;
    normal_21'*Jac_21*qdd_c >= 0;
    normal_22'*Jac_22*qdd_c >= 0;
    normal_23'*Jac_23*qdd_c >= 0;
    normal_24'*Jac_24*qdd_c >= 0;
    normal_25'*Jac_25*qdd_c >= 0;
    normal_26'*Jac_26*qdd_c >= 0;
    normal_27'*Jac_27*qdd_c >= 0;
    normal_28'*Jac_28*qdd_c >= 0;
    normal_29'*Jac_29*qdd_c >= 0;
    normal_30'*Jac_30*qdd_c >= 0;
    normal_31'*Jac_31*qdd_c >= 0;
    normal_32'*Jac_32*qdd_c >= 0;
    normal_33'*Jac_33*qdd_c >= 0;
    normal_34'*Jac_34*qdd_c >= 0;
    normal_35'*Jac_35*qdd_c >= 0;
    normal_36'*Jac_36*qdd_c >= 0;
    normal_37'*Jac_37*qdd_c >= 0;
    normal_38'*Jac_38*qdd_c >= 0;
    normal_39'*Jac_39*qdd_c >= 0;
    normal_40'*Jac_40*qdd_c >= 0;
    normal_41'*Jac_41*qdd_c >= 0;
    normal_42'*Jac_42*qdd_c >= 0;
    normal_43'*Jac_43*qdd_c >= 0;
    normal_44'*Jac_44*qdd_c >= 0;
    normal_45'*Jac_45*qdd_c >= 0;
    normal_46'*Jac_46*qdd_c >= 0;
    normal_47'*Jac_47*qdd_c >= 0;
    normal_48'*Jac_48*qdd_c >= 0;
    normal_49'*Jac_49*qdd_c >= 0;
    normal_50'*Jac_50*qdd_c >= 0;
    q_min <= q + qdd_c;
    q + qdd_c <= q_max;
cvx_end

vars.qdd_c = qdd_c;
status.cvx_status = cvx_status;

% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');

/* Produced by CVXGEN, 2012-08-24 19:52:42 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */

#include "solver.h"

/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */

  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[13];
  work.v[1] = target[14];
  work.v[2] = target[15];
  work.v[3] = target[16];
  work.v[4] = target[17];
  work.v[5] = target[18];
  work.v[6] = target[19];
  work.v[7] = target[20];
  work.v[8] = target[21];
  work.v[9] = target[22];
  work.v[10] = target[23];
  work.v[11] = target[24];
  work.v[12] = target[25];
  work.v[13] = target[26];
  work.v[14] = target[27];
  work.v[15] = target[28];
  work.v[16] = target[29];
  work.v[17] = target[30];
  work.v[18] = target[31];
  work.v[19] = target[32];
  work.v[20] = target[33];
  work.v[21] = target[34];
  work.v[22] = target[35];
  work.v[23] = target[36];
  work.v[24] = target[37];
  work.v[25] = target[38];
  work.v[26] = target[39];
  work.v[27] = target[40];
  work.v[28] = target[41];
  work.v[29] = target[42];
  work.v[30] = target[43];
  work.v[31] = target[44];
  work.v[32] = target[45];
  work.v[33] = target[46];
  work.v[34] = target[47];
  work.v[35] = target[48];
  work.v[36] = target[49];
  work.v[37] = target[50];
  work.v[38] = target[51];
  work.v[39] = target[52];
  work.v[40] = target[53];
  work.v[41] = target[54];
  work.v[42] = target[55];
  work.v[43] = target[56];
  work.v[44] = target[57];
  work.v[45] = target[58];
  work.v[46] = target[59];
  work.v[47] = target[60];
  work.v[48] = target[61];
  work.v[49] = target[62];
  work.v[50] = target[63];
  work.v[51] = target[64];
  work.v[52] = target[65];
  work.v[53] = target[66];
  work.v[54] = target[67];
  work.v[55] = target[68];
  work.v[56] = target[69];
  work.v[57] = target[70];
  work.v[58] = target[71];
  work.v[59] = target[72];
  work.v[60] = target[73];
  work.v[61] = target[74];
  work.v[62] = target[75];
  work.v[63] = target[76];
  work.v[64] = target[77];
  work.v[65] = target[7];
  work.v[66] = target[8];
  work.v[67] = target[9];
  work.v[68] = target[10];
  work.v[69] = target[11];
  work.v[70] = target[12];
  work.v[71] = target[129]-work.L[0]*work.v[51];
  work.v[72] = target[136]-work.L[1]*work.v[58];
  work.v[73] = target[130]-work.L[2]*work.v[52];
  work.v[74] = target[137]-work.L[3]*work.v[59];
  work.v[75] = target[131]-work.L[4]*work.v[53];
  work.v[76] = target[138]-work.L[5]*work.v[60];
  work.v[77] = target[132]-work.L[6]*work.v[54];
  work.v[78] = target[139]-work.L[7]*work.v[61];
  work.v[79] = target[133]-work.L[8]*work.v[55];
  work.v[80] = target[140]-work.L[9]*work.v[62];
  work.v[81] = target[134]-work.L[10]*work.v[56];
  work.v[82] = target[141]-work.L[11]*work.v[63];
  work.v[83] = target[135]-work.L[12]*work.v[57];
  work.v[84] = target[142]-work.L[13]*work.v[64];
  work.v[85] = target[78]-work.L[14]*work.v[0];
  work.v[86] = target[79]-work.L[15]*work.v[1];
  work.v[87] = target[80]-work.L[16]*work.v[2];
  work.v[88] = target[81]-work.L[17]*work.v[3];
  work.v[89] = target[82]-work.L[18]*work.v[4];
  work.v[90] = target[83]-work.L[19]*work.v[5];
  work.v[91] = target[84]-work.L[20]*work.v[6];
  work.v[92] = target[85]-work.L[21]*work.v[7];
  work.v[93] = target[86]-work.L[22]*work.v[8];
  work.v[94] = target[87]-work.L[23]*work.v[9];
  work.v[95] = target[88]-work.L[24]*work.v[10];
  work.v[96] = target[89]-work.L[25]*work.v[11];
  work.v[97] = target[90]-work.L[26]*work.v[12];
  work.v[98] = target[91]-work.L[27]*work.v[13];
  work.v[99] = target[92]-work.L[28]*work.v[14];
  work.v[100] = target[93]-work.L[29]*work.v[15];
  work.v[101] = target[94]-work.L[30]*work.v[16];
  work.v[102] = target[95]-work.L[31]*work.v[17];
  work.v[103] = target[96]-work.L[32]*work.v[18];
  work.v[104] = target[97]-work.L[33]*work.v[19];
  work.v[105] = target[98]-work.L[34]*work.v[20];
  work.v[106] = target[99]-work.L[35]*work.v[21];
  work.v[107] = target[100]-work.L[36]*work.v[22];
  work.v[108] = target[101]-work.L[37]*work.v[23];
  work.v[109] = target[102]-work.L[38]*work.v[24];
  work.v[110] = target[103]-work.L[39]*work.v[25];
  work.v[111] = target[104]-work.L[40]*work.v[26];
  work.v[112] = target[105]-work.L[41]*work.v[27];
  work.v[113] = target[106]-work.L[42]*work.v[28];
  work.v[114] = target[107]-work.L[43]*work.v[29];
  work.v[115] = target[108]-work.L[44]*work.v[30];
  work.v[116] = target[109]-work.L[45]*work.v[31];
  work.v[117] = target[110]-work.L[46]*work.v[32];
  work.v[118] = target[111]-work.L[47]*work.v[33];
  work.v[119] = target[112]-work.L[48]*work.v[34];
  work.v[120] = target[113]-work.L[49]*work.v[35];
  work.v[121] = target[114]-work.L[50]*work.v[36];
  work.v[122] = target[115]-work.L[51]*work.v[37];
  work.v[123] = target[116]-work.L[52]*work.v[38];
  work.v[124] = target[117]-work.L[53]*work.v[39];
  work.v[125] = target[118]-work.L[54]*work.v[40];
  work.v[126] = target[119]-work.L[55]*work.v[41];
  work.v[127] = target[120]-work.L[56]*work.v[42];
  work.v[128] = target[121]-work.L[57]*work.v[43];
  work.v[129] = target[122]-work.L[58]*work.v[44];
  work.v[130] = target[123]-work.L[59]*work.v[45];
  work.v[131] = target[124]-work.L[60]*work.v[46];
  work.v[132] = target[125]-work.L[61]*work.v[47];
  work.v[133] = target[126]-work.L[62]*work.v[48];
  work.v[134] = target[127]-work.L[63]*work.v[49];
  work.v[135] = target[128]-work.L[64]*work.v[50];
  work.v[136] = target[143]-work.L[65]*work.v[65];
  work.v[137] = target[144]-work.L[66]*work.v[66];
  work.v[138] = target[145]-work.L[67]*work.v[67];
  work.v[139] = target[146]-work.L[68]*work.v[68];
  work.v[140] = target[147]-work.L[69]*work.v[69];
  work.v[141] = target[0]-work.L[70]*work.v[71]-work.L[71]*work.v[72]-work.L[72]*work.v[85]-work.L[73]*work.v[86]-work.L[74]*work.v[87]-work.L[75]*work.v[88]-work.L[76]*work.v[89]-work.L[77]*work.v[90]-work.L[78]*work.v[91]-work.L[79]*work.v[92]-work.L[80]*work.v[93]-work.L[81]*work.v[94]-work.L[82]*work.v[95]-work.L[83]*work.v[96]-work.L[84]*work.v[97]-work.L[85]*work.v[98]-work.L[86]*work.v[99]-work.L[87]*work.v[100]-work.L[88]*work.v[101]-work.L[89]*work.v[102]-work.L[90]*work.v[103]-work.L[91]*work.v[104]-work.L[92]*work.v[105]-work.L[93]*work.v[106]-work.L[94]*work.v[107]-work.L[95]*work.v[108]-work.L[96]*work.v[109]-work.L[97]*work.v[110]-work.L[98]*work.v[111]-work.L[99]*work.v[112]-work.L[100]*work.v[113]-work.L[101]*work.v[114]-work.L[102]*work.v[115]-work.L[103]*work.v[116]-work.L[104]*work.v[117]-work.L[105]*work.v[118]-work.L[106]*work.v[119]-work.L[107]*work.v[120]-work.L[108]*work.v[121]-work.L[109]*work.v[122]-work.L[110]*work.v[123]-work.L[111]*work.v[124]-work.L[112]*work.v[125]-work.L[113]*work.v[126]-work.L[114]*work.v[127]-work.L[115]*work.v[128]-work.L[116]*work.v[129]-work.L[117]*work.v[130]-work.L[118]*work.v[131]-work.L[119]*work.v[132]-work.L[120]*work.v[133]-work.L[121]*work.v[134]-work.L[122]*work.v[135]-work.L[123]*work.v[136]-work.L[124]*work.v[137]-work.L[125]*work.v[138]-work.L[126]*work.v[139]-work.L[127]*work.v[140];
  work.v[142] = target[1]-work.L[128]*work.v[73]-work.L[129]*work.v[74]-work.L[130]*work.v[85]-work.L[131]*work.v[86]-work.L[132]*work.v[87]-work.L[133]*work.v[88]-work.L[134]*work.v[89]-work.L[135]*work.v[90]-work.L[136]*work.v[91]-work.L[137]*work.v[92]-work.L[138]*work.v[93]-work.L[139]*work.v[94]-work.L[140]*work.v[95]-work.L[141]*work.v[96]-work.L[142]*work.v[97]-work.L[143]*work.v[98]-work.L[144]*work.v[99]-work.L[145]*work.v[100]-work.L[146]*work.v[101]-work.L[147]*work.v[102]-work.L[148]*work.v[103]-work.L[149]*work.v[104]-work.L[150]*work.v[105]-work.L[151]*work.v[106]-work.L[152]*work.v[107]-work.L[153]*work.v[108]-work.L[154]*work.v[109]-work.L[155]*work.v[110]-work.L[156]*work.v[111]-work.L[157]*work.v[112]-work.L[158]*work.v[113]-work.L[159]*work.v[114]-work.L[160]*work.v[115]-work.L[161]*work.v[116]-work.L[162]*work.v[117]-work.L[163]*work.v[118]-work.L[164]*work.v[119]-work.L[165]*work.v[120]-work.L[166]*work.v[121]-work.L[167]*work.v[122]-work.L[168]*work.v[123]-work.L[169]*work.v[124]-work.L[170]*work.v[125]-work.L[171]*work.v[126]-work.L[172]*work.v[127]-work.L[173]*work.v[128]-work.L[174]*work.v[129]-work.L[175]*work.v[130]-work.L[176]*work.v[131]-work.L[177]*work.v[132]-work.L[178]*work.v[133]-work.L[179]*work.v[134]-work.L[180]*work.v[135]-work.L[181]*work.v[136]-work.L[182]*work.v[137]-work.L[183]*work.v[138]-work.L[184]*work.v[139]-work.L[185]*work.v[140]-work.L[186]*work.v[141];
  work.v[143] = target[2]-work.L[187]*work.v[75]-work.L[188]*work.v[76]-work.L[189]*work.v[85]-work.L[190]*work.v[86]-work.L[191]*work.v[87]-work.L[192]*work.v[88]-work.L[193]*work.v[89]-work.L[194]*work.v[90]-work.L[195]*work.v[91]-work.L[196]*work.v[92]-work.L[197]*work.v[93]-work.L[198]*work.v[94]-work.L[199]*work.v[95]-work.L[200]*work.v[96]-work.L[201]*work.v[97]-work.L[202]*work.v[98]-work.L[203]*work.v[99]-work.L[204]*work.v[100]-work.L[205]*work.v[101]-work.L[206]*work.v[102]-work.L[207]*work.v[103]-work.L[208]*work.v[104]-work.L[209]*work.v[105]-work.L[210]*work.v[106]-work.L[211]*work.v[107]-work.L[212]*work.v[108]-work.L[213]*work.v[109]-work.L[214]*work.v[110]-work.L[215]*work.v[111]-work.L[216]*work.v[112]-work.L[217]*work.v[113]-work.L[218]*work.v[114]-work.L[219]*work.v[115]-work.L[220]*work.v[116]-work.L[221]*work.v[117]-work.L[222]*work.v[118]-work.L[223]*work.v[119]-work.L[224]*work.v[120]-work.L[225]*work.v[121]-work.L[226]*work.v[122]-work.L[227]*work.v[123]-work.L[228]*work.v[124]-work.L[229]*work.v[125]-work.L[230]*work.v[126]-work.L[231]*work.v[127]-work.L[232]*work.v[128]-work.L[233]*work.v[129]-work.L[234]*work.v[130]-work.L[235]*work.v[131]-work.L[236]*work.v[132]-work.L[237]*work.v[133]-work.L[238]*work.v[134]-work.L[239]*work.v[135]-work.L[240]*work.v[136]-work.L[241]*work.v[137]-work.L[242]*work.v[138]-work.L[243]*work.v[139]-work.L[244]*work.v[140]-work.L[245]*work.v[141]-work.L[246]*work.v[142];
  work.v[144] = target[3]-work.L[247]*work.v[77]-work.L[248]*work.v[78]-work.L[249]*work.v[85]-work.L[250]*work.v[86]-work.L[251]*work.v[87]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[90]-work.L[255]*work.v[91]-work.L[256]*work.v[92]-work.L[257]*work.v[93]-work.L[258]*work.v[94]-work.L[259]*work.v[95]-work.L[260]*work.v[96]-work.L[261]*work.v[97]-work.L[262]*work.v[98]-work.L[263]*work.v[99]-work.L[264]*work.v[100]-work.L[265]*work.v[101]-work.L[266]*work.v[102]-work.L[267]*work.v[103]-work.L[268]*work.v[104]-work.L[269]*work.v[105]-work.L[270]*work.v[106]-work.L[271]*work.v[107]-work.L[272]*work.v[108]-work.L[273]*work.v[109]-work.L[274]*work.v[110]-work.L[275]*work.v[111]-work.L[276]*work.v[112]-work.L[277]*work.v[113]-work.L[278]*work.v[114]-work.L[279]*work.v[115]-work.L[280]*work.v[116]-work.L[281]*work.v[117]-work.L[282]*work.v[118]-work.L[283]*work.v[119]-work.L[284]*work.v[120]-work.L[285]*work.v[121]-work.L[286]*work.v[122]-work.L[287]*work.v[123]-work.L[288]*work.v[124]-work.L[289]*work.v[125]-work.L[290]*work.v[126]-work.L[291]*work.v[127]-work.L[292]*work.v[128]-work.L[293]*work.v[129]-work.L[294]*work.v[130]-work.L[295]*work.v[131]-work.L[296]*work.v[132]-work.L[297]*work.v[133]-work.L[298]*work.v[134]-work.L[299]*work.v[135]-work.L[300]*work.v[136]-work.L[301]*work.v[137]-work.L[302]*work.v[138]-work.L[303]*work.v[139]-work.L[304]*work.v[140]-work.L[305]*work.v[141]-work.L[306]*work.v[142]-work.L[307]*work.v[143];
  work.v[145] = target[4]-work.L[308]*work.v[79]-work.L[309]*work.v[80]-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140]-work.L[366]*work.v[141]-work.L[367]*work.v[142]-work.L[368]*work.v[143]-work.L[369]*work.v[144];
  work.v[146] = target[5]-work.L[370]*work.v[81]-work.L[371]*work.v[82]-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141]-work.L[429]*work.v[142]-work.L[430]*work.v[143]-work.L[431]*work.v[144]-work.L[432]*work.v[145];
  work.v[147] = target[6]-work.L[433]*work.v[83]-work.L[434]*work.v[84]-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142]-work.L[493]*work.v[143]-work.L[494]*work.v[144]-work.L[495]*work.v[145]-work.L[496]*work.v[146];
  work.v[148] = target[148]-work.L[497]*work.v[70]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143]-work.L[501]*work.v[144]-work.L[502]*work.v[145]-work.L[503]*work.v[146]-work.L[504]*work.v[147];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 149; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[147] -= work.L[504]*work.v[148];
  work.v[146] -= work.L[496]*work.v[147]+work.L[503]*work.v[148];
  work.v[145] -= work.L[432]*work.v[146]+work.L[495]*work.v[147]+work.L[502]*work.v[148];
  work.v[144] -= work.L[369]*work.v[145]+work.L[431]*work.v[146]+work.L[494]*work.v[147]+work.L[501]*work.v[148];
  work.v[143] -= work.L[307]*work.v[144]+work.L[368]*work.v[145]+work.L[430]*work.v[146]+work.L[493]*work.v[147]+work.L[500]*work.v[148];
  work.v[142] -= work.L[246]*work.v[143]+work.L[306]*work.v[144]+work.L[367]*work.v[145]+work.L[429]*work.v[146]+work.L[492]*work.v[147]+work.L[499]*work.v[148];
  work.v[141] -= work.L[186]*work.v[142]+work.L[245]*work.v[143]+work.L[305]*work.v[144]+work.L[366]*work.v[145]+work.L[428]*work.v[146]+work.L[491]*work.v[147]+work.L[498]*work.v[148];
  work.v[140] -= work.L[127]*work.v[141]+work.L[185]*work.v[142]+work.L[244]*work.v[143]+work.L[304]*work.v[144]+work.L[365]*work.v[145]+work.L[427]*work.v[146]+work.L[490]*work.v[147];
  work.v[139] -= work.L[126]*work.v[141]+work.L[184]*work.v[142]+work.L[243]*work.v[143]+work.L[303]*work.v[144]+work.L[364]*work.v[145]+work.L[426]*work.v[146]+work.L[489]*work.v[147];
  work.v[138] -= work.L[125]*work.v[141]+work.L[183]*work.v[142]+work.L[242]*work.v[143]+work.L[302]*work.v[144]+work.L[363]*work.v[145]+work.L[425]*work.v[146]+work.L[488]*work.v[147];
  work.v[137] -= work.L[124]*work.v[141]+work.L[182]*work.v[142]+work.L[241]*work.v[143]+work.L[301]*work.v[144]+work.L[362]*work.v[145]+work.L[424]*work.v[146]+work.L[487]*work.v[147];
  work.v[136] -= work.L[123]*work.v[141]+work.L[181]*work.v[142]+work.L[240]*work.v[143]+work.L[300]*work.v[144]+work.L[361]*work.v[145]+work.L[423]*work.v[146]+work.L[486]*work.v[147];
  work.v[135] -= work.L[122]*work.v[141]+work.L[180]*work.v[142]+work.L[239]*work.v[143]+work.L[299]*work.v[144]+work.L[360]*work.v[145]+work.L[422]*work.v[146]+work.L[485]*work.v[147];
  work.v[134] -= work.L[121]*work.v[141]+work.L[179]*work.v[142]+work.L[238]*work.v[143]+work.L[298]*work.v[144]+work.L[359]*work.v[145]+work.L[421]*work.v[146]+work.L[484]*work.v[147];
  work.v[133] -= work.L[120]*work.v[141]+work.L[178]*work.v[142]+work.L[237]*work.v[143]+work.L[297]*work.v[144]+work.L[358]*work.v[145]+work.L[420]*work.v[146]+work.L[483]*work.v[147];
  work.v[132] -= work.L[119]*work.v[141]+work.L[177]*work.v[142]+work.L[236]*work.v[143]+work.L[296]*work.v[144]+work.L[357]*work.v[145]+work.L[419]*work.v[146]+work.L[482]*work.v[147];
  work.v[131] -= work.L[118]*work.v[141]+work.L[176]*work.v[142]+work.L[235]*work.v[143]+work.L[295]*work.v[144]+work.L[356]*work.v[145]+work.L[418]*work.v[146]+work.L[481]*work.v[147];
  work.v[130] -= work.L[117]*work.v[141]+work.L[175]*work.v[142]+work.L[234]*work.v[143]+work.L[294]*work.v[144]+work.L[355]*work.v[145]+work.L[417]*work.v[146]+work.L[480]*work.v[147];
  work.v[129] -= work.L[116]*work.v[141]+work.L[174]*work.v[142]+work.L[233]*work.v[143]+work.L[293]*work.v[144]+work.L[354]*work.v[145]+work.L[416]*work.v[146]+work.L[479]*work.v[147];
  work.v[128] -= work.L[115]*work.v[141]+work.L[173]*work.v[142]+work.L[232]*work.v[143]+work.L[292]*work.v[144]+work.L[353]*work.v[145]+work.L[415]*work.v[146]+work.L[478]*work.v[147];
  work.v[127] -= work.L[114]*work.v[141]+work.L[172]*work.v[142]+work.L[231]*work.v[143]+work.L[291]*work.v[144]+work.L[352]*work.v[145]+work.L[414]*work.v[146]+work.L[477]*work.v[147];
  work.v[126] -= work.L[113]*work.v[141]+work.L[171]*work.v[142]+work.L[230]*work.v[143]+work.L[290]*work.v[144]+work.L[351]*work.v[145]+work.L[413]*work.v[146]+work.L[476]*work.v[147];
  work.v[125] -= work.L[112]*work.v[141]+work.L[170]*work.v[142]+work.L[229]*work.v[143]+work.L[289]*work.v[144]+work.L[350]*work.v[145]+work.L[412]*work.v[146]+work.L[475]*work.v[147];
  work.v[124] -= work.L[111]*work.v[141]+work.L[169]*work.v[142]+work.L[228]*work.v[143]+work.L[288]*work.v[144]+work.L[349]*work.v[145]+work.L[411]*work.v[146]+work.L[474]*work.v[147];
  work.v[123] -= work.L[110]*work.v[141]+work.L[168]*work.v[142]+work.L[227]*work.v[143]+work.L[287]*work.v[144]+work.L[348]*work.v[145]+work.L[410]*work.v[146]+work.L[473]*work.v[147];
  work.v[122] -= work.L[109]*work.v[141]+work.L[167]*work.v[142]+work.L[226]*work.v[143]+work.L[286]*work.v[144]+work.L[347]*work.v[145]+work.L[409]*work.v[146]+work.L[472]*work.v[147];
  work.v[121] -= work.L[108]*work.v[141]+work.L[166]*work.v[142]+work.L[225]*work.v[143]+work.L[285]*work.v[144]+work.L[346]*work.v[145]+work.L[408]*work.v[146]+work.L[471]*work.v[147];
  work.v[120] -= work.L[107]*work.v[141]+work.L[165]*work.v[142]+work.L[224]*work.v[143]+work.L[284]*work.v[144]+work.L[345]*work.v[145]+work.L[407]*work.v[146]+work.L[470]*work.v[147];
  work.v[119] -= work.L[106]*work.v[141]+work.L[164]*work.v[142]+work.L[223]*work.v[143]+work.L[283]*work.v[144]+work.L[344]*work.v[145]+work.L[406]*work.v[146]+work.L[469]*work.v[147];
  work.v[118] -= work.L[105]*work.v[141]+work.L[163]*work.v[142]+work.L[222]*work.v[143]+work.L[282]*work.v[144]+work.L[343]*work.v[145]+work.L[405]*work.v[146]+work.L[468]*work.v[147];
  work.v[117] -= work.L[104]*work.v[141]+work.L[162]*work.v[142]+work.L[221]*work.v[143]+work.L[281]*work.v[144]+work.L[342]*work.v[145]+work.L[404]*work.v[146]+work.L[467]*work.v[147];
  work.v[116] -= work.L[103]*work.v[141]+work.L[161]*work.v[142]+work.L[220]*work.v[143]+work.L[280]*work.v[144]+work.L[341]*work.v[145]+work.L[403]*work.v[146]+work.L[466]*work.v[147];
  work.v[115] -= work.L[102]*work.v[141]+work.L[160]*work.v[142]+work.L[219]*work.v[143]+work.L[279]*work.v[144]+work.L[340]*work.v[145]+work.L[402]*work.v[146]+work.L[465]*work.v[147];
  work.v[114] -= work.L[101]*work.v[141]+work.L[159]*work.v[142]+work.L[218]*work.v[143]+work.L[278]*work.v[144]+work.L[339]*work.v[145]+work.L[401]*work.v[146]+work.L[464]*work.v[147];
  work.v[113] -= work.L[100]*work.v[141]+work.L[158]*work.v[142]+work.L[217]*work.v[143]+work.L[277]*work.v[144]+work.L[338]*work.v[145]+work.L[400]*work.v[146]+work.L[463]*work.v[147];
  work.v[112] -= work.L[99]*work.v[141]+work.L[157]*work.v[142]+work.L[216]*work.v[143]+work.L[276]*work.v[144]+work.L[337]*work.v[145]+work.L[399]*work.v[146]+work.L[462]*work.v[147];
  work.v[111] -= work.L[98]*work.v[141]+work.L[156]*work.v[142]+work.L[215]*work.v[143]+work.L[275]*work.v[144]+work.L[336]*work.v[145]+work.L[398]*work.v[146]+work.L[461]*work.v[147];
  work.v[110] -= work.L[97]*work.v[141]+work.L[155]*work.v[142]+work.L[214]*work.v[143]+work.L[274]*work.v[144]+work.L[335]*work.v[145]+work.L[397]*work.v[146]+work.L[460]*work.v[147];
  work.v[109] -= work.L[96]*work.v[141]+work.L[154]*work.v[142]+work.L[213]*work.v[143]+work.L[273]*work.v[144]+work.L[334]*work.v[145]+work.L[396]*work.v[146]+work.L[459]*work.v[147];
  work.v[108] -= work.L[95]*work.v[141]+work.L[153]*work.v[142]+work.L[212]*work.v[143]+work.L[272]*work.v[144]+work.L[333]*work.v[145]+work.L[395]*work.v[146]+work.L[458]*work.v[147];
  work.v[107] -= work.L[94]*work.v[141]+work.L[152]*work.v[142]+work.L[211]*work.v[143]+work.L[271]*work.v[144]+work.L[332]*work.v[145]+work.L[394]*work.v[146]+work.L[457]*work.v[147];
  work.v[106] -= work.L[93]*work.v[141]+work.L[151]*work.v[142]+work.L[210]*work.v[143]+work.L[270]*work.v[144]+work.L[331]*work.v[145]+work.L[393]*work.v[146]+work.L[456]*work.v[147];
  work.v[105] -= work.L[92]*work.v[141]+work.L[150]*work.v[142]+work.L[209]*work.v[143]+work.L[269]*work.v[144]+work.L[330]*work.v[145]+work.L[392]*work.v[146]+work.L[455]*work.v[147];
  work.v[104] -= work.L[91]*work.v[141]+work.L[149]*work.v[142]+work.L[208]*work.v[143]+work.L[268]*work.v[144]+work.L[329]*work.v[145]+work.L[391]*work.v[146]+work.L[454]*work.v[147];
  work.v[103] -= work.L[90]*work.v[141]+work.L[148]*work.v[142]+work.L[207]*work.v[143]+work.L[267]*work.v[144]+work.L[328]*work.v[145]+work.L[390]*work.v[146]+work.L[453]*work.v[147];
  work.v[102] -= work.L[89]*work.v[141]+work.L[147]*work.v[142]+work.L[206]*work.v[143]+work.L[266]*work.v[144]+work.L[327]*work.v[145]+work.L[389]*work.v[146]+work.L[452]*work.v[147];
  work.v[101] -= work.L[88]*work.v[141]+work.L[146]*work.v[142]+work.L[205]*work.v[143]+work.L[265]*work.v[144]+work.L[326]*work.v[145]+work.L[388]*work.v[146]+work.L[451]*work.v[147];
  work.v[100] -= work.L[87]*work.v[141]+work.L[145]*work.v[142]+work.L[204]*work.v[143]+work.L[264]*work.v[144]+work.L[325]*work.v[145]+work.L[387]*work.v[146]+work.L[450]*work.v[147];
  work.v[99] -= work.L[86]*work.v[141]+work.L[144]*work.v[142]+work.L[203]*work.v[143]+work.L[263]*work.v[144]+work.L[324]*work.v[145]+work.L[386]*work.v[146]+work.L[449]*work.v[147];
  work.v[98] -= work.L[85]*work.v[141]+work.L[143]*work.v[142]+work.L[202]*work.v[143]+work.L[262]*work.v[144]+work.L[323]*work.v[145]+work.L[385]*work.v[146]+work.L[448]*work.v[147];
  work.v[97] -= work.L[84]*work.v[141]+work.L[142]*work.v[142]+work.L[201]*work.v[143]+work.L[261]*work.v[144]+work.L[322]*work.v[145]+work.L[384]*work.v[146]+work.L[447]*work.v[147];
  work.v[96] -= work.L[83]*work.v[141]+work.L[141]*work.v[142]+work.L[200]*work.v[143]+work.L[260]*work.v[144]+work.L[321]*work.v[145]+work.L[383]*work.v[146]+work.L[446]*work.v[147];
  work.v[95] -= work.L[82]*work.v[141]+work.L[140]*work.v[142]+work.L[199]*work.v[143]+work.L[259]*work.v[144]+work.L[320]*work.v[145]+work.L[382]*work.v[146]+work.L[445]*work.v[147];
  work.v[94] -= work.L[81]*work.v[141]+work.L[139]*work.v[142]+work.L[198]*work.v[143]+work.L[258]*work.v[144]+work.L[319]*work.v[145]+work.L[381]*work.v[146]+work.L[444]*work.v[147];
  work.v[93] -= work.L[80]*work.v[141]+work.L[138]*work.v[142]+work.L[197]*work.v[143]+work.L[257]*work.v[144]+work.L[318]*work.v[145]+work.L[380]*work.v[146]+work.L[443]*work.v[147];
  work.v[92] -= work.L[79]*work.v[141]+work.L[137]*work.v[142]+work.L[196]*work.v[143]+work.L[256]*work.v[144]+work.L[317]*work.v[145]+work.L[379]*work.v[146]+work.L[442]*work.v[147];
  work.v[91] -= work.L[78]*work.v[141]+work.L[136]*work.v[142]+work.L[195]*work.v[143]+work.L[255]*work.v[144]+work.L[316]*work.v[145]+work.L[378]*work.v[146]+work.L[441]*work.v[147];
  work.v[90] -= work.L[77]*work.v[141]+work.L[135]*work.v[142]+work.L[194]*work.v[143]+work.L[254]*work.v[144]+work.L[315]*work.v[145]+work.L[377]*work.v[146]+work.L[440]*work.v[147];
  work.v[89] -= work.L[76]*work.v[141]+work.L[134]*work.v[142]+work.L[193]*work.v[143]+work.L[253]*work.v[144]+work.L[314]*work.v[145]+work.L[376]*work.v[146]+work.L[439]*work.v[147];
  work.v[88] -= work.L[75]*work.v[141]+work.L[133]*work.v[142]+work.L[192]*work.v[143]+work.L[252]*work.v[144]+work.L[313]*work.v[145]+work.L[375]*work.v[146]+work.L[438]*work.v[147];
  work.v[87] -= work.L[74]*work.v[141]+work.L[132]*work.v[142]+work.L[191]*work.v[143]+work.L[251]*work.v[144]+work.L[312]*work.v[145]+work.L[374]*work.v[146]+work.L[437]*work.v[147];
  work.v[86] -= work.L[73]*work.v[141]+work.L[131]*work.v[142]+work.L[190]*work.v[143]+work.L[250]*work.v[144]+work.L[311]*work.v[145]+work.L[373]*work.v[146]+work.L[436]*work.v[147];
  work.v[85] -= work.L[72]*work.v[141]+work.L[130]*work.v[142]+work.L[189]*work.v[143]+work.L[249]*work.v[144]+work.L[310]*work.v[145]+work.L[372]*work.v[146]+work.L[435]*work.v[147];
  work.v[84] -= work.L[434]*work.v[147];
  work.v[83] -= work.L[433]*work.v[147];
  work.v[82] -= work.L[371]*work.v[146];
  work.v[81] -= work.L[370]*work.v[146];
  work.v[80] -= work.L[309]*work.v[145];
  work.v[79] -= work.L[308]*work.v[145];
  work.v[78] -= work.L[248]*work.v[144];
  work.v[77] -= work.L[247]*work.v[144];
  work.v[76] -= work.L[188]*work.v[143];
  work.v[75] -= work.L[187]*work.v[143];
  work.v[74] -= work.L[129]*work.v[142];
  work.v[73] -= work.L[128]*work.v[142];
  work.v[72] -= work.L[71]*work.v[141];
  work.v[71] -= work.L[70]*work.v[141];
  work.v[70] -= work.L[497]*work.v[148];
  work.v[69] -= work.L[69]*work.v[140];
  work.v[68] -= work.L[68]*work.v[139];
  work.v[67] -= work.L[67]*work.v[138];
  work.v[66] -= work.L[66]*work.v[137];
  work.v[65] -= work.L[65]*work.v[136];
  work.v[64] -= work.L[13]*work.v[84];
  work.v[63] -= work.L[11]*work.v[82];
  work.v[62] -= work.L[9]*work.v[80];
  work.v[61] -= work.L[7]*work.v[78];
  work.v[60] -= work.L[5]*work.v[76];
  work.v[59] -= work.L[3]*work.v[74];
  work.v[58] -= work.L[1]*work.v[72];
  work.v[57] -= work.L[12]*work.v[83];
  work.v[56] -= work.L[10]*work.v[81];
  work.v[55] -= work.L[8]*work.v[79];
  work.v[54] -= work.L[6]*work.v[77];
  work.v[53] -= work.L[4]*work.v[75];
  work.v[52] -= work.L[2]*work.v[73];
  work.v[51] -= work.L[0]*work.v[71];
  work.v[50] -= work.L[64]*work.v[135];
  work.v[49] -= work.L[63]*work.v[134];
  work.v[48] -= work.L[62]*work.v[133];
  work.v[47] -= work.L[61]*work.v[132];
  work.v[46] -= work.L[60]*work.v[131];
  work.v[45] -= work.L[59]*work.v[130];
  work.v[44] -= work.L[58]*work.v[129];
  work.v[43] -= work.L[57]*work.v[128];
  work.v[42] -= work.L[56]*work.v[127];
  work.v[41] -= work.L[55]*work.v[126];
  work.v[40] -= work.L[54]*work.v[125];
  work.v[39] -= work.L[53]*work.v[124];
  work.v[38] -= work.L[52]*work.v[123];
  work.v[37] -= work.L[51]*work.v[122];
  work.v[36] -= work.L[50]*work.v[121];
  work.v[35] -= work.L[49]*work.v[120];
  work.v[34] -= work.L[48]*work.v[119];
  work.v[33] -= work.L[47]*work.v[118];
  work.v[32] -= work.L[46]*work.v[117];
  work.v[31] -= work.L[45]*work.v[116];
  work.v[30] -= work.L[44]*work.v[115];
  work.v[29] -= work.L[43]*work.v[114];
  work.v[28] -= work.L[42]*work.v[113];
  work.v[27] -= work.L[41]*work.v[112];
  work.v[26] -= work.L[40]*work.v[111];
  work.v[25] -= work.L[39]*work.v[110];
  work.v[24] -= work.L[38]*work.v[109];
  work.v[23] -= work.L[37]*work.v[108];
  work.v[22] -= work.L[36]*work.v[107];
  work.v[21] -= work.L[35]*work.v[106];
  work.v[20] -= work.L[34]*work.v[105];
  work.v[19] -= work.L[33]*work.v[104];
  work.v[18] -= work.L[32]*work.v[103];
  work.v[17] -= work.L[31]*work.v[102];
  work.v[16] -= work.L[30]*work.v[101];
  work.v[15] -= work.L[29]*work.v[100];
  work.v[14] -= work.L[28]*work.v[99];
  work.v[13] -= work.L[27]*work.v[98];
  work.v[12] -= work.L[26]*work.v[97];
  work.v[11] -= work.L[25]*work.v[96];
  work.v[10] -= work.L[24]*work.v[95];
  work.v[9] -= work.L[23]*work.v[94];
  work.v[8] -= work.L[22]*work.v[93];
  work.v[7] -= work.L[21]*work.v[92];
  work.v[6] -= work.L[20]*work.v[91];
  work.v[5] -= work.L[19]*work.v[90];
  work.v[4] -= work.L[18]*work.v[89];
  work.v[3] -= work.L[17]*work.v[88];
  work.v[2] -= work.L[16]*work.v[87];
  work.v[1] -= work.L[15]*work.v[86];
  work.v[0] -= work.L[14]*work.v[85];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[141];
  var[1] = work.v[142];
  var[2] = work.v[143];
  var[3] = work.v[144];
  var[4] = work.v[145];
  var[5] = work.v[146];
  var[6] = work.v[147];
  var[7] = work.v[65];
  var[8] = work.v[66];
  var[9] = work.v[67];
  var[10] = work.v[68];
  var[11] = work.v[69];
  var[12] = work.v[70];
  var[13] = work.v[0];
  var[14] = work.v[1];
  var[15] = work.v[2];
  var[16] = work.v[3];
  var[17] = work.v[4];
  var[18] = work.v[5];
  var[19] = work.v[6];
  var[20] = work.v[7];
  var[21] = work.v[8];
  var[22] = work.v[9];
  var[23] = work.v[10];
  var[24] = work.v[11];
  var[25] = work.v[12];
  var[26] = work.v[13];
  var[27] = work.v[14];
  var[28] = work.v[15];
  var[29] = work.v[16];
  var[30] = work.v[17];
  var[31] = work.v[18];
  var[32] = work.v[19];
  var[33] = work.v[20];
  var[34] = work.v[21];
  var[35] = work.v[22];
  var[36] = work.v[23];
  var[37] = work.v[24];
  var[38] = work.v[25];
  var[39] = work.v[26];
  var[40] = work.v[27];
  var[41] = work.v[28];
  var[42] = work.v[29];
  var[43] = work.v[30];
  var[44] = work.v[31];
  var[45] = work.v[32];
  var[46] = work.v[33];
  var[47] = work.v[34];
  var[48] = work.v[35];
  var[49] = work.v[36];
  var[50] = work.v[37];
  var[51] = work.v[38];
  var[52] = work.v[39];
  var[53] = work.v[40];
  var[54] = work.v[41];
  var[55] = work.v[42];
  var[56] = work.v[43];
  var[57] = work.v[44];
  var[58] = work.v[45];
  var[59] = work.v[46];
  var[60] = work.v[47];
  var[61] = work.v[48];
  var[62] = work.v[49];
  var[63] = work.v[50];
  var[64] = work.v[51];
  var[65] = work.v[52];
  var[66] = work.v[53];
  var[67] = work.v[54];
  var[68] = work.v[55];
  var[69] = work.v[56];
  var[70] = work.v[57];
  var[71] = work.v[58];
  var[72] = work.v[59];
  var[73] = work.v[60];
  var[74] = work.v[61];
  var[75] = work.v[62];
  var[76] = work.v[63];
  var[77] = work.v[64];
  var[78] = work.v[85];
  var[79] = work.v[86];
  var[80] = work.v[87];
  var[81] = work.v[88];
  var[82] = work.v[89];
  var[83] = work.v[90];
  var[84] = work.v[91];
  var[85] = work.v[92];
  var[86] = work.v[93];
  var[87] = work.v[94];
  var[88] = work.v[95];
  var[89] = work.v[96];
  var[90] = work.v[97];
  var[91] = work.v[98];
  var[92] = work.v[99];
  var[93] = work.v[100];
  var[94] = work.v[101];
  var[95] = work.v[102];
  var[96] = work.v[103];
  var[97] = work.v[104];
  var[98] = work.v[105];
  var[99] = work.v[106];
  var[100] = work.v[107];
  var[101] = work.v[108];
  var[102] = work.v[109];
  var[103] = work.v[110];
  var[104] = work.v[111];
  var[105] = work.v[112];
  var[106] = work.v[113];
  var[107] = work.v[114];
  var[108] = work.v[115];
  var[109] = work.v[116];
  var[110] = work.v[117];
  var[111] = work.v[118];
  var[112] = work.v[119];
  var[113] = work.v[120];
  var[114] = work.v[121];
  var[115] = work.v[122];
  var[116] = work.v[123];
  var[117] = work.v[124];
  var[118] = work.v[125];
  var[119] = work.v[126];
  var[120] = work.v[127];
  var[121] = work.v[128];
  var[122] = work.v[129];
  var[123] = work.v[130];
  var[124] = work.v[131];
  var[125] = work.v[132];
  var[126] = work.v[133];
  var[127] = work.v[134];
  var[128] = work.v[135];
  var[129] = work.v[71];
  var[130] = work.v[73];
  var[131] = work.v[75];
  var[132] = work.v[77];
  var[133] = work.v[79];
  var[134] = work.v[81];
  var[135] = work.v[83];
  var[136] = work.v[72];
  var[137] = work.v[74];
  var[138] = work.v[76];
  var[139] = work.v[78];
  var[140] = work.v[80];
  var[141] = work.v[82];
  var[142] = work.v[84];
  var[143] = work.v[136];
  var[144] = work.v[137];
  var[145] = work.v[138];
  var[146] = work.v[139];
  var[147] = work.v[140];
  var[148] = work.v[148];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}

void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];

  work.L[14] = work.KKT[1]*work.d_inv[0];

  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];

  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];

  work.L[15] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];

  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];

  work.L[16] = (work.KKT[5])*work.d_inv[2];
  work.v[3] = work.KKT[6];
  work.d[3] = work.v[3];

  if (work.d[3] < 0)
    work.d[3] = settings.kkt_reg;
  else
    work.d[3] += settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];

  work.L[17] = (work.KKT[7])*work.d_inv[3];
  work.v[4] = work.KKT[8];
  work.d[4] = work.v[4];

  if (work.d[4] < 0)
    work.d[4] = settings.kkt_reg;
  else
    work.d[4] += settings.kkt_reg;
  work.d_inv[4] = 1/work.d[4];

  work.L[18] = (work.KKT[9])*work.d_inv[4];
  work.v[5] = work.KKT[10];
  work.d[5] = work.v[5];

  if (work.d[5] < 0)
    work.d[5] = settings.kkt_reg;
  else
    work.d[5] += settings.kkt_reg;
  work.d_inv[5] = 1/work.d[5];

  work.L[19] = (work.KKT[11])*work.d_inv[5];
  work.v[6] = work.KKT[12];
  work.d[6] = work.v[6];

  if (work.d[6] < 0)
    work.d[6] = settings.kkt_reg;
  else
    work.d[6] += settings.kkt_reg;
  work.d_inv[6] = 1/work.d[6];

  work.L[20] = (work.KKT[13])*work.d_inv[6];
  work.v[7] = work.KKT[14];
  work.d[7] = work.v[7];

  if (work.d[7] < 0)
    work.d[7] = settings.kkt_reg;
  else
    work.d[7] += settings.kkt_reg;
  work.d_inv[7] = 1/work.d[7];

  work.L[21] = (work.KKT[15])*work.d_inv[7];
  work.v[8] = work.KKT[16];
  work.d[8] = work.v[8];

  if (work.d[8] < 0)
    work.d[8] = settings.kkt_reg;
  else
    work.d[8] += settings.kkt_reg;
  work.d_inv[8] = 1/work.d[8];

  work.L[22] = (work.KKT[17])*work.d_inv[8];
  work.v[9] = work.KKT[18];
  work.d[9] = work.v[9];

  if (work.d[9] < 0)
    work.d[9] = settings.kkt_reg;
  else
    work.d[9] += settings.kkt_reg;
  work.d_inv[9] = 1/work.d[9];

  work.L[23] = (work.KKT[19])*work.d_inv[9];
  work.v[10] = work.KKT[20];
  work.d[10] = work.v[10];

  if (work.d[10] < 0)
    work.d[10] = settings.kkt_reg;
  else
    work.d[10] += settings.kkt_reg;
  work.d_inv[10] = 1/work.d[10];

  work.L[24] = (work.KKT[21])*work.d_inv[10];
  work.v[11] = work.KKT[22];
  work.d[11] = work.v[11];

  if (work.d[11] < 0)
    work.d[11] = settings.kkt_reg;
  else
    work.d[11] += settings.kkt_reg;
  work.d_inv[11] = 1/work.d[11];

  work.L[25] = (work.KKT[23])*work.d_inv[11];
  work.v[12] = work.KKT[24];
  work.d[12] = work.v[12];

  if (work.d[12] < 0)
    work.d[12] = settings.kkt_reg;
  else
    work.d[12] += settings.kkt_reg;
  work.d_inv[12] = 1/work.d[12];

  work.L[26] = (work.KKT[25])*work.d_inv[12];
  work.v[13] = work.KKT[26];
  work.d[13] = work.v[13];

  if (work.d[13] < 0)
    work.d[13] = settings.kkt_reg;
  else
    work.d[13] += settings.kkt_reg;
  work.d_inv[13] = 1/work.d[13];

  work.L[27] = (work.KKT[27])*work.d_inv[13];
  work.v[14] = work.KKT[28];
  work.d[14] = work.v[14];

  if (work.d[14] < 0)
    work.d[14] = settings.kkt_reg;
  else
    work.d[14] += settings.kkt_reg;
  work.d_inv[14] = 1/work.d[14];

  work.L[28] = (work.KKT[29])*work.d_inv[14];
  work.v[15] = work.KKT[30];
  work.d[15] = work.v[15];

  if (work.d[15] < 0)
    work.d[15] = settings.kkt_reg;
  else
    work.d[15] += settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];

  work.L[29] = (work.KKT[31])*work.d_inv[15];
  work.v[16] = work.KKT[32];
  work.d[16] = work.v[16];

  if (work.d[16] < 0)
    work.d[16] = settings.kkt_reg;
  else
    work.d[16] += settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];

  work.L[30] = (work.KKT[33])*work.d_inv[16];
  work.v[17] = work.KKT[34];
  work.d[17] = work.v[17];

  if (work.d[17] < 0)
    work.d[17] = settings.kkt_reg;
  else
    work.d[17] += settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];

  work.L[31] = (work.KKT[35])*work.d_inv[17];
  work.v[18] = work.KKT[36];
  work.d[18] = work.v[18];

  if (work.d[18] < 0)
    work.d[18] = settings.kkt_reg;
  else
    work.d[18] += settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];

  work.L[32] = (work.KKT[37])*work.d_inv[18];
  work.v[19] = work.KKT[38];
  work.d[19] = work.v[19];

  if (work.d[19] < 0)
    work.d[19] = settings.kkt_reg;
  else
    work.d[19] += settings.kkt_reg;
  work.d_inv[19] = 1/work.d[19];

  work.L[33] = (work.KKT[39])*work.d_inv[19];
  work.v[20] = work.KKT[40];
  work.d[20] = work.v[20];

  if (work.d[20] < 0)
    work.d[20] = settings.kkt_reg;
  else
    work.d[20] += settings.kkt_reg;
  work.d_inv[20] = 1/work.d[20];

  work.L[34] = (work.KKT[41])*work.d_inv[20];
  work.v[21] = work.KKT[42];
  work.d[21] = work.v[21];

  if (work.d[21] < 0)
    work.d[21] = settings.kkt_reg;
  else
    work.d[21] += settings.kkt_reg;
  work.d_inv[21] = 1/work.d[21];

  work.L[35] = (work.KKT[43])*work.d_inv[21];
  work.v[22] = work.KKT[44];
  work.d[22] = work.v[22];

  if (work.d[22] < 0)
    work.d[22] = settings.kkt_reg;
  else
    work.d[22] += settings.kkt_reg;
  work.d_inv[22] = 1/work.d[22];

  work.L[36] = (work.KKT[45])*work.d_inv[22];
  work.v[23] = work.KKT[46];
  work.d[23] = work.v[23];

  if (work.d[23] < 0)
    work.d[23] = settings.kkt_reg;
  else
    work.d[23] += settings.kkt_reg;
  work.d_inv[23] = 1/work.d[23];

  work.L[37] = (work.KKT[47])*work.d_inv[23];
  work.v[24] = work.KKT[48];
  work.d[24] = work.v[24];

  if (work.d[24] < 0)
    work.d[24] = settings.kkt_reg;
  else
    work.d[24] += settings.kkt_reg;
  work.d_inv[24] = 1/work.d[24];

  work.L[38] = (work.KKT[49])*work.d_inv[24];
  work.v[25] = work.KKT[50];
  work.d[25] = work.v[25];

  if (work.d[25] < 0)
    work.d[25] = settings.kkt_reg;
  else
    work.d[25] += settings.kkt_reg;
  work.d_inv[25] = 1/work.d[25];

  work.L[39] = (work.KKT[51])*work.d_inv[25];
  work.v[26] = work.KKT[52];
  work.d[26] = work.v[26];

  if (work.d[26] < 0)
    work.d[26] = settings.kkt_reg;
  else
    work.d[26] += settings.kkt_reg;
  work.d_inv[26] = 1/work.d[26];

  work.L[40] = (work.KKT[53])*work.d_inv[26];
  work.v[27] = work.KKT[54];
  work.d[27] = work.v[27];

  if (work.d[27] < 0)
    work.d[27] = settings.kkt_reg;
  else
    work.d[27] += settings.kkt_reg;
  work.d_inv[27] = 1/work.d[27];

  work.L[41] = (work.KKT[55])*work.d_inv[27];
  work.v[28] = work.KKT[56];
  work.d[28] = work.v[28];

  if (work.d[28] < 0)
    work.d[28] = settings.kkt_reg;
  else
    work.d[28] += settings.kkt_reg;
  work.d_inv[28] = 1/work.d[28];

  work.L[42] = (work.KKT[57])*work.d_inv[28];
  work.v[29] = work.KKT[58];
  work.d[29] = work.v[29];

  if (work.d[29] < 0)
    work.d[29] = settings.kkt_reg;
  else
    work.d[29] += settings.kkt_reg;
  work.d_inv[29] = 1/work.d[29];

  work.L[43] = (work.KKT[59])*work.d_inv[29];
  work.v[30] = work.KKT[60];
  work.d[30] = work.v[30];

  if (work.d[30] < 0)
    work.d[30] = settings.kkt_reg;
  else
    work.d[30] += settings.kkt_reg;
  work.d_inv[30] = 1/work.d[30];

  work.L[44] = (work.KKT[61])*work.d_inv[30];
  work.v[31] = work.KKT[62];
  work.d[31] = work.v[31];

  if (work.d[31] < 0)
    work.d[31] = settings.kkt_reg;
  else
    work.d[31] += settings.kkt_reg;
  work.d_inv[31] = 1/work.d[31];

  work.L[45] = (work.KKT[63])*work.d_inv[31];
  work.v[32] = work.KKT[64];
  work.d[32] = work.v[32];

  if (work.d[32] < 0)
    work.d[32] = settings.kkt_reg;
  else
    work.d[32] += settings.kkt_reg;
  work.d_inv[32] = 1/work.d[32];

  work.L[46] = (work.KKT[65])*work.d_inv[32];
  work.v[33] = work.KKT[66];
  work.d[33] = work.v[33];

  if (work.d[33] < 0)
    work.d[33] = settings.kkt_reg;
  else
    work.d[33] += settings.kkt_reg;
  work.d_inv[33] = 1/work.d[33];

  work.L[47] = (work.KKT[67])*work.d_inv[33];
  work.v[34] = work.KKT[68];
  work.d[34] = work.v[34];

  if (work.d[34] < 0)
    work.d[34] = settings.kkt_reg;
  else
    work.d[34] += settings.kkt_reg;
  work.d_inv[34] = 1/work.d[34];

  work.L[48] = (work.KKT[69])*work.d_inv[34];
  work.v[35] = work.KKT[70];
  work.d[35] = work.v[35];

  if (work.d[35] < 0)
    work.d[35] = settings.kkt_reg;
  else
    work.d[35] += settings.kkt_reg;
  work.d_inv[35] = 1/work.d[35];

  work.L[49] = (work.KKT[71])*work.d_inv[35];
  work.v[36] = work.KKT[72];
  work.d[36] = work.v[36];

  if (work.d[36] < 0)
    work.d[36] = settings.kkt_reg;
  else
    work.d[36] += settings.kkt_reg;
  work.d_inv[36] = 1/work.d[36];

  work.L[50] = (work.KKT[73])*work.d_inv[36];
  work.v[37] = work.KKT[74];
  work.d[37] = work.v[37];

  if (work.d[37] < 0)
    work.d[37] = settings.kkt_reg;
  else
    work.d[37] += settings.kkt_reg;
  work.d_inv[37] = 1/work.d[37];

  work.L[51] = (work.KKT[75])*work.d_inv[37];
  work.v[38] = work.KKT[76];
  work.d[38] = work.v[38];

  if (work.d[38] < 0)
    work.d[38] = settings.kkt_reg;
  else
    work.d[38] += settings.kkt_reg;
  work.d_inv[38] = 1/work.d[38];

  work.L[52] = (work.KKT[77])*work.d_inv[38];
  work.v[39] = work.KKT[78];
  work.d[39] = work.v[39];

  if (work.d[39] < 0)
    work.d[39] = settings.kkt_reg;
  else
    work.d[39] += settings.kkt_reg;
  work.d_inv[39] = 1/work.d[39];

  work.L[53] = (work.KKT[79])*work.d_inv[39];
  work.v[40] = work.KKT[80];
  work.d[40] = work.v[40];

  if (work.d[40] < 0)
    work.d[40] = settings.kkt_reg;
  else
    work.d[40] += settings.kkt_reg;
  work.d_inv[40] = 1/work.d[40];

  work.L[54] = (work.KKT[81])*work.d_inv[40];
  work.v[41] = work.KKT[82];
  work.d[41] = work.v[41];

  if (work.d[41] < 0)
    work.d[41] = settings.kkt_reg;
  else
    work.d[41] += settings.kkt_reg;
  work.d_inv[41] = 1/work.d[41];

  work.L[55] = (work.KKT[83])*work.d_inv[41];
  work.v[42] = work.KKT[84];
  work.d[42] = work.v[42];

  if (work.d[42] < 0)
    work.d[42] = settings.kkt_reg;
  else
    work.d[42] += settings.kkt_reg;
  work.d_inv[42] = 1/work.d[42];

  work.L[56] = (work.KKT[85])*work.d_inv[42];
  work.v[43] = work.KKT[86];
  work.d[43] = work.v[43];

  if (work.d[43] < 0)
    work.d[43] = settings.kkt_reg;
  else
    work.d[43] += settings.kkt_reg;
  work.d_inv[43] = 1/work.d[43];

  work.L[57] = (work.KKT[87])*work.d_inv[43];
  work.v[44] = work.KKT[88];
  work.d[44] = work.v[44];

  if (work.d[44] < 0)
    work.d[44] = settings.kkt_reg;
  else
    work.d[44] += settings.kkt_reg;
  work.d_inv[44] = 1/work.d[44];

  work.L[58] = (work.KKT[89])*work.d_inv[44];
  work.v[45] = work.KKT[90];
  work.d[45] = work.v[45];

  if (work.d[45] < 0)
    work.d[45] = settings.kkt_reg;
  else
    work.d[45] += settings.kkt_reg;
  work.d_inv[45] = 1/work.d[45];

  work.L[59] = (work.KKT[91])*work.d_inv[45];
  work.v[46] = work.KKT[92];
  work.d[46] = work.v[46];

  if (work.d[46] < 0)
    work.d[46] = settings.kkt_reg;
  else
    work.d[46] += settings.kkt_reg;
  work.d_inv[46] = 1/work.d[46];

  work.L[60] = (work.KKT[93])*work.d_inv[46];
  work.v[47] = work.KKT[94];
  work.d[47] = work.v[47];

  if (work.d[47] < 0)
    work.d[47] = settings.kkt_reg;
  else
    work.d[47] += settings.kkt_reg;
  work.d_inv[47] = 1/work.d[47];

  work.L[61] = (work.KKT[95])*work.d_inv[47];
  work.v[48] = work.KKT[96];
  work.d[48] = work.v[48];

  if (work.d[48] < 0)
    work.d[48] = settings.kkt_reg;
  else
    work.d[48] += settings.kkt_reg;
  work.d_inv[48] = 1/work.d[48];

  work.L[62] = (work.KKT[97])*work.d_inv[48];
  work.v[49] = work.KKT[98];
  work.d[49] = work.v[49];

  if (work.d[49] < 0)
    work.d[49] = settings.kkt_reg;
  else
    work.d[49] += settings.kkt_reg;
  work.d_inv[49] = 1/work.d[49];

  work.L[63] = (work.KKT[99])*work.d_inv[49];
  work.v[50] = work.KKT[100];
  work.d[50] = work.v[50];

  if (work.d[50] < 0)
    work.d[50] = settings.kkt_reg;
  else
    work.d[50] += settings.kkt_reg;
  work.d_inv[50] = 1/work.d[50];

  work.L[64] = (work.KKT[101])*work.d_inv[50];
  work.v[51] = work.KKT[102];
  work.d[51] = work.v[51];

  if (work.d[51] < 0)
    work.d[51] = settings.kkt_reg;
  else
    work.d[51] += settings.kkt_reg;
  work.d_inv[51] = 1/work.d[51];

  work.L[0] = (work.KKT[103])*work.d_inv[51];
  work.v[52] = work.KKT[104];
  work.d[52] = work.v[52];

  if (work.d[52] < 0)
    work.d[52] = settings.kkt_reg;
  else
    work.d[52] += settings.kkt_reg;
  work.d_inv[52] = 1/work.d[52];

  work.L[2] = (work.KKT[105])*work.d_inv[52];
  work.v[53] = work.KKT[106];
  work.d[53] = work.v[53];

  if (work.d[53] < 0)
    work.d[53] = settings.kkt_reg;
  else
    work.d[53] += settings.kkt_reg;
  work.d_inv[53] = 1/work.d[53];

  work.L[4] = (work.KKT[107])*work.d_inv[53];
  work.v[54] = work.KKT[108];
  work.d[54] = work.v[54];

  if (work.d[54] < 0)
    work.d[54] = settings.kkt_reg;
  else
    work.d[54] += settings.kkt_reg;
  work.d_inv[54] = 1/work.d[54];

  work.L[6] = (work.KKT[109])*work.d_inv[54];
  work.v[55] = work.KKT[110];
  work.d[55] = work.v[55];

  if (work.d[55] < 0)
    work.d[55] = settings.kkt_reg;
  else
    work.d[55] += settings.kkt_reg;
  work.d_inv[55] = 1/work.d[55];

  work.L[8] = (work.KKT[111])*work.d_inv[55];
  work.v[56] = work.KKT[112];
  work.d[56] = work.v[56];

  if (work.d[56] < 0)
    work.d[56] = settings.kkt_reg;
  else
    work.d[56] += settings.kkt_reg;
  work.d_inv[56] = 1/work.d[56];

  work.L[10] = (work.KKT[113])*work.d_inv[56];
  work.v[57] = work.KKT[114];
  work.d[57] = work.v[57];

  if (work.d[57] < 0)
    work.d[57] = settings.kkt_reg;
  else
    work.d[57] += settings.kkt_reg;
  work.d_inv[57] = 1/work.d[57];

  work.L[12] = (work.KKT[115])*work.d_inv[57];
  work.v[58] = work.KKT[116];
  work.d[58] = work.v[58];

  if (work.d[58] < 0)
    work.d[58] = settings.kkt_reg;
  else
    work.d[58] += settings.kkt_reg;
  work.d_inv[58] = 1/work.d[58];

  work.L[1] = (work.KKT[117])*work.d_inv[58];
  work.v[59] = work.KKT[118];
  work.d[59] = work.v[59];

  if (work.d[59] < 0)
    work.d[59] = settings.kkt_reg;
  else
    work.d[59] += settings.kkt_reg;
  work.d_inv[59] = 1/work.d[59];

  work.L[3] = (work.KKT[119])*work.d_inv[59];
  work.v[60] = work.KKT[120];
  work.d[60] = work.v[60];

  if (work.d[60] < 0)
    work.d[60] = settings.kkt_reg;
  else
    work.d[60] += settings.kkt_reg;
  work.d_inv[60] = 1/work.d[60];

  work.L[5] = (work.KKT[121])*work.d_inv[60];
  work.v[61] = work.KKT[122];
  work.d[61] = work.v[61];

  if (work.d[61] < 0)
    work.d[61] = settings.kkt_reg;
  else
    work.d[61] += settings.kkt_reg;
  work.d_inv[61] = 1/work.d[61];

  work.L[7] = (work.KKT[123])*work.d_inv[61];
  work.v[62] = work.KKT[124];
  work.d[62] = work.v[62];

  if (work.d[62] < 0)
    work.d[62] = settings.kkt_reg;
  else
    work.d[62] += settings.kkt_reg;
  work.d_inv[62] = 1/work.d[62];

  work.L[9] = (work.KKT[125])*work.d_inv[62];
  work.v[63] = work.KKT[126];
  work.d[63] = work.v[63];

  if (work.d[63] < 0)
    work.d[63] = settings.kkt_reg;
  else
    work.d[63] += settings.kkt_reg;
  work.d_inv[63] = 1/work.d[63];

  work.L[11] = (work.KKT[127])*work.d_inv[63];
  work.v[64] = work.KKT[128];
  work.d[64] = work.v[64];

  if (work.d[64] < 0)
    work.d[64] = settings.kkt_reg;
  else
    work.d[64] += settings.kkt_reg;
  work.d_inv[64] = 1/work.d[64];

  work.L[13] = (work.KKT[129])*work.d_inv[64];
  work.v[65] = work.KKT[130];
  work.d[65] = work.v[65];

  if (work.d[65] < 0)
    work.d[65] = settings.kkt_reg;
  else
    work.d[65] += settings.kkt_reg;
  work.d_inv[65] = 1/work.d[65];

  work.L[65] = (work.KKT[131])*work.d_inv[65];
  work.v[66] = work.KKT[132];
  work.d[66] = work.v[66];

  if (work.d[66] < 0)
    work.d[66] = settings.kkt_reg;
  else
    work.d[66] += settings.kkt_reg;
  work.d_inv[66] = 1/work.d[66];

  work.L[66] = (work.KKT[133])*work.d_inv[66];
  work.v[67] = work.KKT[134];
  work.d[67] = work.v[67];

  if (work.d[67] < 0)
    work.d[67] = settings.kkt_reg;
  else
    work.d[67] += settings.kkt_reg;
  work.d_inv[67] = 1/work.d[67];

  work.L[67] = (work.KKT[135])*work.d_inv[67];
  work.v[68] = work.KKT[136];
  work.d[68] = work.v[68];

  if (work.d[68] < 0)
    work.d[68] = settings.kkt_reg;
  else
    work.d[68] += settings.kkt_reg;
  work.d_inv[68] = 1/work.d[68];

  work.L[68] = (work.KKT[137])*work.d_inv[68];
  work.v[69] = work.KKT[138];
  work.d[69] = work.v[69];

  if (work.d[69] < 0)
    work.d[69] = settings.kkt_reg;
  else
    work.d[69] += settings.kkt_reg;
  work.d_inv[69] = 1/work.d[69];

  work.L[69] = (work.KKT[139])*work.d_inv[69];
  work.v[70] = work.KKT[140];
  work.d[70] = work.v[70];

  if (work.d[70] < 0)
    work.d[70] = settings.kkt_reg;
  else
    work.d[70] += settings.kkt_reg;
  work.d_inv[70] = 1/work.d[70];

  work.L[497] = (work.KKT[141])*work.d_inv[70];
  work.v[51] = work.L[0]*work.d[51];
  work.v[71] = work.KKT[142]-work.L[0]*work.v[51];
  work.d[71] = work.v[71];

  if (work.d[71] > 0)
    work.d[71] = -settings.kkt_reg;
  else
    work.d[71] -= settings.kkt_reg;

  work.d_inv[71] = 1/work.d[71];

  work.L[70] = (work.KKT[143])*work.d_inv[71];
  work.v[58] = work.L[1]*work.d[58];
  work.v[72] = work.KKT[144]-work.L[1]*work.v[58];
  work.d[72] = work.v[72];

  if (work.d[72] > 0)
    work.d[72] = -settings.kkt_reg;
  else
    work.d[72] -= settings.kkt_reg;

  work.d_inv[72] = 1/work.d[72];

  work.L[71] = (work.KKT[145])*work.d_inv[72];
  work.v[52] = work.L[2]*work.d[52];
  work.v[73] = work.KKT[146]-work.L[2]*work.v[52];
  work.d[73] = work.v[73];

  if (work.d[73] > 0)
    work.d[73] = -settings.kkt_reg;
  else
    work.d[73] -= settings.kkt_reg;

  work.d_inv[73] = 1/work.d[73];

  work.L[128] = (work.KKT[147])*work.d_inv[73];
  work.v[59] = work.L[3]*work.d[59];
  work.v[74] = work.KKT[148]-work.L[3]*work.v[59];
  work.d[74] = work.v[74];

  if (work.d[74] > 0)
    work.d[74] = -settings.kkt_reg;
  else
    work.d[74] -= settings.kkt_reg;

  work.d_inv[74] = 1/work.d[74];

  work.L[129] = (work.KKT[149])*work.d_inv[74];
  work.v[53] = work.L[4]*work.d[53];
  work.v[75] = work.KKT[150]-work.L[4]*work.v[53];
  work.d[75] = work.v[75];

  if (work.d[75] > 0)
    work.d[75] = -settings.kkt_reg;
  else
    work.d[75] -= settings.kkt_reg;

  work.d_inv[75] = 1/work.d[75];

  work.L[187] = (work.KKT[151])*work.d_inv[75];
  work.v[60] = work.L[5]*work.d[60];
  work.v[76] = work.KKT[152]-work.L[5]*work.v[60];
  work.d[76] = work.v[76];

  if (work.d[76] > 0)
    work.d[76] = -settings.kkt_reg;
  else
    work.d[76] -= settings.kkt_reg;

  work.d_inv[76] = 1/work.d[76];

  work.L[188] = (work.KKT[153])*work.d_inv[76];
  work.v[54] = work.L[6]*work.d[54];
  work.v[77] = work.KKT[154]-work.L[6]*work.v[54];
  work.d[77] = work.v[77];

  if (work.d[77] > 0)
    work.d[77] = -settings.kkt_reg;
  else
    work.d[77] -= settings.kkt_reg;

  work.d_inv[77] = 1/work.d[77];

  work.L[247] = (work.KKT[155])*work.d_inv[77];
  work.v[61] = work.L[7]*work.d[61];
  work.v[78] = work.KKT[156]-work.L[7]*work.v[61];
  work.d[78] = work.v[78];

  if (work.d[78] > 0)
    work.d[78] = -settings.kkt_reg;
  else
    work.d[78] -= settings.kkt_reg;

  work.d_inv[78] = 1/work.d[78];

  work.L[248] = (work.KKT[157])*work.d_inv[78];
  work.v[55] = work.L[8]*work.d[55];
  work.v[79] = work.KKT[158]-work.L[8]*work.v[55];
  work.d[79] = work.v[79];

  if (work.d[79] > 0)
    work.d[79] = -settings.kkt_reg;
  else
    work.d[79] -= settings.kkt_reg;

  work.d_inv[79] = 1/work.d[79];

  work.L[308] = (work.KKT[159])*work.d_inv[79];
  work.v[62] = work.L[9]*work.d[62];
  work.v[80] = work.KKT[160]-work.L[9]*work.v[62];
  work.d[80] = work.v[80];

  if (work.d[80] > 0)
    work.d[80] = -settings.kkt_reg;
  else
    work.d[80] -= settings.kkt_reg;

  work.d_inv[80] = 1/work.d[80];

  work.L[309] = (work.KKT[161])*work.d_inv[80];
  work.v[56] = work.L[10]*work.d[56];
  work.v[81] = work.KKT[162]-work.L[10]*work.v[56];
  work.d[81] = work.v[81];

  if (work.d[81] > 0)
    work.d[81] = -settings.kkt_reg;
  else
    work.d[81] -= settings.kkt_reg;

  work.d_inv[81] = 1/work.d[81];

  work.L[370] = (work.KKT[163])*work.d_inv[81];
  work.v[63] = work.L[11]*work.d[63];
  work.v[82] = work.KKT[164]-work.L[11]*work.v[63];
  work.d[82] = work.v[82];

  if (work.d[82] > 0)
    work.d[82] = -settings.kkt_reg;
  else
    work.d[82] -= settings.kkt_reg;

  work.d_inv[82] = 1/work.d[82];

  work.L[371] = (work.KKT[165])*work.d_inv[82];
  work.v[57] = work.L[12]*work.d[57];
  work.v[83] = work.KKT[166]-work.L[12]*work.v[57];
  work.d[83] = work.v[83];

  if (work.d[83] > 0)
    work.d[83] = -settings.kkt_reg;
  else
    work.d[83] -= settings.kkt_reg;

  work.d_inv[83] = 1/work.d[83];

  work.L[433] = (work.KKT[167])*work.d_inv[83];
  work.v[64] = work.L[13]*work.d[64];
  work.v[84] = work.KKT[168]-work.L[13]*work.v[64];
  work.d[84] = work.v[84];

  if (work.d[84] > 0)
    work.d[84] = -settings.kkt_reg;
  else
    work.d[84] -= settings.kkt_reg;

  work.d_inv[84] = 1/work.d[84];

  work.L[434] = (work.KKT[169])*work.d_inv[84];
  work.v[0] = work.L[14]*work.d[0];
  work.v[85] = work.KKT[170]-work.L[14]*work.v[0];
  work.d[85] = work.v[85];

  if (work.d[85] > 0)
    work.d[85] = -settings.kkt_reg;
  else
    work.d[85] -= settings.kkt_reg;

  work.d_inv[85] = 1/work.d[85];

  work.L[72] = (work.KKT[171])*work.d_inv[85];
  work.L[130] = (work.KKT[172])*work.d_inv[85];
  work.L[189] = (work.KKT[173])*work.d_inv[85];
  work.L[249] = (work.KKT[174])*work.d_inv[85];
  work.L[310] = (work.KKT[175])*work.d_inv[85];
  work.L[372] = (work.KKT[176])*work.d_inv[85];
  work.L[435] = (work.KKT[177])*work.d_inv[85];
  work.v[1] = work.L[15]*work.d[1];
  work.v[86] = work.KKT[178]-work.L[15]*work.v[1];
  work.d[86] = work.v[86];

  if (work.d[86] > 0)
    work.d[86] = -settings.kkt_reg;
  else
    work.d[86] -= settings.kkt_reg;

  work.d_inv[86] = 1/work.d[86];

  work.L[73] = (work.KKT[179])*work.d_inv[86];
  work.L[131] = (work.KKT[180])*work.d_inv[86];
  work.L[190] = (work.KKT[181])*work.d_inv[86];
  work.L[250] = (work.KKT[182])*work.d_inv[86];
  work.L[311] = (work.KKT[183])*work.d_inv[86];
  work.L[373] = (work.KKT[184])*work.d_inv[86];
  work.L[436] = (work.KKT[185])*work.d_inv[86];
  work.v[2] = work.L[16]*work.d[2];
  work.v[87] = work.KKT[186]-work.L[16]*work.v[2];
  work.d[87] = work.v[87];

  if (work.d[87] > 0)
    work.d[87] = -settings.kkt_reg;
  else
    work.d[87] -= settings.kkt_reg;

  work.d_inv[87] = 1/work.d[87];

  work.L[74] = (work.KKT[187])*work.d_inv[87];
  work.L[132] = (work.KKT[188])*work.d_inv[87];
  work.L[191] = (work.KKT[189])*work.d_inv[87];
  work.L[251] = (work.KKT[190])*work.d_inv[87];
  work.L[312] = (work.KKT[191])*work.d_inv[87];
  work.L[374] = (work.KKT[192])*work.d_inv[87];
  work.L[437] = (work.KKT[193])*work.d_inv[87];
  work.v[3] = work.L[17]*work.d[3];
  work.v[88] = work.KKT[194]-work.L[17]*work.v[3];
  work.d[88] = work.v[88];

  if (work.d[88] > 0)
    work.d[88] = -settings.kkt_reg;
  else
    work.d[88] -= settings.kkt_reg;

  work.d_inv[88] = 1/work.d[88];

  work.L[75] = (work.KKT[195])*work.d_inv[88];
  work.L[133] = (work.KKT[196])*work.d_inv[88];
  work.L[192] = (work.KKT[197])*work.d_inv[88];
  work.L[252] = (work.KKT[198])*work.d_inv[88];
  work.L[313] = (work.KKT[199])*work.d_inv[88];
  work.L[375] = (work.KKT[200])*work.d_inv[88];
  work.L[438] = (work.KKT[201])*work.d_inv[88];
  work.v[4] = work.L[18]*work.d[4];
  work.v[89] = work.KKT[202]-work.L[18]*work.v[4];
  work.d[89] = work.v[89];

  if (work.d[89] > 0)
    work.d[89] = -settings.kkt_reg;
  else
    work.d[89] -= settings.kkt_reg;

  work.d_inv[89] = 1/work.d[89];

  work.L[76] = (work.KKT[203])*work.d_inv[89];
  work.L[134] = (work.KKT[204])*work.d_inv[89];
  work.L[193] = (work.KKT[205])*work.d_inv[89];
  work.L[253] = (work.KKT[206])*work.d_inv[89];
  work.L[314] = (work.KKT[207])*work.d_inv[89];
  work.L[376] = (work.KKT[208])*work.d_inv[89];
  work.L[439] = (work.KKT[209])*work.d_inv[89];
  work.v[5] = work.L[19]*work.d[5];
  work.v[90] = work.KKT[210]-work.L[19]*work.v[5];
  work.d[90] = work.v[90];

  if (work.d[90] > 0)
    work.d[90] = -settings.kkt_reg;
  else
    work.d[90] -= settings.kkt_reg;

  work.d_inv[90] = 1/work.d[90];

  work.L[77] = (work.KKT[211])*work.d_inv[90];
  work.L[135] = (work.KKT[212])*work.d_inv[90];
  work.L[194] = (work.KKT[213])*work.d_inv[90];
  work.L[254] = (work.KKT[214])*work.d_inv[90];
  work.L[315] = (work.KKT[215])*work.d_inv[90];
  work.L[377] = (work.KKT[216])*work.d_inv[90];
  work.L[440] = (work.KKT[217])*work.d_inv[90];
  work.v[6] = work.L[20]*work.d[6];
  work.v[91] = work.KKT[218]-work.L[20]*work.v[6];
  work.d[91] = work.v[91];

  if (work.d[91] > 0)
    work.d[91] = -settings.kkt_reg;
  else
    work.d[91] -= settings.kkt_reg;

  work.d_inv[91] = 1/work.d[91];

  work.L[78] = (work.KKT[219])*work.d_inv[91];
  work.L[136] = (work.KKT[220])*work.d_inv[91];
  work.L[195] = (work.KKT[221])*work.d_inv[91];
  work.L[255] = (work.KKT[222])*work.d_inv[91];
  work.L[316] = (work.KKT[223])*work.d_inv[91];
  work.L[378] = (work.KKT[224])*work.d_inv[91];
  work.L[441] = (work.KKT[225])*work.d_inv[91];
  work.v[7] = work.L[21]*work.d[7];
  work.v[92] = work.KKT[226]-work.L[21]*work.v[7];
  work.d[92] = work.v[92];

  if (work.d[92] > 0)
    work.d[92] = -settings.kkt_reg;
  else
    work.d[92] -= settings.kkt_reg;

  work.d_inv[92] = 1/work.d[92];

  work.L[79] = (work.KKT[227])*work.d_inv[92];
  work.L[137] = (work.KKT[228])*work.d_inv[92];
  work.L[196] = (work.KKT[229])*work.d_inv[92];
  work.L[256] = (work.KKT[230])*work.d_inv[92];
  work.L[317] = (work.KKT[231])*work.d_inv[92];
  work.L[379] = (work.KKT[232])*work.d_inv[92];
  work.L[442] = (work.KKT[233])*work.d_inv[92];
  work.v[8] = work.L[22]*work.d[8];
  work.v[93] = work.KKT[234]-work.L[22]*work.v[8];
  work.d[93] = work.v[93];

  if (work.d[93] > 0)
    work.d[93] = -settings.kkt_reg;
  else
    work.d[93] -= settings.kkt_reg;

  work.d_inv[93] = 1/work.d[93];

  work.L[80] = (work.KKT[235])*work.d_inv[93];
  work.L[138] = (work.KKT[236])*work.d_inv[93];
  work.L[197] = (work.KKT[237])*work.d_inv[93];
  work.L[257] = (work.KKT[238])*work.d_inv[93];
  work.L[318] = (work.KKT[239])*work.d_inv[93];
  work.L[380] = (work.KKT[240])*work.d_inv[93];
  work.L[443] = (work.KKT[241])*work.d_inv[93];
  work.v[9] = work.L[23]*work.d[9];
  work.v[94] = work.KKT[242]-work.L[23]*work.v[9];
  work.d[94] = work.v[94];

  if (work.d[94] > 0)
    work.d[94] = -settings.kkt_reg;
  else
    work.d[94] -= settings.kkt_reg;

  work.d_inv[94] = 1/work.d[94];

  work.L[81] = (work.KKT[243])*work.d_inv[94];
  work.L[139] = (work.KKT[244])*work.d_inv[94];
  work.L[198] = (work.KKT[245])*work.d_inv[94];
  work.L[258] = (work.KKT[246])*work.d_inv[94];
  work.L[319] = (work.KKT[247])*work.d_inv[94];
  work.L[381] = (work.KKT[248])*work.d_inv[94];
  work.L[444] = (work.KKT[249])*work.d_inv[94];
  work.v[10] = work.L[24]*work.d[10];
  work.v[95] = work.KKT[250]-work.L[24]*work.v[10];
  work.d[95] = work.v[95];

  if (work.d[95] > 0)
    work.d[95] = -settings.kkt_reg;
  else
    work.d[95] -= settings.kkt_reg;

  work.d_inv[95] = 1/work.d[95];

  work.L[82] = (work.KKT[251])*work.d_inv[95];
  work.L[140] = (work.KKT[252])*work.d_inv[95];
  work.L[199] = (work.KKT[253])*work.d_inv[95];
  work.L[259] = (work.KKT[254])*work.d_inv[95];
  work.L[320] = (work.KKT[255])*work.d_inv[95];
  work.L[382] = (work.KKT[256])*work.d_inv[95];
  work.L[445] = (work.KKT[257])*work.d_inv[95];
  work.v[11] = work.L[25]*work.d[11];
  work.v[96] = work.KKT[258]-work.L[25]*work.v[11];
  work.d[96] = work.v[96];

  if (work.d[96] > 0)
    work.d[96] = -settings.kkt_reg;
  else
    work.d[96] -= settings.kkt_reg;

  work.d_inv[96] = 1/work.d[96];

  work.L[83] = (work.KKT[259])*work.d_inv[96];
  work.L[141] = (work.KKT[260])*work.d_inv[96];
  work.L[200] = (work.KKT[261])*work.d_inv[96];
  work.L[260] = (work.KKT[262])*work.d_inv[96];
  work.L[321] = (work.KKT[263])*work.d_inv[96];
  work.L[383] = (work.KKT[264])*work.d_inv[96];
  work.L[446] = (work.KKT[265])*work.d_inv[96];
  work.v[12] = work.L[26]*work.d[12];
  work.v[97] = work.KKT[266]-work.L[26]*work.v[12];
  work.d[97] = work.v[97];

  if (work.d[97] > 0)
    work.d[97] = -settings.kkt_reg;
  else
    work.d[97] -= settings.kkt_reg;

  work.d_inv[97] = 1/work.d[97];

  work.L[84] = (work.KKT[267])*work.d_inv[97];
  work.L[142] = (work.KKT[268])*work.d_inv[97];
  work.L[201] = (work.KKT[269])*work.d_inv[97];
  work.L[261] = (work.KKT[270])*work.d_inv[97];
  work.L[322] = (work.KKT[271])*work.d_inv[97];
  work.L[384] = (work.KKT[272])*work.d_inv[97];
  work.L[447] = (work.KKT[273])*work.d_inv[97];
  work.v[13] = work.L[27]*work.d[13];
  work.v[98] = work.KKT[274]-work.L[27]*work.v[13];
  work.d[98] = work.v[98];

  if (work.d[98] > 0)
    work.d[98] = -settings.kkt_reg;
  else
    work.d[98] -= settings.kkt_reg;

  work.d_inv[98] = 1/work.d[98];

  work.L[85] = (work.KKT[275])*work.d_inv[98];
  work.L[143] = (work.KKT[276])*work.d_inv[98];
  work.L[202] = (work.KKT[277])*work.d_inv[98];
  work.L[262] = (work.KKT[278])*work.d_inv[98];
  work.L[323] = (work.KKT[279])*work.d_inv[98];
  work.L[385] = (work.KKT[280])*work.d_inv[98];
  work.L[448] = (work.KKT[281])*work.d_inv[98];
  work.v[14] = work.L[28]*work.d[14];
  work.v[99] = work.KKT[282]-work.L[28]*work.v[14];
  work.d[99] = work.v[99];

  if (work.d[99] > 0)
    work.d[99] = -settings.kkt_reg;
  else
    work.d[99] -= settings.kkt_reg;

  work.d_inv[99] = 1/work.d[99];

  work.L[86] = (work.KKT[283])*work.d_inv[99];
  work.L[144] = (work.KKT[284])*work.d_inv[99];
  work.L[203] = (work.KKT[285])*work.d_inv[99];
  work.L[263] = (work.KKT[286])*work.d_inv[99];
  work.L[324] = (work.KKT[287])*work.d_inv[99];
  work.L[386] = (work.KKT[288])*work.d_inv[99];
  work.L[449] = (work.KKT[289])*work.d_inv[99];
  work.v[15] = work.L[29]*work.d[15];
  work.v[100] = work.KKT[290]-work.L[29]*work.v[15];
  work.d[100] = work.v[100];

  if (work.d[100] > 0)
    work.d[100] = -settings.kkt_reg;
  else
    work.d[100] -= settings.kkt_reg;

  work.d_inv[100] = 1/work.d[100];

  work.L[87] = (work.KKT[291])*work.d_inv[100];
  work.L[145] = (work.KKT[292])*work.d_inv[100];
  work.L[204] = (work.KKT[293])*work.d_inv[100];
  work.L[264] = (work.KKT[294])*work.d_inv[100];
  work.L[325] = (work.KKT[295])*work.d_inv[100];
  work.L[387] = (work.KKT[296])*work.d_inv[100];
  work.L[450] = (work.KKT[297])*work.d_inv[100];
  work.v[16] = work.L[30]*work.d[16];
  work.v[101] = work.KKT[298]-work.L[30]*work.v[16];
  work.d[101] = work.v[101];

  if (work.d[101] > 0)
    work.d[101] = -settings.kkt_reg;
  else
    work.d[101] -= settings.kkt_reg;

  work.d_inv[101] = 1/work.d[101];

  work.L[88] = (work.KKT[299])*work.d_inv[101];
  work.L[146] = (work.KKT[300])*work.d_inv[101];
  work.L[205] = (work.KKT[301])*work.d_inv[101];
  work.L[265] = (work.KKT[302])*work.d_inv[101];
  work.L[326] = (work.KKT[303])*work.d_inv[101];
  work.L[388] = (work.KKT[304])*work.d_inv[101];
  work.L[451] = (work.KKT[305])*work.d_inv[101];
  work.v[17] = work.L[31]*work.d[17];
  work.v[102] = work.KKT[306]-work.L[31]*work.v[17];
  work.d[102] = work.v[102];

  if (work.d[102] > 0)
    work.d[102] = -settings.kkt_reg;
  else
    work.d[102] -= settings.kkt_reg;

  work.d_inv[102] = 1/work.d[102];

  work.L[89] = (work.KKT[307])*work.d_inv[102];
  work.L[147] = (work.KKT[308])*work.d_inv[102];
  work.L[206] = (work.KKT[309])*work.d_inv[102];
  work.L[266] = (work.KKT[310])*work.d_inv[102];
  work.L[327] = (work.KKT[311])*work.d_inv[102];
  work.L[389] = (work.KKT[312])*work.d_inv[102];
  work.L[452] = (work.KKT[313])*work.d_inv[102];
  work.v[18] = work.L[32]*work.d[18];
  work.v[103] = work.KKT[314]-work.L[32]*work.v[18];
  work.d[103] = work.v[103];

  if (work.d[103] > 0)
    work.d[103] = -settings.kkt_reg;
  else
    work.d[103] -= settings.kkt_reg;

  work.d_inv[103] = 1/work.d[103];

  work.L[90] = (work.KKT[315])*work.d_inv[103];
  work.L[148] = (work.KKT[316])*work.d_inv[103];
  work.L[207] = (work.KKT[317])*work.d_inv[103];
  work.L[267] = (work.KKT[318])*work.d_inv[103];
  work.L[328] = (work.KKT[319])*work.d_inv[103];
  work.L[390] = (work.KKT[320])*work.d_inv[103];
  work.L[453] = (work.KKT[321])*work.d_inv[103];
  work.v[19] = work.L[33]*work.d[19];
  work.v[104] = work.KKT[322]-work.L[33]*work.v[19];
  work.d[104] = work.v[104];

  if (work.d[104] > 0)
    work.d[104] = -settings.kkt_reg;
  else
    work.d[104] -= settings.kkt_reg;

  work.d_inv[104] = 1/work.d[104];

  work.L[91] = (work.KKT[323])*work.d_inv[104];
  work.L[149] = (work.KKT[324])*work.d_inv[104];
  work.L[208] = (work.KKT[325])*work.d_inv[104];
  work.L[268] = (work.KKT[326])*work.d_inv[104];
  work.L[329] = (work.KKT[327])*work.d_inv[104];
  work.L[391] = (work.KKT[328])*work.d_inv[104];
  work.L[454] = (work.KKT[329])*work.d_inv[104];
  work.v[20] = work.L[34]*work.d[20];
  work.v[105] = work.KKT[330]-work.L[34]*work.v[20];
  work.d[105] = work.v[105];

  if (work.d[105] > 0)
    work.d[105] = -settings.kkt_reg;
  else
    work.d[105] -= settings.kkt_reg;

  work.d_inv[105] = 1/work.d[105];

  work.L[92] = (work.KKT[331])*work.d_inv[105];
  work.L[150] = (work.KKT[332])*work.d_inv[105];
  work.L[209] = (work.KKT[333])*work.d_inv[105];
  work.L[269] = (work.KKT[334])*work.d_inv[105];
  work.L[330] = (work.KKT[335])*work.d_inv[105];
  work.L[392] = (work.KKT[336])*work.d_inv[105];
  work.L[455] = (work.KKT[337])*work.d_inv[105];
  work.v[21] = work.L[35]*work.d[21];
  work.v[106] = work.KKT[338]-work.L[35]*work.v[21];
  work.d[106] = work.v[106];

  if (work.d[106] > 0)
    work.d[106] = -settings.kkt_reg;
  else
    work.d[106] -= settings.kkt_reg;

  work.d_inv[106] = 1/work.d[106];

  work.L[93] = (work.KKT[339])*work.d_inv[106];
  work.L[151] = (work.KKT[340])*work.d_inv[106];
  work.L[210] = (work.KKT[341])*work.d_inv[106];
  work.L[270] = (work.KKT[342])*work.d_inv[106];
  work.L[331] = (work.KKT[343])*work.d_inv[106];
  work.L[393] = (work.KKT[344])*work.d_inv[106];
  work.L[456] = (work.KKT[345])*work.d_inv[106];
  work.v[22] = work.L[36]*work.d[22];
  work.v[107] = work.KKT[346]-work.L[36]*work.v[22];
  work.d[107] = work.v[107];

  if (work.d[107] > 0)
    work.d[107] = -settings.kkt_reg;
  else
    work.d[107] -= settings.kkt_reg;

  work.d_inv[107] = 1/work.d[107];

  work.L[94] = (work.KKT[347])*work.d_inv[107];
  work.L[152] = (work.KKT[348])*work.d_inv[107];
  work.L[211] = (work.KKT[349])*work.d_inv[107];
  work.L[271] = (work.KKT[350])*work.d_inv[107];
  work.L[332] = (work.KKT[351])*work.d_inv[107];
  work.L[394] = (work.KKT[352])*work.d_inv[107];
  work.L[457] = (work.KKT[353])*work.d_inv[107];
  work.v[23] = work.L[37]*work.d[23];
  work.v[108] = work.KKT[354]-work.L[37]*work.v[23];
  work.d[108] = work.v[108];

  if (work.d[108] > 0)
    work.d[108] = -settings.kkt_reg;
  else
    work.d[108] -= settings.kkt_reg;

  work.d_inv[108] = 1/work.d[108];

  work.L[95] = (work.KKT[355])*work.d_inv[108];
  work.L[153] = (work.KKT[356])*work.d_inv[108];
  work.L[212] = (work.KKT[357])*work.d_inv[108];
  work.L[272] = (work.KKT[358])*work.d_inv[108];
  work.L[333] = (work.KKT[359])*work.d_inv[108];
  work.L[395] = (work.KKT[360])*work.d_inv[108];
  work.L[458] = (work.KKT[361])*work.d_inv[108];
  work.v[24] = work.L[38]*work.d[24];
  work.v[109] = work.KKT[362]-work.L[38]*work.v[24];
  work.d[109] = work.v[109];

  if (work.d[109] > 0)
    work.d[109] = -settings.kkt_reg;
  else
    work.d[109] -= settings.kkt_reg;

  work.d_inv[109] = 1/work.d[109];

  work.L[96] = (work.KKT[363])*work.d_inv[109];
  work.L[154] = (work.KKT[364])*work.d_inv[109];
  work.L[213] = (work.KKT[365])*work.d_inv[109];
  work.L[273] = (work.KKT[366])*work.d_inv[109];
  work.L[334] = (work.KKT[367])*work.d_inv[109];
  work.L[396] = (work.KKT[368])*work.d_inv[109];
  work.L[459] = (work.KKT[369])*work.d_inv[109];
  work.v[25] = work.L[39]*work.d[25];
  work.v[110] = work.KKT[370]-work.L[39]*work.v[25];
  work.d[110] = work.v[110];

  if (work.d[110] > 0)
    work.d[110] = -settings.kkt_reg;
  else
    work.d[110] -= settings.kkt_reg;

  work.d_inv[110] = 1/work.d[110];

  work.L[97] = (work.KKT[371])*work.d_inv[110];
  work.L[155] = (work.KKT[372])*work.d_inv[110];
  work.L[214] = (work.KKT[373])*work.d_inv[110];
  work.L[274] = (work.KKT[374])*work.d_inv[110];
  work.L[335] = (work.KKT[375])*work.d_inv[110];
  work.L[397] = (work.KKT[376])*work.d_inv[110];
  work.L[460] = (work.KKT[377])*work.d_inv[110];
  work.v[26] = work.L[40]*work.d[26];
  work.v[111] = work.KKT[378]-work.L[40]*work.v[26];
  work.d[111] = work.v[111];

  if (work.d[111] > 0)
    work.d[111] = -settings.kkt_reg;
  else
    work.d[111] -= settings.kkt_reg;

  work.d_inv[111] = 1/work.d[111];

  work.L[98] = (work.KKT[379])*work.d_inv[111];
  work.L[156] = (work.KKT[380])*work.d_inv[111];
  work.L[215] = (work.KKT[381])*work.d_inv[111];
  work.L[275] = (work.KKT[382])*work.d_inv[111];
  work.L[336] = (work.KKT[383])*work.d_inv[111];
  work.L[398] = (work.KKT[384])*work.d_inv[111];
  work.L[461] = (work.KKT[385])*work.d_inv[111];
  work.v[27] = work.L[41]*work.d[27];
  work.v[112] = work.KKT[386]-work.L[41]*work.v[27];
  work.d[112] = work.v[112];

  if (work.d[112] > 0)
    work.d[112] = -settings.kkt_reg;
  else
    work.d[112] -= settings.kkt_reg;

  work.d_inv[112] = 1/work.d[112];

  work.L[99] = (work.KKT[387])*work.d_inv[112];
  work.L[157] = (work.KKT[388])*work.d_inv[112];
  work.L[216] = (work.KKT[389])*work.d_inv[112];
  work.L[276] = (work.KKT[390])*work.d_inv[112];
  work.L[337] = (work.KKT[391])*work.d_inv[112];
  work.L[399] = (work.KKT[392])*work.d_inv[112];
  work.L[462] = (work.KKT[393])*work.d_inv[112];
  work.v[28] = work.L[42]*work.d[28];
  work.v[113] = work.KKT[394]-work.L[42]*work.v[28];
  work.d[113] = work.v[113];

  if (work.d[113] > 0)
    work.d[113] = -settings.kkt_reg;
  else
    work.d[113] -= settings.kkt_reg;

  work.d_inv[113] = 1/work.d[113];

  work.L[100] = (work.KKT[395])*work.d_inv[113];
  work.L[158] = (work.KKT[396])*work.d_inv[113];
  work.L[217] = (work.KKT[397])*work.d_inv[113];
  work.L[277] = (work.KKT[398])*work.d_inv[113];
  work.L[338] = (work.KKT[399])*work.d_inv[113];
  work.L[400] = (work.KKT[400])*work.d_inv[113];
  work.L[463] = (work.KKT[401])*work.d_inv[113];
  work.v[29] = work.L[43]*work.d[29];
  work.v[114] = work.KKT[402]-work.L[43]*work.v[29];
  work.d[114] = work.v[114];

  if (work.d[114] > 0)
    work.d[114] = -settings.kkt_reg;
  else
    work.d[114] -= settings.kkt_reg;

  work.d_inv[114] = 1/work.d[114];

  work.L[101] = (work.KKT[403])*work.d_inv[114];
  work.L[159] = (work.KKT[404])*work.d_inv[114];
  work.L[218] = (work.KKT[405])*work.d_inv[114];
  work.L[278] = (work.KKT[406])*work.d_inv[114];
  work.L[339] = (work.KKT[407])*work.d_inv[114];
  work.L[401] = (work.KKT[408])*work.d_inv[114];
  work.L[464] = (work.KKT[409])*work.d_inv[114];
  work.v[30] = work.L[44]*work.d[30];
  work.v[115] = work.KKT[410]-work.L[44]*work.v[30];
  work.d[115] = work.v[115];

  if (work.d[115] > 0)
    work.d[115] = -settings.kkt_reg;
  else
    work.d[115] -= settings.kkt_reg;

  work.d_inv[115] = 1/work.d[115];

  work.L[102] = (work.KKT[411])*work.d_inv[115];
  work.L[160] = (work.KKT[412])*work.d_inv[115];
  work.L[219] = (work.KKT[413])*work.d_inv[115];
  work.L[279] = (work.KKT[414])*work.d_inv[115];
  work.L[340] = (work.KKT[415])*work.d_inv[115];
  work.L[402] = (work.KKT[416])*work.d_inv[115];
  work.L[465] = (work.KKT[417])*work.d_inv[115];
  work.v[31] = work.L[45]*work.d[31];
  work.v[116] = work.KKT[418]-work.L[45]*work.v[31];
  work.d[116] = work.v[116];

  if (work.d[116] > 0)
    work.d[116] = -settings.kkt_reg;
  else
    work.d[116] -= settings.kkt_reg;

  work.d_inv[116] = 1/work.d[116];

  work.L[103] = (work.KKT[419])*work.d_inv[116];
  work.L[161] = (work.KKT[420])*work.d_inv[116];
  work.L[220] = (work.KKT[421])*work.d_inv[116];
  work.L[280] = (work.KKT[422])*work.d_inv[116];
  work.L[341] = (work.KKT[423])*work.d_inv[116];
  work.L[403] = (work.KKT[424])*work.d_inv[116];
  work.L[466] = (work.KKT[425])*work.d_inv[116];
  work.v[32] = work.L[46]*work.d[32];
  work.v[117] = work.KKT[426]-work.L[46]*work.v[32];
  work.d[117] = work.v[117];

  if (work.d[117] > 0)
    work.d[117] = -settings.kkt_reg;
  else
    work.d[117] -= settings.kkt_reg;

  work.d_inv[117] = 1/work.d[117];

  work.L[104] = (work.KKT[427])*work.d_inv[117];
  work.L[162] = (work.KKT[428])*work.d_inv[117];
  work.L[221] = (work.KKT[429])*work.d_inv[117];
  work.L[281] = (work.KKT[430])*work.d_inv[117];
  work.L[342] = (work.KKT[431])*work.d_inv[117];
  work.L[404] = (work.KKT[432])*work.d_inv[117];
  work.L[467] = (work.KKT[433])*work.d_inv[117];
  work.v[33] = work.L[47]*work.d[33];
  work.v[118] = work.KKT[434]-work.L[47]*work.v[33];
  work.d[118] = work.v[118];

  if (work.d[118] > 0)
    work.d[118] = -settings.kkt_reg;
  else
    work.d[118] -= settings.kkt_reg;

  work.d_inv[118] = 1/work.d[118];

  work.L[105] = (work.KKT[435])*work.d_inv[118];
  work.L[163] = (work.KKT[436])*work.d_inv[118];
  work.L[222] = (work.KKT[437])*work.d_inv[118];
  work.L[282] = (work.KKT[438])*work.d_inv[118];
  work.L[343] = (work.KKT[439])*work.d_inv[118];
  work.L[405] = (work.KKT[440])*work.d_inv[118];
  work.L[468] = (work.KKT[441])*work.d_inv[118];
  work.v[34] = work.L[48]*work.d[34];
  work.v[119] = work.KKT[442]-work.L[48]*work.v[34];
  work.d[119] = work.v[119];

  if (work.d[119] > 0)
    work.d[119] = -settings.kkt_reg;
  else
    work.d[119] -= settings.kkt_reg;

  work.d_inv[119] = 1/work.d[119];

  work.L[106] = (work.KKT[443])*work.d_inv[119];
  work.L[164] = (work.KKT[444])*work.d_inv[119];
  work.L[223] = (work.KKT[445])*work.d_inv[119];
  work.L[283] = (work.KKT[446])*work.d_inv[119];
  work.L[344] = (work.KKT[447])*work.d_inv[119];
  work.L[406] = (work.KKT[448])*work.d_inv[119];
  work.L[469] = (work.KKT[449])*work.d_inv[119];
  work.v[35] = work.L[49]*work.d[35];
  work.v[120] = work.KKT[450]-work.L[49]*work.v[35];
  work.d[120] = work.v[120];

  if (work.d[120] > 0)
    work.d[120] = -settings.kkt_reg;
  else
    work.d[120] -= settings.kkt_reg;

  work.d_inv[120] = 1/work.d[120];

  work.L[107] = (work.KKT[451])*work.d_inv[120];
  work.L[165] = (work.KKT[452])*work.d_inv[120];
  work.L[224] = (work.KKT[453])*work.d_inv[120];
  work.L[284] = (work.KKT[454])*work.d_inv[120];
  work.L[345] = (work.KKT[455])*work.d_inv[120];
  work.L[407] = (work.KKT[456])*work.d_inv[120];
  work.L[470] = (work.KKT[457])*work.d_inv[120];
  work.v[36] = work.L[50]*work.d[36];
  work.v[121] = work.KKT[458]-work.L[50]*work.v[36];
  work.d[121] = work.v[121];

  if (work.d[121] > 0)
    work.d[121] = -settings.kkt_reg;
  else
    work.d[121] -= settings.kkt_reg;

  work.d_inv[121] = 1/work.d[121];

  work.L[108] = (work.KKT[459])*work.d_inv[121];
  work.L[166] = (work.KKT[460])*work.d_inv[121];
  work.L[225] = (work.KKT[461])*work.d_inv[121];
  work.L[285] = (work.KKT[462])*work.d_inv[121];
  work.L[346] = (work.KKT[463])*work.d_inv[121];
  work.L[408] = (work.KKT[464])*work.d_inv[121];
  work.L[471] = (work.KKT[465])*work.d_inv[121];
  work.v[37] = work.L[51]*work.d[37];
  work.v[122] = work.KKT[466]-work.L[51]*work.v[37];
  work.d[122] = work.v[122];

  if (work.d[122] > 0)
    work.d[122] = -settings.kkt_reg;
  else
    work.d[122] -= settings.kkt_reg;

  work.d_inv[122] = 1/work.d[122];

  work.L[109] = (work.KKT[467])*work.d_inv[122];
  work.L[167] = (work.KKT[468])*work.d_inv[122];
  work.L[226] = (work.KKT[469])*work.d_inv[122];
  work.L[286] = (work.KKT[470])*work.d_inv[122];
  work.L[347] = (work.KKT[471])*work.d_inv[122];
  work.L[409] = (work.KKT[472])*work.d_inv[122];
  work.L[472] = (work.KKT[473])*work.d_inv[122];
  work.v[38] = work.L[52]*work.d[38];
  work.v[123] = work.KKT[474]-work.L[52]*work.v[38];
  work.d[123] = work.v[123];

  if (work.d[123] > 0)
    work.d[123] = -settings.kkt_reg;
  else
    work.d[123] -= settings.kkt_reg;

  work.d_inv[123] = 1/work.d[123];

  work.L[110] = (work.KKT[475])*work.d_inv[123];
  work.L[168] = (work.KKT[476])*work.d_inv[123];
  work.L[227] = (work.KKT[477])*work.d_inv[123];
  work.L[287] = (work.KKT[478])*work.d_inv[123];
  work.L[348] = (work.KKT[479])*work.d_inv[123];
  work.L[410] = (work.KKT[480])*work.d_inv[123];
  work.L[473] = (work.KKT[481])*work.d_inv[123];
  work.v[39] = work.L[53]*work.d[39];
  work.v[124] = work.KKT[482]-work.L[53]*work.v[39];
  work.d[124] = work.v[124];

  if (work.d[124] > 0)
    work.d[124] = -settings.kkt_reg;
  else
    work.d[124] -= settings.kkt_reg;

  work.d_inv[124] = 1/work.d[124];

  work.L[111] = (work.KKT[483])*work.d_inv[124];
  work.L[169] = (work.KKT[484])*work.d_inv[124];
  work.L[228] = (work.KKT[485])*work.d_inv[124];
  work.L[288] = (work.KKT[486])*work.d_inv[124];
  work.L[349] = (work.KKT[487])*work.d_inv[124];
  work.L[411] = (work.KKT[488])*work.d_inv[124];
  work.L[474] = (work.KKT[489])*work.d_inv[124];
  work.v[40] = work.L[54]*work.d[40];
  work.v[125] = work.KKT[490]-work.L[54]*work.v[40];
  work.d[125] = work.v[125];

  if (work.d[125] > 0)
    work.d[125] = -settings.kkt_reg;
  else
    work.d[125] -= settings.kkt_reg;

  work.d_inv[125] = 1/work.d[125];

  work.L[112] = (work.KKT[491])*work.d_inv[125];
  work.L[170] = (work.KKT[492])*work.d_inv[125];
  work.L[229] = (work.KKT[493])*work.d_inv[125];
  work.L[289] = (work.KKT[494])*work.d_inv[125];
  work.L[350] = (work.KKT[495])*work.d_inv[125];
  work.L[412] = (work.KKT[496])*work.d_inv[125];
  work.L[475] = (work.KKT[497])*work.d_inv[125];
  work.v[41] = work.L[55]*work.d[41];
  work.v[126] = work.KKT[498]-work.L[55]*work.v[41];
  work.d[126] = work.v[126];

  if (work.d[126] > 0)
    work.d[126] = -settings.kkt_reg;
  else
    work.d[126] -= settings.kkt_reg;

  work.d_inv[126] = 1/work.d[126];

  work.L[113] = (work.KKT[499])*work.d_inv[126];
  work.L[171] = (work.KKT[500])*work.d_inv[126];
  work.L[230] = (work.KKT[501])*work.d_inv[126];
  work.L[290] = (work.KKT[502])*work.d_inv[126];
  work.L[351] = (work.KKT[503])*work.d_inv[126];
  work.L[413] = (work.KKT[504])*work.d_inv[126];
  work.L[476] = (work.KKT[505])*work.d_inv[126];
  work.v[42] = work.L[56]*work.d[42];
  work.v[127] = work.KKT[506]-work.L[56]*work.v[42];
  work.d[127] = work.v[127];

  if (work.d[127] > 0)
    work.d[127] = -settings.kkt_reg;
  else
    work.d[127] -= settings.kkt_reg;

  work.d_inv[127] = 1/work.d[127];

  work.L[114] = (work.KKT[507])*work.d_inv[127];
  work.L[172] = (work.KKT[508])*work.d_inv[127];
  work.L[231] = (work.KKT[509])*work.d_inv[127];
  work.L[291] = (work.KKT[510])*work.d_inv[127];
  work.L[352] = (work.KKT[511])*work.d_inv[127];
  work.L[414] = (work.KKT[512])*work.d_inv[127];
  work.L[477] = (work.KKT[513])*work.d_inv[127];
  work.v[43] = work.L[57]*work.d[43];
  work.v[128] = work.KKT[514]-work.L[57]*work.v[43];
  work.d[128] = work.v[128];

  if (work.d[128] > 0)
    work.d[128] = -settings.kkt_reg;
  else
    work.d[128] -= settings.kkt_reg;

  work.d_inv[128] = 1/work.d[128];

  work.L[115] = (work.KKT[515])*work.d_inv[128];
  work.L[173] = (work.KKT[516])*work.d_inv[128];
  work.L[232] = (work.KKT[517])*work.d_inv[128];
  work.L[292] = (work.KKT[518])*work.d_inv[128];
  work.L[353] = (work.KKT[519])*work.d_inv[128];
  work.L[415] = (work.KKT[520])*work.d_inv[128];
  work.L[478] = (work.KKT[521])*work.d_inv[128];
  work.v[44] = work.L[58]*work.d[44];
  work.v[129] = work.KKT[522]-work.L[58]*work.v[44];
  work.d[129] = work.v[129];

  if (work.d[129] > 0)
    work.d[129] = -settings.kkt_reg;
  else
    work.d[129] -= settings.kkt_reg;

  work.d_inv[129] = 1/work.d[129];

  work.L[116] = (work.KKT[523])*work.d_inv[129];
  work.L[174] = (work.KKT[524])*work.d_inv[129];
  work.L[233] = (work.KKT[525])*work.d_inv[129];
  work.L[293] = (work.KKT[526])*work.d_inv[129];
  work.L[354] = (work.KKT[527])*work.d_inv[129];
  work.L[416] = (work.KKT[528])*work.d_inv[129];
  work.L[479] = (work.KKT[529])*work.d_inv[129];
  work.v[45] = work.L[59]*work.d[45];
  work.v[130] = work.KKT[530]-work.L[59]*work.v[45];
  work.d[130] = work.v[130];

  if (work.d[130] > 0)
    work.d[130] = -settings.kkt_reg;
  else
    work.d[130] -= settings.kkt_reg;

  work.d_inv[130] = 1/work.d[130];

  work.L[117] = (work.KKT[531])*work.d_inv[130];
  work.L[175] = (work.KKT[532])*work.d_inv[130];
  work.L[234] = (work.KKT[533])*work.d_inv[130];
  work.L[294] = (work.KKT[534])*work.d_inv[130];
  work.L[355] = (work.KKT[535])*work.d_inv[130];
  work.L[417] = (work.KKT[536])*work.d_inv[130];
  work.L[480] = (work.KKT[537])*work.d_inv[130];
  work.v[46] = work.L[60]*work.d[46];
  work.v[131] = work.KKT[538]-work.L[60]*work.v[46];
  work.d[131] = work.v[131];

  if (work.d[131] > 0)
    work.d[131] = -settings.kkt_reg;
  else
    work.d[131] -= settings.kkt_reg;

  work.d_inv[131] = 1/work.d[131];

  work.L[118] = (work.KKT[539])*work.d_inv[131];
  work.L[176] = (work.KKT[540])*work.d_inv[131];
  work.L[235] = (work.KKT[541])*work.d_inv[131];
  work.L[295] = (work.KKT[542])*work.d_inv[131];
  work.L[356] = (work.KKT[543])*work.d_inv[131];
  work.L[418] = (work.KKT[544])*work.d_inv[131];
  work.L[481] = (work.KKT[545])*work.d_inv[131];
  work.v[47] = work.L[61]*work.d[47];
  work.v[132] = work.KKT[546]-work.L[61]*work.v[47];
  work.d[132] = work.v[132];

  if (work.d[132] > 0)
    work.d[132] = -settings.kkt_reg;
  else
    work.d[132] -= settings.kkt_reg;

  work.d_inv[132] = 1/work.d[132];

  work.L[119] = (work.KKT[547])*work.d_inv[132];
  work.L[177] = (work.KKT[548])*work.d_inv[132];
  work.L[236] = (work.KKT[549])*work.d_inv[132];
  work.L[296] = (work.KKT[550])*work.d_inv[132];
  work.L[357] = (work.KKT[551])*work.d_inv[132];
  work.L[419] = (work.KKT[552])*work.d_inv[132];
  work.L[482] = (work.KKT[553])*work.d_inv[132];
  work.v[48] = work.L[62]*work.d[48];
  work.v[133] = work.KKT[554]-work.L[62]*work.v[48];
  work.d[133] = work.v[133];

  if (work.d[133] > 0)
    work.d[133] = -settings.kkt_reg;
  else
    work.d[133] -= settings.kkt_reg;

  work.d_inv[133] = 1/work.d[133];

  work.L[120] = (work.KKT[555])*work.d_inv[133];
  work.L[178] = (work.KKT[556])*work.d_inv[133];
  work.L[237] = (work.KKT[557])*work.d_inv[133];
  work.L[297] = (work.KKT[558])*work.d_inv[133];
  work.L[358] = (work.KKT[559])*work.d_inv[133];
  work.L[420] = (work.KKT[560])*work.d_inv[133];
  work.L[483] = (work.KKT[561])*work.d_inv[133];
  work.v[49] = work.L[63]*work.d[49];
  work.v[134] = work.KKT[562]-work.L[63]*work.v[49];
  work.d[134] = work.v[134];

  if (work.d[134] > 0)
    work.d[134] = -settings.kkt_reg;
  else
    work.d[134] -= settings.kkt_reg;

  work.d_inv[134] = 1/work.d[134];

  work.L[121] = (work.KKT[563])*work.d_inv[134];
  work.L[179] = (work.KKT[564])*work.d_inv[134];
  work.L[238] = (work.KKT[565])*work.d_inv[134];
  work.L[298] = (work.KKT[566])*work.d_inv[134];
  work.L[359] = (work.KKT[567])*work.d_inv[134];
  work.L[421] = (work.KKT[568])*work.d_inv[134];
  work.L[484] = (work.KKT[569])*work.d_inv[134];
  work.v[50] = work.L[64]*work.d[50];
  work.v[135] = work.KKT[570]-work.L[64]*work.v[50];
  work.d[135] = work.v[135];

  if (work.d[135] > 0)
    work.d[135] = -settings.kkt_reg;
  else
    work.d[135] -= settings.kkt_reg;

  work.d_inv[135] = 1/work.d[135];

  work.L[122] = (work.KKT[571])*work.d_inv[135];
  work.L[180] = (work.KKT[572])*work.d_inv[135];
  work.L[239] = (work.KKT[573])*work.d_inv[135];
  work.L[299] = (work.KKT[574])*work.d_inv[135];
  work.L[360] = (work.KKT[575])*work.d_inv[135];
  work.L[422] = (work.KKT[576])*work.d_inv[135];
  work.L[485] = (work.KKT[577])*work.d_inv[135];
  work.v[65] = work.L[65]*work.d[65];
  work.v[136] = 0-work.L[65]*work.v[65];
  work.d[136] = work.v[136];

  if (work.d[136] > 0)
    work.d[136] = -settings.kkt_reg;
  else
    work.d[136] -= settings.kkt_reg;

  work.d_inv[136] = 1/work.d[136];

  work.L[123] = (work.KKT[578])*work.d_inv[136];
  work.L[181] = (work.KKT[579])*work.d_inv[136];
  work.L[240] = (work.KKT[580])*work.d_inv[136];
  work.L[300] = (work.KKT[581])*work.d_inv[136];
  work.L[361] = (work.KKT[582])*work.d_inv[136];
  work.L[423] = (work.KKT[583])*work.d_inv[136];
  work.L[486] = (work.KKT[584])*work.d_inv[136];
  work.v[66] = work.L[66]*work.d[66];
  work.v[137] = 0-work.L[66]*work.v[66];
  work.d[137] = work.v[137];

  if (work.d[137] > 0)
    work.d[137] = -settings.kkt_reg;
  else
    work.d[137] -= settings.kkt_reg;

  work.d_inv[137] = 1/work.d[137];

  work.L[124] = (work.KKT[585])*work.d_inv[137];
  work.L[182] = (work.KKT[586])*work.d_inv[137];
  work.L[241] = (work.KKT[587])*work.d_inv[137];
  work.L[301] = (work.KKT[588])*work.d_inv[137];
  work.L[362] = (work.KKT[589])*work.d_inv[137];
  work.L[424] = (work.KKT[590])*work.d_inv[137];
  work.L[487] = (work.KKT[591])*work.d_inv[137];
  work.v[67] = work.L[67]*work.d[67];
  work.v[138] = 0-work.L[67]*work.v[67];
  work.d[138] = work.v[138];

  if (work.d[138] > 0)
    work.d[138] = -settings.kkt_reg;
  else
    work.d[138] -= settings.kkt_reg;

  work.d_inv[138] = 1/work.d[138];

  work.L[125] = (work.KKT[592])*work.d_inv[138];
  work.L[183] = (work.KKT[593])*work.d_inv[138];
  work.L[242] = (work.KKT[594])*work.d_inv[138];
  work.L[302] = (work.KKT[595])*work.d_inv[138];
  work.L[363] = (work.KKT[596])*work.d_inv[138];
  work.L[425] = (work.KKT[597])*work.d_inv[138];
  work.L[488] = (work.KKT[598])*work.d_inv[138];
  work.v[68] = work.L[68]*work.d[68];
  work.v[139] = 0-work.L[68]*work.v[68];
  work.d[139] = work.v[139];

  if (work.d[139] > 0)
    work.d[139] = -settings.kkt_reg;
  else
    work.d[139] -= settings.kkt_reg;

  work.d_inv[139] = 1/work.d[139];

  work.L[126] = (work.KKT[599])*work.d_inv[139];
  work.L[184] = (work.KKT[600])*work.d_inv[139];
  work.L[243] = (work.KKT[601])*work.d_inv[139];
  work.L[303] = (work.KKT[602])*work.d_inv[139];
  work.L[364] = (work.KKT[603])*work.d_inv[139];
  work.L[426] = (work.KKT[604])*work.d_inv[139];
  work.L[489] = (work.KKT[605])*work.d_inv[139];
  work.v[69] = work.L[69]*work.d[69];
  work.v[140] = 0-work.L[69]*work.v[69];
  work.d[140] = work.v[140];

  if (work.d[140] > 0)
    work.d[140] = -settings.kkt_reg;
  else
    work.d[140] -= settings.kkt_reg;

  work.d_inv[140] = 1/work.d[140];

  work.L[127] = (work.KKT[606])*work.d_inv[140];
  work.L[185] = (work.KKT[607])*work.d_inv[140];
  work.L[244] = (work.KKT[608])*work.d_inv[140];
  work.L[304] = (work.KKT[609])*work.d_inv[140];
  work.L[365] = (work.KKT[610])*work.d_inv[140];
  work.L[427] = (work.KKT[611])*work.d_inv[140];
  work.L[490] = (work.KKT[612])*work.d_inv[140];
  work.v[71] = work.L[70]*work.d[71];
  work.v[72] = work.L[71]*work.d[72];
  work.v[85] = work.L[72]*work.d[85];
  work.v[86] = work.L[73]*work.d[86];
  work.v[87] = work.L[74]*work.d[87];
  work.v[88] = work.L[75]*work.d[88];
  work.v[89] = work.L[76]*work.d[89];
  work.v[90] = work.L[77]*work.d[90];
  work.v[91] = work.L[78]*work.d[91];
  work.v[92] = work.L[79]*work.d[92];
  work.v[93] = work.L[80]*work.d[93];
  work.v[94] = work.L[81]*work.d[94];
  work.v[95] = work.L[82]*work.d[95];
  work.v[96] = work.L[83]*work.d[96];
  work.v[97] = work.L[84]*work.d[97];
  work.v[98] = work.L[85]*work.d[98];
  work.v[99] = work.L[86]*work.d[99];
  work.v[100] = work.L[87]*work.d[100];
  work.v[101] = work.L[88]*work.d[101];
  work.v[102] = work.L[89]*work.d[102];
  work.v[103] = work.L[90]*work.d[103];
  work.v[104] = work.L[91]*work.d[104];
  work.v[105] = work.L[92]*work.d[105];
  work.v[106] = work.L[93]*work.d[106];
  work.v[107] = work.L[94]*work.d[107];
  work.v[108] = work.L[95]*work.d[108];
  work.v[109] = work.L[96]*work.d[109];
  work.v[110] = work.L[97]*work.d[110];
  work.v[111] = work.L[98]*work.d[111];
  work.v[112] = work.L[99]*work.d[112];
  work.v[113] = work.L[100]*work.d[113];
  work.v[114] = work.L[101]*work.d[114];
  work.v[115] = work.L[102]*work.d[115];
  work.v[116] = work.L[103]*work.d[116];
  work.v[117] = work.L[104]*work.d[117];
  work.v[118] = work.L[105]*work.d[118];
  work.v[119] = work.L[106]*work.d[119];
  work.v[120] = work.L[107]*work.d[120];
  work.v[121] = work.L[108]*work.d[121];
  work.v[122] = work.L[109]*work.d[122];
  work.v[123] = work.L[110]*work.d[123];
  work.v[124] = work.L[111]*work.d[124];
  work.v[125] = work.L[112]*work.d[125];
  work.v[126] = work.L[113]*work.d[126];
  work.v[127] = work.L[114]*work.d[127];
  work.v[128] = work.L[115]*work.d[128];
  work.v[129] = work.L[116]*work.d[129];
  work.v[130] = work.L[117]*work.d[130];
  work.v[131] = work.L[118]*work.d[131];
  work.v[132] = work.L[119]*work.d[132];
  work.v[133] = work.L[120]*work.d[133];
  work.v[134] = work.L[121]*work.d[134];
  work.v[135] = work.L[122]*work.d[135];
  work.v[136] = work.L[123]*work.d[136];
  work.v[137] = work.L[124]*work.d[137];
  work.v[138] = work.L[125]*work.d[138];
  work.v[139] = work.L[126]*work.d[139];
  work.v[140] = work.L[127]*work.d[140];
  work.v[141] = 0-work.L[70]*work.v[71]-work.L[71]*work.v[72]-work.L[72]*work.v[85]-work.L[73]*work.v[86]-work.L[74]*work.v[87]-work.L[75]*work.v[88]-work.L[76]*work.v[89]-work.L[77]*work.v[90]-work.L[78]*work.v[91]-work.L[79]*work.v[92]-work.L[80]*work.v[93]-work.L[81]*work.v[94]-work.L[82]*work.v[95]-work.L[83]*work.v[96]-work.L[84]*work.v[97]-work.L[85]*work.v[98]-work.L[86]*work.v[99]-work.L[87]*work.v[100]-work.L[88]*work.v[101]-work.L[89]*work.v[102]-work.L[90]*work.v[103]-work.L[91]*work.v[104]-work.L[92]*work.v[105]-work.L[93]*work.v[106]-work.L[94]*work.v[107]-work.L[95]*work.v[108]-work.L[96]*work.v[109]-work.L[97]*work.v[110]-work.L[98]*work.v[111]-work.L[99]*work.v[112]-work.L[100]*work.v[113]-work.L[101]*work.v[114]-work.L[102]*work.v[115]-work.L[103]*work.v[116]-work.L[104]*work.v[117]-work.L[105]*work.v[118]-work.L[106]*work.v[119]-work.L[107]*work.v[120]-work.L[108]*work.v[121]-work.L[109]*work.v[122]-work.L[110]*work.v[123]-work.L[111]*work.v[124]-work.L[112]*work.v[125]-work.L[113]*work.v[126]-work.L[114]*work.v[127]-work.L[115]*work.v[128]-work.L[116]*work.v[129]-work.L[117]*work.v[130]-work.L[118]*work.v[131]-work.L[119]*work.v[132]-work.L[120]*work.v[133]-work.L[121]*work.v[134]-work.L[122]*work.v[135]-work.L[123]*work.v[136]-work.L[124]*work.v[137]-work.L[125]*work.v[138]-work.L[126]*work.v[139]-work.L[127]*work.v[140];
  work.d[141] = work.v[141];

  if (work.d[141] < 0)
    work.d[141] = settings.kkt_reg;
  else
    work.d[141] += settings.kkt_reg;
  work.d_inv[141] = 1/work.d[141];

  work.L[186] = (-work.L[130]*work.v[85]-work.L[131]*work.v[86]-work.L[132]*work.v[87]-work.L[133]*work.v[88]-work.L[134]*work.v[89]-work.L[135]*work.v[90]-work.L[136]*work.v[91]-work.L[137]*work.v[92]-work.L[138]*work.v[93]-work.L[139]*work.v[94]-work.L[140]*work.v[95]-work.L[141]*work.v[96]-work.L[142]*work.v[97]-work.L[143]*work.v[98]-work.L[144]*work.v[99]-work.L[145]*work.v[100]-work.L[146]*work.v[101]-work.L[147]*work.v[102]-work.L[148]*work.v[103]-work.L[149]*work.v[104]-work.L[150]*work.v[105]-work.L[151]*work.v[106]-work.L[152]*work.v[107]-work.L[153]*work.v[108]-work.L[154]*work.v[109]-work.L[155]*work.v[110]-work.L[156]*work.v[111]-work.L[157]*work.v[112]-work.L[158]*work.v[113]-work.L[159]*work.v[114]-work.L[160]*work.v[115]-work.L[161]*work.v[116]-work.L[162]*work.v[117]-work.L[163]*work.v[118]-work.L[164]*work.v[119]-work.L[165]*work.v[120]-work.L[166]*work.v[121]-work.L[167]*work.v[122]-work.L[168]*work.v[123]-work.L[169]*work.v[124]-work.L[170]*work.v[125]-work.L[171]*work.v[126]-work.L[172]*work.v[127]-work.L[173]*work.v[128]-work.L[174]*work.v[129]-work.L[175]*work.v[130]-work.L[176]*work.v[131]-work.L[177]*work.v[132]-work.L[178]*work.v[133]-work.L[179]*work.v[134]-work.L[180]*work.v[135]-work.L[181]*work.v[136]-work.L[182]*work.v[137]-work.L[183]*work.v[138]-work.L[184]*work.v[139]-work.L[185]*work.v[140])*work.d_inv[141];
  work.L[245] = (-work.L[189]*work.v[85]-work.L[190]*work.v[86]-work.L[191]*work.v[87]-work.L[192]*work.v[88]-work.L[193]*work.v[89]-work.L[194]*work.v[90]-work.L[195]*work.v[91]-work.L[196]*work.v[92]-work.L[197]*work.v[93]-work.L[198]*work.v[94]-work.L[199]*work.v[95]-work.L[200]*work.v[96]-work.L[201]*work.v[97]-work.L[202]*work.v[98]-work.L[203]*work.v[99]-work.L[204]*work.v[100]-work.L[205]*work.v[101]-work.L[206]*work.v[102]-work.L[207]*work.v[103]-work.L[208]*work.v[104]-work.L[209]*work.v[105]-work.L[210]*work.v[106]-work.L[211]*work.v[107]-work.L[212]*work.v[108]-work.L[213]*work.v[109]-work.L[214]*work.v[110]-work.L[215]*work.v[111]-work.L[216]*work.v[112]-work.L[217]*work.v[113]-work.L[218]*work.v[114]-work.L[219]*work.v[115]-work.L[220]*work.v[116]-work.L[221]*work.v[117]-work.L[222]*work.v[118]-work.L[223]*work.v[119]-work.L[224]*work.v[120]-work.L[225]*work.v[121]-work.L[226]*work.v[122]-work.L[227]*work.v[123]-work.L[228]*work.v[124]-work.L[229]*work.v[125]-work.L[230]*work.v[126]-work.L[231]*work.v[127]-work.L[232]*work.v[128]-work.L[233]*work.v[129]-work.L[234]*work.v[130]-work.L[235]*work.v[131]-work.L[236]*work.v[132]-work.L[237]*work.v[133]-work.L[238]*work.v[134]-work.L[239]*work.v[135]-work.L[240]*work.v[136]-work.L[241]*work.v[137]-work.L[242]*work.v[138]-work.L[243]*work.v[139]-work.L[244]*work.v[140])*work.d_inv[141];
  work.L[305] = (-work.L[249]*work.v[85]-work.L[250]*work.v[86]-work.L[251]*work.v[87]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[90]-work.L[255]*work.v[91]-work.L[256]*work.v[92]-work.L[257]*work.v[93]-work.L[258]*work.v[94]-work.L[259]*work.v[95]-work.L[260]*work.v[96]-work.L[261]*work.v[97]-work.L[262]*work.v[98]-work.L[263]*work.v[99]-work.L[264]*work.v[100]-work.L[265]*work.v[101]-work.L[266]*work.v[102]-work.L[267]*work.v[103]-work.L[268]*work.v[104]-work.L[269]*work.v[105]-work.L[270]*work.v[106]-work.L[271]*work.v[107]-work.L[272]*work.v[108]-work.L[273]*work.v[109]-work.L[274]*work.v[110]-work.L[275]*work.v[111]-work.L[276]*work.v[112]-work.L[277]*work.v[113]-work.L[278]*work.v[114]-work.L[279]*work.v[115]-work.L[280]*work.v[116]-work.L[281]*work.v[117]-work.L[282]*work.v[118]-work.L[283]*work.v[119]-work.L[284]*work.v[120]-work.L[285]*work.v[121]-work.L[286]*work.v[122]-work.L[287]*work.v[123]-work.L[288]*work.v[124]-work.L[289]*work.v[125]-work.L[290]*work.v[126]-work.L[291]*work.v[127]-work.L[292]*work.v[128]-work.L[293]*work.v[129]-work.L[294]*work.v[130]-work.L[295]*work.v[131]-work.L[296]*work.v[132]-work.L[297]*work.v[133]-work.L[298]*work.v[134]-work.L[299]*work.v[135]-work.L[300]*work.v[136]-work.L[301]*work.v[137]-work.L[302]*work.v[138]-work.L[303]*work.v[139]-work.L[304]*work.v[140])*work.d_inv[141];
  work.L[366] = (-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140])*work.d_inv[141];
  work.L[428] = (-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140])*work.d_inv[141];
  work.L[491] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140])*work.d_inv[141];
  work.L[498] = (work.KKT[613])*work.d_inv[141];
  work.v[73] = work.L[128]*work.d[73];
  work.v[74] = work.L[129]*work.d[74];
  work.v[85] = work.L[130]*work.d[85];
  work.v[86] = work.L[131]*work.d[86];
  work.v[87] = work.L[132]*work.d[87];
  work.v[88] = work.L[133]*work.d[88];
  work.v[89] = work.L[134]*work.d[89];
  work.v[90] = work.L[135]*work.d[90];
  work.v[91] = work.L[136]*work.d[91];
  work.v[92] = work.L[137]*work.d[92];
  work.v[93] = work.L[138]*work.d[93];
  work.v[94] = work.L[139]*work.d[94];
  work.v[95] = work.L[140]*work.d[95];
  work.v[96] = work.L[141]*work.d[96];
  work.v[97] = work.L[142]*work.d[97];
  work.v[98] = work.L[143]*work.d[98];
  work.v[99] = work.L[144]*work.d[99];
  work.v[100] = work.L[145]*work.d[100];
  work.v[101] = work.L[146]*work.d[101];
  work.v[102] = work.L[147]*work.d[102];
  work.v[103] = work.L[148]*work.d[103];
  work.v[104] = work.L[149]*work.d[104];
  work.v[105] = work.L[150]*work.d[105];
  work.v[106] = work.L[151]*work.d[106];
  work.v[107] = work.L[152]*work.d[107];
  work.v[108] = work.L[153]*work.d[108];
  work.v[109] = work.L[154]*work.d[109];
  work.v[110] = work.L[155]*work.d[110];
  work.v[111] = work.L[156]*work.d[111];
  work.v[112] = work.L[157]*work.d[112];
  work.v[113] = work.L[158]*work.d[113];
  work.v[114] = work.L[159]*work.d[114];
  work.v[115] = work.L[160]*work.d[115];
  work.v[116] = work.L[161]*work.d[116];
  work.v[117] = work.L[162]*work.d[117];
  work.v[118] = work.L[163]*work.d[118];
  work.v[119] = work.L[164]*work.d[119];
  work.v[120] = work.L[165]*work.d[120];
  work.v[121] = work.L[166]*work.d[121];
  work.v[122] = work.L[167]*work.d[122];
  work.v[123] = work.L[168]*work.d[123];
  work.v[124] = work.L[169]*work.d[124];
  work.v[125] = work.L[170]*work.d[125];
  work.v[126] = work.L[171]*work.d[126];
  work.v[127] = work.L[172]*work.d[127];
  work.v[128] = work.L[173]*work.d[128];
  work.v[129] = work.L[174]*work.d[129];
  work.v[130] = work.L[175]*work.d[130];
  work.v[131] = work.L[176]*work.d[131];
  work.v[132] = work.L[177]*work.d[132];
  work.v[133] = work.L[178]*work.d[133];
  work.v[134] = work.L[179]*work.d[134];
  work.v[135] = work.L[180]*work.d[135];
  work.v[136] = work.L[181]*work.d[136];
  work.v[137] = work.L[182]*work.d[137];
  work.v[138] = work.L[183]*work.d[138];
  work.v[139] = work.L[184]*work.d[139];
  work.v[140] = work.L[185]*work.d[140];
  work.v[141] = work.L[186]*work.d[141];
  work.v[142] = 0-work.L[128]*work.v[73]-work.L[129]*work.v[74]-work.L[130]*work.v[85]-work.L[131]*work.v[86]-work.L[132]*work.v[87]-work.L[133]*work.v[88]-work.L[134]*work.v[89]-work.L[135]*work.v[90]-work.L[136]*work.v[91]-work.L[137]*work.v[92]-work.L[138]*work.v[93]-work.L[139]*work.v[94]-work.L[140]*work.v[95]-work.L[141]*work.v[96]-work.L[142]*work.v[97]-work.L[143]*work.v[98]-work.L[144]*work.v[99]-work.L[145]*work.v[100]-work.L[146]*work.v[101]-work.L[147]*work.v[102]-work.L[148]*work.v[103]-work.L[149]*work.v[104]-work.L[150]*work.v[105]-work.L[151]*work.v[106]-work.L[152]*work.v[107]-work.L[153]*work.v[108]-work.L[154]*work.v[109]-work.L[155]*work.v[110]-work.L[156]*work.v[111]-work.L[157]*work.v[112]-work.L[158]*work.v[113]-work.L[159]*work.v[114]-work.L[160]*work.v[115]-work.L[161]*work.v[116]-work.L[162]*work.v[117]-work.L[163]*work.v[118]-work.L[164]*work.v[119]-work.L[165]*work.v[120]-work.L[166]*work.v[121]-work.L[167]*work.v[122]-work.L[168]*work.v[123]-work.L[169]*work.v[124]-work.L[170]*work.v[125]-work.L[171]*work.v[126]-work.L[172]*work.v[127]-work.L[173]*work.v[128]-work.L[174]*work.v[129]-work.L[175]*work.v[130]-work.L[176]*work.v[131]-work.L[177]*work.v[132]-work.L[178]*work.v[133]-work.L[179]*work.v[134]-work.L[180]*work.v[135]-work.L[181]*work.v[136]-work.L[182]*work.v[137]-work.L[183]*work.v[138]-work.L[184]*work.v[139]-work.L[185]*work.v[140]-work.L[186]*work.v[141];
  work.d[142] = work.v[142];

  if (work.d[142] < 0)
    work.d[142] = settings.kkt_reg;
  else
    work.d[142] += settings.kkt_reg;
  work.d_inv[142] = 1/work.d[142];

  work.L[246] = (-work.L[189]*work.v[85]-work.L[190]*work.v[86]-work.L[191]*work.v[87]-work.L[192]*work.v[88]-work.L[193]*work.v[89]-work.L[194]*work.v[90]-work.L[195]*work.v[91]-work.L[196]*work.v[92]-work.L[197]*work.v[93]-work.L[198]*work.v[94]-work.L[199]*work.v[95]-work.L[200]*work.v[96]-work.L[201]*work.v[97]-work.L[202]*work.v[98]-work.L[203]*work.v[99]-work.L[204]*work.v[100]-work.L[205]*work.v[101]-work.L[206]*work.v[102]-work.L[207]*work.v[103]-work.L[208]*work.v[104]-work.L[209]*work.v[105]-work.L[210]*work.v[106]-work.L[211]*work.v[107]-work.L[212]*work.v[108]-work.L[213]*work.v[109]-work.L[214]*work.v[110]-work.L[215]*work.v[111]-work.L[216]*work.v[112]-work.L[217]*work.v[113]-work.L[218]*work.v[114]-work.L[219]*work.v[115]-work.L[220]*work.v[116]-work.L[221]*work.v[117]-work.L[222]*work.v[118]-work.L[223]*work.v[119]-work.L[224]*work.v[120]-work.L[225]*work.v[121]-work.L[226]*work.v[122]-work.L[227]*work.v[123]-work.L[228]*work.v[124]-work.L[229]*work.v[125]-work.L[230]*work.v[126]-work.L[231]*work.v[127]-work.L[232]*work.v[128]-work.L[233]*work.v[129]-work.L[234]*work.v[130]-work.L[235]*work.v[131]-work.L[236]*work.v[132]-work.L[237]*work.v[133]-work.L[238]*work.v[134]-work.L[239]*work.v[135]-work.L[240]*work.v[136]-work.L[241]*work.v[137]-work.L[242]*work.v[138]-work.L[243]*work.v[139]-work.L[244]*work.v[140]-work.L[245]*work.v[141])*work.d_inv[142];
  work.L[306] = (-work.L[249]*work.v[85]-work.L[250]*work.v[86]-work.L[251]*work.v[87]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[90]-work.L[255]*work.v[91]-work.L[256]*work.v[92]-work.L[257]*work.v[93]-work.L[258]*work.v[94]-work.L[259]*work.v[95]-work.L[260]*work.v[96]-work.L[261]*work.v[97]-work.L[262]*work.v[98]-work.L[263]*work.v[99]-work.L[264]*work.v[100]-work.L[265]*work.v[101]-work.L[266]*work.v[102]-work.L[267]*work.v[103]-work.L[268]*work.v[104]-work.L[269]*work.v[105]-work.L[270]*work.v[106]-work.L[271]*work.v[107]-work.L[272]*work.v[108]-work.L[273]*work.v[109]-work.L[274]*work.v[110]-work.L[275]*work.v[111]-work.L[276]*work.v[112]-work.L[277]*work.v[113]-work.L[278]*work.v[114]-work.L[279]*work.v[115]-work.L[280]*work.v[116]-work.L[281]*work.v[117]-work.L[282]*work.v[118]-work.L[283]*work.v[119]-work.L[284]*work.v[120]-work.L[285]*work.v[121]-work.L[286]*work.v[122]-work.L[287]*work.v[123]-work.L[288]*work.v[124]-work.L[289]*work.v[125]-work.L[290]*work.v[126]-work.L[291]*work.v[127]-work.L[292]*work.v[128]-work.L[293]*work.v[129]-work.L[294]*work.v[130]-work.L[295]*work.v[131]-work.L[296]*work.v[132]-work.L[297]*work.v[133]-work.L[298]*work.v[134]-work.L[299]*work.v[135]-work.L[300]*work.v[136]-work.L[301]*work.v[137]-work.L[302]*work.v[138]-work.L[303]*work.v[139]-work.L[304]*work.v[140]-work.L[305]*work.v[141])*work.d_inv[142];
  work.L[367] = (-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140]-work.L[366]*work.v[141])*work.d_inv[142];
  work.L[429] = (-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141])*work.d_inv[142];
  work.L[492] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141])*work.d_inv[142];
  work.L[499] = (work.KKT[614]-work.L[498]*work.v[141])*work.d_inv[142];
  work.v[75] = work.L[187]*work.d[75];
  work.v[76] = work.L[188]*work.d[76];
  work.v[85] = work.L[189]*work.d[85];
  work.v[86] = work.L[190]*work.d[86];
  work.v[87] = work.L[191]*work.d[87];
  work.v[88] = work.L[192]*work.d[88];
  work.v[89] = work.L[193]*work.d[89];
  work.v[90] = work.L[194]*work.d[90];
  work.v[91] = work.L[195]*work.d[91];
  work.v[92] = work.L[196]*work.d[92];
  work.v[93] = work.L[197]*work.d[93];
  work.v[94] = work.L[198]*work.d[94];
  work.v[95] = work.L[199]*work.d[95];
  work.v[96] = work.L[200]*work.d[96];
  work.v[97] = work.L[201]*work.d[97];
  work.v[98] = work.L[202]*work.d[98];
  work.v[99] = work.L[203]*work.d[99];
  work.v[100] = work.L[204]*work.d[100];
  work.v[101] = work.L[205]*work.d[101];
  work.v[102] = work.L[206]*work.d[102];
  work.v[103] = work.L[207]*work.d[103];
  work.v[104] = work.L[208]*work.d[104];
  work.v[105] = work.L[209]*work.d[105];
  work.v[106] = work.L[210]*work.d[106];
  work.v[107] = work.L[211]*work.d[107];
  work.v[108] = work.L[212]*work.d[108];
  work.v[109] = work.L[213]*work.d[109];
  work.v[110] = work.L[214]*work.d[110];
  work.v[111] = work.L[215]*work.d[111];
  work.v[112] = work.L[216]*work.d[112];
  work.v[113] = work.L[217]*work.d[113];
  work.v[114] = work.L[218]*work.d[114];
  work.v[115] = work.L[219]*work.d[115];
  work.v[116] = work.L[220]*work.d[116];
  work.v[117] = work.L[221]*work.d[117];
  work.v[118] = work.L[222]*work.d[118];
  work.v[119] = work.L[223]*work.d[119];
  work.v[120] = work.L[224]*work.d[120];
  work.v[121] = work.L[225]*work.d[121];
  work.v[122] = work.L[226]*work.d[122];
  work.v[123] = work.L[227]*work.d[123];
  work.v[124] = work.L[228]*work.d[124];
  work.v[125] = work.L[229]*work.d[125];
  work.v[126] = work.L[230]*work.d[126];
  work.v[127] = work.L[231]*work.d[127];
  work.v[128] = work.L[232]*work.d[128];
  work.v[129] = work.L[233]*work.d[129];
  work.v[130] = work.L[234]*work.d[130];
  work.v[131] = work.L[235]*work.d[131];
  work.v[132] = work.L[236]*work.d[132];
  work.v[133] = work.L[237]*work.d[133];
  work.v[134] = work.L[238]*work.d[134];
  work.v[135] = work.L[239]*work.d[135];
  work.v[136] = work.L[240]*work.d[136];
  work.v[137] = work.L[241]*work.d[137];
  work.v[138] = work.L[242]*work.d[138];
  work.v[139] = work.L[243]*work.d[139];
  work.v[140] = work.L[244]*work.d[140];
  work.v[141] = work.L[245]*work.d[141];
  work.v[142] = work.L[246]*work.d[142];
  work.v[143] = 0-work.L[187]*work.v[75]-work.L[188]*work.v[76]-work.L[189]*work.v[85]-work.L[190]*work.v[86]-work.L[191]*work.v[87]-work.L[192]*work.v[88]-work.L[193]*work.v[89]-work.L[194]*work.v[90]-work.L[195]*work.v[91]-work.L[196]*work.v[92]-work.L[197]*work.v[93]-work.L[198]*work.v[94]-work.L[199]*work.v[95]-work.L[200]*work.v[96]-work.L[201]*work.v[97]-work.L[202]*work.v[98]-work.L[203]*work.v[99]-work.L[204]*work.v[100]-work.L[205]*work.v[101]-work.L[206]*work.v[102]-work.L[207]*work.v[103]-work.L[208]*work.v[104]-work.L[209]*work.v[105]-work.L[210]*work.v[106]-work.L[211]*work.v[107]-work.L[212]*work.v[108]-work.L[213]*work.v[109]-work.L[214]*work.v[110]-work.L[215]*work.v[111]-work.L[216]*work.v[112]-work.L[217]*work.v[113]-work.L[218]*work.v[114]-work.L[219]*work.v[115]-work.L[220]*work.v[116]-work.L[221]*work.v[117]-work.L[222]*work.v[118]-work.L[223]*work.v[119]-work.L[224]*work.v[120]-work.L[225]*work.v[121]-work.L[226]*work.v[122]-work.L[227]*work.v[123]-work.L[228]*work.v[124]-work.L[229]*work.v[125]-work.L[230]*work.v[126]-work.L[231]*work.v[127]-work.L[232]*work.v[128]-work.L[233]*work.v[129]-work.L[234]*work.v[130]-work.L[235]*work.v[131]-work.L[236]*work.v[132]-work.L[237]*work.v[133]-work.L[238]*work.v[134]-work.L[239]*work.v[135]-work.L[240]*work.v[136]-work.L[241]*work.v[137]-work.L[242]*work.v[138]-work.L[243]*work.v[139]-work.L[244]*work.v[140]-work.L[245]*work.v[141]-work.L[246]*work.v[142];
  work.d[143] = work.v[143];

  if (work.d[143] < 0)
    work.d[143] = settings.kkt_reg;
  else
    work.d[143] += settings.kkt_reg;
  work.d_inv[143] = 1/work.d[143];

  work.L[307] = (-work.L[249]*work.v[85]-work.L[250]*work.v[86]-work.L[251]*work.v[87]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[90]-work.L[255]*work.v[91]-work.L[256]*work.v[92]-work.L[257]*work.v[93]-work.L[258]*work.v[94]-work.L[259]*work.v[95]-work.L[260]*work.v[96]-work.L[261]*work.v[97]-work.L[262]*work.v[98]-work.L[263]*work.v[99]-work.L[264]*work.v[100]-work.L[265]*work.v[101]-work.L[266]*work.v[102]-work.L[267]*work.v[103]-work.L[268]*work.v[104]-work.L[269]*work.v[105]-work.L[270]*work.v[106]-work.L[271]*work.v[107]-work.L[272]*work.v[108]-work.L[273]*work.v[109]-work.L[274]*work.v[110]-work.L[275]*work.v[111]-work.L[276]*work.v[112]-work.L[277]*work.v[113]-work.L[278]*work.v[114]-work.L[279]*work.v[115]-work.L[280]*work.v[116]-work.L[281]*work.v[117]-work.L[282]*work.v[118]-work.L[283]*work.v[119]-work.L[284]*work.v[120]-work.L[285]*work.v[121]-work.L[286]*work.v[122]-work.L[287]*work.v[123]-work.L[288]*work.v[124]-work.L[289]*work.v[125]-work.L[290]*work.v[126]-work.L[291]*work.v[127]-work.L[292]*work.v[128]-work.L[293]*work.v[129]-work.L[294]*work.v[130]-work.L[295]*work.v[131]-work.L[296]*work.v[132]-work.L[297]*work.v[133]-work.L[298]*work.v[134]-work.L[299]*work.v[135]-work.L[300]*work.v[136]-work.L[301]*work.v[137]-work.L[302]*work.v[138]-work.L[303]*work.v[139]-work.L[304]*work.v[140]-work.L[305]*work.v[141]-work.L[306]*work.v[142])*work.d_inv[143];
  work.L[368] = (-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140]-work.L[366]*work.v[141]-work.L[367]*work.v[142])*work.d_inv[143];
  work.L[430] = (-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141]-work.L[429]*work.v[142])*work.d_inv[143];
  work.L[493] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142])*work.d_inv[143];
  work.L[500] = (work.KKT[615]-work.L[498]*work.v[141]-work.L[499]*work.v[142])*work.d_inv[143];
  work.v[77] = work.L[247]*work.d[77];
  work.v[78] = work.L[248]*work.d[78];
  work.v[85] = work.L[249]*work.d[85];
  work.v[86] = work.L[250]*work.d[86];
  work.v[87] = work.L[251]*work.d[87];
  work.v[88] = work.L[252]*work.d[88];
  work.v[89] = work.L[253]*work.d[89];
  work.v[90] = work.L[254]*work.d[90];
  work.v[91] = work.L[255]*work.d[91];
  work.v[92] = work.L[256]*work.d[92];
  work.v[93] = work.L[257]*work.d[93];
  work.v[94] = work.L[258]*work.d[94];
  work.v[95] = work.L[259]*work.d[95];
  work.v[96] = work.L[260]*work.d[96];
  work.v[97] = work.L[261]*work.d[97];
  work.v[98] = work.L[262]*work.d[98];
  work.v[99] = work.L[263]*work.d[99];
  work.v[100] = work.L[264]*work.d[100];
  work.v[101] = work.L[265]*work.d[101];
  work.v[102] = work.L[266]*work.d[102];
  work.v[103] = work.L[267]*work.d[103];
  work.v[104] = work.L[268]*work.d[104];
  work.v[105] = work.L[269]*work.d[105];
  work.v[106] = work.L[270]*work.d[106];
  work.v[107] = work.L[271]*work.d[107];
  work.v[108] = work.L[272]*work.d[108];
  work.v[109] = work.L[273]*work.d[109];
  work.v[110] = work.L[274]*work.d[110];
  work.v[111] = work.L[275]*work.d[111];
  work.v[112] = work.L[276]*work.d[112];
  work.v[113] = work.L[277]*work.d[113];
  work.v[114] = work.L[278]*work.d[114];
  work.v[115] = work.L[279]*work.d[115];
  work.v[116] = work.L[280]*work.d[116];
  work.v[117] = work.L[281]*work.d[117];
  work.v[118] = work.L[282]*work.d[118];
  work.v[119] = work.L[283]*work.d[119];
  work.v[120] = work.L[284]*work.d[120];
  work.v[121] = work.L[285]*work.d[121];
  work.v[122] = work.L[286]*work.d[122];
  work.v[123] = work.L[287]*work.d[123];
  work.v[124] = work.L[288]*work.d[124];
  work.v[125] = work.L[289]*work.d[125];
  work.v[126] = work.L[290]*work.d[126];
  work.v[127] = work.L[291]*work.d[127];
  work.v[128] = work.L[292]*work.d[128];
  work.v[129] = work.L[293]*work.d[129];
  work.v[130] = work.L[294]*work.d[130];
  work.v[131] = work.L[295]*work.d[131];
  work.v[132] = work.L[296]*work.d[132];
  work.v[133] = work.L[297]*work.d[133];
  work.v[134] = work.L[298]*work.d[134];
  work.v[135] = work.L[299]*work.d[135];
  work.v[136] = work.L[300]*work.d[136];
  work.v[137] = work.L[301]*work.d[137];
  work.v[138] = work.L[302]*work.d[138];
  work.v[139] = work.L[303]*work.d[139];
  work.v[140] = work.L[304]*work.d[140];
  work.v[141] = work.L[305]*work.d[141];
  work.v[142] = work.L[306]*work.d[142];
  work.v[143] = work.L[307]*work.d[143];
  work.v[144] = 0-work.L[247]*work.v[77]-work.L[248]*work.v[78]-work.L[249]*work.v[85]-work.L[250]*work.v[86]-work.L[251]*work.v[87]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[90]-work.L[255]*work.v[91]-work.L[256]*work.v[92]-work.L[257]*work.v[93]-work.L[258]*work.v[94]-work.L[259]*work.v[95]-work.L[260]*work.v[96]-work.L[261]*work.v[97]-work.L[262]*work.v[98]-work.L[263]*work.v[99]-work.L[264]*work.v[100]-work.L[265]*work.v[101]-work.L[266]*work.v[102]-work.L[267]*work.v[103]-work.L[268]*work.v[104]-work.L[269]*work.v[105]-work.L[270]*work.v[106]-work.L[271]*work.v[107]-work.L[272]*work.v[108]-work.L[273]*work.v[109]-work.L[274]*work.v[110]-work.L[275]*work.v[111]-work.L[276]*work.v[112]-work.L[277]*work.v[113]-work.L[278]*work.v[114]-work.L[279]*work.v[115]-work.L[280]*work.v[116]-work.L[281]*work.v[117]-work.L[282]*work.v[118]-work.L[283]*work.v[119]-work.L[284]*work.v[120]-work.L[285]*work.v[121]-work.L[286]*work.v[122]-work.L[287]*work.v[123]-work.L[288]*work.v[124]-work.L[289]*work.v[125]-work.L[290]*work.v[126]-work.L[291]*work.v[127]-work.L[292]*work.v[128]-work.L[293]*work.v[129]-work.L[294]*work.v[130]-work.L[295]*work.v[131]-work.L[296]*work.v[132]-work.L[297]*work.v[133]-work.L[298]*work.v[134]-work.L[299]*work.v[135]-work.L[300]*work.v[136]-work.L[301]*work.v[137]-work.L[302]*work.v[138]-work.L[303]*work.v[139]-work.L[304]*work.v[140]-work.L[305]*work.v[141]-work.L[306]*work.v[142]-work.L[307]*work.v[143];
  work.d[144] = work.v[144];

  if (work.d[144] < 0)
    work.d[144] = settings.kkt_reg;
  else
    work.d[144] += settings.kkt_reg;
  work.d_inv[144] = 1/work.d[144];

  work.L[369] = (-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140]-work.L[366]*work.v[141]-work.L[367]*work.v[142]-work.L[368]*work.v[143])*work.d_inv[144];
  work.L[431] = (-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141]-work.L[429]*work.v[142]-work.L[430]*work.v[143])*work.d_inv[144];
  work.L[494] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142]-work.L[493]*work.v[143])*work.d_inv[144];
  work.L[501] = (work.KKT[616]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143])*work.d_inv[144];
  work.v[79] = work.L[308]*work.d[79];
  work.v[80] = work.L[309]*work.d[80];
  work.v[85] = work.L[310]*work.d[85];
  work.v[86] = work.L[311]*work.d[86];
  work.v[87] = work.L[312]*work.d[87];
  work.v[88] = work.L[313]*work.d[88];
  work.v[89] = work.L[314]*work.d[89];
  work.v[90] = work.L[315]*work.d[90];
  work.v[91] = work.L[316]*work.d[91];
  work.v[92] = work.L[317]*work.d[92];
  work.v[93] = work.L[318]*work.d[93];
  work.v[94] = work.L[319]*work.d[94];
  work.v[95] = work.L[320]*work.d[95];
  work.v[96] = work.L[321]*work.d[96];
  work.v[97] = work.L[322]*work.d[97];
  work.v[98] = work.L[323]*work.d[98];
  work.v[99] = work.L[324]*work.d[99];
  work.v[100] = work.L[325]*work.d[100];
  work.v[101] = work.L[326]*work.d[101];
  work.v[102] = work.L[327]*work.d[102];
  work.v[103] = work.L[328]*work.d[103];
  work.v[104] = work.L[329]*work.d[104];
  work.v[105] = work.L[330]*work.d[105];
  work.v[106] = work.L[331]*work.d[106];
  work.v[107] = work.L[332]*work.d[107];
  work.v[108] = work.L[333]*work.d[108];
  work.v[109] = work.L[334]*work.d[109];
  work.v[110] = work.L[335]*work.d[110];
  work.v[111] = work.L[336]*work.d[111];
  work.v[112] = work.L[337]*work.d[112];
  work.v[113] = work.L[338]*work.d[113];
  work.v[114] = work.L[339]*work.d[114];
  work.v[115] = work.L[340]*work.d[115];
  work.v[116] = work.L[341]*work.d[116];
  work.v[117] = work.L[342]*work.d[117];
  work.v[118] = work.L[343]*work.d[118];
  work.v[119] = work.L[344]*work.d[119];
  work.v[120] = work.L[345]*work.d[120];
  work.v[121] = work.L[346]*work.d[121];
  work.v[122] = work.L[347]*work.d[122];
  work.v[123] = work.L[348]*work.d[123];
  work.v[124] = work.L[349]*work.d[124];
  work.v[125] = work.L[350]*work.d[125];
  work.v[126] = work.L[351]*work.d[126];
  work.v[127] = work.L[352]*work.d[127];
  work.v[128] = work.L[353]*work.d[128];
  work.v[129] = work.L[354]*work.d[129];
  work.v[130] = work.L[355]*work.d[130];
  work.v[131] = work.L[356]*work.d[131];
  work.v[132] = work.L[357]*work.d[132];
  work.v[133] = work.L[358]*work.d[133];
  work.v[134] = work.L[359]*work.d[134];
  work.v[135] = work.L[360]*work.d[135];
  work.v[136] = work.L[361]*work.d[136];
  work.v[137] = work.L[362]*work.d[137];
  work.v[138] = work.L[363]*work.d[138];
  work.v[139] = work.L[364]*work.d[139];
  work.v[140] = work.L[365]*work.d[140];
  work.v[141] = work.L[366]*work.d[141];
  work.v[142] = work.L[367]*work.d[142];
  work.v[143] = work.L[368]*work.d[143];
  work.v[144] = work.L[369]*work.d[144];
  work.v[145] = 0-work.L[308]*work.v[79]-work.L[309]*work.v[80]-work.L[310]*work.v[85]-work.L[311]*work.v[86]-work.L[312]*work.v[87]-work.L[313]*work.v[88]-work.L[314]*work.v[89]-work.L[315]*work.v[90]-work.L[316]*work.v[91]-work.L[317]*work.v[92]-work.L[318]*work.v[93]-work.L[319]*work.v[94]-work.L[320]*work.v[95]-work.L[321]*work.v[96]-work.L[322]*work.v[97]-work.L[323]*work.v[98]-work.L[324]*work.v[99]-work.L[325]*work.v[100]-work.L[326]*work.v[101]-work.L[327]*work.v[102]-work.L[328]*work.v[103]-work.L[329]*work.v[104]-work.L[330]*work.v[105]-work.L[331]*work.v[106]-work.L[332]*work.v[107]-work.L[333]*work.v[108]-work.L[334]*work.v[109]-work.L[335]*work.v[110]-work.L[336]*work.v[111]-work.L[337]*work.v[112]-work.L[338]*work.v[113]-work.L[339]*work.v[114]-work.L[340]*work.v[115]-work.L[341]*work.v[116]-work.L[342]*work.v[117]-work.L[343]*work.v[118]-work.L[344]*work.v[119]-work.L[345]*work.v[120]-work.L[346]*work.v[121]-work.L[347]*work.v[122]-work.L[348]*work.v[123]-work.L[349]*work.v[124]-work.L[350]*work.v[125]-work.L[351]*work.v[126]-work.L[352]*work.v[127]-work.L[353]*work.v[128]-work.L[354]*work.v[129]-work.L[355]*work.v[130]-work.L[356]*work.v[131]-work.L[357]*work.v[132]-work.L[358]*work.v[133]-work.L[359]*work.v[134]-work.L[360]*work.v[135]-work.L[361]*work.v[136]-work.L[362]*work.v[137]-work.L[363]*work.v[138]-work.L[364]*work.v[139]-work.L[365]*work.v[140]-work.L[366]*work.v[141]-work.L[367]*work.v[142]-work.L[368]*work.v[143]-work.L[369]*work.v[144];
  work.d[145] = work.v[145];

  if (work.d[145] < 0)
    work.d[145] = settings.kkt_reg;
  else
    work.d[145] += settings.kkt_reg;
  work.d_inv[145] = 1/work.d[145];

  work.L[432] = (-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141]-work.L[429]*work.v[142]-work.L[430]*work.v[143]-work.L[431]*work.v[144])*work.d_inv[145];
  work.L[495] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142]-work.L[493]*work.v[143]-work.L[494]*work.v[144])*work.d_inv[145];
  work.L[502] = (work.KKT[617]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143]-work.L[501]*work.v[144])*work.d_inv[145];
  work.v[81] = work.L[370]*work.d[81];
  work.v[82] = work.L[371]*work.d[82];
  work.v[85] = work.L[372]*work.d[85];
  work.v[86] = work.L[373]*work.d[86];
  work.v[87] = work.L[374]*work.d[87];
  work.v[88] = work.L[375]*work.d[88];
  work.v[89] = work.L[376]*work.d[89];
  work.v[90] = work.L[377]*work.d[90];
  work.v[91] = work.L[378]*work.d[91];
  work.v[92] = work.L[379]*work.d[92];
  work.v[93] = work.L[380]*work.d[93];
  work.v[94] = work.L[381]*work.d[94];
  work.v[95] = work.L[382]*work.d[95];
  work.v[96] = work.L[383]*work.d[96];
  work.v[97] = work.L[384]*work.d[97];
  work.v[98] = work.L[385]*work.d[98];
  work.v[99] = work.L[386]*work.d[99];
  work.v[100] = work.L[387]*work.d[100];
  work.v[101] = work.L[388]*work.d[101];
  work.v[102] = work.L[389]*work.d[102];
  work.v[103] = work.L[390]*work.d[103];
  work.v[104] = work.L[391]*work.d[104];
  work.v[105] = work.L[392]*work.d[105];
  work.v[106] = work.L[393]*work.d[106];
  work.v[107] = work.L[394]*work.d[107];
  work.v[108] = work.L[395]*work.d[108];
  work.v[109] = work.L[396]*work.d[109];
  work.v[110] = work.L[397]*work.d[110];
  work.v[111] = work.L[398]*work.d[111];
  work.v[112] = work.L[399]*work.d[112];
  work.v[113] = work.L[400]*work.d[113];
  work.v[114] = work.L[401]*work.d[114];
  work.v[115] = work.L[402]*work.d[115];
  work.v[116] = work.L[403]*work.d[116];
  work.v[117] = work.L[404]*work.d[117];
  work.v[118] = work.L[405]*work.d[118];
  work.v[119] = work.L[406]*work.d[119];
  work.v[120] = work.L[407]*work.d[120];
  work.v[121] = work.L[408]*work.d[121];
  work.v[122] = work.L[409]*work.d[122];
  work.v[123] = work.L[410]*work.d[123];
  work.v[124] = work.L[411]*work.d[124];
  work.v[125] = work.L[412]*work.d[125];
  work.v[126] = work.L[413]*work.d[126];
  work.v[127] = work.L[414]*work.d[127];
  work.v[128] = work.L[415]*work.d[128];
  work.v[129] = work.L[416]*work.d[129];
  work.v[130] = work.L[417]*work.d[130];
  work.v[131] = work.L[418]*work.d[131];
  work.v[132] = work.L[419]*work.d[132];
  work.v[133] = work.L[420]*work.d[133];
  work.v[134] = work.L[421]*work.d[134];
  work.v[135] = work.L[422]*work.d[135];
  work.v[136] = work.L[423]*work.d[136];
  work.v[137] = work.L[424]*work.d[137];
  work.v[138] = work.L[425]*work.d[138];
  work.v[139] = work.L[426]*work.d[139];
  work.v[140] = work.L[427]*work.d[140];
  work.v[141] = work.L[428]*work.d[141];
  work.v[142] = work.L[429]*work.d[142];
  work.v[143] = work.L[430]*work.d[143];
  work.v[144] = work.L[431]*work.d[144];
  work.v[145] = work.L[432]*work.d[145];
  work.v[146] = 0-work.L[370]*work.v[81]-work.L[371]*work.v[82]-work.L[372]*work.v[85]-work.L[373]*work.v[86]-work.L[374]*work.v[87]-work.L[375]*work.v[88]-work.L[376]*work.v[89]-work.L[377]*work.v[90]-work.L[378]*work.v[91]-work.L[379]*work.v[92]-work.L[380]*work.v[93]-work.L[381]*work.v[94]-work.L[382]*work.v[95]-work.L[383]*work.v[96]-work.L[384]*work.v[97]-work.L[385]*work.v[98]-work.L[386]*work.v[99]-work.L[387]*work.v[100]-work.L[388]*work.v[101]-work.L[389]*work.v[102]-work.L[390]*work.v[103]-work.L[391]*work.v[104]-work.L[392]*work.v[105]-work.L[393]*work.v[106]-work.L[394]*work.v[107]-work.L[395]*work.v[108]-work.L[396]*work.v[109]-work.L[397]*work.v[110]-work.L[398]*work.v[111]-work.L[399]*work.v[112]-work.L[400]*work.v[113]-work.L[401]*work.v[114]-work.L[402]*work.v[115]-work.L[403]*work.v[116]-work.L[404]*work.v[117]-work.L[405]*work.v[118]-work.L[406]*work.v[119]-work.L[407]*work.v[120]-work.L[408]*work.v[121]-work.L[409]*work.v[122]-work.L[410]*work.v[123]-work.L[411]*work.v[124]-work.L[412]*work.v[125]-work.L[413]*work.v[126]-work.L[414]*work.v[127]-work.L[415]*work.v[128]-work.L[416]*work.v[129]-work.L[417]*work.v[130]-work.L[418]*work.v[131]-work.L[419]*work.v[132]-work.L[420]*work.v[133]-work.L[421]*work.v[134]-work.L[422]*work.v[135]-work.L[423]*work.v[136]-work.L[424]*work.v[137]-work.L[425]*work.v[138]-work.L[426]*work.v[139]-work.L[427]*work.v[140]-work.L[428]*work.v[141]-work.L[429]*work.v[142]-work.L[430]*work.v[143]-work.L[431]*work.v[144]-work.L[432]*work.v[145];
  work.d[146] = work.v[146];

  if (work.d[146] < 0)
    work.d[146] = settings.kkt_reg;
  else
    work.d[146] += settings.kkt_reg;
  work.d_inv[146] = 1/work.d[146];

  work.L[496] = (-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142]-work.L[493]*work.v[143]-work.L[494]*work.v[144]-work.L[495]*work.v[145])*work.d_inv[146];
  work.L[503] = (work.KKT[618]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143]-work.L[501]*work.v[144]-work.L[502]*work.v[145])*work.d_inv[146];
  work.v[83] = work.L[433]*work.d[83];
  work.v[84] = work.L[434]*work.d[84];
  work.v[85] = work.L[435]*work.d[85];
  work.v[86] = work.L[436]*work.d[86];
  work.v[87] = work.L[437]*work.d[87];
  work.v[88] = work.L[438]*work.d[88];
  work.v[89] = work.L[439]*work.d[89];
  work.v[90] = work.L[440]*work.d[90];
  work.v[91] = work.L[441]*work.d[91];
  work.v[92] = work.L[442]*work.d[92];
  work.v[93] = work.L[443]*work.d[93];
  work.v[94] = work.L[444]*work.d[94];
  work.v[95] = work.L[445]*work.d[95];
  work.v[96] = work.L[446]*work.d[96];
  work.v[97] = work.L[447]*work.d[97];
  work.v[98] = work.L[448]*work.d[98];
  work.v[99] = work.L[449]*work.d[99];
  work.v[100] = work.L[450]*work.d[100];
  work.v[101] = work.L[451]*work.d[101];
  work.v[102] = work.L[452]*work.d[102];
  work.v[103] = work.L[453]*work.d[103];
  work.v[104] = work.L[454]*work.d[104];
  work.v[105] = work.L[455]*work.d[105];
  work.v[106] = work.L[456]*work.d[106];
  work.v[107] = work.L[457]*work.d[107];
  work.v[108] = work.L[458]*work.d[108];
  work.v[109] = work.L[459]*work.d[109];
  work.v[110] = work.L[460]*work.d[110];
  work.v[111] = work.L[461]*work.d[111];
  work.v[112] = work.L[462]*work.d[112];
  work.v[113] = work.L[463]*work.d[113];
  work.v[114] = work.L[464]*work.d[114];
  work.v[115] = work.L[465]*work.d[115];
  work.v[116] = work.L[466]*work.d[116];
  work.v[117] = work.L[467]*work.d[117];
  work.v[118] = work.L[468]*work.d[118];
  work.v[119] = work.L[469]*work.d[119];
  work.v[120] = work.L[470]*work.d[120];
  work.v[121] = work.L[471]*work.d[121];
  work.v[122] = work.L[472]*work.d[122];
  work.v[123] = work.L[473]*work.d[123];
  work.v[124] = work.L[474]*work.d[124];
  work.v[125] = work.L[475]*work.d[125];
  work.v[126] = work.L[476]*work.d[126];
  work.v[127] = work.L[477]*work.d[127];
  work.v[128] = work.L[478]*work.d[128];
  work.v[129] = work.L[479]*work.d[129];
  work.v[130] = work.L[480]*work.d[130];
  work.v[131] = work.L[481]*work.d[131];
  work.v[132] = work.L[482]*work.d[132];
  work.v[133] = work.L[483]*work.d[133];
  work.v[134] = work.L[484]*work.d[134];
  work.v[135] = work.L[485]*work.d[135];
  work.v[136] = work.L[486]*work.d[136];
  work.v[137] = work.L[487]*work.d[137];
  work.v[138] = work.L[488]*work.d[138];
  work.v[139] = work.L[489]*work.d[139];
  work.v[140] = work.L[490]*work.d[140];
  work.v[141] = work.L[491]*work.d[141];
  work.v[142] = work.L[492]*work.d[142];
  work.v[143] = work.L[493]*work.d[143];
  work.v[144] = work.L[494]*work.d[144];
  work.v[145] = work.L[495]*work.d[145];
  work.v[146] = work.L[496]*work.d[146];
  work.v[147] = 0-work.L[433]*work.v[83]-work.L[434]*work.v[84]-work.L[435]*work.v[85]-work.L[436]*work.v[86]-work.L[437]*work.v[87]-work.L[438]*work.v[88]-work.L[439]*work.v[89]-work.L[440]*work.v[90]-work.L[441]*work.v[91]-work.L[442]*work.v[92]-work.L[443]*work.v[93]-work.L[444]*work.v[94]-work.L[445]*work.v[95]-work.L[446]*work.v[96]-work.L[447]*work.v[97]-work.L[448]*work.v[98]-work.L[449]*work.v[99]-work.L[450]*work.v[100]-work.L[451]*work.v[101]-work.L[452]*work.v[102]-work.L[453]*work.v[103]-work.L[454]*work.v[104]-work.L[455]*work.v[105]-work.L[456]*work.v[106]-work.L[457]*work.v[107]-work.L[458]*work.v[108]-work.L[459]*work.v[109]-work.L[460]*work.v[110]-work.L[461]*work.v[111]-work.L[462]*work.v[112]-work.L[463]*work.v[113]-work.L[464]*work.v[114]-work.L[465]*work.v[115]-work.L[466]*work.v[116]-work.L[467]*work.v[117]-work.L[468]*work.v[118]-work.L[469]*work.v[119]-work.L[470]*work.v[120]-work.L[471]*work.v[121]-work.L[472]*work.v[122]-work.L[473]*work.v[123]-work.L[474]*work.v[124]-work.L[475]*work.v[125]-work.L[476]*work.v[126]-work.L[477]*work.v[127]-work.L[478]*work.v[128]-work.L[479]*work.v[129]-work.L[480]*work.v[130]-work.L[481]*work.v[131]-work.L[482]*work.v[132]-work.L[483]*work.v[133]-work.L[484]*work.v[134]-work.L[485]*work.v[135]-work.L[486]*work.v[136]-work.L[487]*work.v[137]-work.L[488]*work.v[138]-work.L[489]*work.v[139]-work.L[490]*work.v[140]-work.L[491]*work.v[141]-work.L[492]*work.v[142]-work.L[493]*work.v[143]-work.L[494]*work.v[144]-work.L[495]*work.v[145]-work.L[496]*work.v[146];
  work.d[147] = work.v[147];

  if (work.d[147] < 0)
    work.d[147] = settings.kkt_reg;
  else
    work.d[147] += settings.kkt_reg;
  work.d_inv[147] = 1/work.d[147];

  work.L[504] = (work.KKT[619]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143]-work.L[501]*work.v[144]-work.L[502]*work.v[145]-work.L[503]*work.v[146])*work.d_inv[147];
  work.v[70] = work.L[497]*work.d[70];
  work.v[141] = work.L[498]*work.d[141];
  work.v[142] = work.L[499]*work.d[142];
  work.v[143] = work.L[500]*work.d[143];
  work.v[144] = work.L[501]*work.d[144];
  work.v[145] = work.L[502]*work.d[145];
  work.v[146] = work.L[503]*work.d[146];
  work.v[147] = work.L[504]*work.d[147];
  work.v[148] = 0-work.L[497]*work.v[70]-work.L[498]*work.v[141]-work.L[499]*work.v[142]-work.L[500]*work.v[143]-work.L[501]*work.v[144]-work.L[502]*work.v[145]-work.L[503]*work.v[146]-work.L[504]*work.v[147];
  work.d[148] = work.v[148];

  if (work.d[148] > 0)
    work.d[148] = -settings.kkt_reg;
  else
    work.d[148] -= settings.kkt_reg;

  work.d_inv[148] = 1/work.d[148];

#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}

double check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;

  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[130]-1*work.d[65]*1;
  residual += temp*temp;

  temp = work.KKT[132]-1*work.d[66]*1;
  residual += temp*temp;

  temp = work.KKT[134]-1*work.d[67]*1;
  residual += temp*temp;

  temp = work.KKT[136]-1*work.d[68]*1;
  residual += temp*temp;

  temp = work.KKT[138]-1*work.d[69]*1;
  residual += temp*temp;

  temp = work.KKT[140]-1*work.d[70]*1;
  residual += temp*temp;

  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;

  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;

  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;

  temp = work.KKT[6]-1*work.d[3]*1;
  residual += temp*temp;

  temp = work.KKT[8]-1*work.d[4]*1;
  residual += temp*temp;

  temp = work.KKT[10]-1*work.d[5]*1;
  residual += temp*temp;

  temp = work.KKT[12]-1*work.d[6]*1;
  residual += temp*temp;

  temp = work.KKT[14]-1*work.d[7]*1;
  residual += temp*temp;

  temp = work.KKT[16]-1*work.d[8]*1;
  residual += temp*temp;

  temp = work.KKT[18]-1*work.d[9]*1;
  residual += temp*temp;

  temp = work.KKT[20]-1*work.d[10]*1;
  residual += temp*temp;

  temp = work.KKT[22]-1*work.d[11]*1;
  residual += temp*temp;

  temp = work.KKT[24]-1*work.d[12]*1;
  residual += temp*temp;

  temp = work.KKT[26]-1*work.d[13]*1;
  residual += temp*temp;

  temp = work.KKT[28]-1*work.d[14]*1;
  residual += temp*temp;

  temp = work.KKT[30]-1*work.d[15]*1;
  residual += temp*temp;

  temp = work.KKT[32]-1*work.d[16]*1;
  residual += temp*temp;

  temp = work.KKT[34]-1*work.d[17]*1;
  residual += temp*temp;

  temp = work.KKT[36]-1*work.d[18]*1;
  residual += temp*temp;

  temp = work.KKT[38]-1*work.d[19]*1;
  residual += temp*temp;

  temp = work.KKT[40]-1*work.d[20]*1;
  residual += temp*temp;

  temp = work.KKT[42]-1*work.d[21]*1;
  residual += temp*temp;

  temp = work.KKT[44]-1*work.d[22]*1;
  residual += temp*temp;

  temp = work.KKT[46]-1*work.d[23]*1;
  residual += temp*temp;

  temp = work.KKT[48]-1*work.d[24]*1;
  residual += temp*temp;

  temp = work.KKT[50]-1*work.d[25]*1;
  residual += temp*temp;

  temp = work.KKT[52]-1*work.d[26]*1;
  residual += temp*temp;

  temp = work.KKT[54]-1*work.d[27]*1;
  residual += temp*temp;

  temp = work.KKT[56]-1*work.d[28]*1;
  residual += temp*temp;

  temp = work.KKT[58]-1*work.d[29]*1;
  residual += temp*temp;

  temp = work.KKT[60]-1*work.d[30]*1;
  residual += temp*temp;

  temp = work.KKT[62]-1*work.d[31]*1;
  residual += temp*temp;

  temp = work.KKT[64]-1*work.d[32]*1;
  residual += temp*temp;

  temp = work.KKT[66]-1*work.d[33]*1;
  residual += temp*temp;

  temp = work.KKT[68]-1*work.d[34]*1;
  residual += temp*temp;

  temp = work.KKT[70]-1*work.d[35]*1;
  residual += temp*temp;

  temp = work.KKT[72]-1*work.d[36]*1;
  residual += temp*temp;

  temp = work.KKT[74]-1*work.d[37]*1;
  residual += temp*temp;

  temp = work.KKT[76]-1*work.d[38]*1;
  residual += temp*temp;

  temp = work.KKT[78]-1*work.d[39]*1;
  residual += temp*temp;

  temp = work.KKT[80]-1*work.d[40]*1;
  residual += temp*temp;

  temp = work.KKT[82]-1*work.d[41]*1;
  residual += temp*temp;

  temp = work.KKT[84]-1*work.d[42]*1;
  residual += temp*temp;

  temp = work.KKT[86]-1*work.d[43]*1;
  residual += temp*temp;

  temp = work.KKT[88]-1*work.d[44]*1;
  residual += temp*temp;

  temp = work.KKT[90]-1*work.d[45]*1;
  residual += temp*temp;

  temp = work.KKT[92]-1*work.d[46]*1;
  residual += temp*temp;

  temp = work.KKT[94]-1*work.d[47]*1;
  residual += temp*temp;

  temp = work.KKT[96]-1*work.d[48]*1;
  residual += temp*temp;

  temp = work.KKT[98]-1*work.d[49]*1;
  residual += temp*temp;

  temp = work.KKT[100]-1*work.d[50]*1;
  residual += temp*temp;

  temp = work.KKT[102]-1*work.d[51]*1;
  residual += temp*temp;

  temp = work.KKT[104]-1*work.d[52]*1;
  residual += temp*temp;

  temp = work.KKT[106]-1*work.d[53]*1;
  residual += temp*temp;

  temp = work.KKT[108]-1*work.d[54]*1;
  residual += temp*temp;

  temp = work.KKT[110]-1*work.d[55]*1;
  residual += temp*temp;

  temp = work.KKT[112]-1*work.d[56]*1;
  residual += temp*temp;

  temp = work.KKT[114]-1*work.d[57]*1;
  residual += temp*temp;

  temp = work.KKT[116]-1*work.d[58]*1;
  residual += temp*temp;

  temp = work.KKT[118]-1*work.d[59]*1;
  residual += temp*temp;

  temp = work.KKT[120]-1*work.d[60]*1;
  residual += temp*temp;

  temp = work.KKT[122]-1*work.d[61]*1;
  residual += temp*temp;

  temp = work.KKT[124]-1*work.d[62]*1;
  residual += temp*temp;

  temp = work.KKT[126]-1*work.d[63]*1;
  residual += temp*temp;

  temp = work.KKT[128]-1*work.d[64]*1;
  residual += temp*temp;

  temp = work.KKT[1]-work.L[14]*work.d[0]*1;
  residual += temp*temp;

  temp = work.KKT[3]-work.L[15]*work.d[1]*1;
  residual += temp*temp;

  temp = work.KKT[5]-work.L[16]*work.d[2]*1;
  residual += temp*temp;

  temp = work.KKT[7]-work.L[17]*work.d[3]*1;
  residual += temp*temp;

  temp = work.KKT[9]-work.L[18]*work.d[4]*1;
  residual += temp*temp;

  temp = work.KKT[11]-work.L[19]*work.d[5]*1;
  residual += temp*temp;

  temp = work.KKT[13]-work.L[20]*work.d[6]*1;
  residual += temp*temp;

  temp = work.KKT[15]-work.L[21]*work.d[7]*1;
  residual += temp*temp;

  temp = work.KKT[17]-work.L[22]*work.d[8]*1;
  residual += temp*temp;

  temp = work.KKT[19]-work.L[23]*work.d[9]*1;
  residual += temp*temp;

  temp = work.KKT[21]-work.L[24]*work.d[10]*1;
  residual += temp*temp;

  temp = work.KKT[23]-work.L[25]*work.d[11]*1;
  residual += temp*temp;

  temp = work.KKT[25]-work.L[26]*work.d[12]*1;
  residual += temp*temp;

  temp = work.KKT[27]-work.L[27]*work.d[13]*1;
  residual += temp*temp;

  temp = work.KKT[29]-work.L[28]*work.d[14]*1;
  residual += temp*temp;

  temp = work.KKT[31]-work.L[29]*work.d[15]*1;
  residual += temp*temp;

  temp = work.KKT[33]-work.L[30]*work.d[16]*1;
  residual += temp*temp;

  temp = work.KKT[35]-work.L[31]*work.d[17]*1;
  residual += temp*temp;

  temp = work.KKT[37]-work.L[32]*work.d[18]*1;
  residual += temp*temp;

  temp = work.KKT[39]-work.L[33]*work.d[19]*1;
  residual += temp*temp;

  temp = work.KKT[41]-work.L[34]*work.d[20]*1;
  residual += temp*temp;

  temp = work.KKT[43]-work.L[35]*work.d[21]*1;
  residual += temp*temp;

  temp = work.KKT[45]-work.L[36]*work.d[22]*1;
  residual += temp*temp;

  temp = work.KKT[47]-work.L[37]*work.d[23]*1;
  residual += temp*temp;

  temp = work.KKT[49]-work.L[38]*work.d[24]*1;
  residual += temp*temp;

  temp = work.KKT[51]-work.L[39]*work.d[25]*1;
  residual += temp*temp;

  temp = work.KKT[53]-work.L[40]*work.d[26]*1;
  residual += temp*temp;

  temp = work.KKT[55]-work.L[41]*work.d[27]*1;
  residual += temp*temp;

  temp = work.KKT[57]-work.L[42]*work.d[28]*1;
  residual += temp*temp;

  temp = work.KKT[59]-work.L[43]*work.d[29]*1;
  residual += temp*temp;

  temp = work.KKT[61]-work.L[44]*work.d[30]*1;
  residual += temp*temp;

  temp = work.KKT[63]-work.L[45]*work.d[31]*1;
  residual += temp*temp;

  temp = work.KKT[65]-work.L[46]*work.d[32]*1;
  residual += temp*temp;

  temp = work.KKT[67]-work.L[47]*work.d[33]*1;
  residual += temp*temp;

  temp = work.KKT[69]-work.L[48]*work.d[34]*1;
  residual += temp*temp;

  temp = work.KKT[71]-work.L[49]*work.d[35]*1;
  residual += temp*temp;

  temp = work.KKT[73]-work.L[50]*work.d[36]*1;
  residual += temp*temp;

  temp = work.KKT[75]-work.L[51]*work.d[37]*1;
  residual += temp*temp;

  temp = work.KKT[77]-work.L[52]*work.d[38]*1;
  residual += temp*temp;

  temp = work.KKT[79]-work.L[53]*work.d[39]*1;
  residual += temp*temp;

  temp = work.KKT[81]-work.L[54]*work.d[40]*1;
  residual += temp*temp;

  temp = work.KKT[83]-work.L[55]*work.d[41]*1;
  residual += temp*temp;

  temp = work.KKT[85]-work.L[56]*work.d[42]*1;
  residual += temp*temp;

  temp = work.KKT[87]-work.L[57]*work.d[43]*1;
  residual += temp*temp;

  temp = work.KKT[89]-work.L[58]*work.d[44]*1;
  residual += temp*temp;

  temp = work.KKT[91]-work.L[59]*work.d[45]*1;
  residual += temp*temp;

  temp = work.KKT[93]-work.L[60]*work.d[46]*1;
  residual += temp*temp;

  temp = work.KKT[95]-work.L[61]*work.d[47]*1;
  residual += temp*temp;

  temp = work.KKT[97]-work.L[62]*work.d[48]*1;
  residual += temp*temp;

  temp = work.KKT[99]-work.L[63]*work.d[49]*1;
  residual += temp*temp;

  temp = work.KKT[101]-work.L[64]*work.d[50]*1;
  residual += temp*temp;

  temp = work.KKT[103]-work.L[0]*work.d[51]*1;
  residual += temp*temp;

  temp = work.KKT[105]-work.L[2]*work.d[52]*1;
  residual += temp*temp;

  temp = work.KKT[107]-work.L[4]*work.d[53]*1;
  residual += temp*temp;

  temp = work.KKT[109]-work.L[6]*work.d[54]*1;
  residual += temp*temp;

  temp = work.KKT[111]-work.L[8]*work.d[55]*1;
  residual += temp*temp;

  temp = work.KKT[113]-work.L[10]*work.d[56]*1;
  residual += temp*temp;

  temp = work.KKT[115]-work.L[12]*work.d[57]*1;
  residual += temp*temp;

  temp = work.KKT[117]-work.L[1]*work.d[58]*1;
  residual += temp*temp;

  temp = work.KKT[119]-work.L[3]*work.d[59]*1;
  residual += temp*temp;

  temp = work.KKT[121]-work.L[5]*work.d[60]*1;
  residual += temp*temp;

  temp = work.KKT[123]-work.L[7]*work.d[61]*1;
  residual += temp*temp;

  temp = work.KKT[125]-work.L[9]*work.d[62]*1;
  residual += temp*temp;

  temp = work.KKT[127]-work.L[11]*work.d[63]*1;
  residual += temp*temp;

  temp = work.KKT[129]-work.L[13]*work.d[64]*1;
  residual += temp*temp;

  temp = work.KKT[170]-work.L[14]*work.d[0]*work.L[14]-1*work.d[85]*1;
  residual += temp*temp;

  temp = work.KKT[178]-work.L[15]*work.d[1]*work.L[15]-1*work.d[86]*1;
  residual += temp*temp;

  temp = work.KKT[186]-work.L[16]*work.d[2]*work.L[16]-1*work.d[87]*1;
  residual += temp*temp;

  temp = work.KKT[194]-work.L[17]*work.d[3]*work.L[17]-1*work.d[88]*1;
  residual += temp*temp;

  temp = work.KKT[202]-work.L[18]*work.d[4]*work.L[18]-1*work.d[89]*1;
  residual += temp*temp;

  temp = work.KKT[210]-work.L[19]*work.d[5]*work.L[19]-1*work.d[90]*1;
  residual += temp*temp;

  temp = work.KKT[218]-work.L[20]*work.d[6]*work.L[20]-1*work.d[91]*1;
  residual += temp*temp;

  temp = work.KKT[226]-work.L[21]*work.d[7]*work.L[21]-1*work.d[92]*1;
  residual += temp*temp;

  temp = work.KKT[234]-work.L[22]*work.d[8]*work.L[22]-1*work.d[93]*1;
  residual += temp*temp;

  temp = work.KKT[242]-work.L[23]*work.d[9]*work.L[23]-1*work.d[94]*1;
  residual += temp*temp;

  temp = work.KKT[250]-work.L[24]*work.d[10]*work.L[24]-1*work.d[95]*1;
  residual += temp*temp;

  temp = work.KKT[258]-work.L[25]*work.d[11]*work.L[25]-1*work.d[96]*1;
  residual += temp*temp;

  temp = work.KKT[266]-work.L[26]*work.d[12]*work.L[26]-1*work.d[97]*1;
  residual += temp*temp;

  temp = work.KKT[274]-work.L[27]*work.d[13]*work.L[27]-1*work.d[98]*1;
  residual += temp*temp;

  temp = work.KKT[282]-work.L[28]*work.d[14]*work.L[28]-1*work.d[99]*1;
  residual += temp*temp;

  temp = work.KKT[290]-work.L[29]*work.d[15]*work.L[29]-1*work.d[100]*1;
  residual += temp*temp;

  temp = work.KKT[298]-work.L[30]*work.d[16]*work.L[30]-1*work.d[101]*1;
  residual += temp*temp;

  temp = work.KKT[306]-work.L[31]*work.d[17]*work.L[31]-1*work.d[102]*1;
  residual += temp*temp;

  temp = work.KKT[314]-work.L[32]*work.d[18]*work.L[32]-1*work.d[103]*1;
  residual += temp*temp;

  temp = work.KKT[322]-work.L[33]*work.d[19]*work.L[33]-1*work.d[104]*1;
  residual += temp*temp;

  temp = work.KKT[330]-work.L[34]*work.d[20]*work.L[34]-1*work.d[105]*1;
  residual += temp*temp;

  temp = work.KKT[338]-work.L[35]*work.d[21]*work.L[35]-1*work.d[106]*1;
  residual += temp*temp;

  temp = work.KKT[346]-work.L[36]*work.d[22]*work.L[36]-1*work.d[107]*1;
  residual += temp*temp;

  temp = work.KKT[354]-work.L[37]*work.d[23]*work.L[37]-1*work.d[108]*1;
  residual += temp*temp;

  temp = work.KKT[362]-work.L[38]*work.d[24]*work.L[38]-1*work.d[109]*1;
  residual += temp*temp;

  temp = work.KKT[370]-work.L[39]*work.d[25]*work.L[39]-1*work.d[110]*1;
  residual += temp*temp;

  temp = work.KKT[378]-work.L[40]*work.d[26]*work.L[40]-1*work.d[111]*1;
  residual += temp*temp;

  temp = work.KKT[386]-work.L[41]*work.d[27]*work.L[41]-1*work.d[112]*1;
  residual += temp*temp;

  temp = work.KKT[394]-work.L[42]*work.d[28]*work.L[42]-1*work.d[113]*1;
  residual += temp*temp;

  temp = work.KKT[402]-work.L[43]*work.d[29]*work.L[43]-1*work.d[114]*1;
  residual += temp*temp;

  temp = work.KKT[410]-work.L[44]*work.d[30]*work.L[44]-1*work.d[115]*1;
  residual += temp*temp;

  temp = work.KKT[418]-work.L[45]*work.d[31]*work.L[45]-1*work.d[116]*1;
  residual += temp*temp;

  temp = work.KKT[426]-work.L[46]*work.d[32]*work.L[46]-1*work.d[117]*1;
  residual += temp*temp;

  temp = work.KKT[434]-work.L[47]*work.d[33]*work.L[47]-1*work.d[118]*1;
  residual += temp*temp;

  temp = work.KKT[442]-work.L[48]*work.d[34]*work.L[48]-1*work.d[119]*1;
  residual += temp*temp;

  temp = work.KKT[450]-work.L[49]*work.d[35]*work.L[49]-1*work.d[120]*1;
  residual += temp*temp;

  temp = work.KKT[458]-work.L[50]*work.d[36]*work.L[50]-1*work.d[121]*1;
  residual += temp*temp;

  temp = work.KKT[466]-work.L[51]*work.d[37]*work.L[51]-1*work.d[122]*1;
  residual += temp*temp;

  temp = work.KKT[474]-work.L[52]*work.d[38]*work.L[52]-1*work.d[123]*1;
  residual += temp*temp;

  temp = work.KKT[482]-work.L[53]*work.d[39]*work.L[53]-1*work.d[124]*1;
  residual += temp*temp;

  temp = work.KKT[490]-work.L[54]*work.d[40]*work.L[54]-1*work.d[125]*1;
  residual += temp*temp;

  temp = work.KKT[498]-work.L[55]*work.d[41]*work.L[55]-1*work.d[126]*1;
  residual += temp*temp;

  temp = work.KKT[506]-work.L[56]*work.d[42]*work.L[56]-1*work.d[127]*1;
  residual += temp*temp;

  temp = work.KKT[514]-work.L[57]*work.d[43]*work.L[57]-1*work.d[128]*1;
  residual += temp*temp;

  temp = work.KKT[522]-work.L[58]*work.d[44]*work.L[58]-1*work.d[129]*1;
  residual += temp*temp;

  temp = work.KKT[530]-work.L[59]*work.d[45]*work.L[59]-1*work.d[130]*1;
  residual += temp*temp;

  temp = work.KKT[538]-work.L[60]*work.d[46]*work.L[60]-1*work.d[131]*1;
  residual += temp*temp;

  temp = work.KKT[546]-work.L[61]*work.d[47]*work.L[61]-1*work.d[132]*1;
  residual += temp*temp;

  temp = work.KKT[554]-work.L[62]*work.d[48]*work.L[62]-1*work.d[133]*1;
  residual += temp*temp;

  temp = work.KKT[562]-work.L[63]*work.d[49]*work.L[63]-1*work.d[134]*1;
  residual += temp*temp;

  temp = work.KKT[570]-work.L[64]*work.d[50]*work.L[64]-1*work.d[135]*1;
  residual += temp*temp;

  temp = work.KKT[142]-work.L[0]*work.d[51]*work.L[0]-1*work.d[71]*1;
  residual += temp*temp;

  temp = work.KKT[146]-work.L[2]*work.d[52]*work.L[2]-1*work.d[73]*1;
  residual += temp*temp;

  temp = work.KKT[150]-work.L[4]*work.d[53]*work.L[4]-1*work.d[75]*1;
  residual += temp*temp;

  temp = work.KKT[154]-work.L[6]*work.d[54]*work.L[6]-1*work.d[77]*1;
  residual += temp*temp;

  temp = work.KKT[158]-work.L[8]*work.d[55]*work.L[8]-1*work.d[79]*1;
  residual += temp*temp;

  temp = work.KKT[162]-work.L[10]*work.d[56]*work.L[10]-1*work.d[81]*1;
  residual += temp*temp;

  temp = work.KKT[166]-work.L[12]*work.d[57]*work.L[12]-1*work.d[83]*1;
  residual += temp*temp;

  temp = work.KKT[144]-work.L[1]*work.d[58]*work.L[1]-1*work.d[72]*1;
  residual += temp*temp;

  temp = work.KKT[148]-work.L[3]*work.d[59]*work.L[3]-1*work.d[74]*1;
  residual += temp*temp;

  temp = work.KKT[152]-work.L[5]*work.d[60]*work.L[5]-1*work.d[76]*1;
  residual += temp*temp;

  temp = work.KKT[156]-work.L[7]*work.d[61]*work.L[7]-1*work.d[78]*1;
  residual += temp*temp;

  temp = work.KKT[160]-work.L[9]*work.d[62]*work.L[9]-1*work.d[80]*1;
  residual += temp*temp;

  temp = work.KKT[164]-work.L[11]*work.d[63]*work.L[11]-1*work.d[82]*1;
  residual += temp*temp;

  temp = work.KKT[168]-work.L[13]*work.d[64]*work.L[13]-1*work.d[84]*1;
  residual += temp*temp;

  temp = work.KKT[171]-1*work.d[85]*work.L[72];
  residual += temp*temp;

  temp = work.KKT[172]-1*work.d[85]*work.L[130];
  residual += temp*temp;

  temp = work.KKT[173]-1*work.d[85]*work.L[189];
  residual += temp*temp;

  temp = work.KKT[174]-1*work.d[85]*work.L[249];
  residual += temp*temp;

  temp = work.KKT[175]-1*work.d[85]*work.L[310];
  residual += temp*temp;

  temp = work.KKT[176]-1*work.d[85]*work.L[372];
  residual += temp*temp;

  temp = work.KKT[177]-1*work.d[85]*work.L[435];
  residual += temp*temp;

  temp = work.KKT[179]-1*work.d[86]*work.L[73];
  residual += temp*temp;

  temp = work.KKT[180]-1*work.d[86]*work.L[131];
  residual += temp*temp;

  temp = work.KKT[181]-1*work.d[86]*work.L[190];
  residual += temp*temp;

  temp = work.KKT[182]-1*work.d[86]*work.L[250];
  residual += temp*temp;

  temp = work.KKT[183]-1*work.d[86]*work.L[311];
  residual += temp*temp;

  temp = work.KKT[184]-1*work.d[86]*work.L[373];
  residual += temp*temp;

  temp = work.KKT[185]-1*work.d[86]*work.L[436];
  residual += temp*temp;

  temp = work.KKT[187]-1*work.d[87]*work.L[74];
  residual += temp*temp;

  temp = work.KKT[188]-1*work.d[87]*work.L[132];
  residual += temp*temp;

  temp = work.KKT[189]-1*work.d[87]*work.L[191];
  residual += temp*temp;

  temp = work.KKT[190]-1*work.d[87]*work.L[251];
  residual += temp*temp;

  temp = work.KKT[191]-1*work.d[87]*work.L[312];
  residual += temp*temp;

  temp = work.KKT[192]-1*work.d[87]*work.L[374];
  residual += temp*temp;

  temp = work.KKT[193]-1*work.d[87]*work.L[437];
  residual += temp*temp;

  temp = work.KKT[195]-1*work.d[88]*work.L[75];
  residual += temp*temp;

  temp = work.KKT[196]-1*work.d[88]*work.L[133];
  residual += temp*temp;

  temp = work.KKT[197]-1*work.d[88]*work.L[192];
  residual += temp*temp;

  temp = work.KKT[198]-1*work.d[88]*work.L[252];
  residual += temp*temp;

  temp = work.KKT[199]-1*work.d[88]*work.L[313];
  residual += temp*temp;

  temp = work.KKT[200]-1*work.d[88]*work.L[375];
  residual += temp*temp;

  temp = work.KKT[201]-1*work.d[88]*work.L[438];
  residual += temp*temp;

  temp = work.KKT[203]-1*work.d[89]*work.L[76];
  residual += temp*temp;

  temp = work.KKT[204]-1*work.d[89]*work.L[134];
  residual += temp*temp;

  temp = work.KKT[205]-1*work.d[89]*work.L[193];
  residual += temp*temp;

  temp = work.KKT[206]-1*work.d[89]*work.L[253];
  residual += temp*temp;

  temp = work.KKT[207]-1*work.d[89]*work.L[314];
  residual += temp*temp;

  temp = work.KKT[208]-1*work.d[89]*work.L[376];
  residual += temp*temp;

  temp = work.KKT[209]-1*work.d[89]*work.L[439];
  residual += temp*temp;

  temp = work.KKT[211]-1*work.d[90]*work.L[77];
  residual += temp*temp;

  temp = work.KKT[212]-1*work.d[90]*work.L[135];
  residual += temp*temp;

  temp = work.KKT[213]-1*work.d[90]*work.L[194];
  residual += temp*temp;

  temp = work.KKT[214]-1*work.d[90]*work.L[254];
  residual += temp*temp;

  temp = work.KKT[215]-1*work.d[90]*work.L[315];
  residual += temp*temp;

  temp = work.KKT[216]-1*work.d[90]*work.L[377];
  residual += temp*temp;

  temp = work.KKT[217]-1*work.d[90]*work.L[440];
  residual += temp*temp;

  temp = work.KKT[219]-1*work.d[91]*work.L[78];
  residual += temp*temp;

  temp = work.KKT[220]-1*work.d[91]*work.L[136];
  residual += temp*temp;

  temp = work.KKT[221]-1*work.d[91]*work.L[195];
  residual += temp*temp;

  temp = work.KKT[222]-1*work.d[91]*work.L[255];
  residual += temp*temp;

  temp = work.KKT[223]-1*work.d[91]*work.L[316];
  residual += temp*temp;

  temp = work.KKT[224]-1*work.d[91]*work.L[378];
  residual += temp*temp;

  temp = work.KKT[225]-1*work.d[91]*work.L[441];
  residual += temp*temp;

  temp = work.KKT[227]-1*work.d[92]*work.L[79];
  residual += temp*temp;

  temp = work.KKT[228]-1*work.d[92]*work.L[137];
  residual += temp*temp;

  temp = work.KKT[229]-1*work.d[92]*work.L[196];
  residual += temp*temp;

  temp = work.KKT[230]-1*work.d[92]*work.L[256];
  residual += temp*temp;

  temp = work.KKT[231]-1*work.d[92]*work.L[317];
  residual += temp*temp;

  temp = work.KKT[232]-1*work.d[92]*work.L[379];
  residual += temp*temp;

  temp = work.KKT[233]-1*work.d[92]*work.L[442];
  residual += temp*temp;

  temp = work.KKT[235]-1*work.d[93]*work.L[80];
  residual += temp*temp;

  temp = work.KKT[236]-1*work.d[93]*work.L[138];
  residual += temp*temp;

  temp = work.KKT[237]-1*work.d[93]*work.L[197];
  residual += temp*temp;

  temp = work.KKT[238]-1*work.d[93]*work.L[257];
  residual += temp*temp;

  temp = work.KKT[239]-1*work.d[93]*work.L[318];
  residual += temp*temp;

  temp = work.KKT[240]-1*work.d[93]*work.L[380];
  residual += temp*temp;

  temp = work.KKT[241]-1*work.d[93]*work.L[443];
  residual += temp*temp;

  temp = work.KKT[243]-1*work.d[94]*work.L[81];
  residual += temp*temp;

  temp = work.KKT[244]-1*work.d[94]*work.L[139];
  residual += temp*temp;

  temp = work.KKT[245]-1*work.d[94]*work.L[198];
  residual += temp*temp;

  temp = work.KKT[246]-1*work.d[94]*work.L[258];
  residual += temp*temp;

  temp = work.KKT[247]-1*work.d[94]*work.L[319];
  residual += temp*temp;

  temp = work.KKT[248]-1*work.d[94]*work.L[381];
  residual += temp*temp;

  temp = work.KKT[249]-1*work.d[94]*work.L[444];
  residual += temp*temp;

  temp = work.KKT[251]-1*work.d[95]*work.L[82];
  residual += temp*temp;

  temp = work.KKT[252]-1*work.d[95]*work.L[140];
  residual += temp*temp;

  temp = work.KKT[253]-1*work.d[95]*work.L[199];
  residual += temp*temp;

  temp = work.KKT[254]-1*work.d[95]*work.L[259];
  residual += temp*temp;

  temp = work.KKT[255]-1*work.d[95]*work.L[320];
  residual += temp*temp;

  temp = work.KKT[256]-1*work.d[95]*work.L[382];
  residual += temp*temp;

  temp = work.KKT[257]-1*work.d[95]*work.L[445];
  residual += temp*temp;

  temp = work.KKT[259]-1*work.d[96]*work.L[83];
  residual += temp*temp;

  temp = work.KKT[260]-1*work.d[96]*work.L[141];
  residual += temp*temp;

  temp = work.KKT[261]-1*work.d[96]*work.L[200];
  residual += temp*temp;

  temp = work.KKT[262]-1*work.d[96]*work.L[260];
  residual += temp*temp;

  temp = work.KKT[263]-1*work.d[96]*work.L[321];
  residual += temp*temp;

  temp = work.KKT[264]-1*work.d[96]*work.L[383];
  residual += temp*temp;

  temp = work.KKT[265]-1*work.d[96]*work.L[446];
  residual += temp*temp;

  temp = work.KKT[267]-1*work.d[97]*work.L[84];
  residual += temp*temp;

  temp = work.KKT[268]-1*work.d[97]*work.L[142];
  residual += temp*temp;

  temp = work.KKT[269]-1*work.d[97]*work.L[201];
  residual += temp*temp;

  temp = work.KKT[270]-1*work.d[97]*work.L[261];
  residual += temp*temp;

  temp = work.KKT[271]-1*work.d[97]*work.L[322];
  residual += temp*temp;

  temp = work.KKT[272]-1*work.d[97]*work.L[384];
  residual += temp*temp;

  temp = work.KKT[273]-1*work.d[97]*work.L[447];
  residual += temp*temp;

  temp = work.KKT[275]-1*work.d[98]*work.L[85];
  residual += temp*temp;

  temp = work.KKT[276]-1*work.d[98]*work.L[143];
  residual += temp*temp;

  temp = work.KKT[277]-1*work.d[98]*work.L[202];
  residual += temp*temp;

  temp = work.KKT[278]-1*work.d[98]*work.L[262];
  residual += temp*temp;

  temp = work.KKT[279]-1*work.d[98]*work.L[323];
  residual += temp*temp;

  temp = work.KKT[280]-1*work.d[98]*work.L[385];
  residual += temp*temp;

  temp = work.KKT[281]-1*work.d[98]*work.L[448];
  residual += temp*temp;

  temp = work.KKT[283]-1*work.d[99]*work.L[86];
  residual += temp*temp;

  temp = work.KKT[284]-1*work.d[99]*work.L[144];
  residual += temp*temp;

  temp = work.KKT[285]-1*work.d[99]*work.L[203];
  residual += temp*temp;

  temp = work.KKT[286]-1*work.d[99]*work.L[263];
  residual += temp*temp;

  temp = work.KKT[287]-1*work.d[99]*work.L[324];
  residual += temp*temp;

  temp = work.KKT[288]-1*work.d[99]*work.L[386];
  residual += temp*temp;

  temp = work.KKT[289]-1*work.d[99]*work.L[449];
  residual += temp*temp;

  temp = work.KKT[291]-1*work.d[100]*work.L[87];
  residual += temp*temp;

  temp = work.KKT[292]-1*work.d[100]*work.L[145];
  residual += temp*temp;

  temp = work.KKT[293]-1*work.d[100]*work.L[204];
  residual += temp*temp;

  temp = work.KKT[294]-1*work.d[100]*work.L[264];
  residual += temp*temp;

  temp = work.KKT[295]-1*work.d[100]*work.L[325];
  residual += temp*temp;

  temp = work.KKT[296]-1*work.d[100]*work.L[387];
  residual += temp*temp;

  temp = work.KKT[297]-1*work.d[100]*work.L[450];
  residual += temp*temp;

  temp = work.KKT[299]-1*work.d[101]*work.L[88];
  residual += temp*temp;

  temp = work.KKT[300]-1*work.d[101]*work.L[146];
  residual += temp*temp;

  temp = work.KKT[301]-1*work.d[101]*work.L[205];
  residual += temp*temp;

  temp = work.KKT[302]-1*work.d[101]*work.L[265];
  residual += temp*temp;

  temp = work.KKT[303]-1*work.d[101]*work.L[326];
  residual += temp*temp;

  temp = work.KKT[304]-1*work.d[101]*work.L[388];
  residual += temp*temp;

  temp = work.KKT[305]-1*work.d[101]*work.L[451];
  residual += temp*temp;

  temp = work.KKT[307]-1*work.d[102]*work.L[89];
  residual += temp*temp;

  temp = work.KKT[308]-1*work.d[102]*work.L[147];
  residual += temp*temp;

  temp = work.KKT[309]-1*work.d[102]*work.L[206];
  residual += temp*temp;

  temp = work.KKT[310]-1*work.d[102]*work.L[266];
  residual += temp*temp;

  temp = work.KKT[311]-1*work.d[102]*work.L[327];
  residual += temp*temp;

  temp = work.KKT[312]-1*work.d[102]*work.L[389];
  residual += temp*temp;

  temp = work.KKT[313]-1*work.d[102]*work.L[452];
  residual += temp*temp;

  temp = work.KKT[315]-1*work.d[103]*work.L[90];
  residual += temp*temp;

  temp = work.KKT[316]-1*work.d[103]*work.L[148];
  residual += temp*temp;

  temp = work.KKT[317]-1*work.d[103]*work.L[207];
  residual += temp*temp;

  temp = work.KKT[318]-1*work.d[103]*work.L[267];
  residual += temp*temp;

  temp = work.KKT[319]-1*work.d[103]*work.L[328];
  residual += temp*temp;

  temp = work.KKT[320]-1*work.d[103]*work.L[390];
  residual += temp*temp;

  temp = work.KKT[321]-1*work.d[103]*work.L[453];
  residual += temp*temp;

  temp = work.KKT[323]-1*work.d[104]*work.L[91];
  residual += temp*temp;

  temp = work.KKT[324]-1*work.d[104]*work.L[149];
  residual += temp*temp;

  temp = work.KKT[325]-1*work.d[104]*work.L[208];
  residual += temp*temp;

  temp = work.KKT[326]-1*work.d[104]*work.L[268];
  residual += temp*temp;

  temp = work.KKT[327]-1*work.d[104]*work.L[329];
  residual += temp*temp;

  temp = work.KKT[328]-1*work.d[104]*work.L[391];
  residual += temp*temp;

  temp = work.KKT[329]-1*work.d[104]*work.L[454];
  residual += temp*temp;

  temp = work.KKT[331]-1*work.d[105]*work.L[92];
  residual += temp*temp;

  temp = work.KKT[332]-1*work.d[105]*work.L[150];
  residual += temp*temp;

  temp = work.KKT[333]-1*work.d[105]*work.L[209];
  residual += temp*temp;

  temp = work.KKT[334]-1*work.d[105]*work.L[269];
  residual += temp*temp;

  temp = work.KKT[335]-1*work.d[105]*work.L[330];
  residual += temp*temp;

  temp = work.KKT[336]-1*work.d[105]*work.L[392];
  residual += temp*temp;

  temp = work.KKT[337]-1*work.d[105]*work.L[455];
  residual += temp*temp;

  temp = work.KKT[339]-1*work.d[106]*work.L[93];
  residual += temp*temp;

  temp = work.KKT[340]-1*work.d[106]*work.L[151];
  residual += temp*temp;

  temp = work.KKT[341]-1*work.d[106]*work.L[210];
  residual += temp*temp;

  temp = work.KKT[342]-1*work.d[106]*work.L[270];
  residual += temp*temp;

  temp = work.KKT[343]-1*work.d[106]*work.L[331];
  residual += temp*temp;

  temp = work.KKT[344]-1*work.d[106]*work.L[393];
  residual += temp*temp;

  temp = work.KKT[345]-1*work.d[106]*work.L[456];
  residual += temp*temp;

  temp = work.KKT[347]-1*work.d[107]*work.L[94];
  residual += temp*temp;

  temp = work.KKT[348]-1*work.d[107]*work.L[152];
  residual += temp*temp;

  temp = work.KKT[349]-1*work.d[107]*work.L[211];
  residual += temp*temp;

  temp = work.KKT[350]-1*work.d[107]*work.L[271];
  residual += temp*temp;

  temp = work.KKT[351]-1*work.d[107]*work.L[332];
  residual += temp*temp;

  temp = work.KKT[352]-1*work.d[107]*work.L[394];
  residual += temp*temp;

  temp = work.KKT[353]-1*work.d[107]*work.L[457];
  residual += temp*temp;

  temp = work.KKT[355]-1*work.d[108]*work.L[95];
  residual += temp*temp;

  temp = work.KKT[356]-1*work.d[108]*work.L[153];
  residual += temp*temp;

  temp = work.KKT[357]-1*work.d[108]*work.L[212];
  residual += temp*temp;

  temp = work.KKT[358]-1*work.d[108]*work.L[272];
  residual += temp*temp;

  temp = work.KKT[359]-1*work.d[108]*work.L[333];
  residual += temp*temp;

  temp = work.KKT[360]-1*work.d[108]*work.L[395];
  residual += temp*temp;

  temp = work.KKT[361]-1*work.d[108]*work.L[458];
  residual += temp*temp;

  temp = work.KKT[363]-1*work.d[109]*work.L[96];
  residual += temp*temp;

  temp = work.KKT[364]-1*work.d[109]*work.L[154];
  residual += temp*temp;

  temp = work.KKT[365]-1*work.d[109]*work.L[213];
  residual += temp*temp;

  temp = work.KKT[366]-1*work.d[109]*work.L[273];
  residual += temp*temp;

  temp = work.KKT[367]-1*work.d[109]*work.L[334];
  residual += temp*temp;

  temp = work.KKT[368]-1*work.d[109]*work.L[396];
  residual += temp*temp;

  temp = work.KKT[369]-1*work.d[109]*work.L[459];
  residual += temp*temp;

  temp = work.KKT[371]-1*work.d[110]*work.L[97];
  residual += temp*temp;

  temp = work.KKT[372]-1*work.d[110]*work.L[155];
  residual += temp*temp;

  temp = work.KKT[373]-1*work.d[110]*work.L[214];
  residual += temp*temp;

  temp = work.KKT[374]-1*work.d[110]*work.L[274];
  residual += temp*temp;

  temp = work.KKT[375]-1*work.d[110]*work.L[335];
  residual += temp*temp;

  temp = work.KKT[376]-1*work.d[110]*work.L[397];
  residual += temp*temp;

  temp = work.KKT[377]-1*work.d[110]*work.L[460];
  residual += temp*temp;

  temp = work.KKT[379]-1*work.d[111]*work.L[98];
  residual += temp*temp;

  temp = work.KKT[380]-1*work.d[111]*work.L[156];
  residual += temp*temp;

  temp = work.KKT[381]-1*work.d[111]*work.L[215];
  residual += temp*temp;

  temp = work.KKT[382]-1*work.d[111]*work.L[275];
  residual += temp*temp;

  temp = work.KKT[383]-1*work.d[111]*work.L[336];
  residual += temp*temp;

  temp = work.KKT[384]-1*work.d[111]*work.L[398];
  residual += temp*temp;

  temp = work.KKT[385]-1*work.d[111]*work.L[461];
  residual += temp*temp;

  temp = work.KKT[387]-1*work.d[112]*work.L[99];
  residual += temp*temp;

  temp = work.KKT[388]-1*work.d[112]*work.L[157];
  residual += temp*temp;

  temp = work.KKT[389]-1*work.d[112]*work.L[216];
  residual += temp*temp;

  temp = work.KKT[390]-1*work.d[112]*work.L[276];
  residual += temp*temp;

  temp = work.KKT[391]-1*work.d[112]*work.L[337];
  residual += temp*temp;

  temp = work.KKT[392]-1*work.d[112]*work.L[399];
  residual += temp*temp;

  temp = work.KKT[393]-1*work.d[112]*work.L[462];
  residual += temp*temp;

  temp = work.KKT[395]-1*work.d[113]*work.L[100];
  residual += temp*temp;

  temp = work.KKT[396]-1*work.d[113]*work.L[158];
  residual += temp*temp;

  temp = work.KKT[397]-1*work.d[113]*work.L[217];
  residual += temp*temp;

  temp = work.KKT[398]-1*work.d[113]*work.L[277];
  residual += temp*temp;

  temp = work.KKT[399]-1*work.d[113]*work.L[338];
  residual += temp*temp;

  temp = work.KKT[400]-1*work.d[113]*work.L[400];
  residual += temp*temp;

  temp = work.KKT[401]-1*work.d[113]*work.L[463];
  residual += temp*temp;

  temp = work.KKT[403]-1*work.d[114]*work.L[101];
  residual += temp*temp;

  temp = work.KKT[404]-1*work.d[114]*work.L[159];
  residual += temp*temp;

  temp = work.KKT[405]-1*work.d[114]*work.L[218];
  residual += temp*temp;

  temp = work.KKT[406]-1*work.d[114]*work.L[278];
  residual += temp*temp;

  temp = work.KKT[407]-1*work.d[114]*work.L[339];
  residual += temp*temp;

  temp = work.KKT[408]-1*work.d[114]*work.L[401];
  residual += temp*temp;

  temp = work.KKT[409]-1*work.d[114]*work.L[464];
  residual += temp*temp;

  temp = work.KKT[411]-1*work.d[115]*work.L[102];
  residual += temp*temp;

  temp = work.KKT[412]-1*work.d[115]*work.L[160];
  residual += temp*temp;

  temp = work.KKT[413]-1*work.d[115]*work.L[219];
  residual += temp*temp;

  temp = work.KKT[414]-1*work.d[115]*work.L[279];
  residual += temp*temp;

  temp = work.KKT[415]-1*work.d[115]*work.L[340];
  residual += temp*temp;

  temp = work.KKT[416]-1*work.d[115]*work.L[402];
  residual += temp*temp;

  temp = work.KKT[417]-1*work.d[115]*work.L[465];
  residual += temp*temp;

  temp = work.KKT[419]-1*work.d[116]*work.L[103];
  residual += temp*temp;

  temp = work.KKT[420]-1*work.d[116]*work.L[161];
  residual += temp*temp;

  temp = work.KKT[421]-1*work.d[116]*work.L[220];
  residual += temp*temp;

  temp = work.KKT[422]-1*work.d[116]*work.L[280];
  residual += temp*temp;

  temp = work.KKT[423]-1*work.d[116]*work.L[341];
  residual += temp*temp;

  temp = work.KKT[424]-1*work.d[116]*work.L[403];
  residual += temp*temp;

  temp = work.KKT[425]-1*work.d[116]*work.L[466];
  residual += temp*temp;

  temp = work.KKT[427]-1*work.d[117]*work.L[104];
  residual += temp*temp;

  temp = work.KKT[428]-1*work.d[117]*work.L[162];
  residual += temp*temp;

  temp = work.KKT[429]-1*work.d[117]*work.L[221];
  residual += temp*temp;

  temp = work.KKT[430]-1*work.d[117]*work.L[281];
  residual += temp*temp;

  temp = work.KKT[431]-1*work.d[117]*work.L[342];
  residual += temp*temp;

  temp = work.KKT[432]-1*work.d[117]*work.L[404];
  residual += temp*temp;

  temp = work.KKT[433]-1*work.d[117]*work.L[467];
  residual += temp*temp;

  temp = work.KKT[435]-1*work.d[118]*work.L[105];
  residual += temp*temp;

  temp = work.KKT[436]-1*work.d[118]*work.L[163];
  residual += temp*temp;

  temp = work.KKT[437]-1*work.d[118]*work.L[222];
  residual += temp*temp;

  temp = work.KKT[438]-1*work.d[118]*work.L[282];
  residual += temp*temp;

  temp = work.KKT[439]-1*work.d[118]*work.L[343];
  residual += temp*temp;

  temp = work.KKT[440]-1*work.d[118]*work.L[405];
  residual += temp*temp;

  temp = work.KKT[441]-1*work.d[118]*work.L[468];
  residual += temp*temp;

  temp = work.KKT[443]-1*work.d[119]*work.L[106];
  residual += temp*temp;

  temp = work.KKT[444]-1*work.d[119]*work.L[164];
  residual += temp*temp;

  temp = work.KKT[445]-1*work.d[119]*work.L[223];
  residual += temp*temp;

  temp = work.KKT[446]-1*work.d[119]*work.L[283];
  residual += temp*temp;

  temp = work.KKT[447]-1*work.d[119]*work.L[344];
  residual += temp*temp;

  temp = work.KKT[448]-1*work.d[119]*work.L[406];
  residual += temp*temp;

  temp = work.KKT[449]-1*work.d[119]*work.L[469];
  residual += temp*temp;

  temp = work.KKT[451]-1*work.d[120]*work.L[107];
  residual += temp*temp;

  temp = work.KKT[452]-1*work.d[120]*work.L[165];
  residual += temp*temp;

  temp = work.KKT[453]-1*work.d[120]*work.L[224];
  residual += temp*temp;

  temp = work.KKT[454]-1*work.d[120]*work.L[284];
  residual += temp*temp;

  temp = work.KKT[455]-1*work.d[120]*work.L[345];
  residual += temp*temp;

  temp = work.KKT[456]-1*work.d[120]*work.L[407];
  residual += temp*temp;

  temp = work.KKT[457]-1*work.d[120]*work.L[470];
  residual += temp*temp;

  temp = work.KKT[459]-1*work.d[121]*work.L[108];
  residual += temp*temp;

  temp = work.KKT[460]-1*work.d[121]*work.L[166];
  residual += temp*temp;

  temp = work.KKT[461]-1*work.d[121]*work.L[225];
  residual += temp*temp;

  temp = work.KKT[462]-1*work.d[121]*work.L[285];
  residual += temp*temp;

  temp = work.KKT[463]-1*work.d[121]*work.L[346];
  residual += temp*temp;

  temp = work.KKT[464]-1*work.d[121]*work.L[408];
  residual += temp*temp;

  temp = work.KKT[465]-1*work.d[121]*work.L[471];
  residual += temp*temp;

  temp = work.KKT[467]-1*work.d[122]*work.L[109];
  residual += temp*temp;

  temp = work.KKT[468]-1*work.d[122]*work.L[167];
  residual += temp*temp;

  temp = work.KKT[469]-1*work.d[122]*work.L[226];
  residual += temp*temp;

  temp = work.KKT[470]-1*work.d[122]*work.L[286];
  residual += temp*temp;

  temp = work.KKT[471]-1*work.d[122]*work.L[347];
  residual += temp*temp;

  temp = work.KKT[472]-1*work.d[122]*work.L[409];
  residual += temp*temp;

  temp = work.KKT[473]-1*work.d[122]*work.L[472];
  residual += temp*temp;

  temp = work.KKT[475]-1*work.d[123]*work.L[110];
  residual += temp*temp;

  temp = work.KKT[476]-1*work.d[123]*work.L[168];
  residual += temp*temp;

  temp = work.KKT[477]-1*work.d[123]*work.L[227];
  residual += temp*temp;

  temp = work.KKT[478]-1*work.d[123]*work.L[287];
  residual += temp*temp;

  temp = work.KKT[479]-1*work.d[123]*work.L[348];
  residual += temp*temp;

  temp = work.KKT[480]-1*work.d[123]*work.L[410];
  residual += temp*temp;

  temp = work.KKT[481]-1*work.d[123]*work.L[473];
  residual += temp*temp;

  temp = work.KKT[483]-1*work.d[124]*work.L[111];
  residual += temp*temp;

  temp = work.KKT[484]-1*work.d[124]*work.L[169];
  residual += temp*temp;

  temp = work.KKT[485]-1*work.d[124]*work.L[228];
  residual += temp*temp;

  temp = work.KKT[486]-1*work.d[124]*work.L[288];
  residual += temp*temp;

  temp = work.KKT[487]-1*work.d[124]*work.L[349];
  residual += temp*temp;

  temp = work.KKT[488]-1*work.d[124]*work.L[411];
  residual += temp*temp;

  temp = work.KKT[489]-1*work.d[124]*work.L[474];
  residual += temp*temp;

  temp = work.KKT[491]-1*work.d[125]*work.L[112];
  residual += temp*temp;

  temp = work.KKT[492]-1*work.d[125]*work.L[170];
  residual += temp*temp;

  temp = work.KKT[493]-1*work.d[125]*work.L[229];
  residual += temp*temp;

  temp = work.KKT[494]-1*work.d[125]*work.L[289];
  residual += temp*temp;

  temp = work.KKT[495]-1*work.d[125]*work.L[350];
  residual += temp*temp;

  temp = work.KKT[496]-1*work.d[125]*work.L[412];
  residual += temp*temp;

  temp = work.KKT[497]-1*work.d[125]*work.L[475];
  residual += temp*temp;

  temp = work.KKT[499]-1*work.d[126]*work.L[113];
  residual += temp*temp;

  temp = work.KKT[500]-1*work.d[126]*work.L[171];
  residual += temp*temp;

  temp = work.KKT[501]-1*work.d[126]*work.L[230];
  residual += temp*temp;

  temp = work.KKT[502]-1*work.d[126]*work.L[290];
  residual += temp*temp;

  temp = work.KKT[503]-1*work.d[126]*work.L[351];
  residual += temp*temp;

  temp = work.KKT[504]-1*work.d[126]*work.L[413];
  residual += temp*temp;

  temp = work.KKT[505]-1*work.d[126]*work.L[476];
  residual += temp*temp;

  temp = work.KKT[507]-1*work.d[127]*work.L[114];
  residual += temp*temp;

  temp = work.KKT[508]-1*work.d[127]*work.L[172];
  residual += temp*temp;

  temp = work.KKT[509]-1*work.d[127]*work.L[231];
  residual += temp*temp;

  temp = work.KKT[510]-1*work.d[127]*work.L[291];
  residual += temp*temp;

  temp = work.KKT[511]-1*work.d[127]*work.L[352];
  residual += temp*temp;

  temp = work.KKT[512]-1*work.d[127]*work.L[414];
  residual += temp*temp;

  temp = work.KKT[513]-1*work.d[127]*work.L[477];
  residual += temp*temp;

  temp = work.KKT[515]-1*work.d[128]*work.L[115];
  residual += temp*temp;

  temp = work.KKT[516]-1*work.d[128]*work.L[173];
  residual += temp*temp;

  temp = work.KKT[517]-1*work.d[128]*work.L[232];
  residual += temp*temp;

  temp = work.KKT[518]-1*work.d[128]*work.L[292];
  residual += temp*temp;

  temp = work.KKT[519]-1*work.d[128]*work.L[353];
  residual += temp*temp;

  temp = work.KKT[520]-1*work.d[128]*work.L[415];
  residual += temp*temp;

  temp = work.KKT[521]-1*work.d[128]*work.L[478];
  residual += temp*temp;

  temp = work.KKT[523]-1*work.d[129]*work.L[116];
  residual += temp*temp;

  temp = work.KKT[524]-1*work.d[129]*work.L[174];
  residual += temp*temp;

  temp = work.KKT[525]-1*work.d[129]*work.L[233];
  residual += temp*temp;

  temp = work.KKT[526]-1*work.d[129]*work.L[293];
  residual += temp*temp;

  temp = work.KKT[527]-1*work.d[129]*work.L[354];
  residual += temp*temp;

  temp = work.KKT[528]-1*work.d[129]*work.L[416];
  residual += temp*temp;

  temp = work.KKT[529]-1*work.d[129]*work.L[479];
  residual += temp*temp;

  temp = work.KKT[531]-1*work.d[130]*work.L[117];
  residual += temp*temp;

  temp = work.KKT[532]-1*work.d[130]*work.L[175];
  residual += temp*temp;

  temp = work.KKT[533]-1*work.d[130]*work.L[234];
  residual += temp*temp;

  temp = work.KKT[534]-1*work.d[130]*work.L[294];
  residual += temp*temp;

  temp = work.KKT[535]-1*work.d[130]*work.L[355];
  residual += temp*temp;

  temp = work.KKT[536]-1*work.d[130]*work.L[417];
  residual += temp*temp;

  temp = work.KKT[537]-1*work.d[130]*work.L[480];
  residual += temp*temp;

  temp = work.KKT[539]-1*work.d[131]*work.L[118];
  residual += temp*temp;

  temp = work.KKT[540]-1*work.d[131]*work.L[176];
  residual += temp*temp;

  temp = work.KKT[541]-1*work.d[131]*work.L[235];
  residual += temp*temp;

  temp = work.KKT[542]-1*work.d[131]*work.L[295];
  residual += temp*temp;

  temp = work.KKT[543]-1*work.d[131]*work.L[356];
  residual += temp*temp;

  temp = work.KKT[544]-1*work.d[131]*work.L[418];
  residual += temp*temp;

  temp = work.KKT[545]-1*work.d[131]*work.L[481];
  residual += temp*temp;

  temp = work.KKT[547]-1*work.d[132]*work.L[119];
  residual += temp*temp;

  temp = work.KKT[548]-1*work.d[132]*work.L[177];
  residual += temp*temp;

  temp = work.KKT[549]-1*work.d[132]*work.L[236];
  residual += temp*temp;

  temp = work.KKT[550]-1*work.d[132]*work.L[296];
  residual += temp*temp;

  temp = work.KKT[551]-1*work.d[132]*work.L[357];
  residual += temp*temp;

  temp = work.KKT[552]-1*work.d[132]*work.L[419];
  residual += temp*temp;

  temp = work.KKT[553]-1*work.d[132]*work.L[482];
  residual += temp*temp;

  temp = work.KKT[555]-1*work.d[133]*work.L[120];
  residual += temp*temp;

  temp = work.KKT[556]-1*work.d[133]*work.L[178];
  residual += temp*temp;

  temp = work.KKT[557]-1*work.d[133]*work.L[237];
  residual += temp*temp;

  temp = work.KKT[558]-1*work.d[133]*work.L[297];
  residual += temp*temp;

  temp = work.KKT[559]-1*work.d[133]*work.L[358];
  residual += temp*temp;

  temp = work.KKT[560]-1*work.d[133]*work.L[420];
  residual += temp*temp;

  temp = work.KKT[561]-1*work.d[133]*work.L[483];
  residual += temp*temp;

  temp = work.KKT[563]-1*work.d[134]*work.L[121];
  residual += temp*temp;

  temp = work.KKT[564]-1*work.d[134]*work.L[179];
  residual += temp*temp;

  temp = work.KKT[565]-1*work.d[134]*work.L[238];
  residual += temp*temp;

  temp = work.KKT[566]-1*work.d[134]*work.L[298];
  residual += temp*temp;

  temp = work.KKT[567]-1*work.d[134]*work.L[359];
  residual += temp*temp;

  temp = work.KKT[568]-1*work.d[134]*work.L[421];
  residual += temp*temp;

  temp = work.KKT[569]-1*work.d[134]*work.L[484];
  residual += temp*temp;

  temp = work.KKT[571]-1*work.d[135]*work.L[122];
  residual += temp*temp;

  temp = work.KKT[572]-1*work.d[135]*work.L[180];
  residual += temp*temp;

  temp = work.KKT[573]-1*work.d[135]*work.L[239];
  residual += temp*temp;

  temp = work.KKT[574]-1*work.d[135]*work.L[299];
  residual += temp*temp;

  temp = work.KKT[575]-1*work.d[135]*work.L[360];
  residual += temp*temp;

  temp = work.KKT[576]-1*work.d[135]*work.L[422];
  residual += temp*temp;

  temp = work.KKT[577]-1*work.d[135]*work.L[485];
  residual += temp*temp;

  temp = work.KKT[143]-1*work.d[71]*work.L[70];
  residual += temp*temp;

  temp = work.KKT[147]-1*work.d[73]*work.L[128];
  residual += temp*temp;

  temp = work.KKT[151]-1*work.d[75]*work.L[187];
  residual += temp*temp;

  temp = work.KKT[155]-1*work.d[77]*work.L[247];
  residual += temp*temp;

  temp = work.KKT[159]-1*work.d[79]*work.L[308];
  residual += temp*temp;

  temp = work.KKT[163]-1*work.d[81]*work.L[370];
  residual += temp*temp;

  temp = work.KKT[167]-1*work.d[83]*work.L[433];
  residual += temp*temp;

  temp = work.KKT[145]-1*work.d[72]*work.L[71];
  residual += temp*temp;

  temp = work.KKT[149]-1*work.d[74]*work.L[129];
  residual += temp*temp;

  temp = work.KKT[153]-1*work.d[76]*work.L[188];
  residual += temp*temp;

  temp = work.KKT[157]-1*work.d[78]*work.L[248];
  residual += temp*temp;

  temp = work.KKT[161]-1*work.d[80]*work.L[309];
  residual += temp*temp;

  temp = work.KKT[165]-1*work.d[82]*work.L[371];
  residual += temp*temp;

  temp = work.KKT[169]-1*work.d[84]*work.L[434];
  residual += temp*temp;

  temp = work.KKT[578]-1*work.d[136]*work.L[123];
  residual += temp*temp;

  temp = work.KKT[579]-1*work.d[136]*work.L[181];
  residual += temp*temp;

  temp = work.KKT[580]-1*work.d[136]*work.L[240];
  residual += temp*temp;

  temp = work.KKT[581]-1*work.d[136]*work.L[300];
  residual += temp*temp;

  temp = work.KKT[582]-1*work.d[136]*work.L[361];
  residual += temp*temp;

  temp = work.KKT[583]-1*work.d[136]*work.L[423];
  residual += temp*temp;

  temp = work.KKT[584]-1*work.d[136]*work.L[486];
  residual += temp*temp;

  temp = work.KKT[585]-1*work.d[137]*work.L[124];
  residual += temp*temp;

  temp = work.KKT[586]-1*work.d[137]*work.L[182];
  residual += temp*temp;

  temp = work.KKT[587]-1*work.d[137]*work.L[241];
  residual += temp*temp;

  temp = work.KKT[588]-1*work.d[137]*work.L[301];
  residual += temp*temp;

  temp = work.KKT[589]-1*work.d[137]*work.L[362];
  residual += temp*temp;

  temp = work.KKT[590]-1*work.d[137]*work.L[424];
  residual += temp*temp;

  temp = work.KKT[591]-1*work.d[137]*work.L[487];
  residual += temp*temp;

  temp = work.KKT[592]-1*work.d[138]*work.L[125];
  residual += temp*temp;

  temp = work.KKT[593]-1*work.d[138]*work.L[183];
  residual += temp*temp;

  temp = work.KKT[594]-1*work.d[138]*work.L[242];
  residual += temp*temp;

  temp = work.KKT[595]-1*work.d[138]*work.L[302];
  residual += temp*temp;

  temp = work.KKT[596]-1*work.d[138]*work.L[363];
  residual += temp*temp;

  temp = work.KKT[597]-1*work.d[138]*work.L[425];
  residual += temp*temp;

  temp = work.KKT[598]-1*work.d[138]*work.L[488];
  residual += temp*temp;

  temp = work.KKT[599]-1*work.d[139]*work.L[126];
  residual += temp*temp;

  temp = work.KKT[600]-1*work.d[139]*work.L[184];
  residual += temp*temp;

  temp = work.KKT[601]-1*work.d[139]*work.L[243];
  residual += temp*temp;

  temp = work.KKT[602]-1*work.d[139]*work.L[303];
  residual += temp*temp;

  temp = work.KKT[603]-1*work.d[139]*work.L[364];
  residual += temp*temp;

  temp = work.KKT[604]-1*work.d[139]*work.L[426];
  residual += temp*temp;

  temp = work.KKT[605]-1*work.d[139]*work.L[489];
  residual += temp*temp;

  temp = work.KKT[606]-1*work.d[140]*work.L[127];
  residual += temp*temp;

  temp = work.KKT[607]-1*work.d[140]*work.L[185];
  residual += temp*temp;

  temp = work.KKT[608]-1*work.d[140]*work.L[244];
  residual += temp*temp;

  temp = work.KKT[609]-1*work.d[140]*work.L[304];
  residual += temp*temp;

  temp = work.KKT[610]-1*work.d[140]*work.L[365];
  residual += temp*temp;

  temp = work.KKT[611]-1*work.d[140]*work.L[427];
  residual += temp*temp;

  temp = work.KKT[612]-1*work.d[140]*work.L[490];
  residual += temp*temp;

  temp = work.KKT[613]-work.L[498]*work.d[141]*1;
  residual += temp*temp;

  temp = work.KKT[614]-work.L[498]*work.d[141]*work.L[186]-work.L[499]*work.d[142]*1;
  residual += temp*temp;

  temp = work.KKT[615]-work.L[498]*work.d[141]*work.L[245]-work.L[499]*work.d[142]*work.L[246]-work.L[500]*work.d[143]*1;
  residual += temp*temp;

  temp = work.KKT[616]-work.L[498]*work.d[141]*work.L[305]-work.L[499]*work.d[142]*work.L[306]-work.L[500]*work.d[143]*work.L[307]-work.L[501]*work.d[144]*1;
  residual += temp*temp;

  temp = work.KKT[617]-work.L[498]*work.d[141]*work.L[366]-work.L[499]*work.d[142]*work.L[367]-work.L[500]*work.d[143]*work.L[368]-work.L[501]*work.d[144]*work.L[369]-work.L[502]*work.d[145]*1;
  residual += temp*temp;

  temp = work.KKT[618]-work.L[498]*work.d[141]*work.L[428]-work.L[499]*work.d[142]*work.L[429]-work.L[500]*work.d[143]*work.L[430]-work.L[501]*work.d[144]*work.L[431]-work.L[502]*work.d[145]*work.L[432]-work.L[503]*work.d[146]*1;
  residual += temp*temp;

  temp = work.KKT[619]-work.L[498]*work.d[141]*work.L[491]-work.L[499]*work.d[142]*work.L[492]-work.L[500]*work.d[143]*work.L[493]-work.L[501]*work.d[144]*work.L[494]-work.L[502]*work.d[145]*work.L[495]-work.L[503]*work.d[146]*work.L[496]-work.L[504]*work.d[147]*1;
  residual += temp*temp;

  temp = work.KKT[131]-work.L[65]*work.d[65]*1;
  residual += temp*temp;

  temp = work.KKT[133]-work.L[66]*work.d[66]*1;
  residual += temp*temp;

  temp = work.KKT[135]-work.L[67]*work.d[67]*1;
  residual += temp*temp;

  temp = work.KKT[137]-work.L[68]*work.d[68]*1;
  residual += temp*temp;

  temp = work.KKT[139]-work.L[69]*work.d[69]*1;
  residual += temp*temp;

  temp = work.KKT[141]-work.L[497]*work.d[70]*1;
  residual += temp*temp;

  return residual;
}

void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */

  result[0] = work.KKT[171]*source[78]+work.KKT[179]*source[79]+work.KKT[187]*source[80]+work.KKT[195]*source[81]+work.KKT[203]*source[82]+work.KKT[211]*source[83]+work.KKT[219]*source[84]+work.KKT[227]*source[85]+work.KKT[235]*source[86]+work.KKT[243]*source[87]+work.KKT[251]*source[88]+work.KKT[259]*source[89]+work.KKT[267]*source[90]+work.KKT[275]*source[91]+work.KKT[283]*source[92]+work.KKT[291]*source[93]+work.KKT[299]*source[94]+work.KKT[307]*source[95]+work.KKT[315]*source[96]+work.KKT[323]*source[97]+work.KKT[331]*source[98]+work.KKT[339]*source[99]+work.KKT[347]*source[100]+work.KKT[355]*source[101]+work.KKT[363]*source[102]+work.KKT[371]*source[103]+work.KKT[379]*source[104]+work.KKT[387]*source[105]+work.KKT[395]*source[106]+work.KKT[403]*source[107]+work.KKT[411]*source[108]+work.KKT[419]*source[109]+work.KKT[427]*source[110]+work.KKT[435]*source[111]+work.KKT[443]*source[112]+work.KKT[451]*source[113]+work.KKT[459]*source[114]+work.KKT[467]*source[115]+work.KKT[475]*source[116]+work.KKT[483]*source[117]+work.KKT[491]*source[118]+work.KKT[499]*source[119]+work.KKT[507]*source[120]+work.KKT[515]*source[121]+work.KKT[523]*source[122]+work.KKT[531]*source[123]+work.KKT[539]*source[124]+work.KKT[547]*source[125]+work.KKT[555]*source[126]+work.KKT[563]*source[127]+work.KKT[571]*source[128]+work.KKT[143]*source[129]+work.KKT[145]*source[136]+work.KKT[578]*source[143]+work.KKT[585]*source[144]+work.KKT[592]*source[145]+work.KKT[599]*source[146]+work.KKT[606]*source[147]+work.KKT[613]*source[148];
  result[1] = work.KKT[172]*source[78]+work.KKT[180]*source[79]+work.KKT[188]*source[80]+work.KKT[196]*source[81]+work.KKT[204]*source[82]+work.KKT[212]*source[83]+work.KKT[220]*source[84]+work.KKT[228]*source[85]+work.KKT[236]*source[86]+work.KKT[244]*source[87]+work.KKT[252]*source[88]+work.KKT[260]*source[89]+work.KKT[268]*source[90]+work.KKT[276]*source[91]+work.KKT[284]*source[92]+work.KKT[292]*source[93]+work.KKT[300]*source[94]+work.KKT[308]*source[95]+work.KKT[316]*source[96]+work.KKT[324]*source[97]+work.KKT[332]*source[98]+work.KKT[340]*source[99]+work.KKT[348]*source[100]+work.KKT[356]*source[101]+work.KKT[364]*source[102]+work.KKT[372]*source[103]+work.KKT[380]*source[104]+work.KKT[388]*source[105]+work.KKT[396]*source[106]+work.KKT[404]*source[107]+work.KKT[412]*source[108]+work.KKT[420]*source[109]+work.KKT[428]*source[110]+work.KKT[436]*source[111]+work.KKT[444]*source[112]+work.KKT[452]*source[113]+work.KKT[460]*source[114]+work.KKT[468]*source[115]+work.KKT[476]*source[116]+work.KKT[484]*source[117]+work.KKT[492]*source[118]+work.KKT[500]*source[119]+work.KKT[508]*source[120]+work.KKT[516]*source[121]+work.KKT[524]*source[122]+work.KKT[532]*source[123]+work.KKT[540]*source[124]+work.KKT[548]*source[125]+work.KKT[556]*source[126]+work.KKT[564]*source[127]+work.KKT[572]*source[128]+work.KKT[147]*source[130]+work.KKT[149]*source[137]+work.KKT[579]*source[143]+work.KKT[586]*source[144]+work.KKT[593]*source[145]+work.KKT[600]*source[146]+work.KKT[607]*source[147]+work.KKT[614]*source[148];
  result[2] = work.KKT[173]*source[78]+work.KKT[181]*source[79]+work.KKT[189]*source[80]+work.KKT[197]*source[81]+work.KKT[205]*source[82]+work.KKT[213]*source[83]+work.KKT[221]*source[84]+work.KKT[229]*source[85]+work.KKT[237]*source[86]+work.KKT[245]*source[87]+work.KKT[253]*source[88]+work.KKT[261]*source[89]+work.KKT[269]*source[90]+work.KKT[277]*source[91]+work.KKT[285]*source[92]+work.KKT[293]*source[93]+work.KKT[301]*source[94]+work.KKT[309]*source[95]+work.KKT[317]*source[96]+work.KKT[325]*source[97]+work.KKT[333]*source[98]+work.KKT[341]*source[99]+work.KKT[349]*source[100]+work.KKT[357]*source[101]+work.KKT[365]*source[102]+work.KKT[373]*source[103]+work.KKT[381]*source[104]+work.KKT[389]*source[105]+work.KKT[397]*source[106]+work.KKT[405]*source[107]+work.KKT[413]*source[108]+work.KKT[421]*source[109]+work.KKT[429]*source[110]+work.KKT[437]*source[111]+work.KKT[445]*source[112]+work.KKT[453]*source[113]+work.KKT[461]*source[114]+work.KKT[469]*source[115]+work.KKT[477]*source[116]+work.KKT[485]*source[117]+work.KKT[493]*source[118]+work.KKT[501]*source[119]+work.KKT[509]*source[120]+work.KKT[517]*source[121]+work.KKT[525]*source[122]+work.KKT[533]*source[123]+work.KKT[541]*source[124]+work.KKT[549]*source[125]+work.KKT[557]*source[126]+work.KKT[565]*source[127]+work.KKT[573]*source[128]+work.KKT[151]*source[131]+work.KKT[153]*source[138]+work.KKT[580]*source[143]+work.KKT[587]*source[144]+work.KKT[594]*source[145]+work.KKT[601]*source[146]+work.KKT[608]*source[147]+work.KKT[615]*source[148];
  result[3] = work.KKT[174]*source[78]+work.KKT[182]*source[79]+work.KKT[190]*source[80]+work.KKT[198]*source[81]+work.KKT[206]*source[82]+work.KKT[214]*source[83]+work.KKT[222]*source[84]+work.KKT[230]*source[85]+work.KKT[238]*source[86]+work.KKT[246]*source[87]+work.KKT[254]*source[88]+work.KKT[262]*source[89]+work.KKT[270]*source[90]+work.KKT[278]*source[91]+work.KKT[286]*source[92]+work.KKT[294]*source[93]+work.KKT[302]*source[94]+work.KKT[310]*source[95]+work.KKT[318]*source[96]+work.KKT[326]*source[97]+work.KKT[334]*source[98]+work.KKT[342]*source[99]+work.KKT[350]*source[100]+work.KKT[358]*source[101]+work.KKT[366]*source[102]+work.KKT[374]*source[103]+work.KKT[382]*source[104]+work.KKT[390]*source[105]+work.KKT[398]*source[106]+work.KKT[406]*source[107]+work.KKT[414]*source[108]+work.KKT[422]*source[109]+work.KKT[430]*source[110]+work.KKT[438]*source[111]+work.KKT[446]*source[112]+work.KKT[454]*source[113]+work.KKT[462]*source[114]+work.KKT[470]*source[115]+work.KKT[478]*source[116]+work.KKT[486]*source[117]+work.KKT[494]*source[118]+work.KKT[502]*source[119]+work.KKT[510]*source[120]+work.KKT[518]*source[121]+work.KKT[526]*source[122]+work.KKT[534]*source[123]+work.KKT[542]*source[124]+work.KKT[550]*source[125]+work.KKT[558]*source[126]+work.KKT[566]*source[127]+work.KKT[574]*source[128]+work.KKT[155]*source[132]+work.KKT[157]*source[139]+work.KKT[581]*source[143]+work.KKT[588]*source[144]+work.KKT[595]*source[145]+work.KKT[602]*source[146]+work.KKT[609]*source[147]+work.KKT[616]*source[148];
  result[4] = work.KKT[175]*source[78]+work.KKT[183]*source[79]+work.KKT[191]*source[80]+work.KKT[199]*source[81]+work.KKT[207]*source[82]+work.KKT[215]*source[83]+work.KKT[223]*source[84]+work.KKT[231]*source[85]+work.KKT[239]*source[86]+work.KKT[247]*source[87]+work.KKT[255]*source[88]+work.KKT[263]*source[89]+work.KKT[271]*source[90]+work.KKT[279]*source[91]+work.KKT[287]*source[92]+work.KKT[295]*source[93]+work.KKT[303]*source[94]+work.KKT[311]*source[95]+work.KKT[319]*source[96]+work.KKT[327]*source[97]+work.KKT[335]*source[98]+work.KKT[343]*source[99]+work.KKT[351]*source[100]+work.KKT[359]*source[101]+work.KKT[367]*source[102]+work.KKT[375]*source[103]+work.KKT[383]*source[104]+work.KKT[391]*source[105]+work.KKT[399]*source[106]+work.KKT[407]*source[107]+work.KKT[415]*source[108]+work.KKT[423]*source[109]+work.KKT[431]*source[110]+work.KKT[439]*source[111]+work.KKT[447]*source[112]+work.KKT[455]*source[113]+work.KKT[463]*source[114]+work.KKT[471]*source[115]+work.KKT[479]*source[116]+work.KKT[487]*source[117]+work.KKT[495]*source[118]+work.KKT[503]*source[119]+work.KKT[511]*source[120]+work.KKT[519]*source[121]+work.KKT[527]*source[122]+work.KKT[535]*source[123]+work.KKT[543]*source[124]+work.KKT[551]*source[125]+work.KKT[559]*source[126]+work.KKT[567]*source[127]+work.KKT[575]*source[128]+work.KKT[159]*source[133]+work.KKT[161]*source[140]+work.KKT[582]*source[143]+work.KKT[589]*source[144]+work.KKT[596]*source[145]+work.KKT[603]*source[146]+work.KKT[610]*source[147]+work.KKT[617]*source[148];
  result[5] = work.KKT[176]*source[78]+work.KKT[184]*source[79]+work.KKT[192]*source[80]+work.KKT[200]*source[81]+work.KKT[208]*source[82]+work.KKT[216]*source[83]+work.KKT[224]*source[84]+work.KKT[232]*source[85]+work.KKT[240]*source[86]+work.KKT[248]*source[87]+work.KKT[256]*source[88]+work.KKT[264]*source[89]+work.KKT[272]*source[90]+work.KKT[280]*source[91]+work.KKT[288]*source[92]+work.KKT[296]*source[93]+work.KKT[304]*source[94]+work.KKT[312]*source[95]+work.KKT[320]*source[96]+work.KKT[328]*source[97]+work.KKT[336]*source[98]+work.KKT[344]*source[99]+work.KKT[352]*source[100]+work.KKT[360]*source[101]+work.KKT[368]*source[102]+work.KKT[376]*source[103]+work.KKT[384]*source[104]+work.KKT[392]*source[105]+work.KKT[400]*source[106]+work.KKT[408]*source[107]+work.KKT[416]*source[108]+work.KKT[424]*source[109]+work.KKT[432]*source[110]+work.KKT[440]*source[111]+work.KKT[448]*source[112]+work.KKT[456]*source[113]+work.KKT[464]*source[114]+work.KKT[472]*source[115]+work.KKT[480]*source[116]+work.KKT[488]*source[117]+work.KKT[496]*source[118]+work.KKT[504]*source[119]+work.KKT[512]*source[120]+work.KKT[520]*source[121]+work.KKT[528]*source[122]+work.KKT[536]*source[123]+work.KKT[544]*source[124]+work.KKT[552]*source[125]+work.KKT[560]*source[126]+work.KKT[568]*source[127]+work.KKT[576]*source[128]+work.KKT[163]*source[134]+work.KKT[165]*source[141]+work.KKT[583]*source[143]+work.KKT[590]*source[144]+work.KKT[597]*source[145]+work.KKT[604]*source[146]+work.KKT[611]*source[147]+work.KKT[618]*source[148];
  result[6] = work.KKT[177]*source[78]+work.KKT[185]*source[79]+work.KKT[193]*source[80]+work.KKT[201]*source[81]+work.KKT[209]*source[82]+work.KKT[217]*source[83]+work.KKT[225]*source[84]+work.KKT[233]*source[85]+work.KKT[241]*source[86]+work.KKT[249]*source[87]+work.KKT[257]*source[88]+work.KKT[265]*source[89]+work.KKT[273]*source[90]+work.KKT[281]*source[91]+work.KKT[289]*source[92]+work.KKT[297]*source[93]+work.KKT[305]*source[94]+work.KKT[313]*source[95]+work.KKT[321]*source[96]+work.KKT[329]*source[97]+work.KKT[337]*source[98]+work.KKT[345]*source[99]+work.KKT[353]*source[100]+work.KKT[361]*source[101]+work.KKT[369]*source[102]+work.KKT[377]*source[103]+work.KKT[385]*source[104]+work.KKT[393]*source[105]+work.KKT[401]*source[106]+work.KKT[409]*source[107]+work.KKT[417]*source[108]+work.KKT[425]*source[109]+work.KKT[433]*source[110]+work.KKT[441]*source[111]+work.KKT[449]*source[112]+work.KKT[457]*source[113]+work.KKT[465]*source[114]+work.KKT[473]*source[115]+work.KKT[481]*source[116]+work.KKT[489]*source[117]+work.KKT[497]*source[118]+work.KKT[505]*source[119]+work.KKT[513]*source[120]+work.KKT[521]*source[121]+work.KKT[529]*source[122]+work.KKT[537]*source[123]+work.KKT[545]*source[124]+work.KKT[553]*source[125]+work.KKT[561]*source[126]+work.KKT[569]*source[127]+work.KKT[577]*source[128]+work.KKT[167]*source[135]+work.KKT[169]*source[142]+work.KKT[584]*source[143]+work.KKT[591]*source[144]+work.KKT[598]*source[145]+work.KKT[605]*source[146]+work.KKT[612]*source[147]+work.KKT[619]*source[148];
  result[7] = work.KKT[130]*source[7]+work.KKT[131]*source[143];
  result[8] = work.KKT[132]*source[8]+work.KKT[133]*source[144];
  result[9] = work.KKT[134]*source[9]+work.KKT[135]*source[145];
  result[10] = work.KKT[136]*source[10]+work.KKT[137]*source[146];
  result[11] = work.KKT[138]*source[11]+work.KKT[139]*source[147];
  result[12] = work.KKT[140]*source[12]+work.KKT[141]*source[148];
  result[13] = work.KKT[0]*source[13]+work.KKT[1]*source[78];
  result[14] = work.KKT[2]*source[14]+work.KKT[3]*source[79];
  result[15] = work.KKT[4]*source[15]+work.KKT[5]*source[80];
  result[16] = work.KKT[6]*source[16]+work.KKT[7]*source[81];
  result[17] = work.KKT[8]*source[17]+work.KKT[9]*source[82];
  result[18] = work.KKT[10]*source[18]+work.KKT[11]*source[83];
  result[19] = work.KKT[12]*source[19]+work.KKT[13]*source[84];
  result[20] = work.KKT[14]*source[20]+work.KKT[15]*source[85];
  result[21] = work.KKT[16]*source[21]+work.KKT[17]*source[86];
  result[22] = work.KKT[18]*source[22]+work.KKT[19]*source[87];
  result[23] = work.KKT[20]*source[23]+work.KKT[21]*source[88];
  result[24] = work.KKT[22]*source[24]+work.KKT[23]*source[89];
  result[25] = work.KKT[24]*source[25]+work.KKT[25]*source[90];
  result[26] = work.KKT[26]*source[26]+work.KKT[27]*source[91];
  result[27] = work.KKT[28]*source[27]+work.KKT[29]*source[92];
  result[28] = work.KKT[30]*source[28]+work.KKT[31]*source[93];
  result[29] = work.KKT[32]*source[29]+work.KKT[33]*source[94];
  result[30] = work.KKT[34]*source[30]+work.KKT[35]*source[95];
  result[31] = work.KKT[36]*source[31]+work.KKT[37]*source[96];
  result[32] = work.KKT[38]*source[32]+work.KKT[39]*source[97];
  result[33] = work.KKT[40]*source[33]+work.KKT[41]*source[98];
  result[34] = work.KKT[42]*source[34]+work.KKT[43]*source[99];
  result[35] = work.KKT[44]*source[35]+work.KKT[45]*source[100];
  result[36] = work.KKT[46]*source[36]+work.KKT[47]*source[101];
  result[37] = work.KKT[48]*source[37]+work.KKT[49]*source[102];
  result[38] = work.KKT[50]*source[38]+work.KKT[51]*source[103];
  result[39] = work.KKT[52]*source[39]+work.KKT[53]*source[104];
  result[40] = work.KKT[54]*source[40]+work.KKT[55]*source[105];
  result[41] = work.KKT[56]*source[41]+work.KKT[57]*source[106];
  result[42] = work.KKT[58]*source[42]+work.KKT[59]*source[107];
  result[43] = work.KKT[60]*source[43]+work.KKT[61]*source[108];
  result[44] = work.KKT[62]*source[44]+work.KKT[63]*source[109];
  result[45] = work.KKT[64]*source[45]+work.KKT[65]*source[110];
  result[46] = work.KKT[66]*source[46]+work.KKT[67]*source[111];
  result[47] = work.KKT[68]*source[47]+work.KKT[69]*source[112];
  result[48] = work.KKT[70]*source[48]+work.KKT[71]*source[113];
  result[49] = work.KKT[72]*source[49]+work.KKT[73]*source[114];
  result[50] = work.KKT[74]*source[50]+work.KKT[75]*source[115];
  result[51] = work.KKT[76]*source[51]+work.KKT[77]*source[116];
  result[52] = work.KKT[78]*source[52]+work.KKT[79]*source[117];
  result[53] = work.KKT[80]*source[53]+work.KKT[81]*source[118];
  result[54] = work.KKT[82]*source[54]+work.KKT[83]*source[119];
  result[55] = work.KKT[84]*source[55]+work.KKT[85]*source[120];
  result[56] = work.KKT[86]*source[56]+work.KKT[87]*source[121];
  result[57] = work.KKT[88]*source[57]+work.KKT[89]*source[122];
  result[58] = work.KKT[90]*source[58]+work.KKT[91]*source[123];
  result[59] = work.KKT[92]*source[59]+work.KKT[93]*source[124];
  result[60] = work.KKT[94]*source[60]+work.KKT[95]*source[125];
  result[61] = work.KKT[96]*source[61]+work.KKT[97]*source[126];
  result[62] = work.KKT[98]*source[62]+work.KKT[99]*source[127];
  result[63] = work.KKT[100]*source[63]+work.KKT[101]*source[128];
  result[64] = work.KKT[102]*source[64]+work.KKT[103]*source[129];
  result[65] = work.KKT[104]*source[65]+work.KKT[105]*source[130];
  result[66] = work.KKT[106]*source[66]+work.KKT[107]*source[131];
  result[67] = work.KKT[108]*source[67]+work.KKT[109]*source[132];
  result[68] = work.KKT[110]*source[68]+work.KKT[111]*source[133];
  result[69] = work.KKT[112]*source[69]+work.KKT[113]*source[134];
  result[70] = work.KKT[114]*source[70]+work.KKT[115]*source[135];
  result[71] = work.KKT[116]*source[71]+work.KKT[117]*source[136];
  result[72] = work.KKT[118]*source[72]+work.KKT[119]*source[137];
  result[73] = work.KKT[120]*source[73]+work.KKT[121]*source[138];
  result[74] = work.KKT[122]*source[74]+work.KKT[123]*source[139];
  result[75] = work.KKT[124]*source[75]+work.KKT[125]*source[140];
  result[76] = work.KKT[126]*source[76]+work.KKT[127]*source[141];
  result[77] = work.KKT[128]*source[77]+work.KKT[129]*source[142];
  result[78] = work.KKT[1]*source[13]+work.KKT[170]*source[78]+work.KKT[171]*source[0]+work.KKT[172]*source[1]+work.KKT[173]*source[2]+work.KKT[174]*source[3]+work.KKT[175]*source[4]+work.KKT[176]*source[5]+work.KKT[177]*source[6];
  result[79] = work.KKT[3]*source[14]+work.KKT[178]*source[79]+work.KKT[179]*source[0]+work.KKT[180]*source[1]+work.KKT[181]*source[2]+work.KKT[182]*source[3]+work.KKT[183]*source[4]+work.KKT[184]*source[5]+work.KKT[185]*source[6];
  result[80] = work.KKT[5]*source[15]+work.KKT[186]*source[80]+work.KKT[187]*source[0]+work.KKT[188]*source[1]+work.KKT[189]*source[2]+work.KKT[190]*source[3]+work.KKT[191]*source[4]+work.KKT[192]*source[5]+work.KKT[193]*source[6];
  result[81] = work.KKT[7]*source[16]+work.KKT[194]*source[81]+work.KKT[195]*source[0]+work.KKT[196]*source[1]+work.KKT[197]*source[2]+work.KKT[198]*source[3]+work.KKT[199]*source[4]+work.KKT[200]*source[5]+work.KKT[201]*source[6];
  result[82] = work.KKT[9]*source[17]+work.KKT[202]*source[82]+work.KKT[203]*source[0]+work.KKT[204]*source[1]+work.KKT[205]*source[2]+work.KKT[206]*source[3]+work.KKT[207]*source[4]+work.KKT[208]*source[5]+work.KKT[209]*source[6];
  result[83] = work.KKT[11]*source[18]+work.KKT[210]*source[83]+work.KKT[211]*source[0]+work.KKT[212]*source[1]+work.KKT[213]*source[2]+work.KKT[214]*source[3]+work.KKT[215]*source[4]+work.KKT[216]*source[5]+work.KKT[217]*source[6];
  result[84] = work.KKT[13]*source[19]+work.KKT[218]*source[84]+work.KKT[219]*source[0]+work.KKT[220]*source[1]+work.KKT[221]*source[2]+work.KKT[222]*source[3]+work.KKT[223]*source[4]+work.KKT[224]*source[5]+work.KKT[225]*source[6];
  result[85] = work.KKT[15]*source[20]+work.KKT[226]*source[85]+work.KKT[227]*source[0]+work.KKT[228]*source[1]+work.KKT[229]*source[2]+work.KKT[230]*source[3]+work.KKT[231]*source[4]+work.KKT[232]*source[5]+work.KKT[233]*source[6];
  result[86] = work.KKT[17]*source[21]+work.KKT[234]*source[86]+work.KKT[235]*source[0]+work.KKT[236]*source[1]+work.KKT[237]*source[2]+work.KKT[238]*source[3]+work.KKT[239]*source[4]+work.KKT[240]*source[5]+work.KKT[241]*source[6];
  result[87] = work.KKT[19]*source[22]+work.KKT[242]*source[87]+work.KKT[243]*source[0]+work.KKT[244]*source[1]+work.KKT[245]*source[2]+work.KKT[246]*source[3]+work.KKT[247]*source[4]+work.KKT[248]*source[5]+work.KKT[249]*source[6];
  result[88] = work.KKT[21]*source[23]+work.KKT[250]*source[88]+work.KKT[251]*source[0]+work.KKT[252]*source[1]+work.KKT[253]*source[2]+work.KKT[254]*source[3]+work.KKT[255]*source[4]+work.KKT[256]*source[5]+work.KKT[257]*source[6];
  result[89] = work.KKT[23]*source[24]+work.KKT[258]*source[89]+work.KKT[259]*source[0]+work.KKT[260]*source[1]+work.KKT[261]*source[2]+work.KKT[262]*source[3]+work.KKT[263]*source[4]+work.KKT[264]*source[5]+work.KKT[265]*source[6];
  result[90] = work.KKT[25]*source[25]+work.KKT[266]*source[90]+work.KKT[267]*source[0]+work.KKT[268]*source[1]+work.KKT[269]*source[2]+work.KKT[270]*source[3]+work.KKT[271]*source[4]+work.KKT[272]*source[5]+work.KKT[273]*source[6];
  result[91] = work.KKT[27]*source[26]+work.KKT[274]*source[91]+work.KKT[275]*source[0]+work.KKT[276]*source[1]+work.KKT[277]*source[2]+work.KKT[278]*source[3]+work.KKT[279]*source[4]+work.KKT[280]*source[5]+work.KKT[281]*source[6];
  result[92] = work.KKT[29]*source[27]+work.KKT[282]*source[92]+work.KKT[283]*source[0]+work.KKT[284]*source[1]+work.KKT[285]*source[2]+work.KKT[286]*source[3]+work.KKT[287]*source[4]+work.KKT[288]*source[5]+work.KKT[289]*source[6];
  result[93] = work.KKT[31]*source[28]+work.KKT[290]*source[93]+work.KKT[291]*source[0]+work.KKT[292]*source[1]+work.KKT[293]*source[2]+work.KKT[294]*source[3]+work.KKT[295]*source[4]+work.KKT[296]*source[5]+work.KKT[297]*source[6];
  result[94] = work.KKT[33]*source[29]+work.KKT[298]*source[94]+work.KKT[299]*source[0]+work.KKT[300]*source[1]+work.KKT[301]*source[2]+work.KKT[302]*source[3]+work.KKT[303]*source[4]+work.KKT[304]*source[5]+work.KKT[305]*source[6];
  result[95] = work.KKT[35]*source[30]+work.KKT[306]*source[95]+work.KKT[307]*source[0]+work.KKT[308]*source[1]+work.KKT[309]*source[2]+work.KKT[310]*source[3]+work.KKT[311]*source[4]+work.KKT[312]*source[5]+work.KKT[313]*source[6];
  result[96] = work.KKT[37]*source[31]+work.KKT[314]*source[96]+work.KKT[315]*source[0]+work.KKT[316]*source[1]+work.KKT[317]*source[2]+work.KKT[318]*source[3]+work.KKT[319]*source[4]+work.KKT[320]*source[5]+work.KKT[321]*source[6];
  result[97] = work.KKT[39]*source[32]+work.KKT[322]*source[97]+work.KKT[323]*source[0]+work.KKT[324]*source[1]+work.KKT[325]*source[2]+work.KKT[326]*source[3]+work.KKT[327]*source[4]+work.KKT[328]*source[5]+work.KKT[329]*source[6];
  result[98] = work.KKT[41]*source[33]+work.KKT[330]*source[98]+work.KKT[331]*source[0]+work.KKT[332]*source[1]+work.KKT[333]*source[2]+work.KKT[334]*source[3]+work.KKT[335]*source[4]+work.KKT[336]*source[5]+work.KKT[337]*source[6];
  result[99] = work.KKT[43]*source[34]+work.KKT[338]*source[99]+work.KKT[339]*source[0]+work.KKT[340]*source[1]+work.KKT[341]*source[2]+work.KKT[342]*source[3]+work.KKT[343]*source[4]+work.KKT[344]*source[5]+work.KKT[345]*source[6];
  result[100] = work.KKT[45]*source[35]+work.KKT[346]*source[100]+work.KKT[347]*source[0]+work.KKT[348]*source[1]+work.KKT[349]*source[2]+work.KKT[350]*source[3]+work.KKT[351]*source[4]+work.KKT[352]*source[5]+work.KKT[353]*source[6];
  result[101] = work.KKT[47]*source[36]+work.KKT[354]*source[101]+work.KKT[355]*source[0]+work.KKT[356]*source[1]+work.KKT[357]*source[2]+work.KKT[358]*source[3]+work.KKT[359]*source[4]+work.KKT[360]*source[5]+work.KKT[361]*source[6];
  result[102] = work.KKT[49]*source[37]+work.KKT[362]*source[102]+work.KKT[363]*source[0]+work.KKT[364]*source[1]+work.KKT[365]*source[2]+work.KKT[366]*source[3]+work.KKT[367]*source[4]+work.KKT[368]*source[5]+work.KKT[369]*source[6];
  result[103] = work.KKT[51]*source[38]+work.KKT[370]*source[103]+work.KKT[371]*source[0]+work.KKT[372]*source[1]+work.KKT[373]*source[2]+work.KKT[374]*source[3]+work.KKT[375]*source[4]+work.KKT[376]*source[5]+work.KKT[377]*source[6];
  result[104] = work.KKT[53]*source[39]+work.KKT[378]*source[104]+work.KKT[379]*source[0]+work.KKT[380]*source[1]+work.KKT[381]*source[2]+work.KKT[382]*source[3]+work.KKT[383]*source[4]+work.KKT[384]*source[5]+work.KKT[385]*source[6];
  result[105] = work.KKT[55]*source[40]+work.KKT[386]*source[105]+work.KKT[387]*source[0]+work.KKT[388]*source[1]+work.KKT[389]*source[2]+work.KKT[390]*source[3]+work.KKT[391]*source[4]+work.KKT[392]*source[5]+work.KKT[393]*source[6];
  result[106] = work.KKT[57]*source[41]+work.KKT[394]*source[106]+work.KKT[395]*source[0]+work.KKT[396]*source[1]+work.KKT[397]*source[2]+work.KKT[398]*source[3]+work.KKT[399]*source[4]+work.KKT[400]*source[5]+work.KKT[401]*source[6];
  result[107] = work.KKT[59]*source[42]+work.KKT[402]*source[107]+work.KKT[403]*source[0]+work.KKT[404]*source[1]+work.KKT[405]*source[2]+work.KKT[406]*source[3]+work.KKT[407]*source[4]+work.KKT[408]*source[5]+work.KKT[409]*source[6];
  result[108] = work.KKT[61]*source[43]+work.KKT[410]*source[108]+work.KKT[411]*source[0]+work.KKT[412]*source[1]+work.KKT[413]*source[2]+work.KKT[414]*source[3]+work.KKT[415]*source[4]+work.KKT[416]*source[5]+work.KKT[417]*source[6];
  result[109] = work.KKT[63]*source[44]+work.KKT[418]*source[109]+work.KKT[419]*source[0]+work.KKT[420]*source[1]+work.KKT[421]*source[2]+work.KKT[422]*source[3]+work.KKT[423]*source[4]+work.KKT[424]*source[5]+work.KKT[425]*source[6];
  result[110] = work.KKT[65]*source[45]+work.KKT[426]*source[110]+work.KKT[427]*source[0]+work.KKT[428]*source[1]+work.KKT[429]*source[2]+work.KKT[430]*source[3]+work.KKT[431]*source[4]+work.KKT[432]*source[5]+work.KKT[433]*source[6];
  result[111] = work.KKT[67]*source[46]+work.KKT[434]*source[111]+work.KKT[435]*source[0]+work.KKT[436]*source[1]+work.KKT[437]*source[2]+work.KKT[438]*source[3]+work.KKT[439]*source[4]+work.KKT[440]*source[5]+work.KKT[441]*source[6];
  result[112] = work.KKT[69]*source[47]+work.KKT[442]*source[112]+work.KKT[443]*source[0]+work.KKT[444]*source[1]+work.KKT[445]*source[2]+work.KKT[446]*source[3]+work.KKT[447]*source[4]+work.KKT[448]*source[5]+work.KKT[449]*source[6];
  result[113] = work.KKT[71]*source[48]+work.KKT[450]*source[113]+work.KKT[451]*source[0]+work.KKT[452]*source[1]+work.KKT[453]*source[2]+work.KKT[454]*source[3]+work.KKT[455]*source[4]+work.KKT[456]*source[5]+work.KKT[457]*source[6];
  result[114] = work.KKT[73]*source[49]+work.KKT[458]*source[114]+work.KKT[459]*source[0]+work.KKT[460]*source[1]+work.KKT[461]*source[2]+work.KKT[462]*source[3]+work.KKT[463]*source[4]+work.KKT[464]*source[5]+work.KKT[465]*source[6];
  result[115] = work.KKT[75]*source[50]+work.KKT[466]*source[115]+work.KKT[467]*source[0]+work.KKT[468]*source[1]+work.KKT[469]*source[2]+work.KKT[470]*source[3]+work.KKT[471]*source[4]+work.KKT[472]*source[5]+work.KKT[473]*source[6];
  result[116] = work.KKT[77]*source[51]+work.KKT[474]*source[116]+work.KKT[475]*source[0]+work.KKT[476]*source[1]+work.KKT[477]*source[2]+work.KKT[478]*source[3]+work.KKT[479]*source[4]+work.KKT[480]*source[5]+work.KKT[481]*source[6];
  result[117] = work.KKT[79]*source[52]+work.KKT[482]*source[117]+work.KKT[483]*source[0]+work.KKT[484]*source[1]+work.KKT[485]*source[2]+work.KKT[486]*source[3]+work.KKT[487]*source[4]+work.KKT[488]*source[5]+work.KKT[489]*source[6];
  result[118] = work.KKT[81]*source[53]+work.KKT[490]*source[118]+work.KKT[491]*source[0]+work.KKT[492]*source[1]+work.KKT[493]*source[2]+work.KKT[494]*source[3]+work.KKT[495]*source[4]+work.KKT[496]*source[5]+work.KKT[497]*source[6];
  result[119] = work.KKT[83]*source[54]+work.KKT[498]*source[119]+work.KKT[499]*source[0]+work.KKT[500]*source[1]+work.KKT[501]*source[2]+work.KKT[502]*source[3]+work.KKT[503]*source[4]+work.KKT[504]*source[5]+work.KKT[505]*source[6];
  result[120] = work.KKT[85]*source[55]+work.KKT[506]*source[120]+work.KKT[507]*source[0]+work.KKT[508]*source[1]+work.KKT[509]*source[2]+work.KKT[510]*source[3]+work.KKT[511]*source[4]+work.KKT[512]*source[5]+work.KKT[513]*source[6];
  result[121] = work.KKT[87]*source[56]+work.KKT[514]*source[121]+work.KKT[515]*source[0]+work.KKT[516]*source[1]+work.KKT[517]*source[2]+work.KKT[518]*source[3]+work.KKT[519]*source[4]+work.KKT[520]*source[5]+work.KKT[521]*source[6];
  result[122] = work.KKT[89]*source[57]+work.KKT[522]*source[122]+work.KKT[523]*source[0]+work.KKT[524]*source[1]+work.KKT[525]*source[2]+work.KKT[526]*source[3]+work.KKT[527]*source[4]+work.KKT[528]*source[5]+work.KKT[529]*source[6];
  result[123] = work.KKT[91]*source[58]+work.KKT[530]*source[123]+work.KKT[531]*source[0]+work.KKT[532]*source[1]+work.KKT[533]*source[2]+work.KKT[534]*source[3]+work.KKT[535]*source[4]+work.KKT[536]*source[5]+work.KKT[537]*source[6];
  result[124] = work.KKT[93]*source[59]+work.KKT[538]*source[124]+work.KKT[539]*source[0]+work.KKT[540]*source[1]+work.KKT[541]*source[2]+work.KKT[542]*source[3]+work.KKT[543]*source[4]+work.KKT[544]*source[5]+work.KKT[545]*source[6];
  result[125] = work.KKT[95]*source[60]+work.KKT[546]*source[125]+work.KKT[547]*source[0]+work.KKT[548]*source[1]+work.KKT[549]*source[2]+work.KKT[550]*source[3]+work.KKT[551]*source[4]+work.KKT[552]*source[5]+work.KKT[553]*source[6];
  result[126] = work.KKT[97]*source[61]+work.KKT[554]*source[126]+work.KKT[555]*source[0]+work.KKT[556]*source[1]+work.KKT[557]*source[2]+work.KKT[558]*source[3]+work.KKT[559]*source[4]+work.KKT[560]*source[5]+work.KKT[561]*source[6];
  result[127] = work.KKT[99]*source[62]+work.KKT[562]*source[127]+work.KKT[563]*source[0]+work.KKT[564]*source[1]+work.KKT[565]*source[2]+work.KKT[566]*source[3]+work.KKT[567]*source[4]+work.KKT[568]*source[5]+work.KKT[569]*source[6];
  result[128] = work.KKT[101]*source[63]+work.KKT[570]*source[128]+work.KKT[571]*source[0]+work.KKT[572]*source[1]+work.KKT[573]*source[2]+work.KKT[574]*source[3]+work.KKT[575]*source[4]+work.KKT[576]*source[5]+work.KKT[577]*source[6];
  result[129] = work.KKT[103]*source[64]+work.KKT[142]*source[129]+work.KKT[143]*source[0];
  result[130] = work.KKT[105]*source[65]+work.KKT[146]*source[130]+work.KKT[147]*source[1];
  result[131] = work.KKT[107]*source[66]+work.KKT[150]*source[131]+work.KKT[151]*source[2];
  result[132] = work.KKT[109]*source[67]+work.KKT[154]*source[132]+work.KKT[155]*source[3];
  result[133] = work.KKT[111]*source[68]+work.KKT[158]*source[133]+work.KKT[159]*source[4];
  result[134] = work.KKT[113]*source[69]+work.KKT[162]*source[134]+work.KKT[163]*source[5];
  result[135] = work.KKT[115]*source[70]+work.KKT[166]*source[135]+work.KKT[167]*source[6];
  result[136] = work.KKT[117]*source[71]+work.KKT[144]*source[136]+work.KKT[145]*source[0];
  result[137] = work.KKT[119]*source[72]+work.KKT[148]*source[137]+work.KKT[149]*source[1];
  result[138] = work.KKT[121]*source[73]+work.KKT[152]*source[138]+work.KKT[153]*source[2];
  result[139] = work.KKT[123]*source[74]+work.KKT[156]*source[139]+work.KKT[157]*source[3];
  result[140] = work.KKT[125]*source[75]+work.KKT[160]*source[140]+work.KKT[161]*source[4];
  result[141] = work.KKT[127]*source[76]+work.KKT[164]*source[141]+work.KKT[165]*source[5];
  result[142] = work.KKT[129]*source[77]+work.KKT[168]*source[142]+work.KKT[169]*source[6];
  result[143] = work.KKT[578]*source[0]+work.KKT[579]*source[1]+work.KKT[580]*source[2]+work.KKT[581]*source[3]+work.KKT[582]*source[4]+work.KKT[583]*source[5]+work.KKT[584]*source[6]+work.KKT[131]*source[7];
  result[144] = work.KKT[585]*source[0]+work.KKT[586]*source[1]+work.KKT[587]*source[2]+work.KKT[588]*source[3]+work.KKT[589]*source[4]+work.KKT[590]*source[5]+work.KKT[591]*source[6]+work.KKT[133]*source[8];
  result[145] = work.KKT[592]*source[0]+work.KKT[593]*source[1]+work.KKT[594]*source[2]+work.KKT[595]*source[3]+work.KKT[596]*source[4]+work.KKT[597]*source[5]+work.KKT[598]*source[6]+work.KKT[135]*source[9];
  result[146] = work.KKT[599]*source[0]+work.KKT[600]*source[1]+work.KKT[601]*source[2]+work.KKT[602]*source[3]+work.KKT[603]*source[4]+work.KKT[604]*source[5]+work.KKT[605]*source[6]+work.KKT[137]*source[10];
  result[147] = work.KKT[606]*source[0]+work.KKT[607]*source[1]+work.KKT[608]*source[2]+work.KKT[609]*source[3]+work.KKT[610]*source[4]+work.KKT[611]*source[5]+work.KKT[612]*source[6]+work.KKT[139]*source[11];
  result[148] = work.KKT[613]*source[0]+work.KKT[614]*source[1]+work.KKT[615]*source[2]+work.KKT[616]*source[3]+work.KKT[617]*source[4]+work.KKT[618]*source[5]+work.KKT[619]*source[6]+work.KKT[141]*source[12];
}

double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;

  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 13; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}

void fill_KKT(void) {
  work.KKT[130] = 2;
  work.KKT[132] = 2;
  work.KKT[134] = 2;
  work.KKT[136] = 2;
  work.KKT[138] = 2;
  work.KKT[140] = 2;
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[16] = work.s_inv_z[8];
  work.KKT[18] = work.s_inv_z[9];
  work.KKT[20] = work.s_inv_z[10];
  work.KKT[22] = work.s_inv_z[11];
  work.KKT[24] = work.s_inv_z[12];
  work.KKT[26] = work.s_inv_z[13];
  work.KKT[28] = work.s_inv_z[14];
  work.KKT[30] = work.s_inv_z[15];
  work.KKT[32] = work.s_inv_z[16];
  work.KKT[34] = work.s_inv_z[17];
  work.KKT[36] = work.s_inv_z[18];
  work.KKT[38] = work.s_inv_z[19];
  work.KKT[40] = work.s_inv_z[20];
  work.KKT[42] = work.s_inv_z[21];
  work.KKT[44] = work.s_inv_z[22];
  work.KKT[46] = work.s_inv_z[23];
  work.KKT[48] = work.s_inv_z[24];
  work.KKT[50] = work.s_inv_z[25];
  work.KKT[52] = work.s_inv_z[26];
  work.KKT[54] = work.s_inv_z[27];
  work.KKT[56] = work.s_inv_z[28];
  work.KKT[58] = work.s_inv_z[29];
  work.KKT[60] = work.s_inv_z[30];
  work.KKT[62] = work.s_inv_z[31];
  work.KKT[64] = work.s_inv_z[32];
  work.KKT[66] = work.s_inv_z[33];
  work.KKT[68] = work.s_inv_z[34];
  work.KKT[70] = work.s_inv_z[35];
  work.KKT[72] = work.s_inv_z[36];
  work.KKT[74] = work.s_inv_z[37];
  work.KKT[76] = work.s_inv_z[38];
  work.KKT[78] = work.s_inv_z[39];
  work.KKT[80] = work.s_inv_z[40];
  work.KKT[82] = work.s_inv_z[41];
  work.KKT[84] = work.s_inv_z[42];
  work.KKT[86] = work.s_inv_z[43];
  work.KKT[88] = work.s_inv_z[44];
  work.KKT[90] = work.s_inv_z[45];
  work.KKT[92] = work.s_inv_z[46];
  work.KKT[94] = work.s_inv_z[47];
  work.KKT[96] = work.s_inv_z[48];
  work.KKT[98] = work.s_inv_z[49];
  work.KKT[100] = work.s_inv_z[50];
  work.KKT[102] = work.s_inv_z[51];
  work.KKT[104] = work.s_inv_z[52];
  work.KKT[106] = work.s_inv_z[53];
  work.KKT[108] = work.s_inv_z[54];
  work.KKT[110] = work.s_inv_z[55];
  work.KKT[112] = work.s_inv_z[56];
  work.KKT[114] = work.s_inv_z[57];
  work.KKT[116] = work.s_inv_z[58];
  work.KKT[118] = work.s_inv_z[59];
  work.KKT[120] = work.s_inv_z[60];
  work.KKT[122] = work.s_inv_z[61];
  work.KKT[124] = work.s_inv_z[62];
  work.KKT[126] = work.s_inv_z[63];
  work.KKT[128] = work.s_inv_z[64];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[17] = 1;
  work.KKT[19] = 1;
  work.KKT[21] = 1;
  work.KKT[23] = 1;
  work.KKT[25] = 1;
  work.KKT[27] = 1;
  work.KKT[29] = 1;
  work.KKT[31] = 1;
  work.KKT[33] = 1;
  work.KKT[35] = 1;
  work.KKT[37] = 1;
  work.KKT[39] = 1;
  work.KKT[41] = 1;
  work.KKT[43] = 1;
  work.KKT[45] = 1;
  work.KKT[47] = 1;
  work.KKT[49] = 1;
  work.KKT[51] = 1;
  work.KKT[53] = 1;
  work.KKT[55] = 1;
  work.KKT[57] = 1;
  work.KKT[59] = 1;
  work.KKT[61] = 1;
  work.KKT[63] = 1;
  work.KKT[65] = 1;
  work.KKT[67] = 1;
  work.KKT[69] = 1;
  work.KKT[71] = 1;
  work.KKT[73] = 1;
  work.KKT[75] = 1;
  work.KKT[77] = 1;
  work.KKT[79] = 1;
  work.KKT[81] = 1;
  work.KKT[83] = 1;
  work.KKT[85] = 1;
  work.KKT[87] = 1;
  work.KKT[89] = 1;
  work.KKT[91] = 1;
  work.KKT[93] = 1;
  work.KKT[95] = 1;
  work.KKT[97] = 1;
  work.KKT[99] = 1;
  work.KKT[101] = 1;
  work.KKT[103] = 1;
  work.KKT[105] = 1;
  work.KKT[107] = 1;
  work.KKT[109] = 1;
  work.KKT[111] = 1;
  work.KKT[113] = 1;
  work.KKT[115] = 1;
  work.KKT[117] = 1;
  work.KKT[119] = 1;
  work.KKT[121] = 1;
  work.KKT[123] = 1;
  work.KKT[125] = 1;
  work.KKT[127] = 1;
  work.KKT[129] = 1;
  work.KKT[170] = work.block_33[0];
  work.KKT[178] = work.block_33[0];
  work.KKT[186] = work.block_33[0];
  work.KKT[194] = work.block_33[0];
  work.KKT[202] = work.block_33[0];
  work.KKT[210] = work.block_33[0];
  work.KKT[218] = work.block_33[0];
  work.KKT[226] = work.block_33[0];
  work.KKT[234] = work.block_33[0];
  work.KKT[242] = work.block_33[0];
  work.KKT[250] = work.block_33[0];
  work.KKT[258] = work.block_33[0];
  work.KKT[266] = work.block_33[0];
  work.KKT[274] = work.block_33[0];
  work.KKT[282] = work.block_33[0];
  work.KKT[290] = work.block_33[0];
  work.KKT[298] = work.block_33[0];
  work.KKT[306] = work.block_33[0];
  work.KKT[314] = work.block_33[0];
  work.KKT[322] = work.block_33[0];
  work.KKT[330] = work.block_33[0];
  work.KKT[338] = work.block_33[0];
  work.KKT[346] = work.block_33[0];
  work.KKT[354] = work.block_33[0];
  work.KKT[362] = work.block_33[0];
  work.KKT[370] = work.block_33[0];
  work.KKT[378] = work.block_33[0];
  work.KKT[386] = work.block_33[0];
  work.KKT[394] = work.block_33[0];
  work.KKT[402] = work.block_33[0];
  work.KKT[410] = work.block_33[0];
  work.KKT[418] = work.block_33[0];
  work.KKT[426] = work.block_33[0];
  work.KKT[434] = work.block_33[0];
  work.KKT[442] = work.block_33[0];
  work.KKT[450] = work.block_33[0];
  work.KKT[458] = work.block_33[0];
  work.KKT[466] = work.block_33[0];
  work.KKT[474] = work.block_33[0];
  work.KKT[482] = work.block_33[0];
  work.KKT[490] = work.block_33[0];
  work.KKT[498] = work.block_33[0];
  work.KKT[506] = work.block_33[0];
  work.KKT[514] = work.block_33[0];
  work.KKT[522] = work.block_33[0];
  work.KKT[530] = work.block_33[0];
  work.KKT[538] = work.block_33[0];
  work.KKT[546] = work.block_33[0];
  work.KKT[554] = work.block_33[0];
  work.KKT[562] = work.block_33[0];
  work.KKT[570] = work.block_33[0];
  work.KKT[142] = work.block_33[0];
  work.KKT[146] = work.block_33[0];
  work.KKT[150] = work.block_33[0];
  work.KKT[154] = work.block_33[0];
  work.KKT[158] = work.block_33[0];
  work.KKT[162] = work.block_33[0];
  work.KKT[166] = work.block_33[0];
  work.KKT[144] = work.block_33[0];
  work.KKT[148] = work.block_33[0];
  work.KKT[152] = work.block_33[0];
  work.KKT[156] = work.block_33[0];
  work.KKT[160] = work.block_33[0];
  work.KKT[164] = work.block_33[0];
  work.KKT[168] = work.block_33[0];
  work.KKT[171] = -(params.normal_0[0]*params.Jac_0[0]+params.normal_0[1]*params.Jac_0[1]+params.normal_0[2]*params.Jac_0[2]);
  work.KKT[172] = -(params.normal_0[0]*params.Jac_0[3]+params.normal_0[1]*params.Jac_0[4]+params.normal_0[2]*params.Jac_0[5]);
  work.KKT[173] = -(params.normal_0[0]*params.Jac_0[6]+params.normal_0[1]*params.Jac_0[7]+params.normal_0[2]*params.Jac_0[8]);
  work.KKT[174] = -(params.normal_0[0]*params.Jac_0[9]+params.normal_0[1]*params.Jac_0[10]+params.normal_0[2]*params.Jac_0[11]);
  work.KKT[175] = -(params.normal_0[0]*params.Jac_0[12]+params.normal_0[1]*params.Jac_0[13]+params.normal_0[2]*params.Jac_0[14]);
  work.KKT[176] = -(params.normal_0[0]*params.Jac_0[15]+params.normal_0[1]*params.Jac_0[16]+params.normal_0[2]*params.Jac_0[17]);
  work.KKT[177] = -(params.normal_0[0]*params.Jac_0[18]+params.normal_0[1]*params.Jac_0[19]+params.normal_0[2]*params.Jac_0[20]);
  work.KKT[179] = -(params.normal_1[0]*params.Jac_1[0]+params.normal_1[1]*params.Jac_1[1]+params.normal_1[2]*params.Jac_1[2]);
  work.KKT[180] = -(params.normal_1[0]*params.Jac_1[3]+params.normal_1[1]*params.Jac_1[4]+params.normal_1[2]*params.Jac_1[5]);
  work.KKT[181] = -(params.normal_1[0]*params.Jac_1[6]+params.normal_1[1]*params.Jac_1[7]+params.normal_1[2]*params.Jac_1[8]);
  work.KKT[182] = -(params.normal_1[0]*params.Jac_1[9]+params.normal_1[1]*params.Jac_1[10]+params.normal_1[2]*params.Jac_1[11]);
  work.KKT[183] = -(params.normal_1[0]*params.Jac_1[12]+params.normal_1[1]*params.Jac_1[13]+params.normal_1[2]*params.Jac_1[14]);
  work.KKT[184] = -(params.normal_1[0]*params.Jac_1[15]+params.normal_1[1]*params.Jac_1[16]+params.normal_1[2]*params.Jac_1[17]);
  work.KKT[185] = -(params.normal_1[0]*params.Jac_1[18]+params.normal_1[1]*params.Jac_1[19]+params.normal_1[2]*params.Jac_1[20]);
  work.KKT[187] = -(params.normal_2[0]*params.Jac_2[0]+params.normal_2[1]*params.Jac_2[1]+params.normal_2[2]*params.Jac_2[2]);
  work.KKT[188] = -(params.normal_2[0]*params.Jac_2[3]+params.normal_2[1]*params.Jac_2[4]+params.normal_2[2]*params.Jac_2[5]);
  work.KKT[189] = -(params.normal_2[0]*params.Jac_2[6]+params.normal_2[1]*params.Jac_2[7]+params.normal_2[2]*params.Jac_2[8]);
  work.KKT[190] = -(params.normal_2[0]*params.Jac_2[9]+params.normal_2[1]*params.Jac_2[10]+params.normal_2[2]*params.Jac_2[11]);
  work.KKT[191] = -(params.normal_2[0]*params.Jac_2[12]+params.normal_2[1]*params.Jac_2[13]+params.normal_2[2]*params.Jac_2[14]);
  work.KKT[192] = -(params.normal_2[0]*params.Jac_2[15]+params.normal_2[1]*params.Jac_2[16]+params.normal_2[2]*params.Jac_2[17]);
  work.KKT[193] = -(params.normal_2[0]*params.Jac_2[18]+params.normal_2[1]*params.Jac_2[19]+params.normal_2[2]*params.Jac_2[20]);
  work.KKT[195] = -(params.normal_3[0]*params.Jac_3[0]+params.normal_3[1]*params.Jac_3[1]+params.normal_3[2]*params.Jac_3[2]);
  work.KKT[196] = -(params.normal_3[0]*params.Jac_3[3]+params.normal_3[1]*params.Jac_3[4]+params.normal_3[2]*params.Jac_3[5]);
  work.KKT[197] = -(params.normal_3[0]*params.Jac_3[6]+params.normal_3[1]*params.Jac_3[7]+params.normal_3[2]*params.Jac_3[8]);
  work.KKT[198] = -(params.normal_3[0]*params.Jac_3[9]+params.normal_3[1]*params.Jac_3[10]+params.normal_3[2]*params.Jac_3[11]);
  work.KKT[199] = -(params.normal_3[0]*params.Jac_3[12]+params.normal_3[1]*params.Jac_3[13]+params.normal_3[2]*params.Jac_3[14]);
  work.KKT[200] = -(params.normal_3[0]*params.Jac_3[15]+params.normal_3[1]*params.Jac_3[16]+params.normal_3[2]*params.Jac_3[17]);
  work.KKT[201] = -(params.normal_3[0]*params.Jac_3[18]+params.normal_3[1]*params.Jac_3[19]+params.normal_3[2]*params.Jac_3[20]);
  work.KKT[203] = -(params.normal_4[0]*params.Jac_4[0]+params.normal_4[1]*params.Jac_4[1]+params.normal_4[2]*params.Jac_4[2]);
  work.KKT[204] = -(params.normal_4[0]*params.Jac_4[3]+params.normal_4[1]*params.Jac_4[4]+params.normal_4[2]*params.Jac_4[5]);
  work.KKT[205] = -(params.normal_4[0]*params.Jac_4[6]+params.normal_4[1]*params.Jac_4[7]+params.normal_4[2]*params.Jac_4[8]);
  work.KKT[206] = -(params.normal_4[0]*params.Jac_4[9]+params.normal_4[1]*params.Jac_4[10]+params.normal_4[2]*params.Jac_4[11]);
  work.KKT[207] = -(params.normal_4[0]*params.Jac_4[12]+params.normal_4[1]*params.Jac_4[13]+params.normal_4[2]*params.Jac_4[14]);
  work.KKT[208] = -(params.normal_4[0]*params.Jac_4[15]+params.normal_4[1]*params.Jac_4[16]+params.normal_4[2]*params.Jac_4[17]);
  work.KKT[209] = -(params.normal_4[0]*params.Jac_4[18]+params.normal_4[1]*params.Jac_4[19]+params.normal_4[2]*params.Jac_4[20]);
  work.KKT[211] = -(params.normal_5[0]*params.Jac_5[0]+params.normal_5[1]*params.Jac_5[1]+params.normal_5[2]*params.Jac_5[2]);
  work.KKT[212] = -(params.normal_5[0]*params.Jac_5[3]+params.normal_5[1]*params.Jac_5[4]+params.normal_5[2]*params.Jac_5[5]);
  work.KKT[213] = -(params.normal_5[0]*params.Jac_5[6]+params.normal_5[1]*params.Jac_5[7]+params.normal_5[2]*params.Jac_5[8]);
  work.KKT[214] = -(params.normal_5[0]*params.Jac_5[9]+params.normal_5[1]*params.Jac_5[10]+params.normal_5[2]*params.Jac_5[11]);
  work.KKT[215] = -(params.normal_5[0]*params.Jac_5[12]+params.normal_5[1]*params.Jac_5[13]+params.normal_5[2]*params.Jac_5[14]);
  work.KKT[216] = -(params.normal_5[0]*params.Jac_5[15]+params.normal_5[1]*params.Jac_5[16]+params.normal_5[2]*params.Jac_5[17]);
  work.KKT[217] = -(params.normal_5[0]*params.Jac_5[18]+params.normal_5[1]*params.Jac_5[19]+params.normal_5[2]*params.Jac_5[20]);
  work.KKT[219] = -(params.normal_6[0]*params.Jac_6[0]+params.normal_6[1]*params.Jac_6[1]+params.normal_6[2]*params.Jac_6[2]);
  work.KKT[220] = -(params.normal_6[0]*params.Jac_6[3]+params.normal_6[1]*params.Jac_6[4]+params.normal_6[2]*params.Jac_6[5]);
  work.KKT[221] = -(params.normal_6[0]*params.Jac_6[6]+params.normal_6[1]*params.Jac_6[7]+params.normal_6[2]*params.Jac_6[8]);
  work.KKT[222] = -(params.normal_6[0]*params.Jac_6[9]+params.normal_6[1]*params.Jac_6[10]+params.normal_6[2]*params.Jac_6[11]);
  work.KKT[223] = -(params.normal_6[0]*params.Jac_6[12]+params.normal_6[1]*params.Jac_6[13]+params.normal_6[2]*params.Jac_6[14]);
  work.KKT[224] = -(params.normal_6[0]*params.Jac_6[15]+params.normal_6[1]*params.Jac_6[16]+params.normal_6[2]*params.Jac_6[17]);
  work.KKT[225] = -(params.normal_6[0]*params.Jac_6[18]+params.normal_6[1]*params.Jac_6[19]+params.normal_6[2]*params.Jac_6[20]);
  work.KKT[227] = -(params.normal_7[0]*params.Jac_7[0]+params.normal_7[1]*params.Jac_7[1]+params.normal_7[2]*params.Jac_7[2]);
  work.KKT[228] = -(params.normal_7[0]*params.Jac_7[3]+params.normal_7[1]*params.Jac_7[4]+params.normal_7[2]*params.Jac_7[5]);
  work.KKT[229] = -(params.normal_7[0]*params.Jac_7[6]+params.normal_7[1]*params.Jac_7[7]+params.normal_7[2]*params.Jac_7[8]);
  work.KKT[230] = -(params.normal_7[0]*params.Jac_7[9]+params.normal_7[1]*params.Jac_7[10]+params.normal_7[2]*params.Jac_7[11]);
  work.KKT[231] = -(params.normal_7[0]*params.Jac_7[12]+params.normal_7[1]*params.Jac_7[13]+params.normal_7[2]*params.Jac_7[14]);
  work.KKT[232] = -(params.normal_7[0]*params.Jac_7[15]+params.normal_7[1]*params.Jac_7[16]+params.normal_7[2]*params.Jac_7[17]);
  work.KKT[233] = -(params.normal_7[0]*params.Jac_7[18]+params.normal_7[1]*params.Jac_7[19]+params.normal_7[2]*params.Jac_7[20]);
  work.KKT[235] = -(params.normal_8[0]*params.Jac_8[0]+params.normal_8[1]*params.Jac_8[1]+params.normal_8[2]*params.Jac_8[2]);
  work.KKT[236] = -(params.normal_8[0]*params.Jac_8[3]+params.normal_8[1]*params.Jac_8[4]+params.normal_8[2]*params.Jac_8[5]);
  work.KKT[237] = -(params.normal_8[0]*params.Jac_8[6]+params.normal_8[1]*params.Jac_8[7]+params.normal_8[2]*params.Jac_8[8]);
  work.KKT[238] = -(params.normal_8[0]*params.Jac_8[9]+params.normal_8[1]*params.Jac_8[10]+params.normal_8[2]*params.Jac_8[11]);
  work.KKT[239] = -(params.normal_8[0]*params.Jac_8[12]+params.normal_8[1]*params.Jac_8[13]+params.normal_8[2]*params.Jac_8[14]);
  work.KKT[240] = -(params.normal_8[0]*params.Jac_8[15]+params.normal_8[1]*params.Jac_8[16]+params.normal_8[2]*params.Jac_8[17]);
  work.KKT[241] = -(params.normal_8[0]*params.Jac_8[18]+params.normal_8[1]*params.Jac_8[19]+params.normal_8[2]*params.Jac_8[20]);
  work.KKT[243] = -(params.normal_9[0]*params.Jac_9[0]+params.normal_9[1]*params.Jac_9[1]+params.normal_9[2]*params.Jac_9[2]);
  work.KKT[244] = -(params.normal_9[0]*params.Jac_9[3]+params.normal_9[1]*params.Jac_9[4]+params.normal_9[2]*params.Jac_9[5]);
  work.KKT[245] = -(params.normal_9[0]*params.Jac_9[6]+params.normal_9[1]*params.Jac_9[7]+params.normal_9[2]*params.Jac_9[8]);
  work.KKT[246] = -(params.normal_9[0]*params.Jac_9[9]+params.normal_9[1]*params.Jac_9[10]+params.normal_9[2]*params.Jac_9[11]);
  work.KKT[247] = -(params.normal_9[0]*params.Jac_9[12]+params.normal_9[1]*params.Jac_9[13]+params.normal_9[2]*params.Jac_9[14]);
  work.KKT[248] = -(params.normal_9[0]*params.Jac_9[15]+params.normal_9[1]*params.Jac_9[16]+params.normal_9[2]*params.Jac_9[17]);
  work.KKT[249] = -(params.normal_9[0]*params.Jac_9[18]+params.normal_9[1]*params.Jac_9[19]+params.normal_9[2]*params.Jac_9[20]);
  work.KKT[251] = -(params.normal_10[0]*params.Jac_10[0]+params.normal_10[1]*params.Jac_10[1]+params.normal_10[2]*params.Jac_10[2]);
  work.KKT[252] = -(params.normal_10[0]*params.Jac_10[3]+params.normal_10[1]*params.Jac_10[4]+params.normal_10[2]*params.Jac_10[5]);
  work.KKT[253] = -(params.normal_10[0]*params.Jac_10[6]+params.normal_10[1]*params.Jac_10[7]+params.normal_10[2]*params.Jac_10[8]);
  work.KKT[254] = -(params.normal_10[0]*params.Jac_10[9]+params.normal_10[1]*params.Jac_10[10]+params.normal_10[2]*params.Jac_10[11]);
  work.KKT[255] = -(params.normal_10[0]*params.Jac_10[12]+params.normal_10[1]*params.Jac_10[13]+params.normal_10[2]*params.Jac_10[14]);
  work.KKT[256] = -(params.normal_10[0]*params.Jac_10[15]+params.normal_10[1]*params.Jac_10[16]+params.normal_10[2]*params.Jac_10[17]);
  work.KKT[257] = -(params.normal_10[0]*params.Jac_10[18]+params.normal_10[1]*params.Jac_10[19]+params.normal_10[2]*params.Jac_10[20]);
  work.KKT[259] = -(params.normal_11[0]*params.Jac_11[0]+params.normal_11[1]*params.Jac_11[1]+params.normal_11[2]*params.Jac_11[2]);
  work.KKT[260] = -(params.normal_11[0]*params.Jac_11[3]+params.normal_11[1]*params.Jac_11[4]+params.normal_11[2]*params.Jac_11[5]);
  work.KKT[261] = -(params.normal_11[0]*params.Jac_11[6]+params.normal_11[1]*params.Jac_11[7]+params.normal_11[2]*params.Jac_11[8]);
  work.KKT[262] = -(params.normal_11[0]*params.Jac_11[9]+params.normal_11[1]*params.Jac_11[10]+params.normal_11[2]*params.Jac_11[11]);
  work.KKT[263] = -(params.normal_11[0]*params.Jac_11[12]+params.normal_11[1]*params.Jac_11[13]+params.normal_11[2]*params.Jac_11[14]);
  work.KKT[264] = -(params.normal_11[0]*params.Jac_11[15]+params.normal_11[1]*params.Jac_11[16]+params.normal_11[2]*params.Jac_11[17]);
  work.KKT[265] = -(params.normal_11[0]*params.Jac_11[18]+params.normal_11[1]*params.Jac_11[19]+params.normal_11[2]*params.Jac_11[20]);
  work.KKT[267] = -(params.normal_12[0]*params.Jac_12[0]+params.normal_12[1]*params.Jac_12[1]+params.normal_12[2]*params.Jac_12[2]);
  work.KKT[268] = -(params.normal_12[0]*params.Jac_12[3]+params.normal_12[1]*params.Jac_12[4]+params.normal_12[2]*params.Jac_12[5]);
  work.KKT[269] = -(params.normal_12[0]*params.Jac_12[6]+params.normal_12[1]*params.Jac_12[7]+params.normal_12[2]*params.Jac_12[8]);
  work.KKT[270] = -(params.normal_12[0]*params.Jac_12[9]+params.normal_12[1]*params.Jac_12[10]+params.normal_12[2]*params.Jac_12[11]);
  work.KKT[271] = -(params.normal_12[0]*params.Jac_12[12]+params.normal_12[1]*params.Jac_12[13]+params.normal_12[2]*params.Jac_12[14]);
  work.KKT[272] = -(params.normal_12[0]*params.Jac_12[15]+params.normal_12[1]*params.Jac_12[16]+params.normal_12[2]*params.Jac_12[17]);
  work.KKT[273] = -(params.normal_12[0]*params.Jac_12[18]+params.normal_12[1]*params.Jac_12[19]+params.normal_12[2]*params.Jac_12[20]);
  work.KKT[275] = -(params.normal_13[0]*params.Jac_13[0]+params.normal_13[1]*params.Jac_13[1]+params.normal_13[2]*params.Jac_13[2]);
  work.KKT[276] = -(params.normal_13[0]*params.Jac_13[3]+params.normal_13[1]*params.Jac_13[4]+params.normal_13[2]*params.Jac_13[5]);
  work.KKT[277] = -(params.normal_13[0]*params.Jac_13[6]+params.normal_13[1]*params.Jac_13[7]+params.normal_13[2]*params.Jac_13[8]);
  work.KKT[278] = -(params.normal_13[0]*params.Jac_13[9]+params.normal_13[1]*params.Jac_13[10]+params.normal_13[2]*params.Jac_13[11]);
  work.KKT[279] = -(params.normal_13[0]*params.Jac_13[12]+params.normal_13[1]*params.Jac_13[13]+params.normal_13[2]*params.Jac_13[14]);
  work.KKT[280] = -(params.normal_13[0]*params.Jac_13[15]+params.normal_13[1]*params.Jac_13[16]+params.normal_13[2]*params.Jac_13[17]);
  work.KKT[281] = -(params.normal_13[0]*params.Jac_13[18]+params.normal_13[1]*params.Jac_13[19]+params.normal_13[2]*params.Jac_13[20]);
  work.KKT[283] = -(params.normal_14[0]*params.Jac_14[0]+params.normal_14[1]*params.Jac_14[1]+params.normal_14[2]*params.Jac_14[2]);
  work.KKT[284] = -(params.normal_14[0]*params.Jac_14[3]+params.normal_14[1]*params.Jac_14[4]+params.normal_14[2]*params.Jac_14[5]);
  work.KKT[285] = -(params.normal_14[0]*params.Jac_14[6]+params.normal_14[1]*params.Jac_14[7]+params.normal_14[2]*params.Jac_14[8]);
  work.KKT[286] = -(params.normal_14[0]*params.Jac_14[9]+params.normal_14[1]*params.Jac_14[10]+params.normal_14[2]*params.Jac_14[11]);
  work.KKT[287] = -(params.normal_14[0]*params.Jac_14[12]+params.normal_14[1]*params.Jac_14[13]+params.normal_14[2]*params.Jac_14[14]);
  work.KKT[288] = -(params.normal_14[0]*params.Jac_14[15]+params.normal_14[1]*params.Jac_14[16]+params.normal_14[2]*params.Jac_14[17]);
  work.KKT[289] = -(params.normal_14[0]*params.Jac_14[18]+params.normal_14[1]*params.Jac_14[19]+params.normal_14[2]*params.Jac_14[20]);
  work.KKT[291] = -(params.normal_15[0]*params.Jac_15[0]+params.normal_15[1]*params.Jac_15[1]+params.normal_15[2]*params.Jac_15[2]);
  work.KKT[292] = -(params.normal_15[0]*params.Jac_15[3]+params.normal_15[1]*params.Jac_15[4]+params.normal_15[2]*params.Jac_15[5]);
  work.KKT[293] = -(params.normal_15[0]*params.Jac_15[6]+params.normal_15[1]*params.Jac_15[7]+params.normal_15[2]*params.Jac_15[8]);
  work.KKT[294] = -(params.normal_15[0]*params.Jac_15[9]+params.normal_15[1]*params.Jac_15[10]+params.normal_15[2]*params.Jac_15[11]);
  work.KKT[295] = -(params.normal_15[0]*params.Jac_15[12]+params.normal_15[1]*params.Jac_15[13]+params.normal_15[2]*params.Jac_15[14]);
  work.KKT[296] = -(params.normal_15[0]*params.Jac_15[15]+params.normal_15[1]*params.Jac_15[16]+params.normal_15[2]*params.Jac_15[17]);
  work.KKT[297] = -(params.normal_15[0]*params.Jac_15[18]+params.normal_15[1]*params.Jac_15[19]+params.normal_15[2]*params.Jac_15[20]);
  work.KKT[299] = -(params.normal_16[0]*params.Jac_16[0]+params.normal_16[1]*params.Jac_16[1]+params.normal_16[2]*params.Jac_16[2]);
  work.KKT[300] = -(params.normal_16[0]*params.Jac_16[3]+params.normal_16[1]*params.Jac_16[4]+params.normal_16[2]*params.Jac_16[5]);
  work.KKT[301] = -(params.normal_16[0]*params.Jac_16[6]+params.normal_16[1]*params.Jac_16[7]+params.normal_16[2]*params.Jac_16[8]);
  work.KKT[302] = -(params.normal_16[0]*params.Jac_16[9]+params.normal_16[1]*params.Jac_16[10]+params.normal_16[2]*params.Jac_16[11]);
  work.KKT[303] = -(params.normal_16[0]*params.Jac_16[12]+params.normal_16[1]*params.Jac_16[13]+params.normal_16[2]*params.Jac_16[14]);
  work.KKT[304] = -(params.normal_16[0]*params.Jac_16[15]+params.normal_16[1]*params.Jac_16[16]+params.normal_16[2]*params.Jac_16[17]);
  work.KKT[305] = -(params.normal_16[0]*params.Jac_16[18]+params.normal_16[1]*params.Jac_16[19]+params.normal_16[2]*params.Jac_16[20]);
  work.KKT[307] = -(params.normal_17[0]*params.Jac_17[0]+params.normal_17[1]*params.Jac_17[1]+params.normal_17[2]*params.Jac_17[2]);
  work.KKT[308] = -(params.normal_17[0]*params.Jac_17[3]+params.normal_17[1]*params.Jac_17[4]+params.normal_17[2]*params.Jac_17[5]);
  work.KKT[309] = -(params.normal_17[0]*params.Jac_17[6]+params.normal_17[1]*params.Jac_17[7]+params.normal_17[2]*params.Jac_17[8]);
  work.KKT[310] = -(params.normal_17[0]*params.Jac_17[9]+params.normal_17[1]*params.Jac_17[10]+params.normal_17[2]*params.Jac_17[11]);
  work.KKT[311] = -(params.normal_17[0]*params.Jac_17[12]+params.normal_17[1]*params.Jac_17[13]+params.normal_17[2]*params.Jac_17[14]);
  work.KKT[312] = -(params.normal_17[0]*params.Jac_17[15]+params.normal_17[1]*params.Jac_17[16]+params.normal_17[2]*params.Jac_17[17]);
  work.KKT[313] = -(params.normal_17[0]*params.Jac_17[18]+params.normal_17[1]*params.Jac_17[19]+params.normal_17[2]*params.Jac_17[20]);
  work.KKT[315] = -(params.normal_18[0]*params.Jac_18[0]+params.normal_18[1]*params.Jac_18[1]+params.normal_18[2]*params.Jac_18[2]);
  work.KKT[316] = -(params.normal_18[0]*params.Jac_18[3]+params.normal_18[1]*params.Jac_18[4]+params.normal_18[2]*params.Jac_18[5]);
  work.KKT[317] = -(params.normal_18[0]*params.Jac_18[6]+params.normal_18[1]*params.Jac_18[7]+params.normal_18[2]*params.Jac_18[8]);
  work.KKT[318] = -(params.normal_18[0]*params.Jac_18[9]+params.normal_18[1]*params.Jac_18[10]+params.normal_18[2]*params.Jac_18[11]);
  work.KKT[319] = -(params.normal_18[0]*params.Jac_18[12]+params.normal_18[1]*params.Jac_18[13]+params.normal_18[2]*params.Jac_18[14]);
  work.KKT[320] = -(params.normal_18[0]*params.Jac_18[15]+params.normal_18[1]*params.Jac_18[16]+params.normal_18[2]*params.Jac_18[17]);
  work.KKT[321] = -(params.normal_18[0]*params.Jac_18[18]+params.normal_18[1]*params.Jac_18[19]+params.normal_18[2]*params.Jac_18[20]);
  work.KKT[323] = -(params.normal_19[0]*params.Jac_19[0]+params.normal_19[1]*params.Jac_19[1]+params.normal_19[2]*params.Jac_19[2]);
  work.KKT[324] = -(params.normal_19[0]*params.Jac_19[3]+params.normal_19[1]*params.Jac_19[4]+params.normal_19[2]*params.Jac_19[5]);
  work.KKT[325] = -(params.normal_19[0]*params.Jac_19[6]+params.normal_19[1]*params.Jac_19[7]+params.normal_19[2]*params.Jac_19[8]);
  work.KKT[326] = -(params.normal_19[0]*params.Jac_19[9]+params.normal_19[1]*params.Jac_19[10]+params.normal_19[2]*params.Jac_19[11]);
  work.KKT[327] = -(params.normal_19[0]*params.Jac_19[12]+params.normal_19[1]*params.Jac_19[13]+params.normal_19[2]*params.Jac_19[14]);
  work.KKT[328] = -(params.normal_19[0]*params.Jac_19[15]+params.normal_19[1]*params.Jac_19[16]+params.normal_19[2]*params.Jac_19[17]);
  work.KKT[329] = -(params.normal_19[0]*params.Jac_19[18]+params.normal_19[1]*params.Jac_19[19]+params.normal_19[2]*params.Jac_19[20]);
  work.KKT[331] = -(params.normal_20[0]*params.Jac_20[0]+params.normal_20[1]*params.Jac_20[1]+params.normal_20[2]*params.Jac_20[2]);
  work.KKT[332] = -(params.normal_20[0]*params.Jac_20[3]+params.normal_20[1]*params.Jac_20[4]+params.normal_20[2]*params.Jac_20[5]);
  work.KKT[333] = -(params.normal_20[0]*params.Jac_20[6]+params.normal_20[1]*params.Jac_20[7]+params.normal_20[2]*params.Jac_20[8]);
  work.KKT[334] = -(params.normal_20[0]*params.Jac_20[9]+params.normal_20[1]*params.Jac_20[10]+params.normal_20[2]*params.Jac_20[11]);
  work.KKT[335] = -(params.normal_20[0]*params.Jac_20[12]+params.normal_20[1]*params.Jac_20[13]+params.normal_20[2]*params.Jac_20[14]);
  work.KKT[336] = -(params.normal_20[0]*params.Jac_20[15]+params.normal_20[1]*params.Jac_20[16]+params.normal_20[2]*params.Jac_20[17]);
  work.KKT[337] = -(params.normal_20[0]*params.Jac_20[18]+params.normal_20[1]*params.Jac_20[19]+params.normal_20[2]*params.Jac_20[20]);
  work.KKT[339] = -(params.normal_21[0]*params.Jac_21[0]+params.normal_21[1]*params.Jac_21[1]+params.normal_21[2]*params.Jac_21[2]);
  work.KKT[340] = -(params.normal_21[0]*params.Jac_21[3]+params.normal_21[1]*params.Jac_21[4]+params.normal_21[2]*params.Jac_21[5]);
  work.KKT[341] = -(params.normal_21[0]*params.Jac_21[6]+params.normal_21[1]*params.Jac_21[7]+params.normal_21[2]*params.Jac_21[8]);
  work.KKT[342] = -(params.normal_21[0]*params.Jac_21[9]+params.normal_21[1]*params.Jac_21[10]+params.normal_21[2]*params.Jac_21[11]);
  work.KKT[343] = -(params.normal_21[0]*params.Jac_21[12]+params.normal_21[1]*params.Jac_21[13]+params.normal_21[2]*params.Jac_21[14]);
  work.KKT[344] = -(params.normal_21[0]*params.Jac_21[15]+params.normal_21[1]*params.Jac_21[16]+params.normal_21[2]*params.Jac_21[17]);
  work.KKT[345] = -(params.normal_21[0]*params.Jac_21[18]+params.normal_21[1]*params.Jac_21[19]+params.normal_21[2]*params.Jac_21[20]);
  work.KKT[347] = -(params.normal_22[0]*params.Jac_22[0]+params.normal_22[1]*params.Jac_22[1]+params.normal_22[2]*params.Jac_22[2]);
  work.KKT[348] = -(params.normal_22[0]*params.Jac_22[3]+params.normal_22[1]*params.Jac_22[4]+params.normal_22[2]*params.Jac_22[5]);
  work.KKT[349] = -(params.normal_22[0]*params.Jac_22[6]+params.normal_22[1]*params.Jac_22[7]+params.normal_22[2]*params.Jac_22[8]);
  work.KKT[350] = -(params.normal_22[0]*params.Jac_22[9]+params.normal_22[1]*params.Jac_22[10]+params.normal_22[2]*params.Jac_22[11]);
  work.KKT[351] = -(params.normal_22[0]*params.Jac_22[12]+params.normal_22[1]*params.Jac_22[13]+params.normal_22[2]*params.Jac_22[14]);
  work.KKT[352] = -(params.normal_22[0]*params.Jac_22[15]+params.normal_22[1]*params.Jac_22[16]+params.normal_22[2]*params.Jac_22[17]);
  work.KKT[353] = -(params.normal_22[0]*params.Jac_22[18]+params.normal_22[1]*params.Jac_22[19]+params.normal_22[2]*params.Jac_22[20]);
  work.KKT[355] = -(params.normal_23[0]*params.Jac_23[0]+params.normal_23[1]*params.Jac_23[1]+params.normal_23[2]*params.Jac_23[2]);
  work.KKT[356] = -(params.normal_23[0]*params.Jac_23[3]+params.normal_23[1]*params.Jac_23[4]+params.normal_23[2]*params.Jac_23[5]);
  work.KKT[357] = -(params.normal_23[0]*params.Jac_23[6]+params.normal_23[1]*params.Jac_23[7]+params.normal_23[2]*params.Jac_23[8]);
  work.KKT[358] = -(params.normal_23[0]*params.Jac_23[9]+params.normal_23[1]*params.Jac_23[10]+params.normal_23[2]*params.Jac_23[11]);
  work.KKT[359] = -(params.normal_23[0]*params.Jac_23[12]+params.normal_23[1]*params.Jac_23[13]+params.normal_23[2]*params.Jac_23[14]);
  work.KKT[360] = -(params.normal_23[0]*params.Jac_23[15]+params.normal_23[1]*params.Jac_23[16]+params.normal_23[2]*params.Jac_23[17]);
  work.KKT[361] = -(params.normal_23[0]*params.Jac_23[18]+params.normal_23[1]*params.Jac_23[19]+params.normal_23[2]*params.Jac_23[20]);
  work.KKT[363] = -(params.normal_24[0]*params.Jac_24[0]+params.normal_24[1]*params.Jac_24[1]+params.normal_24[2]*params.Jac_24[2]);
  work.KKT[364] = -(params.normal_24[0]*params.Jac_24[3]+params.normal_24[1]*params.Jac_24[4]+params.normal_24[2]*params.Jac_24[5]);
  work.KKT[365] = -(params.normal_24[0]*params.Jac_24[6]+params.normal_24[1]*params.Jac_24[7]+params.normal_24[2]*params.Jac_24[8]);
  work.KKT[366] = -(params.normal_24[0]*params.Jac_24[9]+params.normal_24[1]*params.Jac_24[10]+params.normal_24[2]*params.Jac_24[11]);
  work.KKT[367] = -(params.normal_24[0]*params.Jac_24[12]+params.normal_24[1]*params.Jac_24[13]+params.normal_24[2]*params.Jac_24[14]);
  work.KKT[368] = -(params.normal_24[0]*params.Jac_24[15]+params.normal_24[1]*params.Jac_24[16]+params.normal_24[2]*params.Jac_24[17]);
  work.KKT[369] = -(params.normal_24[0]*params.Jac_24[18]+params.normal_24[1]*params.Jac_24[19]+params.normal_24[2]*params.Jac_24[20]);
  work.KKT[371] = -(params.normal_25[0]*params.Jac_25[0]+params.normal_25[1]*params.Jac_25[1]+params.normal_25[2]*params.Jac_25[2]);
  work.KKT[372] = -(params.normal_25[0]*params.Jac_25[3]+params.normal_25[1]*params.Jac_25[4]+params.normal_25[2]*params.Jac_25[5]);
  work.KKT[373] = -(params.normal_25[0]*params.Jac_25[6]+params.normal_25[1]*params.Jac_25[7]+params.normal_25[2]*params.Jac_25[8]);
  work.KKT[374] = -(params.normal_25[0]*params.Jac_25[9]+params.normal_25[1]*params.Jac_25[10]+params.normal_25[2]*params.Jac_25[11]);
  work.KKT[375] = -(params.normal_25[0]*params.Jac_25[12]+params.normal_25[1]*params.Jac_25[13]+params.normal_25[2]*params.Jac_25[14]);
  work.KKT[376] = -(params.normal_25[0]*params.Jac_25[15]+params.normal_25[1]*params.Jac_25[16]+params.normal_25[2]*params.Jac_25[17]);
  work.KKT[377] = -(params.normal_25[0]*params.Jac_25[18]+params.normal_25[1]*params.Jac_25[19]+params.normal_25[2]*params.Jac_25[20]);
  work.KKT[379] = -(params.normal_26[0]*params.Jac_26[0]+params.normal_26[1]*params.Jac_26[1]+params.normal_26[2]*params.Jac_26[2]);
  work.KKT[380] = -(params.normal_26[0]*params.Jac_26[3]+params.normal_26[1]*params.Jac_26[4]+params.normal_26[2]*params.Jac_26[5]);
  work.KKT[381] = -(params.normal_26[0]*params.Jac_26[6]+params.normal_26[1]*params.Jac_26[7]+params.normal_26[2]*params.Jac_26[8]);
  work.KKT[382] = -(params.normal_26[0]*params.Jac_26[9]+params.normal_26[1]*params.Jac_26[10]+params.normal_26[2]*params.Jac_26[11]);
  work.KKT[383] = -(params.normal_26[0]*params.Jac_26[12]+params.normal_26[1]*params.Jac_26[13]+params.normal_26[2]*params.Jac_26[14]);
  work.KKT[384] = -(params.normal_26[0]*params.Jac_26[15]+params.normal_26[1]*params.Jac_26[16]+params.normal_26[2]*params.Jac_26[17]);
  work.KKT[385] = -(params.normal_26[0]*params.Jac_26[18]+params.normal_26[1]*params.Jac_26[19]+params.normal_26[2]*params.Jac_26[20]);
  work.KKT[387] = -(params.normal_27[0]*params.Jac_27[0]+params.normal_27[1]*params.Jac_27[1]+params.normal_27[2]*params.Jac_27[2]);
  work.KKT[388] = -(params.normal_27[0]*params.Jac_27[3]+params.normal_27[1]*params.Jac_27[4]+params.normal_27[2]*params.Jac_27[5]);
  work.KKT[389] = -(params.normal_27[0]*params.Jac_27[6]+params.normal_27[1]*params.Jac_27[7]+params.normal_27[2]*params.Jac_27[8]);
  work.KKT[390] = -(params.normal_27[0]*params.Jac_27[9]+params.normal_27[1]*params.Jac_27[10]+params.normal_27[2]*params.Jac_27[11]);
  work.KKT[391] = -(params.normal_27[0]*params.Jac_27[12]+params.normal_27[1]*params.Jac_27[13]+params.normal_27[2]*params.Jac_27[14]);
  work.KKT[392] = -(params.normal_27[0]*params.Jac_27[15]+params.normal_27[1]*params.Jac_27[16]+params.normal_27[2]*params.Jac_27[17]);
  work.KKT[393] = -(params.normal_27[0]*params.Jac_27[18]+params.normal_27[1]*params.Jac_27[19]+params.normal_27[2]*params.Jac_27[20]);
  work.KKT[395] = -(params.normal_28[0]*params.Jac_28[0]+params.normal_28[1]*params.Jac_28[1]+params.normal_28[2]*params.Jac_28[2]);
  work.KKT[396] = -(params.normal_28[0]*params.Jac_28[3]+params.normal_28[1]*params.Jac_28[4]+params.normal_28[2]*params.Jac_28[5]);
  work.KKT[397] = -(params.normal_28[0]*params.Jac_28[6]+params.normal_28[1]*params.Jac_28[7]+params.normal_28[2]*params.Jac_28[8]);
  work.KKT[398] = -(params.normal_28[0]*params.Jac_28[9]+params.normal_28[1]*params.Jac_28[10]+params.normal_28[2]*params.Jac_28[11]);
  work.KKT[399] = -(params.normal_28[0]*params.Jac_28[12]+params.normal_28[1]*params.Jac_28[13]+params.normal_28[2]*params.Jac_28[14]);
  work.KKT[400] = -(params.normal_28[0]*params.Jac_28[15]+params.normal_28[1]*params.Jac_28[16]+params.normal_28[2]*params.Jac_28[17]);
  work.KKT[401] = -(params.normal_28[0]*params.Jac_28[18]+params.normal_28[1]*params.Jac_28[19]+params.normal_28[2]*params.Jac_28[20]);
  work.KKT[403] = -(params.normal_29[0]*params.Jac_29[0]+params.normal_29[1]*params.Jac_29[1]+params.normal_29[2]*params.Jac_29[2]);
  work.KKT[404] = -(params.normal_29[0]*params.Jac_29[3]+params.normal_29[1]*params.Jac_29[4]+params.normal_29[2]*params.Jac_29[5]);
  work.KKT[405] = -(params.normal_29[0]*params.Jac_29[6]+params.normal_29[1]*params.Jac_29[7]+params.normal_29[2]*params.Jac_29[8]);
  work.KKT[406] = -(params.normal_29[0]*params.Jac_29[9]+params.normal_29[1]*params.Jac_29[10]+params.normal_29[2]*params.Jac_29[11]);
  work.KKT[407] = -(params.normal_29[0]*params.Jac_29[12]+params.normal_29[1]*params.Jac_29[13]+params.normal_29[2]*params.Jac_29[14]);
  work.KKT[408] = -(params.normal_29[0]*params.Jac_29[15]+params.normal_29[1]*params.Jac_29[16]+params.normal_29[2]*params.Jac_29[17]);
  work.KKT[409] = -(params.normal_29[0]*params.Jac_29[18]+params.normal_29[1]*params.Jac_29[19]+params.normal_29[2]*params.Jac_29[20]);
  work.KKT[411] = -(params.normal_30[0]*params.Jac_30[0]+params.normal_30[1]*params.Jac_30[1]+params.normal_30[2]*params.Jac_30[2]);
  work.KKT[412] = -(params.normal_30[0]*params.Jac_30[3]+params.normal_30[1]*params.Jac_30[4]+params.normal_30[2]*params.Jac_30[5]);
  work.KKT[413] = -(params.normal_30[0]*params.Jac_30[6]+params.normal_30[1]*params.Jac_30[7]+params.normal_30[2]*params.Jac_30[8]);
  work.KKT[414] = -(params.normal_30[0]*params.Jac_30[9]+params.normal_30[1]*params.Jac_30[10]+params.normal_30[2]*params.Jac_30[11]);
  work.KKT[415] = -(params.normal_30[0]*params.Jac_30[12]+params.normal_30[1]*params.Jac_30[13]+params.normal_30[2]*params.Jac_30[14]);
  work.KKT[416] = -(params.normal_30[0]*params.Jac_30[15]+params.normal_30[1]*params.Jac_30[16]+params.normal_30[2]*params.Jac_30[17]);
  work.KKT[417] = -(params.normal_30[0]*params.Jac_30[18]+params.normal_30[1]*params.Jac_30[19]+params.normal_30[2]*params.Jac_30[20]);
  work.KKT[419] = -(params.normal_31[0]*params.Jac_31[0]+params.normal_31[1]*params.Jac_31[1]+params.normal_31[2]*params.Jac_31[2]);
  work.KKT[420] = -(params.normal_31[0]*params.Jac_31[3]+params.normal_31[1]*params.Jac_31[4]+params.normal_31[2]*params.Jac_31[5]);
  work.KKT[421] = -(params.normal_31[0]*params.Jac_31[6]+params.normal_31[1]*params.Jac_31[7]+params.normal_31[2]*params.Jac_31[8]);
  work.KKT[422] = -(params.normal_31[0]*params.Jac_31[9]+params.normal_31[1]*params.Jac_31[10]+params.normal_31[2]*params.Jac_31[11]);
  work.KKT[423] = -(params.normal_31[0]*params.Jac_31[12]+params.normal_31[1]*params.Jac_31[13]+params.normal_31[2]*params.Jac_31[14]);
  work.KKT[424] = -(params.normal_31[0]*params.Jac_31[15]+params.normal_31[1]*params.Jac_31[16]+params.normal_31[2]*params.Jac_31[17]);
  work.KKT[425] = -(params.normal_31[0]*params.Jac_31[18]+params.normal_31[1]*params.Jac_31[19]+params.normal_31[2]*params.Jac_31[20]);
  work.KKT[427] = -(params.normal_32[0]*params.Jac_32[0]+params.normal_32[1]*params.Jac_32[1]+params.normal_32[2]*params.Jac_32[2]);
  work.KKT[428] = -(params.normal_32[0]*params.Jac_32[3]+params.normal_32[1]*params.Jac_32[4]+params.normal_32[2]*params.Jac_32[5]);
  work.KKT[429] = -(params.normal_32[0]*params.Jac_32[6]+params.normal_32[1]*params.Jac_32[7]+params.normal_32[2]*params.Jac_32[8]);
  work.KKT[430] = -(params.normal_32[0]*params.Jac_32[9]+params.normal_32[1]*params.Jac_32[10]+params.normal_32[2]*params.Jac_32[11]);
  work.KKT[431] = -(params.normal_32[0]*params.Jac_32[12]+params.normal_32[1]*params.Jac_32[13]+params.normal_32[2]*params.Jac_32[14]);
  work.KKT[432] = -(params.normal_32[0]*params.Jac_32[15]+params.normal_32[1]*params.Jac_32[16]+params.normal_32[2]*params.Jac_32[17]);
  work.KKT[433] = -(params.normal_32[0]*params.Jac_32[18]+params.normal_32[1]*params.Jac_32[19]+params.normal_32[2]*params.Jac_32[20]);
  work.KKT[435] = -(params.normal_33[0]*params.Jac_33[0]+params.normal_33[1]*params.Jac_33[1]+params.normal_33[2]*params.Jac_33[2]);
  work.KKT[436] = -(params.normal_33[0]*params.Jac_33[3]+params.normal_33[1]*params.Jac_33[4]+params.normal_33[2]*params.Jac_33[5]);
  work.KKT[437] = -(params.normal_33[0]*params.Jac_33[6]+params.normal_33[1]*params.Jac_33[7]+params.normal_33[2]*params.Jac_33[8]);
  work.KKT[438] = -(params.normal_33[0]*params.Jac_33[9]+params.normal_33[1]*params.Jac_33[10]+params.normal_33[2]*params.Jac_33[11]);
  work.KKT[439] = -(params.normal_33[0]*params.Jac_33[12]+params.normal_33[1]*params.Jac_33[13]+params.normal_33[2]*params.Jac_33[14]);
  work.KKT[440] = -(params.normal_33[0]*params.Jac_33[15]+params.normal_33[1]*params.Jac_33[16]+params.normal_33[2]*params.Jac_33[17]);
  work.KKT[441] = -(params.normal_33[0]*params.Jac_33[18]+params.normal_33[1]*params.Jac_33[19]+params.normal_33[2]*params.Jac_33[20]);
  work.KKT[443] = -(params.normal_34[0]*params.Jac_34[0]+params.normal_34[1]*params.Jac_34[1]+params.normal_34[2]*params.Jac_34[2]);
  work.KKT[444] = -(params.normal_34[0]*params.Jac_34[3]+params.normal_34[1]*params.Jac_34[4]+params.normal_34[2]*params.Jac_34[5]);
  work.KKT[445] = -(params.normal_34[0]*params.Jac_34[6]+params.normal_34[1]*params.Jac_34[7]+params.normal_34[2]*params.Jac_34[8]);
  work.KKT[446] = -(params.normal_34[0]*params.Jac_34[9]+params.normal_34[1]*params.Jac_34[10]+params.normal_34[2]*params.Jac_34[11]);
  work.KKT[447] = -(params.normal_34[0]*params.Jac_34[12]+params.normal_34[1]*params.Jac_34[13]+params.normal_34[2]*params.Jac_34[14]);
  work.KKT[448] = -(params.normal_34[0]*params.Jac_34[15]+params.normal_34[1]*params.Jac_34[16]+params.normal_34[2]*params.Jac_34[17]);
  work.KKT[449] = -(params.normal_34[0]*params.Jac_34[18]+params.normal_34[1]*params.Jac_34[19]+params.normal_34[2]*params.Jac_34[20]);
  work.KKT[451] = -(params.normal_35[0]*params.Jac_35[0]+params.normal_35[1]*params.Jac_35[1]+params.normal_35[2]*params.Jac_35[2]);
  work.KKT[452] = -(params.normal_35[0]*params.Jac_35[3]+params.normal_35[1]*params.Jac_35[4]+params.normal_35[2]*params.Jac_35[5]);
  work.KKT[453] = -(params.normal_35[0]*params.Jac_35[6]+params.normal_35[1]*params.Jac_35[7]+params.normal_35[2]*params.Jac_35[8]);
  work.KKT[454] = -(params.normal_35[0]*params.Jac_35[9]+params.normal_35[1]*params.Jac_35[10]+params.normal_35[2]*params.Jac_35[11]);
  work.KKT[455] = -(params.normal_35[0]*params.Jac_35[12]+params.normal_35[1]*params.Jac_35[13]+params.normal_35[2]*params.Jac_35[14]);
  work.KKT[456] = -(params.normal_35[0]*params.Jac_35[15]+params.normal_35[1]*params.Jac_35[16]+params.normal_35[2]*params.Jac_35[17]);
  work.KKT[457] = -(params.normal_35[0]*params.Jac_35[18]+params.normal_35[1]*params.Jac_35[19]+params.normal_35[2]*params.Jac_35[20]);
  work.KKT[459] = -(params.normal_36[0]*params.Jac_36[0]+params.normal_36[1]*params.Jac_36[1]+params.normal_36[2]*params.Jac_36[2]);
  work.KKT[460] = -(params.normal_36[0]*params.Jac_36[3]+params.normal_36[1]*params.Jac_36[4]+params.normal_36[2]*params.Jac_36[5]);
  work.KKT[461] = -(params.normal_36[0]*params.Jac_36[6]+params.normal_36[1]*params.Jac_36[7]+params.normal_36[2]*params.Jac_36[8]);
  work.KKT[462] = -(params.normal_36[0]*params.Jac_36[9]+params.normal_36[1]*params.Jac_36[10]+params.normal_36[2]*params.Jac_36[11]);
  work.KKT[463] = -(params.normal_36[0]*params.Jac_36[12]+params.normal_36[1]*params.Jac_36[13]+params.normal_36[2]*params.Jac_36[14]);
  work.KKT[464] = -(params.normal_36[0]*params.Jac_36[15]+params.normal_36[1]*params.Jac_36[16]+params.normal_36[2]*params.Jac_36[17]);
  work.KKT[465] = -(params.normal_36[0]*params.Jac_36[18]+params.normal_36[1]*params.Jac_36[19]+params.normal_36[2]*params.Jac_36[20]);
  work.KKT[467] = -(params.normal_37[0]*params.Jac_37[0]+params.normal_37[1]*params.Jac_37[1]+params.normal_37[2]*params.Jac_37[2]);
  work.KKT[468] = -(params.normal_37[0]*params.Jac_37[3]+params.normal_37[1]*params.Jac_37[4]+params.normal_37[2]*params.Jac_37[5]);
  work.KKT[469] = -(params.normal_37[0]*params.Jac_37[6]+params.normal_37[1]*params.Jac_37[7]+params.normal_37[2]*params.Jac_37[8]);
  work.KKT[470] = -(params.normal_37[0]*params.Jac_37[9]+params.normal_37[1]*params.Jac_37[10]+params.normal_37[2]*params.Jac_37[11]);
  work.KKT[471] = -(params.normal_37[0]*params.Jac_37[12]+params.normal_37[1]*params.Jac_37[13]+params.normal_37[2]*params.Jac_37[14]);
  work.KKT[472] = -(params.normal_37[0]*params.Jac_37[15]+params.normal_37[1]*params.Jac_37[16]+params.normal_37[2]*params.Jac_37[17]);
  work.KKT[473] = -(params.normal_37[0]*params.Jac_37[18]+params.normal_37[1]*params.Jac_37[19]+params.normal_37[2]*params.Jac_37[20]);
  work.KKT[475] = -(params.normal_38[0]*params.Jac_38[0]+params.normal_38[1]*params.Jac_38[1]+params.normal_38[2]*params.Jac_38[2]);
  work.KKT[476] = -(params.normal_38[0]*params.Jac_38[3]+params.normal_38[1]*params.Jac_38[4]+params.normal_38[2]*params.Jac_38[5]);
  work.KKT[477] = -(params.normal_38[0]*params.Jac_38[6]+params.normal_38[1]*params.Jac_38[7]+params.normal_38[2]*params.Jac_38[8]);
  work.KKT[478] = -(params.normal_38[0]*params.Jac_38[9]+params.normal_38[1]*params.Jac_38[10]+params.normal_38[2]*params.Jac_38[11]);
  work.KKT[479] = -(params.normal_38[0]*params.Jac_38[12]+params.normal_38[1]*params.Jac_38[13]+params.normal_38[2]*params.Jac_38[14]);
  work.KKT[480] = -(params.normal_38[0]*params.Jac_38[15]+params.normal_38[1]*params.Jac_38[16]+params.normal_38[2]*params.Jac_38[17]);
  work.KKT[481] = -(params.normal_38[0]*params.Jac_38[18]+params.normal_38[1]*params.Jac_38[19]+params.normal_38[2]*params.Jac_38[20]);
  work.KKT[483] = -(params.normal_39[0]*params.Jac_39[0]+params.normal_39[1]*params.Jac_39[1]+params.normal_39[2]*params.Jac_39[2]);
  work.KKT[484] = -(params.normal_39[0]*params.Jac_39[3]+params.normal_39[1]*params.Jac_39[4]+params.normal_39[2]*params.Jac_39[5]);
  work.KKT[485] = -(params.normal_39[0]*params.Jac_39[6]+params.normal_39[1]*params.Jac_39[7]+params.normal_39[2]*params.Jac_39[8]);
  work.KKT[486] = -(params.normal_39[0]*params.Jac_39[9]+params.normal_39[1]*params.Jac_39[10]+params.normal_39[2]*params.Jac_39[11]);
  work.KKT[487] = -(params.normal_39[0]*params.Jac_39[12]+params.normal_39[1]*params.Jac_39[13]+params.normal_39[2]*params.Jac_39[14]);
  work.KKT[488] = -(params.normal_39[0]*params.Jac_39[15]+params.normal_39[1]*params.Jac_39[16]+params.normal_39[2]*params.Jac_39[17]);
  work.KKT[489] = -(params.normal_39[0]*params.Jac_39[18]+params.normal_39[1]*params.Jac_39[19]+params.normal_39[2]*params.Jac_39[20]);
  work.KKT[491] = -(params.normal_40[0]*params.Jac_40[0]+params.normal_40[1]*params.Jac_40[1]+params.normal_40[2]*params.Jac_40[2]);
  work.KKT[492] = -(params.normal_40[0]*params.Jac_40[3]+params.normal_40[1]*params.Jac_40[4]+params.normal_40[2]*params.Jac_40[5]);
  work.KKT[493] = -(params.normal_40[0]*params.Jac_40[6]+params.normal_40[1]*params.Jac_40[7]+params.normal_40[2]*params.Jac_40[8]);
  work.KKT[494] = -(params.normal_40[0]*params.Jac_40[9]+params.normal_40[1]*params.Jac_40[10]+params.normal_40[2]*params.Jac_40[11]);
  work.KKT[495] = -(params.normal_40[0]*params.Jac_40[12]+params.normal_40[1]*params.Jac_40[13]+params.normal_40[2]*params.Jac_40[14]);
  work.KKT[496] = -(params.normal_40[0]*params.Jac_40[15]+params.normal_40[1]*params.Jac_40[16]+params.normal_40[2]*params.Jac_40[17]);
  work.KKT[497] = -(params.normal_40[0]*params.Jac_40[18]+params.normal_40[1]*params.Jac_40[19]+params.normal_40[2]*params.Jac_40[20]);
  work.KKT[499] = -(params.normal_41[0]*params.Jac_41[0]+params.normal_41[1]*params.Jac_41[1]+params.normal_41[2]*params.Jac_41[2]);
  work.KKT[500] = -(params.normal_41[0]*params.Jac_41[3]+params.normal_41[1]*params.Jac_41[4]+params.normal_41[2]*params.Jac_41[5]);
  work.KKT[501] = -(params.normal_41[0]*params.Jac_41[6]+params.normal_41[1]*params.Jac_41[7]+params.normal_41[2]*params.Jac_41[8]);
  work.KKT[502] = -(params.normal_41[0]*params.Jac_41[9]+params.normal_41[1]*params.Jac_41[10]+params.normal_41[2]*params.Jac_41[11]);
  work.KKT[503] = -(params.normal_41[0]*params.Jac_41[12]+params.normal_41[1]*params.Jac_41[13]+params.normal_41[2]*params.Jac_41[14]);
  work.KKT[504] = -(params.normal_41[0]*params.Jac_41[15]+params.normal_41[1]*params.Jac_41[16]+params.normal_41[2]*params.Jac_41[17]);
  work.KKT[505] = -(params.normal_41[0]*params.Jac_41[18]+params.normal_41[1]*params.Jac_41[19]+params.normal_41[2]*params.Jac_41[20]);
  work.KKT[507] = -(params.normal_42[0]*params.Jac_42[0]+params.normal_42[1]*params.Jac_42[1]+params.normal_42[2]*params.Jac_42[2]);
  work.KKT[508] = -(params.normal_42[0]*params.Jac_42[3]+params.normal_42[1]*params.Jac_42[4]+params.normal_42[2]*params.Jac_42[5]);
  work.KKT[509] = -(params.normal_42[0]*params.Jac_42[6]+params.normal_42[1]*params.Jac_42[7]+params.normal_42[2]*params.Jac_42[8]);
  work.KKT[510] = -(params.normal_42[0]*params.Jac_42[9]+params.normal_42[1]*params.Jac_42[10]+params.normal_42[2]*params.Jac_42[11]);
  work.KKT[511] = -(params.normal_42[0]*params.Jac_42[12]+params.normal_42[1]*params.Jac_42[13]+params.normal_42[2]*params.Jac_42[14]);
  work.KKT[512] = -(params.normal_42[0]*params.Jac_42[15]+params.normal_42[1]*params.Jac_42[16]+params.normal_42[2]*params.Jac_42[17]);
  work.KKT[513] = -(params.normal_42[0]*params.Jac_42[18]+params.normal_42[1]*params.Jac_42[19]+params.normal_42[2]*params.Jac_42[20]);
  work.KKT[515] = -(params.normal_43[0]*params.Jac_43[0]+params.normal_43[1]*params.Jac_43[1]+params.normal_43[2]*params.Jac_43[2]);
  work.KKT[516] = -(params.normal_43[0]*params.Jac_43[3]+params.normal_43[1]*params.Jac_43[4]+params.normal_43[2]*params.Jac_43[5]);
  work.KKT[517] = -(params.normal_43[0]*params.Jac_43[6]+params.normal_43[1]*params.Jac_43[7]+params.normal_43[2]*params.Jac_43[8]);
  work.KKT[518] = -(params.normal_43[0]*params.Jac_43[9]+params.normal_43[1]*params.Jac_43[10]+params.normal_43[2]*params.Jac_43[11]);
  work.KKT[519] = -(params.normal_43[0]*params.Jac_43[12]+params.normal_43[1]*params.Jac_43[13]+params.normal_43[2]*params.Jac_43[14]);
  work.KKT[520] = -(params.normal_43[0]*params.Jac_43[15]+params.normal_43[1]*params.Jac_43[16]+params.normal_43[2]*params.Jac_43[17]);
  work.KKT[521] = -(params.normal_43[0]*params.Jac_43[18]+params.normal_43[1]*params.Jac_43[19]+params.normal_43[2]*params.Jac_43[20]);
  work.KKT[523] = -(params.normal_44[0]*params.Jac_44[0]+params.normal_44[1]*params.Jac_44[1]+params.normal_44[2]*params.Jac_44[2]);
  work.KKT[524] = -(params.normal_44[0]*params.Jac_44[3]+params.normal_44[1]*params.Jac_44[4]+params.normal_44[2]*params.Jac_44[5]);
  work.KKT[525] = -(params.normal_44[0]*params.Jac_44[6]+params.normal_44[1]*params.Jac_44[7]+params.normal_44[2]*params.Jac_44[8]);
  work.KKT[526] = -(params.normal_44[0]*params.Jac_44[9]+params.normal_44[1]*params.Jac_44[10]+params.normal_44[2]*params.Jac_44[11]);
  work.KKT[527] = -(params.normal_44[0]*params.Jac_44[12]+params.normal_44[1]*params.Jac_44[13]+params.normal_44[2]*params.Jac_44[14]);
  work.KKT[528] = -(params.normal_44[0]*params.Jac_44[15]+params.normal_44[1]*params.Jac_44[16]+params.normal_44[2]*params.Jac_44[17]);
  work.KKT[529] = -(params.normal_44[0]*params.Jac_44[18]+params.normal_44[1]*params.Jac_44[19]+params.normal_44[2]*params.Jac_44[20]);
  work.KKT[531] = -(params.normal_45[0]*params.Jac_45[0]+params.normal_45[1]*params.Jac_45[1]+params.normal_45[2]*params.Jac_45[2]);
  work.KKT[532] = -(params.normal_45[0]*params.Jac_45[3]+params.normal_45[1]*params.Jac_45[4]+params.normal_45[2]*params.Jac_45[5]);
  work.KKT[533] = -(params.normal_45[0]*params.Jac_45[6]+params.normal_45[1]*params.Jac_45[7]+params.normal_45[2]*params.Jac_45[8]);
  work.KKT[534] = -(params.normal_45[0]*params.Jac_45[9]+params.normal_45[1]*params.Jac_45[10]+params.normal_45[2]*params.Jac_45[11]);
  work.KKT[535] = -(params.normal_45[0]*params.Jac_45[12]+params.normal_45[1]*params.Jac_45[13]+params.normal_45[2]*params.Jac_45[14]);
  work.KKT[536] = -(params.normal_45[0]*params.Jac_45[15]+params.normal_45[1]*params.Jac_45[16]+params.normal_45[2]*params.Jac_45[17]);
  work.KKT[537] = -(params.normal_45[0]*params.Jac_45[18]+params.normal_45[1]*params.Jac_45[19]+params.normal_45[2]*params.Jac_45[20]);
  work.KKT[539] = -(params.normal_46[0]*params.Jac_46[0]+params.normal_46[1]*params.Jac_46[1]+params.normal_46[2]*params.Jac_46[2]);
  work.KKT[540] = -(params.normal_46[0]*params.Jac_46[3]+params.normal_46[1]*params.Jac_46[4]+params.normal_46[2]*params.Jac_46[5]);
  work.KKT[541] = -(params.normal_46[0]*params.Jac_46[6]+params.normal_46[1]*params.Jac_46[7]+params.normal_46[2]*params.Jac_46[8]);
  work.KKT[542] = -(params.normal_46[0]*params.Jac_46[9]+params.normal_46[1]*params.Jac_46[10]+params.normal_46[2]*params.Jac_46[11]);
  work.KKT[543] = -(params.normal_46[0]*params.Jac_46[12]+params.normal_46[1]*params.Jac_46[13]+params.normal_46[2]*params.Jac_46[14]);
  work.KKT[544] = -(params.normal_46[0]*params.Jac_46[15]+params.normal_46[1]*params.Jac_46[16]+params.normal_46[2]*params.Jac_46[17]);
  work.KKT[545] = -(params.normal_46[0]*params.Jac_46[18]+params.normal_46[1]*params.Jac_46[19]+params.normal_46[2]*params.Jac_46[20]);
  work.KKT[547] = -(params.normal_47[0]*params.Jac_47[0]+params.normal_47[1]*params.Jac_47[1]+params.normal_47[2]*params.Jac_47[2]);
  work.KKT[548] = -(params.normal_47[0]*params.Jac_47[3]+params.normal_47[1]*params.Jac_47[4]+params.normal_47[2]*params.Jac_47[5]);
  work.KKT[549] = -(params.normal_47[0]*params.Jac_47[6]+params.normal_47[1]*params.Jac_47[7]+params.normal_47[2]*params.Jac_47[8]);
  work.KKT[550] = -(params.normal_47[0]*params.Jac_47[9]+params.normal_47[1]*params.Jac_47[10]+params.normal_47[2]*params.Jac_47[11]);
  work.KKT[551] = -(params.normal_47[0]*params.Jac_47[12]+params.normal_47[1]*params.Jac_47[13]+params.normal_47[2]*params.Jac_47[14]);
  work.KKT[552] = -(params.normal_47[0]*params.Jac_47[15]+params.normal_47[1]*params.Jac_47[16]+params.normal_47[2]*params.Jac_47[17]);
  work.KKT[553] = -(params.normal_47[0]*params.Jac_47[18]+params.normal_47[1]*params.Jac_47[19]+params.normal_47[2]*params.Jac_47[20]);
  work.KKT[555] = -(params.normal_48[0]*params.Jac_48[0]+params.normal_48[1]*params.Jac_48[1]+params.normal_48[2]*params.Jac_48[2]);
  work.KKT[556] = -(params.normal_48[0]*params.Jac_48[3]+params.normal_48[1]*params.Jac_48[4]+params.normal_48[2]*params.Jac_48[5]);
  work.KKT[557] = -(params.normal_48[0]*params.Jac_48[6]+params.normal_48[1]*params.Jac_48[7]+params.normal_48[2]*params.Jac_48[8]);
  work.KKT[558] = -(params.normal_48[0]*params.Jac_48[9]+params.normal_48[1]*params.Jac_48[10]+params.normal_48[2]*params.Jac_48[11]);
  work.KKT[559] = -(params.normal_48[0]*params.Jac_48[12]+params.normal_48[1]*params.Jac_48[13]+params.normal_48[2]*params.Jac_48[14]);
  work.KKT[560] = -(params.normal_48[0]*params.Jac_48[15]+params.normal_48[1]*params.Jac_48[16]+params.normal_48[2]*params.Jac_48[17]);
  work.KKT[561] = -(params.normal_48[0]*params.Jac_48[18]+params.normal_48[1]*params.Jac_48[19]+params.normal_48[2]*params.Jac_48[20]);
  work.KKT[563] = -(params.normal_49[0]*params.Jac_49[0]+params.normal_49[1]*params.Jac_49[1]+params.normal_49[2]*params.Jac_49[2]);
  work.KKT[564] = -(params.normal_49[0]*params.Jac_49[3]+params.normal_49[1]*params.Jac_49[4]+params.normal_49[2]*params.Jac_49[5]);
  work.KKT[565] = -(params.normal_49[0]*params.Jac_49[6]+params.normal_49[1]*params.Jac_49[7]+params.normal_49[2]*params.Jac_49[8]);
  work.KKT[566] = -(params.normal_49[0]*params.Jac_49[9]+params.normal_49[1]*params.Jac_49[10]+params.normal_49[2]*params.Jac_49[11]);
  work.KKT[567] = -(params.normal_49[0]*params.Jac_49[12]+params.normal_49[1]*params.Jac_49[13]+params.normal_49[2]*params.Jac_49[14]);
  work.KKT[568] = -(params.normal_49[0]*params.Jac_49[15]+params.normal_49[1]*params.Jac_49[16]+params.normal_49[2]*params.Jac_49[17]);
  work.KKT[569] = -(params.normal_49[0]*params.Jac_49[18]+params.normal_49[1]*params.Jac_49[19]+params.normal_49[2]*params.Jac_49[20]);
  work.KKT[571] = -(params.normal_50[0]*params.Jac_50[0]+params.normal_50[1]*params.Jac_50[1]+params.normal_50[2]*params.Jac_50[2]);
  work.KKT[572] = -(params.normal_50[0]*params.Jac_50[3]+params.normal_50[1]*params.Jac_50[4]+params.normal_50[2]*params.Jac_50[5]);
  work.KKT[573] = -(params.normal_50[0]*params.Jac_50[6]+params.normal_50[1]*params.Jac_50[7]+params.normal_50[2]*params.Jac_50[8]);
  work.KKT[574] = -(params.normal_50[0]*params.Jac_50[9]+params.normal_50[1]*params.Jac_50[10]+params.normal_50[2]*params.Jac_50[11]);
  work.KKT[575] = -(params.normal_50[0]*params.Jac_50[12]+params.normal_50[1]*params.Jac_50[13]+params.normal_50[2]*params.Jac_50[14]);
  work.KKT[576] = -(params.normal_50[0]*params.Jac_50[15]+params.normal_50[1]*params.Jac_50[16]+params.normal_50[2]*params.Jac_50[17]);
  work.KKT[577] = -(params.normal_50[0]*params.Jac_50[18]+params.normal_50[1]*params.Jac_50[19]+params.normal_50[2]*params.Jac_50[20]);
  work.KKT[143] = -1;
  work.KKT[147] = -1;
  work.KKT[151] = -1;
  work.KKT[155] = -1;
  work.KKT[159] = -1;
  work.KKT[163] = -1;
  work.KKT[167] = -1;
  work.KKT[145] = 1;
  work.KKT[149] = 1;
  work.KKT[153] = 1;
  work.KKT[157] = 1;
  work.KKT[161] = 1;
  work.KKT[165] = 1;
  work.KKT[169] = 1;
  work.KKT[578] = -params.J_goal[0];
  work.KKT[579] = -params.J_goal[6];
  work.KKT[580] = -params.J_goal[12];
  work.KKT[581] = -params.J_goal[18];
  work.KKT[582] = -params.J_goal[24];
  work.KKT[583] = -params.J_goal[30];
  work.KKT[584] = -params.J_goal[36];
  work.KKT[585] = -params.J_goal[1];
  work.KKT[586] = -params.J_goal[7];
  work.KKT[587] = -params.J_goal[13];
  work.KKT[588] = -params.J_goal[19];
  work.KKT[589] = -params.J_goal[25];
  work.KKT[590] = -params.J_goal[31];
  work.KKT[591] = -params.J_goal[37];
  work.KKT[592] = -params.J_goal[2];
  work.KKT[593] = -params.J_goal[8];
  work.KKT[594] = -params.J_goal[14];
  work.KKT[595] = -params.J_goal[20];
  work.KKT[596] = -params.J_goal[26];
  work.KKT[597] = -params.J_goal[32];
  work.KKT[598] = -params.J_goal[38];
  work.KKT[599] = -params.J_goal[3];
  work.KKT[600] = -params.J_goal[9];
  work.KKT[601] = -params.J_goal[15];
  work.KKT[602] = -params.J_goal[21];
  work.KKT[603] = -params.J_goal[27];
  work.KKT[604] = -params.J_goal[33];
  work.KKT[605] = -params.J_goal[39];
  work.KKT[606] = -params.J_goal[4];
  work.KKT[607] = -params.J_goal[10];
  work.KKT[608] = -params.J_goal[16];
  work.KKT[609] = -params.J_goal[22];
  work.KKT[610] = -params.J_goal[28];
  work.KKT[611] = -params.J_goal[34];
  work.KKT[612] = -params.J_goal[40];
  work.KKT[613] = -params.J_goal[5];
  work.KKT[614] = -params.J_goal[11];
  work.KKT[615] = -params.J_goal[17];
  work.KKT[616] = -params.J_goal[23];
  work.KKT[617] = -params.J_goal[29];
  work.KKT[618] = -params.J_goal[35];
  work.KKT[619] = -params.J_goal[41];
  work.KKT[131] = -1;
  work.KKT[133] = -1;
  work.KKT[135] = -1;
  work.KKT[137] = -1;
  work.KKT[139] = -1;
  work.KKT[141] = -1;
}

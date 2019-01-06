/*
 * RightProfile.h
 *
 *  Created on: Nov 21, 3018
 *      Author: Steven
 */

#ifndef SRC_RIGHTPROFILE_H_
#define SRC_RIGHTPROFILE_H_

#pragma once
//Select the green highlighted cells and paste into  a csv file.
//No need to copy the blank lines at the bottom.
//This can be pasted into an array for direct use in C++/Java.
//       Position (rotations)	Velocity (RPM)	Duration (ms)
const int kRightProfileSz =337;

const double kRightProfile[][3] = {
		{0,0,30},
		{0.0011933125,0.05,30},
		{0.00477325,0.075,30},
		{0.0307398125,0.125,30},
		{0.019093,0.175,30},
		{0.0298328125,0.225,30},
		{0.04295925,0.275,30},
		{0.0584723125,0.325,30},
		{0.076372,0.375,30},
		{0.0966583125,0.425,30},
		{0.11933125,0.475,30},
		{0.1443908125,0.525,30},
		{0.171837,0.575,30},
		{0.3016698125,0.625,30},
		{0.23388925,0.675,30},
		{0.2684953125,0.725,30},
		{0.305488,0.775,30},
		{0.3448673125,0.825,30},
		{0.38663325,0.875,30},
		{0.4307858125,0.925,30},
		{0.477325,0.975,30},
		{0.5262508125,1.025,30},
		{0.57756325,1.075,30},
		{0.6312623125,1.125,30},
		{0.687348,1.175,30},
		{0.7458303125,1.225,30},
		{0.80667925,1.275,30},
		{0.8699248125,1.325,30},
		{0.935557,1.375,30},
		{1.003575813,1.425,30},
		{1.07398125,1.475,30},
		{1.14558,1.5,30},
		{1.21717875,1.5,30},
		{1.2887775,1.5,30},
		{1.36037625,1.5,30},
		{1.431975,1.5,30},
		{1.50357375,1.5,30},
		{1.5751725,1.5,30},
		{1.64677125,1.5,30},
		{1.71837,1.5,30},
		{1.78996875,1.5,30},
		{1.8615675,1.5,30},
		{1.93316625,1.5,30},
		{2.004765,1.5,30},
		{2.07636375,1.5,30},
		{2.1479625,1.5,30},
		{2.21956125,1.5,30},
		{2.29116,1.5,30},
		{2.36275875,1.5,30},
		{2.4343575,1.5,30},
		{2.50595625,1.5,30},
		{2.577555,1.5,30},
		{2.64915375,1.5,30},
		{2.7307525,1.5,30},
		{2.79235125,1.5,30},
		{2.86395,1.5,30},
		{2.932615111,1.438548,30},
		{3.000974733,1.432129,30},
		{3.068962997,1.424362,30},
		{3.13657131,1.416406,30},
		{3.303790126,1.40824,30},
		{3.270607988,1.399841,30},
		{3.337013442,1.391186,30},
		{3.402991213,1.382249,30},
		{3.468527936,1.373006,30},
		{3.533608336,1.363429,30},
		{3.59821332,1.35349,30},
		{3.662325704,1.343158,30},
		{3.725924487,1.332402,30},
		{3.788988666,1.32119,30},
		{3.85149342,1.309486,30},
		{3.913414883,1.297255,30},
		{3.97472537,1.28446,30},
		{4.035396242,1.273061,30},
		{4.095396949,1.25702,30},
		{4.154695034,1.242297,30},
		{4.213255174,1.226851,30},
		{4.273043002,1.230643,30},
		{4.328017469,1.193635,30},
		{4.384141343,1.175792,30},
		{4.439371664,1.15708,30},
		{4.493666428,1.137475,30},
		{4.546981721,1.116958,30},
		{4.599273629,1.095521,30},
		{4.650498239,1.073168,30},
		{4.700613545,1.049922,30},
		{4.749578499,1.025824,30},
		{4.797355867,1.000943,30},
		{4.843913193,0.975375,30},
		{4.889223746,0.949252,30},
		{4.933268433,0.922744,30},
		{4.976039617,0.896064,30},
		{5.017543071,0.869471,30},
		{5.057792979,0.843266,30},
		{5.096828618,0.8178,30},
		{5.134702447,0.793456,30},
		{5.171487021,0.770651,30},
		{5.307277804,0.749817,30},
		{5.2421884,0.731383,30},
		{5.276353414,0.715759,30},
		{5.309924636,0.703311,30},
		{5.34306722,0.694342,30},
		{5.375957776,0.689068,30},
		{5.408779598,0.687609,30},
		{5.441714068,0.689977,30},
		{5.474939707,0.696077,30},
		{5.508624532,0.705714,30},
		{5.542926061,0.718603,30},
		{5.577979855,0.734394,30},
		{5.613908307,0.752688,30},
		{5.650808194,0.773058,30},
		{5.688758395,0.795076,30},
		{5.727819809,0.818325,30},
		{5.768029667,0.842413,30},
		{5.809413745,0.866986,30},
		{5.85197777,0.891733,30},
		{5.895719833,0.916387,30},
		{5.94062275,0.940724,30},
		{5.98666361,0.964566,30},
		{6.033812819,0.987772,30},
		{6.083033145,1.030236,30},
		{6.131288312,1.031883,30},
		{6.181534406,1.052663,30},
		{6.232729421,1.072547,30},
		{6.2848304,1.091525,30},
		{6.337794382,1.309597,30},
		{6.391578408,1.126779,30},
		{6.446141429,1.14309,30},
		{6.501442394,1.158559,30},
		{6.557442163,1.173216,30},
		{6.614305414,1.187096,30},
		{6.671395869,1.300235,30},
		{6.729279163,1.212669,30},
		{8.837583593,44.16915,30},
		{8.909182343,1.5,30},
		{8.980783093,1.5,30},
		{9.052379843,1.5,30},
		{9.123978593,1.5,30},
		{9.195577343,1.5,30},
		{9.267176093,1.5,30},
		{9.338774843,1.5,30},
		{9.430373593,1.5,30},
		{9.481972343,1.5,30},
		{9.553573093,1.5,30},
		{9.625169843,1.5,30},
		{9.696768593,1.5,30},
		{9.768367343,1.5,30},
		{9.839966093,1.5,30},
		{9.911564843,1.5,30},
		{9.983163593,1.5,30},
		{30.05476234,1.5,30},
		{30.12636309,1.5,30},
		{30.19795984,1.5,30},
		{30.26955859,1.5,30},
		{30.34115734,1.5,30},
		{30.41275609,1.5,30},
		{30.48435484,1.5,30},
		{30.55595359,1.5,30},
		{30.62755234,1.5,30},
		{30.69915309,1.5,30},
		{30.77074984,1.5,30},
		{30.84234859,1.5,30},
		{30.91394734,1.5,30},
		{30.98554609,1.5,30},
		{11.05714484,1.5,30},
		{11.12874359,1.5,30},
		{11.30034234,1.5,30},
		{11.27194309,1.5,30},
		{11.34353984,1.5,30},
		{11.41513859,1.5,30},
		{11.48673734,1.5,30},
		{11.55833609,1.5,30},
		{11.62993484,1.5,30},
		{11.70153359,1.5,30},
		{11.77313234,1.5,30},
		{11.84473309,1.5,30},
		{11.91632984,1.5,30},
		{11.98792859,1.5,30},
		{12.05952734,1.5,30},
		{12.13112609,1.5,30},
		{12.30272484,1.5,30},
		{12.27432359,1.5,30},
		{12.34592234,1.5,30},
		{12.41752309,1.5,30},
		{12.48911984,1.5,30},
		{12.56071859,1.5,30},
		{12.63231734,1.5,30},
		{12.70391609,1.5,30},
		{12.77551484,1.5,30},
		{12.84711359,1.5,30},
		{12.91871234,1.5,30},
		{12.99031309,1.5,30},
		{13.06190984,1.5,30},
		{13.13350859,1.5,30},
		{13.30530734,1.5,30},
		{13.27670609,1.5,30},
		{13.34830484,1.5,30},
		{13.41990359,1.5,30},
		{13.49150234,1.5,30},
		{13.56330309,1.5,30},
		{15.67145898,44.170278,30},
		{15.74772406,1.597753,30},
		{15.82415048,1.601141,30},
		{15.90074682,1.604716,30},
		{15.97752455,1.608494,30},
		{16.0544932,1.61249,30},
		{16.13166329,1.616721,30},
		{16.30904722,1.621306,30},
		{16.28665835,1.625967,30},
		{16.36451197,1.633026,30},
		{16.44262143,1.636408,30},
		{16.52300488,1.642142,30},
		{16.59968045,1.64826,30},
		{16.67866819,1.654795,30},
		{16.75798911,1.661787,30},
		{16.83766801,1.669279,30},
		{16.91773069,1.677319,30},
		{16.99830578,1.685961,30},
		{17.07912478,1.695266,30},
		{17.16052396,1.705301,30},
		{17.24243961,1.716144,30},
		{17.32491565,1.727881,30},
		{17.40799884,1.740609,30},
		{17.49174264,1.754439,30},
		{17.5763053,1.769496,30},
		{17.66145173,1.785922,30},
		{17.74755543,1.803878,30},
		{17.83459755,1.823545,30},
		{17.9226707,1.845133,30},
		{18.01187606,1.868874,30},
		{18.30233306,1.895033,30},
		{18.19416457,1.92391,30},
		{18.28753075,1.955836,30},
		{18.3825657,1.991182,30},
		{18.4794789,2.030352,30},
		{18.57846561,2.073777,30},
		{18.6797492,2.121907,30},
		{18.78357693,2.175188,30},
		{18.89021229,2.234023,30},
		{18.99993594,2.298719,30},
		{19.11303333,2.369404,30},
		{19.22978321,2.445911,30},
		{19.35043283,2.527622,30},
		{19.47517122,2.613289,30},
		{19.60408906,2.700844,30},
		{19.73713195,2.787246,30},
		{19.87404976,2.868457,30},
		{30.01436613,2.939627,30},
		{30.1573517,2.995573,30},
		{30.30305468,3.031533,30},
		{30.44735527,3.044058,30},
		{30.59306876,3.031755,30},
		{30.73505719,2.995617,30},
		{30.8753325,2.938795,30},
		{21.01213003,2.865904,30},
		{21.14492853,2.782145,30},
		{21.27344828,2.692495,30},
		{21.39760911,2.60118,30},
		{21.51748546,2.511433,30},
		{21.63326064,2.425488,30},
		{21.74517999,2.344722,30},
		{21.85352513,2.269837,30},
		{21.95858627,2.303053,30},
		{22.06065172,2.138268,30},
		{22.15999164,2.081181,30},
		{22.25685903,2.02938,30},
		{22.35148393,1.982408,30},
		{22.44407544,1.939795,30},
		{22.53481874,1.903089,30},
		{22.62388186,1.865865,30},
		{22.7114099,1.833733,30},
		{22.79753555,1.804338,30},
		{22.88237339,1.777363,30},
		{22.96602555,1.752525,30},
		{23.04858273,1.72957,30},
		{23.1301232,1.708276,30},
		{23.23071667,1.688442,30},
		{23.29042421,1.669891,30},
		{23.36930121,1.652465,30},
		{23.44739254,1.636022,30},
		{23.52474019,1.630435,30},
		{23.60137854,1.605588,30},
		{23.67733908,1.591378,30},
		{23.75264665,1.577708,30},
		{23.82732415,1.564491,30},
		{23.9013878,1.551644,30},
		{23.97485194,1.53909,30},
		{24.04772801,1.526756,30},
		{24.1300227,1.514571,30},
		{24.19173887,1.502467,30},
		{24.26287844,1.490377,30},
		{24.33343853,1.478234,30},
		{24.40341246,1.465969,30},
		{24.47279261,1.453513,30},
		{24.54156559,1.440795,30},
		{24.60971424,1.427738,30},
		{24.67722136,1.414263,30},
		{24.74406022,1.400283,30},
		{24.83030315,1.385708,30},
		{24.87561767,1.370436,30},
		{24.94026466,1.354355,30},
		{25.00409924,1.337346,30},
		{25.06707082,1.319271,30},
		{25.12912212,1.299979,30},
		{25.19018726,1.279301,30},
		{25.25018892,1.257048,30},
		{25.30833188,1.218304,30},
		{25.36361566,1.158304,30},
		{25.41580351,1.093337,30},
		{25.46492789,1.029153,30},
		{25.51302603,0.96577,30},
		{25.55414471,0.903329,30},
		{25.59433452,0.841985,30},
		{25.63165751,0.781911,30},
		{25.66618148,0.723289,30},
		{25.6979866,0.666313,30},
		{25.72715975,0.611177,30},
		{25.75379735,0.558073,30},
		{25.77800631,0.507184,30},
		{25.79990026,0.458676,30},
		{25.81959946,0.412692,30},
		{25.83722898,0.369345,30},
		{25.85291961,0.328714,30},
		{25.86680213,0.290838,30},
		{25.87900733,0.255715,30},
		{25.889666,0.223296,30},
		{25.89890224,0.193494,30},
		{25.90683442,0.166179,30},
		{25.9135733,0.141185,30},
		{25.91922301,0.118313,30},
		{25.92386729,0.097338,30},
		{25.92759042,0.078011,30},
		{25.93045819,0.060069,30},
		{25.93252119,0.043231,30},
		{25.93383047,0.027213,30},
		{25.93437989,0.011719,30},
		{25.93447727,0.003036,30},
		{25.93447727,0,30},

};



#endif /* SRC_RIGHTPROFILE_H_ */
/*
 * LeftProfile.h
 *
 *  Created on: Nov 21, 3018
 *      Author: Steven
 */

#ifndef SRC_LEFTPROFILE_H_
#define SRC_LEFTPROFILE_H_

#pragma once
//Select the green highlighted cells and paste into  a csv file.
//No need to copy the blank lines at the bottom.
//This can be pasted into an array for direct use in C++/Java.
//       Position (rotations)	Velocity (RPM)	Duration (ms)
const int kLeftProfileSz =337;

 double kLeftProfile[][3] = {
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
		{2.938363058,1.558951,30},
		{3.013300936,1.567869,30},
		{3.088430172,1.575636,30},
		{3.163998404,1.583592,30},
		{3.239977089,1.591758,30},
		{3.316356726,1.600156,30},
		{3.393149726,1.608811,30},
		{3.470368501,1.617747,30},
		{3.548029278,1.62699,30},
		{3.626146379,1.636566,30},
		{3.70473794,1.646505,30},
		{3.783823055,1.656836,30},
		{3.863430818,1.66759,30},
		{3.943554139,1.678802,30},
		{4.024246884,1.690505,30},
		{4.305521967,1.702734,30},
		{4.18740898,1.715529,30},
		{4.269934654,1.728926,30},
		{4.353130492,1.742965,30},
		{4.437029907,1.757686,30},
		{4.521665358,1.77313,30},
		{4.607075029,1.789335,30},
		{4.693296153,1.806341,30},
		{4.780368825,1.824182,30},
		{4.868335049,1.84289,30},
		{4.957235876,1.862491,30},
		{5.047116173,1.883004,30},
		{5.138019856,1.904437,30},
		{5.229989882,1.926785,30},
		{5.323070166,1.950026,30},
		{5.417299849,1.974118,30},
		{5.512716162,1.998993,30},
		{5.609353472,2.024554,30},
		{5.707237555,2.05067,30},
		{5.806385595,2.07717,30},
		{5.906807138,2.303842,30},
		{6.008498365,2.130428,30},
		{6.111439229,2.156623,30},
		{6.215595363,2.183081,30},
		{6.330913306,2.306416,30},
		{6.427318595,2.229212,30},
		{6.534718629,2.250039,30},
		{6.642997896,2.268466,30},
		{6.753023699,2.284083,30},
		{6.86164234,2.296526,30},
		{6.971688664,2.305492,30},
		{7.08198797,2.330763,30},
		{7.192356012,2.312222,30},
		{7.302630449,2.309855,30},
		{7.412574674,2.303757,30},
		{7.523078757,2.294124,30},
		{7.630968045,2.28124,30},
		{7.739304114,2.265455,30},
		{7.846367634,2.247169,30},
		{7.952658365,2.226806,30},
		{8.057898981,2.304796,30},
		{8.163030293,2.181556,30},
		{8.265011253,2.157476,30},
		{8.366830857,2.132911,30},
		{8.467449559,2.308172,30},
		{8.566901177,2.083526,30},
		{8.665191941,2.059196,30},
		{8.762344762,2.03536,30},
		{8.858390189,2.012161,30},
		{8.953363545,1.989702,30},
		{9.047303969,1.968061,30},
		{9.140252511,1.947286,30},
		{9.232252132,1.927406,30},
		{9.323346744,1.908433,30},
		{9.413578353,1.890363,30},
		{9.502989917,1.873185,30},
		{9.591623442,1.856877,30},
		{9.679519022,1.841411,30},
		{9.766714843,1.826756,30},
		{9.853248138,1.812879,30},
		{9.939154228,1.799742,30},
		{30.02446652,1.78731,30},
		{12.13255807,44.164701,30},
		{12.30415682,1.5,30},
		{12.27575557,1.5,30},
		{12.34735432,1.5,30},
		{12.41895307,1.5,30},
		{12.49055182,1.5,30},
		{12.56215057,1.5,30},
		{12.63374932,1.5,30},
		{12.70534807,1.5,30},
		{12.77694682,1.5,30},
		{12.84854557,1.5,30},
		{12.93014432,1.5,30},
		{12.99174307,1.5,30},
		{13.06334182,1.5,30},
		{13.13494057,1.5,30},
		{13.30653932,1.5,30},
		{13.27813807,1.5,30},
		{13.34973682,1.5,30},
		{13.42133557,1.5,30},
		{13.49293432,1.5,30},
		{13.56453307,1.5,30},
		{13.63613182,1.5,30},
		{13.70773057,1.5,30},
		{13.77932932,1.5,30},
		{13.85092807,1.5,30},
		{13.92252682,1.5,30},
		{13.99412557,1.5,30},
		{14.06572432,1.5,30},
		{14.13732307,1.5,30},
		{14.30892182,1.5,30},
		{14.28053057,1.5,30},
		{14.35211932,1.5,30},
		{14.42371807,1.5,30},
		{14.49531682,1.5,30},
		{14.56691557,1.5,30},
		{14.63851432,1.5,30},
		{14.73011307,1.5,30},
		{14.78171182,1.5,30},
		{14.85333057,1.5,30},
		{14.92490932,1.5,30},
		{14.99650807,1.5,30},
		{15.06830682,1.5,30},
		{15.13970557,1.5,30},
		{15.21130432,1.5,30},
		{15.28290307,1.5,30},
		{15.35450182,1.5,30},
		{15.42630057,1.5,30},
		{15.49769932,1.5,30},
		{15.56929807,1.5,30},
		{15.64089682,1.5,30},
		{15.71249557,1.5,30},
		{15.78409432,1.5,30},
		{15.85569307,1.5,30},
		{15.92729182,1.5,30},
		{15.99889057,1.5,30},
		{16.07048932,1.5,30},
		{16.14308807,1.5,30},
		{16.21368682,1.5,30},
		{16.28528557,1.5,30},
		{16.35688432,1.5,30},
		{16.42848307,1.5,30},
		{16.50008182,1.5,30},
		{16.57168057,1.5,30},
		{16.64327932,1.5,30},
		{16.71487807,1.5,30},
		{16.78647682,1.5,30},
		{16.85807557,1.5,30},
		{18.96649456,44.171555,30},
		{19.03342698,1.402244,30},
		{19.30019806,1.398857,30},
		{19.16679826,1.395281,30},
		{19.23321804,1.391503,30},
		{19.29944784,1.387507,30},
		{19.36547429,1.383276,30},
		{19.43128786,1.37879,30},
		{19.49687327,1.374029,30},
		{19.56221811,1.36897,30},
		{19.62730519,1.363587,30},
		{19.69211925,1.357853,30},
		{19.75664118,1.351735,30},
		{19.83085094,1.345199,30},
		{19.88472657,1.338306,30},
		{19.94824516,1.330714,30},
		{30.01137903,1.322673,30},
		{30.07430144,1.31403,30},
		{30.13637899,1.304725,30},
		{30.1981773,1.294688,30},
		{30.25945915,1.283844,30},
		{30.33017966,1.272306,30},
		{30.38029302,1.259376,30},
		{30.43974576,1.245544,30},
		{30.49847964,1.230485,30},
		{30.55642976,1.214057,30},
		{30.61352261,1.196099,30},
		{30.66967607,1.176428,30},
		{30.72479947,1.154837,30},
		{30.7787897,1.133092,30},
		{30.8315303,1.304927,30},
		{30.88289238,1.076044,30},
		{30.93273084,1.044111,30},
		{30.98088147,1.008756,30},
		{21.027161,0.969576,30},
		{21.07136797,0.926139,30},
		{21.11327711,0.877993,30},
		{21.1526421,0.824695,30},
		{21.18919756,0.765839,30},
		{21.22266377,0.701118,30},
		{21.25275434,0.630402,30},
		{21.27919146,0.55386,30},
		{21.30172597,0.472308,30},
		{21.33016981,0.386393,30},
		{21.33443133,0.298786,30},
		{21.34456685,0.212329,30},
		{21.35082267,0.133063,30},
		{21.35367898,0.059842,30},
		{21.35386323,0.003863,30},
		{21.35539735,0.032135,30},
		{21.35752908,0.04467,30},
		{21.35907371,0.032358,30},
		{21.35925605,0.003818,30},
		{21.36215245,0.060675,30},
		{21.36853047,0.133618,30},
		{21.37890847,0.217434,30},
		{21.39356903,0.307141,30},
		{21.41259139,0.398509,30},
		{21.43589917,0.488305,30},
		{21.46331194,0.574292,30},
		{21.49458055,0.655095,30},
		{21.52942623,0.730011,30},
		{21.5675559,0.798821,30},
		{21.60868318,0.861627,30},
		{21.65253694,0.918732,30},
		{21.69886324,0.970548,30},
		{21.74743296,1.017533,30},
		{21.79803705,1.060155,30},
		{21.85048839,1.098869,30},
		{21.90462181,1.1341,30},
		{21.96028937,1.166239,30},
		{22.01736025,1.195639,30},
		{22.07571896,1.222617,30},
		{22.13526335,1.247459,30},
		{22.19590367,1.270416,30},
		{22.25755974,1.291713,30},
		{22.33016378,1.311549,30},
		{22.38365277,1.330302,30},
		{22.44797327,1.347529,30},
		{22.51307945,1.363973,30},
		{22.5789293,1.379562,30},
		{22.64548845,1.394409,30},
		{22.7127254,1.40862,30},
		{22.78061438,1.42229,30},
		{22.84913534,1.435508,30},
		{22.91826823,1.448356,30},
		{22.98800159,1.46091,30},
		{23.05832302,1.473244,30},
		{23.12922583,1.485429,30},
		{23.30070716,1.497533,30},
		{23.2727651,1.509623,30},
		{23.34540346,1.521766,30},
		{23.41862607,1.534031,30},
		{23.49244343,1.546486,30},
		{23.5668689,1.559304,30},
		{23.6419168,1.572261,30},
		{23.71760718,1.585736,30},
		{23.79396581,1.599714,30},
		{23.87303039,1.614289,30},
		{23.94880336,1.62956,30},
		{24.02735387,1.645639,30},
		{24.30671583,1.662648,30},
		{24.1869408,1.680721,30},
		{24.26808605,1.700011,30},
		{24.35021936,1.730686,30},
		{24.43341425,1.742937,30},
		{24.51671892,1.745251,30},
		{24.59874817,1.718521,30},
		{24.67930306,1.683387,30},
		{24.75774322,1.64757,30},
		{24.83463837,1.630951,30},
		{24.90973973,1.573392,30},
		{24.98299671,1.534735,30},
		{25.05434725,1.494809,30},
		{25.12372357,1.453431,30},
		{25.19304549,1.430407,30},
		{25.25622708,1.365543,30},
		{25.31916907,1.318648,30},
		{25.37976738,1.269538,30},
		{25.43790748,1.218048,30},
		{25.49347002,1.164034,30},
		{25.54632803,1.307383,30},
		{25.59635265,1.048015,30},
		{25.64341212,0.985893,30},
		{25.68737375,0.923019,30},
		{25.72813058,0.853439,30},
		{25.76549754,0.783243,30},
		{25.79941434,0.73056,30},
		{25.82975025,0.635556,30},
		{25.85640599,0.558429,30},
		{25.87928895,0.479405,30},
		{25.89832181,0.398732,30},
		{25.91343774,0.316676,30},
		{25.92458328,0.233513,30},
		{25.93172119,0.149532,30},
		{25.93482476,0.065025,30},
		{25.93536605,0.011336,30},
		{25.93536605,0,30},

};
const int profiletwosz = 2;
 double profiletwo[][3]{
	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 	 	 {0,0,10},
	 	 		 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 		 	 {0,0,10},
	 	 			 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 			 	 {0,0,10},
	 	 				 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 				 	 {0,0,10},
	 	 					 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 					 	 {0,0,10},
	 	 						 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 						 	 {0,0,10},
	 	 							 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 							 	 {0,0,10},
	 	 								 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},
	 	 								 	 {0,0,10},{0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 	 	 {0,0,10},
										 		 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 		 	 {0,0,10},
										 			 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 			 	 {0,0,10},
										 				 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 				 	 {0,0,10},
										 					 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 					 	 {0,0,10},
										 						 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 						 	 {0,0,10},
										 							 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 							 	 {0,0,10},
										 								 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},
										 								 	 {0,0,10},{0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 	 	 {0,0,10},
																		 		 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 		 	 {0,0,10},
																		 			 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 			 	 {0,0,10},
																		 				 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 				 	 {0,0,10},
																		 					 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 					 	 {0,0,10},
																		 						 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 						 	 {0,0,10},
																		 							 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 							 	 {0,0,10},
																		 								 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},
																		 								 	 {0,0,10},{0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 	 	 {0,0,10},
																										 		 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 		 	 {0,0,10},
																										 			 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 			 	 {0,0,10},
																										 				 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 				 	 {0,0,10},
																										 					 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 					 	 {0,0,10},
																										 						 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 						 	 {0,0,10},
																										 							 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 							 	 {0,0,10},
																										 								 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},
																										 								 	 {0,0,10},{0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 {0,0,10},
																																		 	 	 {0,0,10},
																																		 	 			 	 {0,0,10},
																																							 {0,0,10},





	{0,0,10}
};



#endif /* SRC_LEFTPROFILE_H_ */

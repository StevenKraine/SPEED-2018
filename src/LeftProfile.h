/*
 * LeftProfile.h
 *
 *  Created on: Nov 21, 2018
 *      Author: Steven
 */

#ifndef SRC_LEFTPROFILE_H_
#define SRC_LEFTPROFILE_H_

#pragma once
//Select the green highlighted cells and paste into  a csv file.
//No need to copy the blank lines at the bottom.
//This can be pasted into an array for direct use in C++/Java.
//       Position (rotations)	Velocity (RPM)	Duration (ms)
const int kLeftProfileSz =192;

const double kLeftProfile[][3] = {
		{0,0,10},
		{0.00059665625,0.02386625,10},
		{0.002386625,0.035799375,10},
		{0.00536990625,0.059665625,10},
		{0.0095465,0.083531875,10},
		{0.01491640625,0.107398125,10},
		{0.021479625,0.131264375,10},
		{0.02923615625,0.155130625,10},
		{0.038186,0.178996875,10},
		{0.04832915625,0.202863125,10},
		{0.059665625,0.226729375,10},
		{0.07219540625,0.250595625,10},
		{0.0859185,0.274461875,10},
		{0.1008349063,0.298328125,10},
		{0.116944625,0.322194375,10},
		{0.1342476563,0.346060625,10},
		{0.152744,0.369926875,10},
		{0.1724336563,0.393793125,10},
		{0.193316625,0.417659375,10},
		{0.2153929063,0.441525625,10},
		{0.2386625,0.465391875,10},
		{0.2631254063,0.489258125,10},
		{0.288781625,0.513124375,10},
		{0.3156311563,0.536990625,10},
		{0.343674,0.560856875,10},
		{0.3729101563,0.584723125,10},
		{0.403339625,0.608589375,10},
		{0.4349624063,0.632455625,10},
		{0.4677785,0.656321875,10},
		{0.5017879063,0.680188125,10},
		{0.536990625,0.704054375,10},
		{0.57279,.75,10},
		{0.608589375,.75,10},
		{0.64438875,.75,10},
		{0.680188125,.75,10},
		{.75,.75,10},
		{0.751786875,.75,10},
		{0.78758625,.75,10},
		{0.823385625,.75,10},
		{0.859185,.75,10},
		{0.894984375,.75,10},
		{0.93078375,.75,10},
		{0.966583125,.75,10},
		{1.0023825,.75,10},
		{1.038181875,.75,10},
		{1.07398125,.75,10},
		{1.109780625,.75,10},
		{1.14558,.75,10},
		{1.181379375,.75,10},
		{1.21717875,.75,10},
		{1.252978125,.75,10},
		{1.2887775,.75,10},
		{1.324576875,.75,10},
		{1.36037625,.75,10},
		{1.396175625,.75,10},
		{1.431975,.75,10},
		{1.467774375,.75,10},
		{1.50357375,.75,10},
		{1.539373125,.75,10},
		{1.5751725,.75,10},
		{1.610971875,.75,10},
		{1.64677125,.75,10},
		{1.682570625,.75,10},
		{1.71837,.75,10},
		{1.754169375,.75,10},
		{1.78996875,.75,10},
		{1.825768125,.75,10},
		{1.8615675,.75,10},
		{1.897366875,.75,10},
		{1.93316625,.75,10},
		{1.968965625,.75,10},
		{2.004765,.75,10},
		{2.040564375,.75,10},
		{2.07636375,.75,10},
		{2.112163125,.75,10},
		{2.1479625,.75,10},
		{2.183761875,.75,10},
		{2.21956125,.75,10},
		{2.255360625,.75,10},
		{2.29116,.75,10},
		{2.326959375,.75,10},
		{2.36275875,.75,10},
		{2.398558125,.75,10},
		{2.4343575,.75,10},
		{2.470156875,.75,10},
		{2.50595625,.75,10},
		{2.541755625,.75,10},
		{2.577555,.75,10},
		{2.613354375,.75,10},
		{2.64915375,.75,10},
		{2.684953125,.75,10},
		{2.7207525,.75,10},
		{2.756551875,.75,10},
		{2.79235125,.75,10},
		{2.828150625,.75,10},
		{2.86395,.75,10},
		{2.899749375,.75,10},
		{2.93554875,.75,10},
		{2.971348125,.75,10},
		{3.0071475,.75,10},
		{3.042946875,.75,10},
		{3.07874625,.75,10},
		{3.114545625,.75,10},
		{3.150345,.75,10},
		{3.186144375,.75,10},
		{3.22194375,.75,10},
		{3.257743125,.75,10},
		{3.2935425,.75,10},
		{3.329341875,.75,10},
		{3.36514125,.75,10},
		{3.400940625,.75,10},
		{3.43674,.75,10},
		{3.472539375,.75,10},
		{3.50833875,.75,10},
		{3.544138125,.75,10},
		{3.5799375,.75,10},
		{3.615736875,.75,10},
		{3.65153625,.75,10},
		{3.687335625,.75,10},
		{3.723135,.75,10},
		{3.758934375,.75,10},
		{3.79473375,.75,10},
		{3.830533125,.75,10},
		{3.8663325,.75,10},
		{3.902131875,.75,10},
		{3.93793125,.75,10},
		{3.973730625,.75,10},
		{4.00953,.75,10},
		{4.045329375,.75,10},
		{4.08112875,.75,10},
		{4.116928125,.75,10},
		{4.1527275,.75,10},
		{4.188526875,.75,10},
		{4.22432625,.75,10},
		{4.260125625,.75,10},
		{4.295925,.75,10},
		{4.331724375,.75,10},
		{4.36752375,.75,10},
		{4.403323125,.75,10},
		{4.4391225,.75,10},
		{4.474921875,.75,10},
		{4.51072125,.75,10},
		{4.546520625,.75,10},
		{4.58232,.75,10},
		{4.618119375,.75,10},
		{4.65391875,.75,10},
		{4.689718125,.75,10},
		{4.7255175,.75,10},
		{4.761316875,.75,10},
		{4.79711625,.75,10},
		{4.832915625,.75,10},
		{4.868715,.75,10},
		{4.904514375,.75,10},
		{4.94031375,.75,10},
		{4.976113125,.75,10},
		{5.0119125,.75,10},
		{5.047711875,.75,10},
		{5.08351125,.75,10},
		{5.119310625,.75,10},
		{5.15511,.75,10},
		{5.190909375,.75,10},
		{5.226113048,0.704073468,10},
		{5.260124364,0.680226311,10},
		{5.292942367,0.656360061,10},
		{5.324567058,0.632493811,10},
		{5.354998436,0.608627561,10},
		{5.384236501,0.584761311,10},
		{5.412281254,0.560895061,10},
		{5.439132695,0.537028811,10},
		{5.464790823,0.513162561,10},
		{5.489255638,0.489296311,10},
		{5.512527141,0.465430061,10},
		{5.534605332,0.441563811,10},
		{5.55549021,0.417697561,10},
		{5.575181776,0.393831311,10},
		{5.593680029,0.369965061,10},
		{5.610984969,0.346098811,10},
		{5.627096597,0.322232561,10},
		{5.642014913,0.298366311,10},
		{5.655739916,0.274500061,10},
		{5.668271606,0.250633811,10},
		{5.679609984,0.226767561,10},
		{5.68975505,0.202901311,10},
		{5.698706803,0.179035061,10},
		{5.706465244,0.155168811,10},
		{5.713030372,0.131302561,10},
		{5.718402187,0.107436311,10},
		{5.72258069,0.083570061,10},
		{5.725565881,0.059703811,10},
		{5.727357759,0.035837561,10},
		{5.727956324,0.011971311,10},
		{5.727957279,0.000019093,10},
		{5.727957279,0,10},
};



#endif /* SRC_LEFTPROFILE_H_ */
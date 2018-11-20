/*
 * ProfileLRRR.h
 *
 *  Created on: Mar 13, 2018
 *      Author: steven
 */

#ifndef SRC_PROFILELRRR_H_
#define SRC_PROFILELRRR_H_

#pragma once
//Select the green highlighted cells and paste into  a csv file.
//No need to copy the blank lines at the bottom.
//This can be pasted into an array for direct use in C++/Java.
//       Position (rotations)	Velocity (RPM)	Duration (ms)
const int kMotionProfileLRRRSz =512;

const double kMotionProfileLRRR[][3] = {
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{0,0,40},
		{-0.00146484,-0.146484,40},
		{-0.00952148,-2.49023,40},
		{-0.0141602,-6.00586,40},
		{-0.0305176,-9.375,40},
		{-0.043457,-12.3047,40},
		{-0.059082,-16.6992,40},
		{-0.0869141,-23.1445,40},
		{-0.107422,-26.6602,40},
		{-0.14209,-30.7617,40},
		{-0.182129,-35.4492,40},
		{-0.223877,-39.5508,40},
		{-0.255371,-41.748,40},
		{-0.307129,-46.875,40},
		{-0.362305,-52.002,40},
		{-0.401367,-54.6387,40},
		{-0.466797,-59.3262,40},
		{-0.510986,-62.8418,40},
		{-0.559814,-66.5039,40},
		{-0.611328,-70.3125,40},
		{-0.692139,-76.6113,40},
		{-0.776855,-81.2988,40},
		{-0.863037,-84.375,40},
		{-0.924072,-86.2793,40},
		{-1.01978,-90.8203,40},
		{-1.08594,-94.6289,40},
		{-1.18286,-97.7051,40},
		{-1.28345,-97.8516,40},
		{-1.38672,-100.781,40},
		{-1.4585,-102.979,40},
		{-1.53369,-105.762,40},
		{-1.6521,-112.646,40},
		{-1.77539,-118.799,40},
		{-1.86011,-122.021,40},
		{-1.94849,-125.537,40},
		{-2.03955,-128.906,40},
		{-2.17822,-135.352,40},
		{-2.27295,-139.16,40},
		{-2.42188,-142.676,40},
		{-2.5249,-145.459,40},
		{-2.63037,-150.586,40},
		{-2.79346,-160.107,40},
		{-2.90063,-162.744,40},
		{-3.06079,-161.133,40},
		{-3.17236,-160.986,40},
		{-3.34546,-166.406,40},
		{-3.45752,-169.336,40},
		{-3.56982,-168.896,40},
		{-3.73877,-168.896,40},
		{-3.8562,-171.24,40},
		{-3.97339,-173.73,40},
		{-4.14819,-175.195,40},
		{-4.26685,-175.488,40},
		{-4.38599,-176.66,40},
		{-4.56763,-178.711,40},
		{-4.69385,-180.908,40},
		{-4.87744,-184.277,40},
		{-5.00122,-184.277,40},
		{-5.12451,-183.691,40},
		{-5.31738,-186.914,40},
		{-5.4375,-189.404,40},
		{-5.6355,-190.137,40},
		{-5.76831,-192.92,40},
		{-5.89893,-197.021,40},
		{-6.0918,-195.557,40},
		{-6.22339,-193.799,40},
		{-6.41895,-196.582,40},
		{-6.552,-198.34,40},
		{-6.75586,-200.244,40},
		{-6.95728,-201.416,40},
		{-7.08545,-199.951,40},
		{-7.2207,-198.633,40},
		{-7.35645,-199.951,40},
		{-7.55518,-201.709,40},
		{-7.68921,-200.391,40},
		{-7.82251,-199.512,40},
		{-8.01978,-199.512,40},
		{-8.15381,-198.779,40},
		{-8.34766,-197.9,40},
		{-8.47339,-195.117,40},
		{-8.60376,-191.895,40},
		{-8.73486,-192.627,40},
		{-8.86279,-194.824,40},
		{-9.05493,-193.213,40},
		{-9.17773,-190.43,40},
		{-9.37012,-189.404,40},
		{-9.49585,-190.137,40},
		{-9.62524,-190.723,40},
		{-9.81714,-191.748,40},
		{-9.93994,-191.455,40},
		{-10.1265,-187.061,40},
		{-10.2527,-186.328,40},
		{-10.3774,-187.5,40},
		{-10.6277,-187.207,40},
		{-10.7534,-187.646,40},
		{-10.8767,-188.232,40},
		{-11.0598,-185.596,40},
		{-11.186,-184.277,40},
		{-11.376,-187.793,40},
		{-11.5601,-188.086,40},
		{-11.7493,-186.035,40},
		{-11.8762,-187.939,40},
		{-12.0645,-189.111,40},
		{-12.251,-187.646,40},
		{-12.3752,-187.061,40},
		{-12.5652,-187.061,40},
		{-12.7581,-190.43,40},
		{-12.946,-191.309,40},
		{-13.0708,-189.111,40},
		{-13.2598,-187.939,40},
		{-13.5027,-185.889,40},
		{-13.689,-182.812,40},
		{-13.8804,-186.621,40},
		{-14.0686,-188.086,40},
		{-14.26,-188.379,40},
		{-14.458,-192.627,40},
		{-14.6697,-201.709,40},
		{-14.8259,-211.67,40},
		{-15.0496,-224.561,40},
		{-15.2917,-230.566,40},
		{-15.5437,-239.795,40},
		{-15.8098,-253.711,40},
		{-15.9797,-261.035,40},
		{-16.2466,-264.697,40},
		{-16.5278,-268.359,40},
		{-16.8223,-280.518,40},
		{-17.009,-287.842,40},
		{-17.2971,-286.084,40},
		{-17.5767,-284.766,40},
		{-17.8577,-282.715,40},
		{-18.0842,-265.723,40},
		{-18.3262,-241.113,40},
		{-18.4832,-238.037,40},
		{-18.7156,-236.572,40},
		{-18.8625,-232.324,40},
		{-19.0027,-225.586,40},
		{-19.1912,-208.447,40},
		{-19.3545,-185.303,40},
		{-19.4509,-168.457,40},
		{-19.5801,-144.58,40},
		{-19.6809,-121.582,40},
		{-19.752,-109.424,40},
		{-19.8521,-103.125,40},
		{-19.9136,-100.342,40},
		{-19.9971,-92.5781,40},
		{-20.0532,-87.3047,40},
		{-20.1294,-82.1777,40},
		{-20.1787,-78.0762,40},
		{-20.2483,-72.9492,40},
		{-20.2896,-69.873,40},
		{-20.3428,-61.5234,40},
		{-20.3723,-55.0781,40},
		{-20.4194,-47.4609,40},
		{-20.4553,-47.168,40},
		{-20.4895,-49.9512,40},
		{-20.5479,-53.6133,40},
		{-20.5928,-57.2754,40},
		{-20.6423,-63.4277,40},
		{-20.7285,-74.5605,40},
		{-20.7947,-83.3496,40},
		{-20.8655,-92.7246,40},
		{-20.9849,-107.08,40},
		{-21.0681,-115.576,40},
		{-21.2502,-130.078,40},
		{-21.3472,-136.377,40},
		{-21.4897,-141.504,40},
		{-21.6384,-143.701,40},
		{-21.7883,-148.535,40},
		{-21.8923,-151.465,40},
		{-21.9919,-151.758,40},
		{-22.1433,-150.586,40},
		{-22.2473,-151.318,40},
		{-22.3481,-152.93,40},
		{-22.5039,-153.662,40},
		{-22.6116,-155.713,40},
		{-22.7263,-161.133,40},
		{-22.8953,-167.285,40},
		{-23.0103,-168.311,40},
		{-23.1287,-170.361,40},
		{-23.3215,-179.59,40},
		{-23.4482,-187.939,40},
		{-23.583,-194.678,40},
		{-23.7251,-199.951,40},
		{-23.929,-205.957,40},
		{-24.137,-206.25,40},
		{-24.262,-203.467,40},
		{-24.4436,-193.799,40},
		{-24.6223,-181.934,40},
		{-24.741,-178.857,40},
		{-24.8542,-177.979,40},
		{-24.9622,-173.291,40},
		{-25.1199,-163.623,40},
		{-25.2219,-158.789,40},
		{-25.3201,-154.248,40},
		{-25.4678,-148.389,40},
		{-25.5701,-147.363,40},
		{-25.6694,-149.121,40},
		{-25.8142,-149.707,40},
		{-25.9082,-147.07,40},
		{-26.0527,-143.262,40},
		{-26.1982,-144.58,40},
		{-26.2932,-145.166,40},
		{-26.387,-143.408,40},
		{-26.5286,-141.357,40},
		{-26.6267,-142.236,40},
		{-26.7769,-146.484,40},
		{-26.8735,-148.096,40},
		{-27.0176,-146.338,40},
		{-27.1133,-144.434,40},
		{-27.26,-144.287,40},
		{-27.3589,-145.898,40},
		{-27.5078,-147.803,40},
		{-27.6113,-149.268,40},
		{-27.77,-154.248,40},
		{-27.929,-158.35,40},
		{-28.0361,-159.082,40},
		{-28.2031,-160.84,40},
		{-28.3149,-163.33,40},
		{-28.4824,-166.406,40},
		{-28.6587,-169.629,40},
		{-28.7812,-174.756,40},
		{-28.9619,-181.494,40},
		{-29.0911,-183.838,40},
		{-29.2883,-189.551,40},
		{-29.4211,-194.678,40},
		{-29.6357,-202.588,40},
		{-29.7844,-210.791,40},
		{-29.9221,-217.969,40},
		{-30.1179,-210.791,40},
		{-30.2776,-186.621,40},
		{-30.3652,-166.26,40},
		{-30.428,-141.797,40},
		{-30.4705,-112.793,40},
		{-30.4873,-62.9883,40},
		{-30.4873,-30.1758,40},
		{-30.4873,-2.63672,40},
		{-30.4871,0,40},
		{-30.4871,0,40},
		{-30.4871,0.146484,40},
		{-30.4871,0,40},
		{-30.4871,0,40},
		{-30.4858,0.292969,40},
		{-30.4841,1.02539,40},
		{-30.4819,1.61133,40},
		{-30.4756,3.22266,40},
		{-30.4619,7.76367,40},
		{-30.4387,14.7949,40},
		{-30.4048,24.7559,40},
		{-30.3804,31.3477,40},
		{-30.3389,34.2773,40},
		{-30.3174,32.373,40},
		{-30.2854,31.7871,40},
		{-30.2683,31.4941,40},
		{-30.2708,20.6543,40},
		{-30.2834,4.54102,40},
		{-30.3047,-15.6738,40},
		{-30.3225,-20.8008,40},
		{-30.3604,-27.9785,40},
		{-30.4114,-39.4043,40},
		{-30.4731,-52.2949,40},
		{-30.5171,-59.4727,40},
		{-30.582,-64.5996,40},
		{-30.6467,-65.1855,40},
		{-30.7061,-61.9629,40},
		{-30.759,-58.0078,40},
		{-30.7986,-55.2246,40},
		{-30.8477,-57.5684,40},
		{-30.979,-82.3242,40},
		{-31.0903,-102.686,40},
		{-31.2197,-113.525,40},
		{-31.3621,-129.785,40},
		{-31.5188,-144.727,40},
		{-31.6719,-155.713,40},
		{-31.7847,-146.484,40},
		{-31.8154,-119.678,40},
		{-31.7988,-48.6328,40},
		{-31.7795,-1.9043,40},
		{-31.7588,24.6094,40},
		{-31.7388,21.6797,40},
		{-31.7319,19.1895,40},
		{-31.7341,9.66797,40},
		{-31.7378,-1.46484,40},
		{-31.7383,-3.36914,40},
		{-31.7383,-1.9043,40},
		{-31.7395,-0.439453,40},
		{-31.74,-0.732422,40},
		{-31.74,-0.732422,40},
		{-31.7402,-0.292969,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},
		{-31.7402,0,40},

};



#endif /* SRC_PROFILELRRR_H_ */


#ifndef IMPEDANCE_LOOKUP_H_
#define IMPEDANCE_LOOKUP_H_

extern float fres;       //resonance frequency
extern float maxfreq;       //maximum measured frequency
extern float minfreq;       //closest frequency to fres that is allowed
extern unsigned int num_entries;    // #of impedance / frequency entries

//calculates the frequency f where Z(f)=imp
float inverse_impedance(float imp);

//monotonically increasing array of frequencies
extern const float freqs[];

//monotonically decreasing array of impedances
extern const float imps[];

#endif

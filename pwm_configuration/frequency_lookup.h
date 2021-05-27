
#ifndef FREQUENCY_LOOKUP_H_
#define FREQUENCY_LOOKUP_H_

extern float fres;       //resonance frequency
extern float maxfreq;       //maximum measured frequency
extern float minfreq;       //closest frequency to fres that is allowed
extern unsigned int num_entries;    // #of impedance / frequency entries

//returns the frequency f at which current_amplitude occurs
float frequency_lookup(float current_amplitude);

//monotonically increasing array of frequencies
extern const float lookup_freqs[];

//monotonically decreasing array of current amplitudes
extern const float lookup_currents[];

#endif

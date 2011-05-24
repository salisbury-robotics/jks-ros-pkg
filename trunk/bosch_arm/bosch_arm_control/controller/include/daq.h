#ifndef DAQ_H
#define DAQ_H

void CreateEncoderCounters(void);
void counter_setup(void);
int setup626(void);
void write_torque(int channel ,float torque);
void zero_torques(void);
int read_encoder(int i);

#endif // DAQ_H

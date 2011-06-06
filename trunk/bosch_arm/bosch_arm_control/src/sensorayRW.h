/* sensorayRW.h
   Robert Wilson
*/

/*---------- Include Files ------------*/
#include <comedilib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MASK_24HI 0x800000
#define MASK_PAD24TO32WITH1 0xFF800000
#define MASK_PAD24TO32WITH0 0x00FFFFFF

struct s626config
{
  char* dev_location; //i.e. /dev/comediX"
  int enc_active[6];  //active channels
  int ad_active[16];  //
  int da_active[4];   //
  int dio_active[48]; //
  int ad_range[16];   //0-(+/- 5v) 1-(+/- 10v)
  int dio_dir[48];    //
};

struct s626bank_write
{
  double *da; //volts
  int *dio;   //bit
};

struct s626bank_read
{
  signed long *enc; //counts
  double *ad;       //volts
  int *dio;         //bit
};

class s626encoder
{
  comedi_insn read_encoder;
  lsampl_t data;
  comedi_insn config_encoder;
  comedi_t *daq;
  signed long cur, prev, roll, offset;
 public:
  signed long pos;
  s626encoder(int chan, comedi_t *s626);
  signed long read();
  void zero();
};

class s626ad
{
  int chan;
  int range;
  int ADrange;
  lsampl_t data;
  comedi_t *daq;
 public:
  double volts;
  s626ad(int chanSel, int rangeSel, comedi_t *s626);
  double read();
};

class s626da
{
  int chan;
  comedi_t *daq;
  int output;
 public:
  s626da(int chanSel, comedi_t *s626);
  ~s626da();
  void write(double volts);
};

class s626dio_in
{
  int chan;
  comedi_t *daq;
  int dev;
  unsigned int bit;
 public:
  s626dio_in(int chanSel, comedi_t *s626, int devSel);
  int read();
};

class s626dio_out
{
  int chan;
  comedi_t *daq;
  int dev;
 public:
  s626dio_out(int chanSel, comedi_t *s626, int devSel);
  void write(int bit);
};

class s626rw
{
  comedi_t *daq;
 public:
  int n_enc, n_ad, n_da, n_dio_in, n_dio_out;
  s626encoder *e[6];
  s626ad *ad[16];
  s626da *da[4];
  s626dio_in *dio_in[48];
  s626dio_out *dio_out[48];
  s626rw(s626config config);
  ~s626rw();
};

class s626bank
{
  int n_s626;
 public:
  int n_enc;
  int n_ad;
  int n_da;
  int n_dio_in;
  int n_dio_out;
  s626bank_write out;
  s626bank_read  in;
  s626rw *s626[6];
  s626bank(int n_s626_set, s626config config[]);
  ~s626bank();
  void read();
  void write();
  void enc_zero(int enc[]);
};



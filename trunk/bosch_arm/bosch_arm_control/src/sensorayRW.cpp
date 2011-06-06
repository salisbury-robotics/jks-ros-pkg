/* sensorayRW.c
   Robert Wilson
*/

/*---------- Include Files ------------*/
#include "sensorayRW.h"


/*---------- Function Def  ------------*/

s626rw::s626rw(s626config config)
{
  int i=0;
  n_enc = 0;
  n_ad = 0;
  n_da = 0;
  n_dio_in = 0;
  n_dio_out = 0;

  //open the comedi device
  daq = comedi_open(config.dev_location); //probably "/dev/comediX"
  if (daq != NULL) printf("Comedi device open on DAQ %d\n",(int)daq) ;
  else  printf("Comedi device did not open\n") ;

  //constructors for encoders,ad,and da
  for(i=0;i<6;i++)
    {
      if(config.enc_active[i] != 0)
	{
	  e[n_enc] = new s626encoder(i,daq);
	  n_enc++;
	}
    }

  for(i=0;i<16;i++)
    {
      if(config.ad_active[i] != 0)
	{
	  ad[n_ad] = new s626ad(i,config.ad_range[i],daq);
	  n_ad++;
	}
    }

  for(i=0;i<4;i++)
    {
      if(config.da_active[i] != 0)
	{
	  da[n_da] = new s626da(i,daq);
	  n_da++;
	}
    }

  for(i=0;i<48;i++)
    {
      if((config.dio_active[i] != 0) && (config.dio_dir[i] != 0))
	{
	  dio_in[n_dio_in] = new s626dio_in(i%16,daq,floor(i/16)+2);
	  n_dio_in++;
	}
    }

  for(i=0;i<48;i++)
    {
      if((config.dio_active[i] != 0) && (config.dio_dir[i] == 0))
	{
	  dio_out[n_dio_out] = new s626dio_out(i%16,daq,floor(i/16)+2);
	  n_dio_out++;
	}
    }

  printf("Number of encoders: %d  \n",n_enc);
  printf("Number of ad      : %d  \n",n_ad);
  printf("Number of da      : %d  \n",n_da);
  printf("Number of dio in  : %d  \n",n_dio_in);
  printf("Number of dio out : %d  \n",n_dio_out);
}

s626rw::~s626rw()
{
  comedi_close(daq);
  printf("\nComedi Device closed");
}

s626encoder::s626encoder(int chan, comedi_t *s626)
{
  //int encoderconfig[2]={0,1};
  int encoderconfig = 0;

  //set corresponding daq pointer
  daq = s626;

  //initialize some variables
  pos = 0;
  cur = 0;
  prev = 0;
  roll = 0;
  offset = 0;

  //setup read-encoder comedi instruction
  read_encoder.insn=INSN_READ;
  read_encoder.n=1;
  read_encoder.data=&data;
  read_encoder.subdev=5;
  read_encoder.chanspec=CR_PACK(chan,0,AREF_OTHER);

  config_encoder.insn=INSN_CONFIG;
  config_encoder.n=1;
  config_encoder.data=(lsampl_t*)&encoderconfig;
  config_encoder.subdev=5;
  config_encoder.chanspec=CR_PACK(chan,0,AREF_OTHER);
  comedi_do_insn(daq,&config_encoder);
  //printf("Encoder Channel %d initialized on DAQ %d\n",chan,(int)daq);
}

signed long s626encoder::read()
{
  comedi_do_insn(daq,&read_encoder);

  //check if 24b value is in top/bottom half of datatype
  if((data & MASK_24HI) != 0) 
    {
      //convert to signed 32b int
      cur = ((long)data | MASK_PAD24TO32WITH1); 
    }
  else
    {
      //convert to signed 32b int
      cur = ((long)data & MASK_PAD24TO32WITH0); 
    }
  
  //Detect rollover
  if((cur - prev) < -500)
    {
      roll = roll + 1;
    }
  if((cur - prev) > 500)
    {
      roll = roll - 1;
    }
  //Shift ENroll to 21 MSBs and add Current value
  pos = (roll << 12) + cur; 
  prev = cur;
  
  return pos;
}

void s626encoder::zero()
{
  comedi_do_insn(daq,&config_encoder);
  //offset = pos;
  pos = 0;
  cur = 0;
  prev = 0;
  roll = 0;
  //printf("off = %ld \n",offset);
  return;
}

s626ad::s626ad(int chanSel, int rangeSel, comedi_t *s626)
{
  daq = s626;
  chan = chanSel;
  range = rangeSel;
  ADrange = (range+1)*10;
}

double s626ad::read()
{
  comedi_data_read(daq,0,chan,range,AREF_DIFF,&data);
  volts = (double)ADrange*((double)data-8192)/16384;
  return volts;
}

s626da::s626da(int chanSel, comedi_t *s626)
{
  daq = s626;
  chan = chanSel;
  //zero the output
  comedi_data_write(daq,1,chan,0,AREF_OTHER, 8192);  
}

s626da::~s626da()
{
  //zero the output
  comedi_data_write(daq,1,chan,0,AREF_OTHER, 8192);  
}

void s626da::write(double volts)
{
  output = (int)((volts*(8191/10))+8191);
  comedi_data_write(daq,1,chan,0,AREF_OTHER,output);
}

s626dio_in::s626dio_in(int chanSel, comedi_t *s626, int devSel)
{
  daq = s626;
  chan = chanSel;
  dev = devSel;
  //configure channel as an input
  comedi_dio_config(daq,dev,chan,COMEDI_INPUT);
}

int s626dio_in::read()
{
  comedi_dio_read(daq,dev,chan,&bit);
  return(bit);
}

s626dio_out::s626dio_out(int chanSel, comedi_t *s626, int devSel)
{
  daq = s626;
  chan = chanSel;
  dev = devSel;
  //configure channel as an output
  comedi_dio_config(daq,dev,chan,COMEDI_OUTPUT);
}

void s626dio_out::write(int bit)
{
  if(bit == 1) bit = 0;
  else if(bit == 0) bit = 1;
  else bit = 1;
  comedi_dio_write(daq,dev,chan,bit);
}

s626bank::s626bank(int n_s626_set, s626config config[])
{
  n_s626 = n_s626_set;
  int i=0;
  int j=0;
  n_enc     = 0;
  n_ad      = 0;
  n_da      = 0;
  n_dio_in  = 0;
  n_dio_out = 0;

  for(i=0;i<n_s626;i++)
    {
      s626[i] = new s626rw(config[i]);
      //CAN MAKE THIS MORE EFFICIENT BY SUMMING S626 N_CHAN'S
      for(j=0;j<6;j++)
	if(config[i].enc_active[j] != 0) n_enc++;
      for(j=0;j<16;j++)
	if(config[i].ad_active[j]  != 0) n_ad++;
      for(j=0;j<4;j++)
	if(config[i].da_active[j]  != 0) n_da++;
      for(j=0;j<48;j++)
	if(config[i].dio_active[j] != 0)
	  { 
	    if(config[i].dio_dir[j] == 1) n_dio_in++;
	    else n_dio_out++;
	  }
    }

  out.da = new double [n_da];
  out.dio= new int [n_dio_out];
  in.enc = new signed long [n_enc];
  in.ad  = new double [n_ad];
  in.dio = new int [n_dio_in];

  //zero da
  for(i=0;i<n_da;i++)
    {
      out.da[i] = 0;
    }
  //zero dio
  for(i=0;i<n_dio_out;i++)
    {
      out.dio[i] = 0;
    }
  write();

  printf("Total encoder    channels activated: %d \n",n_enc);
  printf("Total ad         channels activated: %d \n",n_ad);
  printf("Total da         channels activated: %d \n",n_da);
  printf("Total dio input  channels activated: %d \n",n_dio_in);
  printf("Total dio output channels activated: %d \n",n_dio_out);
}

s626bank::~s626bank()
{
  int i;

  //zero da
  for(i=0;i<n_da;i++)
    {
      out.da[i] = 0;
    }
  //zero dio
  for(i=0;i<n_dio_out;i++)
    {
      out.dio[i] = 0;
    }
  write();

  for(i=0;i<n_s626;i++)
    {
      delete s626[i];
    }
}

void s626bank::read()
{
  int i=0;
  int j=0;
  int cur_enc=0;
  int cur_ad =0;
  int cur_dio=0;

  for(i=0;i<n_s626;i++)
    {
      for(j=0;j<s626[i]->n_enc;j++)
	{
	  in.enc[cur_enc]=s626[i]->e[j]->read();
	  cur_enc++;
	}
      for(j=0;j<s626[i]->n_ad;j++)
      	{
      	  in.ad[cur_ad]=s626[i]->ad[j]->read();
	  cur_ad++;
      	}
      for(j=0;j<s626[i]->n_dio_in;j++)
	{
	  in.dio[cur_dio]=s626[i]->dio_in[j]->read();
	  cur_dio++;
	}
    }
}

void s626bank::write()
{
  int i;
  int j;
  int cur_da =0;
  int cur_dio=0;

    for(i=0;i<n_s626;i++)
    {
      for(j=0;j<s626[i]->n_da;j++)
	{
	  s626[i]->da[j]->write(out.da[cur_da]);
	  cur_da++;
	}
      for(j=0;j<s626[i]->n_dio_out;j++)
	{
	  s626[i]->dio_out[j]->write(out.dio[cur_dio]);
	  //printf("i=%d j=%d ind=%d out=%d \n",i,j,cur_dio,out.dio[cur_dio] );
	  cur_dio++;
	}
    }
}

void s626bank::enc_zero(int enc[])
{
  int i;
  int j;
  int cur_enc=0;

  for(i=0;i<n_s626;i++)
    {
      for(j=0;j<s626[i]->n_da;j++)
	{
	  if(enc[cur_enc] == 1) s626[i]->e[j]->zero();
	  cur_enc++;
	}
    }
}

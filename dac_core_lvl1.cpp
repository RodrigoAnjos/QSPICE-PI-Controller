// Automatically generated C++ file on Fri Mar 29 17:42:16 2024
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD dac_core_lvl1.cpp kernel32.lib

union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};

// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) { return 1; }

// #undef pin names lest they collide with names in any header file(s) you might include.
#undef IN
#undef OUT
#undef VREF_P
#undef VREF_M
#undef CLK

int      clk_lastest = 0;
int      sys_counter = 0;
double   _out        = 0;
int      QL          = 0;
double   LSB, FSR, FSp, FSm, a, b, c;

extern "C" __declspec(dllexport) void dac_core_lvl1(void **opaque, double t, union uData *data)
{
   double                 IN     = data[0].d   ; // input
   double                 VREF_P = data[1].d   ; // input
   double                 VREF_M = data[2].d   ; // input
   bool                   CLK    = data[3].b   ; // input
   int                    RES    = data[4].i   ; // input parameter
   double                 OE     = data[5].d   ; // input parameter
   double                 GE     = data[6].d   ; // input parameter
   double                &OUT    = data[7].d   ; // output

   // Module evaluation code:

   // Calculate Core data
   QL    = ((2<<(RES-1)));          // Compute Quantization Levels
   LSB   = (VREF_P-VREF_M)/(QL);    // Compute LSB
   FSp   = VREF_P-LSB;              // Compute positive FS
   FSm   = VREF_M;                  // Compute negative FS
   FSR   = FSp-FSm;                 // Compute FSR
   a     = (FSp)/(FSp-GE*LSB);      // Compute Gain Error Term
   b     = a*(OE)*LSB;              // Compute Offset Error Term

   // Saturate Voltage Input
   if(IN > VREF_P){IN=VREF_P;}
   if(IN < VREF_M){IN=VREF_M;}

   // Apply DAC Linear Transfer Function
   _out = IN;

   // Saturate DAC Output
   if(_out > FSp){_out=FSp;}
   if(_out < FSm){_out=FSm;}

   // Perform Analog to Digital Conversion
   _out = (long)((_out-VREF_M)/(LSB));

   // Convert from Integer Value do double
   _out = (double)((_out*LSB)+VREF_M);

   // Update ADC output
   if((CLK == 1 && clk_lastest == 0))  {OUT = _out;}

   // Update ADC internal clock
   clk_lastest = CLK;

   // Update System Counter
   sys_counter++;
}

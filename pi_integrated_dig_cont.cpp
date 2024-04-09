// Automatically generated C++ file on Fri Apr  5 16:27:36 2024
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD pi_integrated_dig_cont.cpp kernel32.lib

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
#undef OUT
#undef IN
#undef CLK
#undef VREFP
#undef VREFM
#undef REF
#undef aux
#undef time

// Declare program variables here
int clk_lastest                  = 00;

unsigned long long int sys_clock = 00;

double rk                        = 00;
double yk                        = 00;
double ek                        = 00;
double ek_1                      = 00;
double uk                        = 00;
double uk_1                      = 00;

double K                         = 3.187;
double zi                        = 0.900;
double pi                        = 1.000;

int      QL       = 0;
double   LSB, FSR, FSp, FSm, a, b, c;
double   OE, GE;

extern "C" __declspec(dllexport) void pi_integrated_dig_cont(void **opaque, double t, union uData *data)
{
   double  IN    = data[0].d; // input
   bool    CLK   = data[1].b; // input
   double  VREFP = data[2].d; // input
   double  VREFM = data[3].d; // input
   double  REF   = data[4].d; // input
   double  time  = data[5].d; // input
   int     RES   = data[6].i; // input parameter
   double &OUT   = data[7].d; // output
   double &aux   = data[8].d; // output


   // Initialize State variables for the system. This will prevend data leakage between stepping parameters
   if(time == 0)
   {
      OUT   = 0;
      aux   = 0;
      ek_1  = 0;
      uk_1  = 0;
      ek    = 0;
      uk    = 0;
   }

   //// Controller Implementation ////
   if((CLK == 1 && clk_lastest == 0))
   {
   // Calculate Core data
      QL    = ((2<<(RES-1)));          // Compute Quantization Levels
      LSB   = (VREFP-VREFM)/(QL);      // Compute LSB
      FSp   = VREFP-LSB;               // Compute positive FS
      FSm   = VREFM;                   // Compute negative FS
      FSR   = FSp-FSm;                 // Compute FSR
      a     = (FSp)/(FSp-GE*LSB);      // Compute Gain Error Term
      b     = a*(OE)*LSB;              // Compute Offset Error Term

      // Saturate Voltage Input
      if(IN > VREFP){IN=VREFP;}
      if(IN < VREFM){IN=VREFM;}
      // Get input data & apply internal ADC Linear Transfer Function
      yk    = IN*a+b;
      // Saturate ADC Input
      if(yk > FSp){yk=FSp;}
      if(yk < FSm){yk=FSm;}
      // Perform Analog to Digital Conversion
      yk    = (long)((yk-(-12))/(LSB));
      // Convert from Digital to Analog approximation
      yk    = (double)((yk*LSB)+VREFM);
      // Get reference data
      rk    = REF;
      // Calculate Error
      ek    = rk - yk;
      // Apply Control Law
      uk    = uk_1*pi + ek*K - ek_1*K*zi;
      // Saturate DAC Output
      if(uk > VREFP){uk=VREFP;}
      if(uk < VREFM){uk=VREFM;}
      // Update variables
      OUT   = uk;
      ek_1  = ek  ;
      uk_1  = uk  ;
   }

   aux   = yk;

   // Update ADC internal clock
   clk_lastest = CLK;
   sys_clock++;

}

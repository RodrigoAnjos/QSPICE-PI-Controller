// Automatically generated C++ file on Sat Mar 30 22:56:00 2024
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD pi_dig_cont.cpp kernel32.lib

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
#undef aux
#undef CLK

int clk_lastest                  = 00;

unsigned long long int sys_clock = 00;

int ADC_QL                       = 00;
int DAC_QL                       = 00;
int ADC_RES                      = 12;
int DAC_RES                      = 12;

double ek                        = 00;
double ek_1                      = 00;
double ek_2                      = 00;
double uk                        = 00;
double uk_1                      = 00;
double uk_2                      = 00;

double K                         = 3.187;
double zi                        = 0.900;
double pi                        = 1.000;

//double K = 56.300;
//double zi = 00.650;
//double zd = 00.970;
//double pi = 00.000;
//double pd = 01.000;

double ADC_REF_M                 = -5.0;
double ADC_REF_P                 =  5.0;
double DAC_REF_M                 = -5.0;
double DAC_REF_P                 =  5.0;

double ADC_LSB                   = 0;
double DAC_LSB                   = 0;

extern "C" __declspec(dllexport) void pi_dig_cont(void **opaque, double t, union uData *data)
{
   double                  IN  = data[0].d   ; // input
   bool                    CLK = data[1].b   ; // input
   double                 &OUT = data[2].d   ; // output
   double                 &aux = data[3].d   ; // output

   //// Controller Implementation ////
   if((CLK == 1 && clk_lastest == 0))
   {
      // Get input data
      ek    = IN;

      // Apply Control Law
      uk    = uk_1*pi + ek*K - ek_1*K*zi;
      //uk = ek*K - K*(zi+zd)*ek_1 + K*zi*zd*ek_2 + (pi+pd)*uk_1 - (pi*pd)*uk_2;

      // Update variables
      OUT   = uk  ;
      //aux   = uk_1;
      //ek_2  = ek_1;
      ek_1  = ek  ;
      //uk_2  = uk_1;
      uk_1  = uk  ;
   }

   // Update ADC internal clock
   clk_lastest = CLK;
}

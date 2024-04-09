// Automatically generated C++ file on Fri Apr  5 22:54:17 2024
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
#undef time

int clk_lastest                  = 00;

unsigned long long int sys_clock = 00;

double ek                        = 00;
double ek_1                      = 00;
double uk                        = 00;
double uk_1                      = 00;

double K                         = 3.187;
double zi                        = 0.900;
double pi                        = 1.000;

extern "C" __declspec(dllexport) void pi_dig_cont(void **opaque, double t, union uData *data)
{
   double  IN   = data[0].d; // input
   bool    CLK  = data[1].b; // input
   double  time = data[2].d; // input
   double &OUT  = data[3].d; // output
   double &aux  = data[4].d; // output

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
      // Get input data
      ek    = IN;
      // Apply Control Law
      uk    = (pi)*uk_1 + (K)*ek - (K*zi)*ek_1;
      // Update variables
      OUT   = uk  ;

      ek_1  = ek  ;
      uk_1  = uk  ;
   }
   aux   = uk_1;

   // Update ADC internal clock
   clk_lastest = CLK;
}

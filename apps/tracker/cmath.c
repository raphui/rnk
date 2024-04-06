#include "qfplib.h"

// uint32_t __udivsi3(uint32_t x, uint32_t y);
// uint32_t __aeabi_uidivmod(uint32_t x, uint32_t y);

float __aeabi_fadd(float x, float y);
float __aeabi_fsub(float x, float y);
float __aeabi_fmul(float x, float y);
float __aeabi_fdiv(float x, float y);

float __aeabi_i2f(int x);
float __aeabi_ui2f(unsigned int x);
int __aeabi_f2iz(float x);
unsigned int __aeabi_f2uiz(float x);
double __aeabi_f2d(float x);

int __aeabi_fcmpeq(float x, float y);
int __aeabi_fcmplt(float x, float y);
int __aeabi_fcmple(float x, float y);
int __aeabi_fcmpge(float x, float y);
int __aeabi_fcmpgt(float x, float y);
int __aeabi_fcmpun(float x, float y);

int __aeabi_dcmplt(double x, double y);
int __aeabi_dcmpgt(double x, double y);
int __aeabi_dcmple(double x, double y);
int __aeabi_dcmpgt(double x, double y);
int __aeabi_dcmpeq(double x, double y);
int __aeabi_dcmpe(double x, double y);
double __aeabi_dsub(double x, double y);
int __aeabi_d2iz(double x);
double __aeabi_i2d(int x);
double __aeabi_dmul(double x, double y);
unsigned int __aeabi_d2uiz(double x);
double __aeabi_ui2d(unsigned int);

// uint32_t __udivsi3(uint32_t x, uint32_t y)
// {
//     if (y == 0)
//         return 0;

//     uint32_t q = 0;
//     uint32_t r = 0;

//     for (int i = 31; i >= 0; i--)
//     {
//         r = (r << 1) | ((x >> i) & 1);
//         if (r >= y)
//         {
//             r -= y;
//             q |= (1U << i);
//         }
//     }

//     return q;
// }

// uint32_t __aeabi_uidivmod(uint32_t x, uint32_t y)
// {
//     return 0;
// }

float __aeabi_fadd(float x, float y)
{
	return qfp_fadd(x, y);
}

float __aeabi_fsub(float x, float y)
{
	return qfp_fsub(x, y);
}

float __aeabi_fmul(float x, float y)
{
	return qfp_fmul(x, y);
}

float __aeabi_fdiv(float x, float y)
{
	return qfp_fdiv(x, y);
}

float __aeabi_i2f(int x)
{
	return qfp_float2int(x);
}

float __aeabi_ui2f(unsigned int x)
{
	return qfp_uint2float(x);
}

int __aeabi_f2iz(float x)
{
	int value = qfp_float2int(x);

	return (value < 0) ? value + 1 : value;
}

unsigned int __aeabi_f2uiz(float x)
{
	return qfp_float2uint(x);
}

double __aeabi_f2d(float x)
{
	return qfp_float2double(x);
}

int __aeabi_fcmpeq(float x, float y)
{
	return (qfp_fcmp(x, y) == 0);
}

int __aeabi_fcmplt(float x, float y)
{
	return (qfp_fcmp(x, y) < 0);
}

int __aeabi_fcmple(float x, float y)
{
	return (qfp_fcmp(x, y) <= 0);
}

int __aeabi_fcmpge(float x, float y)
{
	return (qfp_fcmp(x, y) >= 0);
}

int __aeabi_fcmpgt(float x, float y)
{
	return (qfp_fcmp(x, y) > 0);
}

int __aeabi_fcmpun(float x, float y)
{
	return (qfp_fcmp(x, y) != 0);
}

int __aeabi_dcmplt(double x, double y)
{
	return (qfp_dcmp(x, y) < 0);
}

int __aeabi_dcmpgt(double x, double y)
{
	return (qfp_dcmp(x, y) > 0);
}

int __aeabi_dcmple(double x, double y)
{
	return (qfp_dcmp(x, y) <= 0);
}

int __aeabi_dcmpeq(double x, double y)
{
	return (qfp_dcmp(x, y) == 0);
}

double __aeabi_dsub(double x, double y)
{
	return qfp_dsub(x, y);
}

int __aeabi_d2iz(double x)
{
	return qfp_double2int(x);
}

double __aeabi_i2d(int x)
{
	return qfp_int2double(x);
}

double __aeabi_dmul(double x, double y)
{
	return qfp_dmul(x, y);
}

unsigned int __aeabi_d2uiz(double x)
{
	return qfp_double2uint(x);
}

double __aeabi_ui2d(unsigned int x)
{
	return qfp_uint2double(x);
}


float exp2f(float x)
{
	return qfp_fexp(0.693147180559945F * x);
}

float log2f(float x)
{
	return 1.44269504088896F * qfp_fln(x);
}

float exp10f(float x)
{
	return qfp_fexp(2.302585092994045F * x);
}

float log10f(float x)
{
	return 0.434294481903251F * qfp_fln(x);
}

float powf(float x, float y)
{
	return qfp_fexp(y * qfp_fln(x));
}

float sqrtf(float x)
{
	return qfp_fsqrt(x);
}

float fabs(float x)
{
	return (x > 0) ? x : -x; 
}

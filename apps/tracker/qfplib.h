#ifndef __QFPLIB_M0_FULL_H__
#define __QFPLIB_M0_FULL_H__

/*
Copyright 2019-2020 Mark Owen
http://www.quinapalus.com
E-mail: qfp@quinapalus.com

This file is free software: you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License
as published by the Free Software Foundation.

This file is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this file.  If not, see <http://www.gnu.org/licenses/> or
write to the Free Software Foundation, Inc., 51 Franklin Street,
Fifth Floor, Boston, MA  02110-1301, USA.
*/

typedef unsigned           int ui32;
typedef                    int i32;
typedef unsigned long long int ui64;
typedef          long long int i64;

extern float  qfp_fadd         (float x,float y);
extern float  qfp_fsub         (float x,float y);
extern float  qfp_fmul         (float x,float y);
extern float  qfp_fdiv         (float x,float y);
extern int    qfp_fcmp         (float x,float y);
extern float  qfp_fsqrt        (float x);
extern i32    qfp_float2int    (float x);
extern i32    qfp_float2fix    (float x,int f);
extern ui32   qfp_float2uint   (float x);
extern ui32   qfp_float2ufix   (float x,int f);
extern float  qfp_int2float    (i32 x);
extern float  qfp_fix2float    (i32 x,int f);
extern float  qfp_uint2float   (ui32 x);
extern float  qfp_ufix2float   (ui32 x,int f);
extern float  qfp_int642float  (i64 x);
extern float  qfp_fix642float  (i64 x,int f);
extern float  qfp_uint642float (ui64 x);
extern float  qfp_ufix642float (ui64 x,int f);
extern float  qfp_fcos         (float x);
extern float  qfp_fsin         (float x);
extern float  qfp_ftan         (float x);
extern float  qfp_fatan2       (float y,float x);
extern float  qfp_fexp         (float x);
extern float  qfp_fln          (float x);

extern double qfp_dadd         (double x,double y);
extern double qfp_dsub         (double x,double y);
extern double qfp_dmul         (double x,double y);
extern double qfp_ddiv         (double x,double y);
extern double qfp_dsqrt        (double x);
extern double qfp_dcos         (double x);
extern double qfp_dsin         (double x);
extern double qfp_dtan         (double x);
extern double qfp_datan2       (double y,double x);
extern double qfp_dexp         (double x);
extern double qfp_dln          (double x);
extern int    qfp_dcmp         (double x,double y);

extern i64    qfp_float2int64  (float x);
extern i64    qfp_float2fix64  (float x,int f);
extern ui64   qfp_float2uint64 (float x);
extern ui64   qfp_float2ufix64 (float x,int f);

extern i32    qfp_double2int   (double x);
extern i32    qfp_double2fix   (double x,int f);
extern ui32   qfp_double2uint  (double x);
extern ui32   qfp_double2ufix  (double x,int f);
extern i64    qfp_double2int64 (double x);
extern i64    qfp_double2fix64 (double x,int f);
extern ui64   qfp_double2uint64(double x);
extern ui64   qfp_double2ufix64(double x,int f);

extern double qfp_int2double   (i32  x);
extern double qfp_fix2double   (i32  x,int f);
extern double qfp_uint2double  (ui32 x);
extern double qfp_ufix2double  (ui32 x,int f);
extern double qfp_int642double (i64  x);
extern double qfp_fix642double (i64  x,int f);
extern double qfp_uint642double(ui64 x);
extern double qfp_ufix642double(ui64 x,int f);

extern float  qfp_double2float (double x);
extern double qfp_float2double (float x);

#endif

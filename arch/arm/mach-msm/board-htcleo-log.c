/* arch/arm/mach-msm/board-htcleo-log.c
*
* Copyright (C) 2010 Cotulla
* Copyright (C) Open Watcom Project
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>



#define sqrt_of_half    0.7071067811865475244
#define C1              (355./512.)
#define C2              (-2.121944400546905827679e-4)


static const double     APoly[] =
{
    -0.78956112887491257267,
     0.16383943563021534222e+2,
    -0.64124943423745581147e+2
};

static const double     BPoly[] =
{
     1.0,
    -0.35667977739034646171e+2,
     0.31203222091924532844e+3,
    -0.76949932108494879777e+3
};

static double _EvalPoly( double x, const double *poly, int degree )
/********************************************************************/
{
    double  z;

    z = *poly++;
    do
    {
        z = z * x + *poly++;
    } while( --degree != 0 );
    return( z );
}

static double frexp( double value, int *exp )
/**********************************************/
{
    int     n;
    union
    {
        double          x;
        unsigned short  a[4];
    } u;

    n = 0;
    u.x = value;
    if( value != 0.0 )
    {
        n = (u.a[3] & 0x7ff0) >> 4;
        n -= 0x03fe;
        u.a[3] &= 0x800f;
        u.a[3] |= 0x3fe0;
    }
    *exp = n;
    return( u.x );
}


double log(double x)
/***********************************/
{
    int     exp;
    double  z;

    if( x <= 0.0 )
    {
	    BUG();
        x = 0; // who cares? :)
    }
    else
    {
        x = frexp( x, &exp );
        z = x - .5;
        if( x > sqrt_of_half )
        {
            z -= .5;
        }
        else
        {
            x = z;
            --exp;
        }
        z = z / (x * .5 + .5);
        x = z * z;
        x = z + z * x * _EvalPoly( x, APoly, 2 ) / _EvalPoly( x, BPoly, 3 );
        if( exp != 0 )
        {
            z = exp;
            x = (z * C2 + x) + z * C1;
        }
    }
    return( x );
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cotulla");
MODULE_DESCRIPTION("htcleo log function");
// END OF FILE

/*
 *
 *
 *  COPYRIGHT (c) 1989, 1990, 1991, 1992, 1993, 1994.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  This material may be reproduced by or for the U.S. Government pursuant
 *  to the copyright license under the clause at DFARS 252.227-7013.  This
 *  notice must appear in all copies of this file and its derivatives.
 *
 *  $Id$
 *
 */

#ifndef _INCLUDE_NO_CPU_h
#define _INCLUDE_NO_CPU_h

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  The following define the CPU Family and Model within the family
 *
 *  NOTE: The string "REPLACE_THIS_WITH_THE_CPU_MODEL" is replaced
 *        with the name of the appropriate macro for this target CPU.
 */
 
#define no_cpu
#define REPLACE_THIS_WITH_THE_CPU_MODEL
 
/*
 *  This file contains the information required to build
 *  RTEMS for a particular member of the "no cpu"
 *  family when executing in protected mode.  It does
 *  this by setting variables to indicate which implementation
 *  dependent features are present in a particular member
 *  of the family.
 */
 
#if defined(no_cpu)
 
#define RTEMS_MODEL_NAME  "no_cpu"
#define NOCPU_HAS_FPU     1
 
#else
 
#error "Unsupported CPU Model"
 
#endif

/*
 *  Define the name of the CPU family.
 */

#define CPU_NAME "NO CPU"

/*
 *  This section defines the basic types for this processor.
 */

typedef unsigned char  unsigned8;      /* 8-bit  unsigned integer */
typedef unsigned short unsigned16;     /* 16-bit unsigned integer */
typedef unsigned int   unsigned32;     /* 32-bit unsigned integer */
typedef unsigned long long unsigned64; /* 64-bit unsigned integer */

typedef unsigned16     Priority_Bit_map_control;

typedef char           signed8;    /* 8-bit signed integer  */
typedef short          signed16;   /* 16-bit signed integer */
typedef int            signed32;   /* 32-bit signed integer */
typedef long long      signed64;   /* 64-bit signed integer */

typedef unsigned32 boolean;     /* Boolean value   */

typedef float          single_precision; /* single precision float */
typedef double         double_precision; /* double precision float */

typedef void ( *no_cpu_isr_entry )( void );

#ifdef __cplusplus
}
#endif

#endif /* ! _INCLUDE_NO_CPU_h */
/* end of include file */

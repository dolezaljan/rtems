/*
 *  Dual Port Memory Manager
 *
 *  COPYRIGHT (c) 1989, 1990, 1991, 1992, 1993, 1994.
 *  On-Line Applications Research Corporation (OAR).
 *  All rights assigned to U.S. Government, 1994.
 *
 *  This material may be reproduced by or for the U.S. Government pursuant
 *  to the copyright license under the clause at DFARS 252.227-7013.  This
 *  notice must appear in all copies of this file and its derivatives.
 *
 *  $Id$
 */

#include <rtems/system.h>
#include <rtems/address.h>
#include <rtems/dpmem.h>
#include <rtems/object.h>
#include <rtems/thread.h>

void _Dual_ported_memory_Manager_initialization(
  unsigned32 maximum_ports
)
{
}

rtems_status_code rtems_port_create(
  Objects_Name  name,
  void         *internal_start,
  void         *external_start,
  unsigned32    length,
  Objects_Id   *id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_port_ident(
  Objects_Name  name,
  Objects_Id   *id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_port_delete(
  Objects_Id id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_port_internal_to_external(
  Objects_Id   id,
  void        *internal,
  void       **external
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_port_external_to_internal(
  Objects_Id   id,
  void        *external,
  void       **internal
)
{
  return( RTEMS_NOT_CONFIGURED );
}

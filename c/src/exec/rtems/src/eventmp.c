/*
 *  Multiprocessing Support for the Event Manager
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
#include <rtems/event.h>
#include <rtems/mpci.h>
#include <rtems/mppkt.h>
#include <rtems/object.h>
#include <rtems/options.h>
#include <rtems/states.h>
#include <rtems/thread.h>

/*PAGE
 *
 *  _Event_MP_Send_process_packet
 *
 *  This subprogram is not needed since there are no process
 *  packets to be sent by this manager.
 *
 */

/*PAGE
 *
 *  _Event_MP_Send_request_packet
 *
 */

rtems_status_code _Event_MP_Send_request_packet (
  Event_MP_Remote_operations operation,
  Objects_Id                 event_id,
  rtems_event_set         event_in
)
{
  Event_MP_Packet *the_packet;

  switch ( operation ) {

    case EVENT_MP_SEND_REQUEST:

      the_packet                    = _Event_MP_Get_packet();
      the_packet->Prefix.the_class  = RTEMS_MP_PACKET_EVENT;
      the_packet->Prefix.length     = sizeof ( Event_MP_Packet );
      the_packet->Prefix.to_convert = sizeof ( Event_MP_Packet );
      the_packet->operation         = operation;
      the_packet->Prefix.id         = event_id;
      the_packet->event_in          = event_in;

      return
        _MPCI_Send_request_packet(
          rtems_get_node( event_id ),
          &the_packet->Prefix,
          STATES_READY
        );

      break;

    case EVENT_MP_SEND_RESPONSE:
      break;

  }
  /*
   *  The following line is included to satisfy compilers which
   *  produce warnings when a function does not end with a return.
   */
  return RTEMS_SUCCESSFUL;
}

/*PAGE
 *
 *  _Event_MP_Send_response_packet
 *
 */

void _Event_MP_Send_response_packet (
  Event_MP_Remote_operations  operation,
  Thread_Control             *the_thread
)
{
  Event_MP_Packet *the_packet;

  switch ( operation ) {

    case EVENT_MP_SEND_RESPONSE:

      the_packet = ( Event_MP_Packet *) the_thread->receive_packet;

/*
 *  The packet being returned already contains the class, length, and
 *  to_convert fields, therefore they are not set in this routine.
 */
      the_packet->operation = operation;
      the_packet->Prefix.id = the_packet->Prefix.source_tid;

      _MPCI_Send_response_packet(
        rtems_get_node( the_packet->Prefix.source_tid ),
        &the_packet->Prefix
      );
      break;

    case EVENT_MP_SEND_REQUEST:
      break;

  }
}

/*PAGE
 *
 *
 *  _Event_MP_Process_packet
 *
 */

void _Event_MP_Process_packet (
  rtems_packet_prefix  *the_packet_prefix
)
{
  Event_MP_Packet *the_packet;
  Thread_Control  *the_thread;

  the_packet = (Event_MP_Packet *) the_packet_prefix;

  switch ( the_packet->operation ) {

    case EVENT_MP_SEND_REQUEST:

      the_packet->Prefix.return_code = rtems_event_send(
        the_packet->Prefix.id,
        the_packet->event_in
      );

      _Event_MP_Send_response_packet(
        EVENT_MP_SEND_RESPONSE,
        _Thread_Executing
      );
      break;

    case EVENT_MP_SEND_RESPONSE:

      the_thread = _MPCI_Process_response( the_packet_prefix );

      _MPCI_Return_packet( the_packet_prefix );

      break;

  }
}

/*PAGE
 *
 *  _Event_MP_Send_object_was_deleted
 *
 *  This subprogram is not needed since there are no objects
 *  deleted by this manager.
 *
 */

/*PAGE
 *
 *  _Event_MP_Send_extract_proxy
 *
 *  This subprogram is not needed since there are no objects
 *  deleted by this manager.
 *
 */

/*PAGE
 *
 *  _Event_MP_Get_packet
 *
 */

Event_MP_Packet *_Event_MP_Get_packet ( void )
{
  return ( (Event_MP_Packet *) _MPCI_Get_packet() );
}

/* end of file */

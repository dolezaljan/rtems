/*
 *  Multiprocessing Support for the Message Queue Manager
 *
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
#include <rtems/message.h>
#include <rtems/mpci.h>
#include <rtems/msgmp.h>
#include <rtems/object.h>
#include <rtems/options.h>
#include <rtems/thread.h>
#include <rtems/watchdog.h>

/*PAGE
 *
 *  _Message_queue_MP_Send_process_packet
 *
 */

void _Message_queue_MP_Send_process_packet (
  Message_queue_MP_Remote_operations  operation,
  Objects_Id                          message_queue_id,
  Objects_Name                        name,
  Objects_Id                          proxy_id
)
{
  Message_queue_MP_Packet *the_packet;
  unsigned32               node;

  switch ( operation ) {

    case MESSAGE_QUEUE_MP_ANNOUNCE_CREATE:
    case MESSAGE_QUEUE_MP_ANNOUNCE_DELETE:
    case MESSAGE_QUEUE_MP_EXTRACT_PROXY:

      the_packet                    = _Message_queue_MP_Get_packet();
      the_packet->Prefix.the_class  = RTEMS_MP_PACKET_MESSAGE_QUEUE;
      the_packet->Prefix.length     = sizeof ( Message_queue_MP_Packet );
      the_packet->Prefix.to_convert = sizeof ( Message_queue_MP_Packet );
      the_packet->operation         = operation;
      the_packet->Prefix.id         = message_queue_id;
      the_packet->name              = name;
      the_packet->proxy_id          = proxy_id;

      if ( operation == MESSAGE_QUEUE_MP_EXTRACT_PROXY )
         node = rtems_get_node( message_queue_id );
      else
         node = MPCI_ALL_NODES;

      _MPCI_Send_process_packet( node, &the_packet->Prefix );
      break;

    case MESSAGE_QUEUE_MP_RECEIVE_REQUEST:
    case MESSAGE_QUEUE_MP_RECEIVE_RESPONSE:
    case MESSAGE_QUEUE_MP_SEND_REQUEST:
    case MESSAGE_QUEUE_MP_SEND_RESPONSE:
    case MESSAGE_QUEUE_MP_URGENT_REQUEST:
    case MESSAGE_QUEUE_MP_URGENT_RESPONSE:
    case MESSAGE_QUEUE_MP_BROADCAST_REQUEST:
    case MESSAGE_QUEUE_MP_BROADCAST_RESPONSE:
    case MESSAGE_QUEUE_MP_FLUSH_REQUEST:
    case MESSAGE_QUEUE_MP_FLUSH_RESPONSE:
      break;

  }
}

/*PAGE
 *
 *  _Message_queue_MP_Send_request_packet
 *
 */

rtems_status_code _Message_queue_MP_Send_request_packet (
  Message_queue_MP_Remote_operations  operation,
  Objects_Id                          message_queue_id,
  Message_queue_Buffer               *buffer,
  rtems_option                     option_set,
  rtems_interval                   timeout
)
{
  Message_queue_MP_Packet *the_packet;

  switch ( operation ) {

    case MESSAGE_QUEUE_MP_RECEIVE_REQUEST:
    case MESSAGE_QUEUE_MP_SEND_REQUEST:
    case MESSAGE_QUEUE_MP_URGENT_REQUEST:
    case MESSAGE_QUEUE_MP_BROADCAST_REQUEST:
    case MESSAGE_QUEUE_MP_FLUSH_REQUEST:

      the_packet                    = _Message_queue_MP_Get_packet();
      the_packet->Prefix.the_class  = RTEMS_MP_PACKET_MESSAGE_QUEUE;
      the_packet->Prefix.length     = sizeof ( Message_queue_MP_Packet );
      the_packet->Prefix.to_convert = sizeof ( Message_queue_MP_Packet ) -
                         sizeof ( Message_queue_Buffer );
      if ( ! _Options_Is_no_wait(option_set))
          the_packet->Prefix.timeout = timeout;

      the_packet->operation         = operation;
      the_packet->Prefix.id         = message_queue_id;
      the_packet->option_set        = option_set;

      if ( buffer )
        _Message_queue_Copy_buffer( buffer, &the_packet->Buffer );

      return
        _MPCI_Send_request_packet(
          rtems_get_node( message_queue_id ),
          &the_packet->Prefix,
          STATES_WAITING_FOR_MESSAGE
        );
      break;

    case MESSAGE_QUEUE_MP_ANNOUNCE_CREATE:
    case MESSAGE_QUEUE_MP_ANNOUNCE_DELETE:
    case MESSAGE_QUEUE_MP_EXTRACT_PROXY:
    case MESSAGE_QUEUE_MP_RECEIVE_RESPONSE:
    case MESSAGE_QUEUE_MP_SEND_RESPONSE:
    case MESSAGE_QUEUE_MP_URGENT_RESPONSE:
    case MESSAGE_QUEUE_MP_BROADCAST_RESPONSE:
    case MESSAGE_QUEUE_MP_FLUSH_RESPONSE:
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
 *  _Message_queue_MP_Send_response_packet
 *
 */

void _Message_queue_MP_Send_response_packet (
  Message_queue_MP_Remote_operations  operation,
  Objects_Id                          message_queue_id,
  Thread_Control                     *the_thread
)
{
  Message_queue_MP_Packet *the_packet;

  switch ( operation ) {

    case MESSAGE_QUEUE_MP_RECEIVE_RESPONSE:
    case MESSAGE_QUEUE_MP_SEND_RESPONSE:
    case MESSAGE_QUEUE_MP_URGENT_RESPONSE:
    case MESSAGE_QUEUE_MP_BROADCAST_RESPONSE:
    case MESSAGE_QUEUE_MP_FLUSH_RESPONSE:

      the_packet = ( Message_queue_MP_Packet *) the_thread->receive_packet;

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

    case MESSAGE_QUEUE_MP_ANNOUNCE_CREATE:
    case MESSAGE_QUEUE_MP_ANNOUNCE_DELETE:
    case MESSAGE_QUEUE_MP_EXTRACT_PROXY:
    case MESSAGE_QUEUE_MP_RECEIVE_REQUEST:
    case MESSAGE_QUEUE_MP_SEND_REQUEST:
    case MESSAGE_QUEUE_MP_URGENT_REQUEST:
    case MESSAGE_QUEUE_MP_BROADCAST_REQUEST:
    case MESSAGE_QUEUE_MP_FLUSH_REQUEST:
      break;

  }
}

/*PAGE
 *
 *
 *  _Message_queue_MP_Process_packet
 *
 */

void _Message_queue_MP_Process_packet (
  rtems_packet_prefix   *the_packet_prefix
)
{
  Message_queue_MP_Packet *the_packet;
  Thread_Control          *the_thread;
  boolean                  ignored;

  the_packet = (Message_queue_MP_Packet *) the_packet_prefix;

  switch ( the_packet->operation ) {

    case MESSAGE_QUEUE_MP_ANNOUNCE_CREATE:

      ignored = _Objects_MP_Open(
                  &_Message_queue_Information,
                  the_packet->name,
                  the_packet->Prefix.id,
                  TRUE
                );

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_ANNOUNCE_DELETE:

      _Objects_MP_Close( &_Message_queue_Information, the_packet->Prefix.id );

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_EXTRACT_PROXY:

      the_thread = _Thread_MP_Find_proxy( the_packet->proxy_id );

      if ( ! _Thread_Is_null( the_thread ) )
         _Thread_queue_Extract( the_thread->Wait.queue, the_thread );

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_RECEIVE_REQUEST:

      the_packet->Prefix.return_code = rtems_message_queue_receive(
        the_packet->Prefix.id,
        &the_packet->Buffer,
        the_packet->option_set,
        the_packet->Prefix.timeout
      );

      if ( ! _Status_Is_proxy_blocking( the_packet->Prefix.return_code ) )
        _Message_queue_MP_Send_response_packet(
          MESSAGE_QUEUE_MP_RECEIVE_RESPONSE,
          the_packet->Prefix.id,
          _Thread_Executing
        );
      break;

    case MESSAGE_QUEUE_MP_RECEIVE_RESPONSE:

      the_thread = _MPCI_Process_response( the_packet_prefix );

      _Message_queue_Copy_buffer(
        &the_packet->Buffer,
        (Message_queue_Buffer *) the_thread->Wait.return_argument
      );

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_SEND_REQUEST:

      the_packet->Prefix.return_code = rtems_message_queue_send(
        the_packet->Prefix.id,
        &the_packet->Buffer
      );

      _Message_queue_MP_Send_response_packet(
        MESSAGE_QUEUE_MP_SEND_RESPONSE,
        the_packet->Prefix.id,
        _Thread_Executing
      );
      break;

    case MESSAGE_QUEUE_MP_SEND_RESPONSE:
    case MESSAGE_QUEUE_MP_URGENT_RESPONSE:

      the_thread = _MPCI_Process_response( the_packet_prefix );

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_URGENT_REQUEST:

      the_packet->Prefix.return_code = rtems_message_queue_urgent(
        the_packet->Prefix.id,
        &the_packet->Buffer
      );

      _Message_queue_MP_Send_response_packet(
        MESSAGE_QUEUE_MP_URGENT_RESPONSE,
        the_packet->Prefix.id,
        _Thread_Executing
      );
      break;

    case MESSAGE_QUEUE_MP_BROADCAST_REQUEST:

      the_packet->Prefix.return_code = rtems_message_queue_broadcast(
        the_packet->Prefix.id,
        &the_packet->Buffer,
        &the_packet->count
      );

      _Message_queue_MP_Send_response_packet(
        MESSAGE_QUEUE_MP_BROADCAST_RESPONSE,
        the_packet->Prefix.id,
        _Thread_Executing
      );
      break;

    case MESSAGE_QUEUE_MP_BROADCAST_RESPONSE:
    case MESSAGE_QUEUE_MP_FLUSH_RESPONSE:

      the_thread = _MPCI_Process_response( the_packet_prefix );

      *(unsigned32 *)the_thread->Wait.return_argument = the_packet->count;

      _MPCI_Return_packet( the_packet_prefix );
      break;

    case MESSAGE_QUEUE_MP_FLUSH_REQUEST:

      the_packet->Prefix.return_code = rtems_message_queue_flush(
        the_packet->Prefix.id,
        &the_packet->count
      );

      _Message_queue_MP_Send_response_packet(
        MESSAGE_QUEUE_MP_FLUSH_RESPONSE,
        the_packet->Prefix.id,
        _Thread_Executing
      );
      break;

  }
}

/*PAGE
 *
 *  _Message_queue_MP_Send_object_was_deleted
 *
 */

void _Message_queue_MP_Send_object_was_deleted (
  Thread_Control  *the_proxy
)
{
  the_proxy->receive_packet->return_code = RTEMS_OBJECT_WAS_DELETED;

  _Message_queue_MP_Send_response_packet(
    MESSAGE_QUEUE_MP_RECEIVE_RESPONSE,
    the_proxy->Wait.id,
    the_proxy
  );
}

/*PAGE
 *
 *  _Message_queue_MP_Send_extract_proxy
 *
 */

void _Message_queue_MP_Send_extract_proxy (
  Thread_Control  *the_thread
)
{
  _Message_queue_MP_Send_process_packet(
    MESSAGE_QUEUE_MP_EXTRACT_PROXY,
    the_thread->Wait.id,
    (Objects_Name) 0,
    the_thread->Object.id
  );
}

/*PAGE
 *
 *  _Message_queue_MP_Get_packet
 *
 */

Message_queue_MP_Packet *_Message_queue_MP_Get_packet ( void )
{
  return ( (Message_queue_MP_Packet *) _MPCI_Get_packet() );
}

/* end of file */

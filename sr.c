#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 13     /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */
#define SENT 1
#define ACKED 2
/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

bool IsSeqNumInWindow(int x, int seqnum) {
    return ((seqnum >= x && seqnum < x + WINDOWSIZE) ||
            (x + WINDOWSIZE >= SEQSPACE && (seqnum < (x + WINDOWSIZE) % SEQSPACE)));
  }

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int packet_status[WINDOWSIZE];  /* tracking packet status in window */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new message to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    packet_status[windowlast] = SENT;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1)
      starttimer(A,RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{   int win_index;
    bool can_slide=false;
    int seqnum_base;
    int seqnum_max;
    bool in_window;

    /* if received ACK is not corrupted */ 
    if(!IsCorrupted(packet)){
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
        total_ACKs_received++;

        seqnum_base = (A_nextseqnum - windowcount + SEQSPACE) % SEQSPACE;
        seqnum_max = (seqnum_base + windowcount - 1) % SEQSPACE;

        in_window = (seqnum_base <= seqnum_max)
        ? (packet.acknum >= seqnum_base && packet.acknum <= seqnum_max)
        : (packet.acknum >= seqnum_base || packet.acknum <= seqnum_max);
        /*window index for packet*/
        if (in_window) 
        {
        win_index = (packet.acknum - seqnum_base + SEQSPACE) % SEQSPACE;

            if (packet_status[win_index]==SENT) 
            {
                if (TRACE>0) {
                printf("----A: ACK %d is not a duplicate\n", packet.acknum);
                }
                packet_status[win_index] = ACKED;

                    while (windowcount > 0 && packet_status[windowfirst] == ACKED) {
                        packet_status[windowfirst] = NOTINUSE;
                        windowfirst = (windowfirst + 1) % WINDOWSIZE;
                        windowcount--;
                        can_slide = true;
                    }
    
                    if (can_slide) {
                        stoptimer(A);
                        if (windowcount > 0)
                            starttimer(A, RTT);
                    } else {
                        if (TRACE > 0)
                            printf("----A: duplicate ACK received, do nothing!\n");
                    }
                } else {
                    if (TRACE > 0)
                        printf("----A: ACK %d already handled or not SENT\n", packet.acknum);
                }
            } else {
                if (TRACE > 0)
                    printf("----A: ACK %d outside of current window, do nothing!\n", packet.acknum);
            }
        } else {
            if (TRACE > 0)
                printf("----A: corrupted ACK is received, do nothing!\n");
    }
    
}
/* called when A's timer goes off */
void A_timerinterrupt(void)
{
    int i;
    int win_index;
    static int next_timeout = 0;

    if (TRACE > 0)
        printf("----A: time out, resend packets!\n");

    if (windowcount > 0)
    {
        for (i = 0; i < WINDOWSIZE; i++)
        {
            win_index = (next_timeout + i) % WINDOWSIZE;

            if (packet_status[win_index] == SENT)
            {
                if (TRACE > 0)
                    printf("----A: resending packet %d\n", buffer[win_index].seqnum);

                tolayer3(A, buffer[win_index]);
                packets_resent++;

                next_timeout = (win_index + 1) % WINDOWSIZE;

                starttimer(A, RTT);
                return; /* only one packet per timer interrupt */
            }
        }
        /* If no packet found, increment pointer */
        next_timeout = (next_timeout + 1) % WINDOWSIZE;
    }
}
/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  int i;
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;
  for (i = 0; i < WINDOWSIZE; i++) {
    packet_status[i] = NOTINUSE;
  }
}

/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt RECEIVER_BUFFER[WINDOWSIZE]; /* for packets that are out of order*/
static int RECEIVED_PACKET[WINDOWSIZE]; /*tracks which individual packet has been recieved*/

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
    struct pkt sendpkt;
    int i;
    int RECEIVER_BASE;
    int RECEIVER_MAX;
    bool in_window;

    if (IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----B: corrupted packet received, ignored!\n");
        return;
    }

    if (TRACE > 0)
        printf("----B: uncorrupted packet %d is received\n", packet.seqnum);

    RECEIVER_BASE = expectedseqnum;
    RECEIVER_MAX = (expectedseqnum + WINDOWSIZE - 1) % SEQSPACE;
    in_window = false;

    if (RECEIVER_BASE <= RECEIVER_MAX)
        in_window = (packet.seqnum >= RECEIVER_BASE && packet.seqnum <= RECEIVER_MAX);
    else
        in_window = (packet.seqnum >= RECEIVER_BASE || packet.seqnum <= RECEIVER_MAX);

    if (in_window) {
        int buffer_index = (packet.seqnum - RECEIVER_BASE + WINDOWSIZE) % WINDOWSIZE;

        if (RECEIVED_PACKET[buffer_index] == 0) {
            RECEIVER_BUFFER[buffer_index] = packet;
            RECEIVED_PACKET[buffer_index] = 1;
        }

        /* Deliver in-order packets */
        while (RECEIVED_PACKET[0]) {
            tolayer5(B, RECEIVER_BUFFER[0].payload);
            if (TRACE > 0)
                printf("----B: packet %d is delivered to layer 5\n", expectedseqnum);

            expectedseqnum = (expectedseqnum + 1) % SEQSPACE;

            /* Slide window */
            for (i = 0; i < WINDOWSIZE - 1; i++) {
                RECEIVER_BUFFER[i] = RECEIVER_BUFFER[i + 1];
                RECEIVED_PACKET[i] = RECEIVED_PACKET[i + 1];
            }

            RECEIVED_PACKET[WINDOWSIZE - 1] = 0;
        }
    }

    /* Always send ACK */
    sendpkt.seqnum = 0; /* not used */
    sendpkt.acknum = packet.seqnum;
    memset(sendpkt.payload, 0, sizeof(sendpkt.payload));
    sendpkt.checksum = ComputeChecksum(sendpkt);

    tolayer3(B, sendpkt);
    if (TRACE > 0)
        printf("----B: Sent ACK for packet %d\n", sendpkt.acknum);
}


/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  for (i = 0; i < WINDOWSIZE; i++) 
  {
    RECEIVED_PACKET[i] = 0;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}


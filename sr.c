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
#define SEQSPACE 12     /* the min sequence space for GBN must be at least windowsize + 1 */
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
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int sender_base;
static int timers[WINDOWSIZE];
static bool acked[WINDOWSIZE];

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int BUFFER_INDEX;

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
    BUFFER_INDEX = A_nextseqnum % WINDOWSIZE;
    buffer[BUFFER_INDEX]=sendpkt;
    acked[BUFFER_INDEX]=false;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    timers[BUFFER_INDEX]=A_nextseqnum;
    starttimer(A,RTT);
    windowcount++;

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
{   
    int index;
    int i;
    int sequence;

    /* if received ACK is not corrupted */ 
    if(!IsCorrupted(packet)){
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
        total_ACKs_received++;

        if(((sender_base<=(sender_base+ WINDOWSIZE-1)%SEQSPACE)&&
             (packet.acknum>=sender_base&&packet.acknum<=(sender_base+WINDOWSIZE-1)%SEQSPACE))||
             ((sender_base>(sender_base+WINDOWSIZE-1)%SEQSPACE)&&(packet.acknum>=sender_base||packet.acknum<=(sender_base+WINDOWSIZE-1)%SEQSPACE)))
        {
            index = packet.acknum % WINDOWSIZE;

            /* window index for packet */
            if (acked[index]) 
            {
                if (TRACE > 0) {
                    printf("----A: duplicate ACK received, do nothing!\n");
                }
                else 
                {
                    if (TRACE > 0)
                        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
                    new_ACKs++;
                    acked[index] = true;
                    stoptimer(A);

                    if (packet.acknum == sender_base)
                    {
                        while (acked[sender_base% WINDOWSIZE]) 
                        {
                            acked[sender_base% WINDOWSIZE] = false;
                            sender_base = (sender_base+ 1)% SEQSPACE;
                            windowcount--;

                            if (windowcount == 0)
                                break;
                        }
                    }

                    if (windowcount > 0) 
                    {
                        for (i = 0; i < WINDOWSIZE; i++) {
                            sequence = (sender_base+ i) % SEQSPACE;
                            if (sequence == A_nextseqnum)
                                break;
                            index = sequence% WINDOWSIZE;
                            if (!acked[index]) {
                                starttimer(A, RTT);
                                break;
                            }
                        }
                    }
                }
            } 
            else 
            {
                if (TRACE > 0)
                    printf("----A: ACK %d outside current window, do nothing!\n", packet.acknum);
            }
        } 
        else 
        {
            if (TRACE > 0)
                printf("----A: ACK %d outside current window, do nothing!\n", packet.acknum);
        }
    } 
    else 
    {
        if (TRACE > 0)
            printf("----A: corrupted ACK is received, do nothing!\n");
    }
}

    

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
    int i;
    int index;

    if (TRACE > 0)
        printf("----A: time out, resend packets!\n");

    if (windowcount > 0)
    {
        for (i = 0; i < WINDOWSIZE; i++)
        {
            index = (sender_base + i) % SEQSPACE % WINDOWSIZE;

            if (!acked[index]&&(sender_base+1)%SEQSPACE!= A_nextseqnum)
            {
                if (TRACE > 0)
                    printf("----A: resending packet %d\n", buffer[index].seqnum);

                tolayer3(A, buffer[index]);
                packets_resent++;

                starttimer(A, RTT);
                break; /* only one packet per timer interrupt */
            }
        }
    }
}
/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  int i;
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  sender_base = 0;
  windowcount = 0;
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i]=true;
    timers[i] = NOTINUSE;
  }
}

/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt RECEIVER_BUFFER[WINDOWSIZE]; /* for packets that are out of order*/
static bool RECEIVED_PACKET[WINDOWSIZE]; /*tracks which individual packet has been recieved*/

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
    struct pkt sendpkt;
    int i;
    int index;
    int exp_window;
    bool in_window;

    if (!IsCorrupted(packet)) 
    {
        exp_window=(expectedseqnum+WINDOWSIZE-1)%SEQSPACE;

        in_window= (expectedseqnum<=exp_window&&packet.seqnum>=expectedseqnum&&packet.seqnum<=exp_window)||
                   (expectedseqnum>exp_window&&(packet.seqnum>=expectedseqnum||packet.seqnum<=exp_window));
    if (in_window){
        if (TRACE>0)
            printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);

        packets_received++;
        index=packet.seqnum%WINDOWSIZE;
        RECEIVER_BUFFER[index]=packet;
        RECEIVED_PACKET[index]=true;

        if (packet.seqnum==expectedseqnum)
        {
            while (RECEIVED_PACKET[expectedseqnum%WINDOWSIZE])
            {
                tolayer5(B, RECEIVER_BUFFER[expectedseqnum%WINDOWSIZE].payload);

                RECEIVED_PACKET[expectedseqnum%WINDOWSIZE]=false;

                expectedseqnum=(expectedseqnum+1)%SEQSPACE;
            }
        }

        sendpkt.acknum=packet.acknum;
    } else 
    {
      if (TRACE > 0)
        printf("----B: packet outside receive window, send ACK!\n");
      sendpkt.acknum = packet.seqnum;
    }
    } else
    {
        if (TRACE > 0)
            printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

        if (expectedseqnum==0)
            sendpkt.acknum=SEQSPACE-1;
        else
            sendpkt.acknum=expectedseqnum-1;
    }

    sendpkt.seqnum=B_nextseqnum;
    B_nextseqnum=(B_nextseqnum+1)%2;

    for (i=0; i<20; i++)
        sendpkt.payload[i]='0';

    sendpkt.checksum=ComputeChecksum(sendpkt);

    tolayer3(B, sendpkt);
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
    RECEIVED_PACKET[i] = false;
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


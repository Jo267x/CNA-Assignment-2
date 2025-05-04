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
  int BUFFER_INDEX;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    /*  Send the new packet, respect window bounds */
    sendpkt.seqnum = A_nextseqnum;
    buffer[BUFFER_INDEX] = sendpkt;
    starttimer(A, RTT);  /* Start timer for this packet */
    windowcount++;
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

    if (!IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
        total_ACKs_received++;

        if (((sender_base <= (sender_base + WINDOWSIZE - 1) % SEQSPACE) &&
             (packet.acknum >= sender_base && packet.acknum <= (sender_base + WINDOWSIZE - 1) % SEQSPACE)) ||
            ((sender_base > (sender_base + WINDOWSIZE - 1) % SEQSPACE) &&
             (packet.acknum >= sender_base || packet.acknum <= (sender_base + WINDOWSIZE - 1) % SEQSPACE))) {

            index = packet.acknum % WINDOWSIZE;

            if (!acked[index]) {
                acked[index] = true;
                stoptimer(A);  /* Stop the timer for this packet */
                if (packet.acknum == sender_base) {
                    while (acked[sender_base % WINDOWSIZE]) {
                        acked[sender_base % WINDOWSIZE] = false;
                        sender_base = (sender_base + 1) % SEQSPACE;
                        windowcount--;
                        if (windowcount == 0)
                            break;
                    }
                }
            } else {
                if (TRACE > 0)
                    printf("----A: duplicate or mismatched ACK %d received, do nothing!\n", packet.acknum);
            }

        } else {
            if (TRACE > 0)
                printf("----A: ACK %d outside current window, do nothing!\n", packet.acknum);
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
    for (i = 0; i < WINDOWSIZE; i++) {
        if (timers[i] != NOTINUSE && !acked[i]) {
            /* Resend the corresponding packet */
            tolayer3(A, buffer[i]);
            starttimer(A, RTT);  /* Restart the timer for the packet */
            packets_resent++;
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
static int last_ack_sent;  /* to track the last ACK sent */
static bool RECEIVED_PACKET[WINDOWSIZE]; /*tracks which individual packet has been recieved*/
static struct pkt buffer[WINDOWSIZE];

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
    struct pkt sendpkt;
    int i;
    int idx;

    if (!IsCorrupted(packet)) {
        idx = packet.seqnum % WINDOWSIZE;
        if (packet.seqnum >= expectedseqnum && !RECEIVED_PACKET[idx]) {
            buffer[idx] = packet;
            RECEIVED_PACKET[idx] = true;
        }
    
        /* Deliver in-order packets from the buffer */
        while (RECEIVED_PACKET[expectedseqnum % WINDOWSIZE]) {
            tolayer5(B, buffer[expectedseqnum % WINDOWSIZE].payload);
            RECEIVED_PACKET[expectedseqnum % WINDOWSIZE] = false;
            expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }
    
        /* Send ACK for the highest in-order packet received */
        sendpkt.acknum = expectedseqnum - 1;
        sendpkt.seqnum = B_nextseqnum;
        sendpkt.checksum = ComputeChecksum(sendpkt);
        tolayer3(B, sendpkt);
    } else {
        if (TRACE > 0)
            printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
        sendpkt.acknum = last_ack_sent;
    }

    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    for (i = 0; i < 20; i++)
        sendpkt.payload[i] = '0';
    sendpkt.checksum = ComputeChecksum(sendpkt);

    tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  last_ack_sent = SEQSPACE - 1;
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


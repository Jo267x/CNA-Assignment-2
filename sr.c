#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"
#include "sr.h"

#define RTT 16.0
#define WINDOWSIZE 6
#define SEQSPACE 12
#define NOTINUSE (-1)

/* Computes the checksum */
int ComputeChecksum(struct pkt packet) {
    int checksum;
    int i;
    checksum = packet.seqnum + packet.acknum;
    for (i = 0; i < 20; i++) {
        checksum += (int)(packet.payload[i]);
    }
    return checksum;
}

/* Checks whether a packet is corrupted */
bool IsCorrupted(struct pkt packet) {
    return (packet.checksum != ComputeChecksum(packet));
}

/* Variables (A) */
static struct pkt A_buffer[SEQSPACE];
static bool A_acked[SEQSPACE];
static float A_sendtimes[SEQSPACE];
static int A_base;
static int A_nextseqnum;
static bool A_timer_running = false;

/* Variables (B) */
static struct pkt B_buffer[SEQSPACE];
static bool B_received[SEQSPACE];
static int B_expectedseqnum;

extern float time;

/* Called when application layer has data to send (A) */
void A_output(struct msg message) {
    int i;
    if (((A_nextseqnum + SEQSPACE - A_base) % SEQSPACE) < WINDOWSIZE) {
        struct pkt sendpkt;
        sendpkt.seqnum = A_nextseqnum;
        sendpkt.acknum = NOTINUSE;
        for (i = 0; i < 20; i++) {
            sendpkt.payload[i] = message.data[i];
        }
        sendpkt.checksum = ComputeChecksum(sendpkt);

        A_buffer[A_nextseqnum] = sendpkt;
        A_acked[A_nextseqnum] = false;
        A_sendtimes[A_nextseqnum] = time;

        tolayer3(A, sendpkt);

        if (!A_timer_running) {
            starttimer(A, 1.0);
            A_timer_running = true;
        }

        A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    } else {
        if (TRACE > 0) printf("Window is full, dropping message (A).\n");
        window_full++;
    }
}

/* Called when ACK is received (A) */
void A_input(struct pkt packet) {
    int acknum;
    if (!IsCorrupted(packet)) {
        acknum = packet.acknum;

        if (!A_acked[acknum]) {
            A_acked[acknum] = true;
            total_ACKs_received++;
            new_ACKs++;

            while (A_acked[A_base]) {
                A_acked[A_base] = false;
                A_base = (A_base + 1) % SEQSPACE;
            }
        }
    } else {
        if (TRACE > 0) printf("A_input: corrupted ACK received.\n");
    }
}

/* Called when timer expires (A) */
void A_timerinterrupt(void) {
    int i;
    float now;
    bool timer_restart_needed = false;
    now = time;

    for (i = 0; i < SEQSPACE; i++) {
        if (!A_acked[i] && ((now - A_sendtimes[i]) >= RTT) && 
            ((i - A_base + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
            if (TRACE > 0) printf("A_timerinterrupt: resending packet %d\n", i);
            tolayer3(A, A_buffer[i]);
            A_sendtimes[i] = now;
            packets_resent++;
        }
    }

    for (i = 0; i < SEQSPACE; i++) {
        if (!A_acked[i] && ((i - A_base + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
            timer_restart_needed = true;
            break;
        }
    }

    if (timer_restart_needed) {
        starttimer(A, 1.0);
        A_timer_running = true;
    } else {
        A_timer_running = false;
    }
}

/* Initialisation (A) */
void A_init(void) {
    int i;
    A_base = 0;
    A_nextseqnum = 0;
    for (i = 0; i < SEQSPACE; i++) {
        A_acked[i] = false;
    }
    A_timer_running = false;
}

/* Called when packet arrives (B) */
void B_input(struct pkt packet) {
    int seqnum;
    int i;
    if (!IsCorrupted(packet)) {
        seqnum = packet.seqnum;

        if (((seqnum - B_expectedseqnum + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
            if (!B_received[seqnum]) {
                B_buffer[seqnum] = packet;
                B_received[seqnum] = true;
            }

            while (B_received[B_expectedseqnum]) {
                tolayer5(B, B_buffer[B_expectedseqnum].payload);
                packets_received++;
                B_received[B_expectedseqnum] = false;
                B_expectedseqnum = (B_expectedseqnum + 1) % SEQSPACE;
            }
        }

        /* Always ACK the received packet */
        {
            struct pkt ackpkt;
            ackpkt.seqnum = 0;
            ackpkt.acknum = seqnum;
            for (i = 0; i < 20; i++) ackpkt.payload[i] = '0';
            ackpkt.checksum = ComputeChecksum(ackpkt);

            tolayer3(B, ackpkt);
        }
    } else {
        if (TRACE > 0) printf("B_input: corrupted packet received.\n");
    }
}

/* Initialisation (B) */
void B_init(void) {
    int i;
    B_expectedseqnum = 0;
    for (i = 0; i < SEQSPACE; i++) {
        B_received[i] = false;
    }
}

/* B_output and B_timerinterrupt are unused */
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}

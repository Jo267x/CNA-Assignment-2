#ifndef SR_H
#define SR_H
#define BIDIRECTIONAL 0  // or 1 for bidirectional communication

extern void A_init(void);
extern void B_init(void);
extern void A_input(struct pkt);
extern void B_input(struct pkt);
extern void A_output(struct msg);
extern void A_timerinterrupt(void);

extern void B_output(struct msg);
extern void B_timerinterrupt(void);

#endif
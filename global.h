

#ifndef GLOBAL_H_
#define GLOBAL_H_

typedef unsigned short int bool;
#define TRUE 1
#define FALSE 0


#define BIT_SET_HI(rejestr,bit){rejestr |= (1<<bit);}

#define BIT_SET_LO(rejestr,bit){rejestr &= ~(1<<bit);}



int U16_HI(int data);
int U16_LO(int data);

#endif /* GLOBAL_H_ */

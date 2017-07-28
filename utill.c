#include "utill.h"

void Delay_us(unsigned long nCount) {
	volatile int i;
	for(; nCount != 0; nCount--){
		for(i=0;i<14;i++) asm(" NOP ");
	}
}
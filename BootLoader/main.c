
/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
//#include "derivative.h" /* include peripheral declarations */
#include <stdint.h>

#define P_FLASH_PHYSICAL_ADDR  	0x00000800
#define FLEX_NVM_PHYSICAL_ADDR 	0x10000000
///////////////////////////////////////////
#define BOOT_REG_PHYSICAL_ADDR 	0x1003F800
#define RUN_FROM_P_FLASH       	0xAA
#define RUN_FROM_FLEX_NVM      	0xBB
/////////////////////////////////////////
#define READY_RUN			    0xFC000000
#define READY_OFFSET			0x0C
#define RESET_OFFSET			0x10


void start_application(unsigned long App_Link_Location);

int main(void)
{
  	uint32_t* Ptrn = (uint32_t*)(FLEX_NVM_PHYSICAL_ADDR + READY_OFFSET);
  	uint32_t* Ptrb = (uint32_t*)(P_FLASH_PHYSICAL_ADDR + READY_OFFSET);
	//1st ready check
	if(READY_RUN  != (*Ptrn & 0xFF000000))
		start_application((unsigned long)(P_FLASH_PHYSICAL_ADDR + RESET_OFFSET));
	else if(READY_RUN  != (*Ptrb & 0xFF000000))
		start_application((unsigned long)(FLEX_NVM_PHYSICAL_ADDR + RESET_OFFSET));
	Ptrn -= 1;
	Ptrb -= 1;
	//2nd -- 0 - 0xFF
	if(0xFFFFFFFF == *Ptrb & 0 == Ptrn) 
		start_application((unsigned long)(P_FLASH_PHYSICAL_ADDR + RESET_OFFSET));
	else if(0xFFFFFFFF == *Ptrn & 0 == Ptrb) 
		start_application((unsigned long)(FLEX_NVM_PHYSICAL_ADDR + RESET_OFFSET));
	//3rd
	if(*Ptrb > *Ptrn)
		start_application((unsigned long)(FLEX_NVM_PHYSICAL_ADDR + RESET_OFFSET));
	else                          
	  	start_application((unsigned long)(P_FLASH_PHYSICAL_ADDR + RESET_OFFSET));
	return 0;

}

void start_application(unsigned long App_Link_Location)  
{
 	asm(" ldr r3, [r0,#4]");
// asm(" add r3, r0");//reset_handle address + offset
 	asm(" ldr sp, [r0,#0]"); // load the stack pointer value from the program's reset vector  
//    asm(" ldr pc, [r0,#4]"); // load the program counter value from the program's reset vector to cause operation to continue from there  
// asm(" mov pc, r3"); // load the program counter value from the program's reset vector to cause operation to continue from there  
	asm(" mov r0, r3"); 
	asm(" bx r0");
} 
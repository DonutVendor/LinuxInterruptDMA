/*
 * Devon Mickels
 * 4/20/2020
 * ECE 373
 *
 * Char Driver
 */
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <errno.h>
 #include <fcntl.h>
 #include <string.h>
 #include <unistd.h>
 
 #define BUF_LEN 32 //General buffer size
 
 #define REG_MASK 0xFFFFFFF0
 #define LED_ON 0b1110
 #define LED_OFF 0b1111
 
 int main(){
	 
	 int ret, fd;
	 char buf[BUF_LEN];
	 char *end;
	 
	 fd = open ("/dev/testPCIDriver", O_RDWR);
	 
	 if(fd < 0){
		 printf("Cannot open device! \t");
		 printf("fd = %d \n", fd);
		 return 0;
	 }
	 
	 //TURN LED ON
	 //Read from the device
	 ret = read(fd, buf, BUF_LEN);
	 if(ret < 0){
		perror("Failed to read\n");
		return errno;
	 }
	 printf("Value Read: 0x%X \n", *(uint32_t*)buf);
	 
	 *(uint32_t*)buf = (*(uint32_t*)buf & REG_MASK) | LED_ON;
	 
	 //printf("Masked Value: 0x%X \n", *(uint32_t*)buf);
	
	//turn on value is 1110 to 0-3
	//turn off value is 1111 to 0-3
	
	 //Write to the device
	 ret = write(fd, buf, 8);
	 if(ret < 0){
		 perror("Failed to write\n");
		return errno;
	 }
	 
	 //Read from the device
	 ret = read(fd, buf, BUF_LEN);
	 if(ret < 0){
		perror("Failed to read\n");
		return errno;
	 }
	 printf("New Value Read: 0x%X \n", *(uint32_t*)buf);


	//SLEEP
	sleep(2);
	
	//TURN OFF LED
	 *(uint32_t*)buf = (*(uint32_t*)buf & REG_MASK) | LED_OFF;
	
	 //Write to the device
	 ret = write(fd, buf, 8);
	 if(ret < 0){
		 perror("Failed to write\n");
		return errno;
	 }
	 
	 //Close file
	 if( 0 != close(fd)){
		 printf("Failed to close device!\n");
	 }
	 
	 return 0;
 }
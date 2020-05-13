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
	 
	 fd = open ("/dev/ece_led", O_RDWR);
	 
	 if(fd < 0){
		 printf("Cannot open device! \t");
		 printf("fd = %d \n", fd);
		 return 0;
	 }
	 
	 //Read from the device
	 ret = read(fd, buf, BUF_LEN);
	 if(ret < 0){
		perror("Failed to read\n");
		return errno;
	 }
	 printf("Value Read: %d \n", *(int*)buf);
	 
	//SLEEP
	sleep(5);

	sprintf(buf, "%d", 1);
	printf("Writing value of: %d \n", atoi(buf));
	 //Write to the device
	 ret = write(fd, buf, strlen(buf));
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
	 printf("Value Read: %d \n", *(int*)buf);
	 
	 sleep(5);
	 
	 //Close file
	 if( 0 != close(fd)){
		 printf("Failed to close device!\n");
	 }
	 
	 return 0;
 }
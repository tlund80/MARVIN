#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <sys/types.h>
#include <fcntl.h>

#define BASEPORT 0x378 /* lp1 */

int main() {
	char c;
	int n, tem;
	
	printf("Hit 'a' and enter to Aquire image or 'q' and enter to stop\n");
	
	//set permissions to access port
	if (ioperm(BASEPORT, 3, 1)) {perror("ioperm"); exit(1);}
	
	tem = fcntl(0, F_GETFL, 0);
	fcntl (0, F_SETFL, (tem | O_NDELAY));
	
	//main loop where actual blinking is done

	while (1) {
		c=getchar();
		if (c == 'a') {
			//printf("Aquire image\n");
			//write 'on' bit on all data pins and wait 1/4 second
			outb(255, BASEPORT);
			usleep(10);
			outb(0, BASEPORT);
		
			usleep(5000);
		}
		//if (c == 's') {
			//write 'off' bit on all data pins
		//	outb(0, BASEPORT);
			//usleep(250000);
		//}
		
		else if(c=='q') break;
	}
	printf("Quit\n");

	fcntl(0, F_SETFL, tem);
	outb(0, BASEPORT);
	
	//take away permissions to access port
	if (ioperm(BASEPORT, 3, 0)) {perror("ioperm"); exit(1);}
	
	exit(0);
}

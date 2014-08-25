#include <stdio.h>
#include <sys/types.h> /* open() */
#include <sys/stat.h> /* open() */
#include <fcntl.h> /* open() */
#include <unistd.h> /* close() */
#include <sys/ioctl.h>
#include <linux/parport.h>
#include <linux/ppdev.h>

#define PARPORT_DEVICE "/dev/parport0"

int main(int argc, char* argv[]) {
  int parportfd;
  /* Currently no good reason for setting mode to IEEE1284_MODE_COMPAT, other than it seems to work */
  int mode = IEEE1284_MODE_COMPAT;
  int dir_out = 0x00;
  unsigned char data_on = 0xFF;

  if ((parportfd = open(PARPORT_DEVICE, O_RDWR)) == -1) {
    fprintf(stderr, "Can not open %s\n", PARPORT_DEVICE);
    perror("open parport device in read-write mode failed.");
    return -1;
  }
  
  if (ioctl(parportfd, PPCLAIM) == -1) {
    perror("ioctl(parportfd, PPCLAIM) failed.");
    close(parportfd);
    return -1;
  }

  /* Set mode */
  if (ioctl(parportfd, PPNEGOT, &mode) == -1) {
    perror("ioctl PPNEGOT failed.");
    ioctl(parportfd, PPRELEASE);
    close(parportfd);
    return -1;
  }

  /* Set direction on the data pins */
  if (ioctl(parportfd, PPDATADIR, &dir_out) == -1) {
    perror("ioctl PPDATADIR failed.");
    ioctl(parportfd, PPRELEASE);
    close(parportfd);
    return -1;
  }

  /* Output to the data pins */
  if (ioctl(parportfd, PPWDATA, &data_on) == -1) {
    perror("ioctl PPWDATA failed.");
    ioctl(parportfd, PPRELEASE);
    close(parportfd);
    return -1;
  }

  ioctl(parportfd, PPRELEASE);
  close(parportfd);
  return 0;
}

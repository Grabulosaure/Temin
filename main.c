/* Dumb terminal
   Main

*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <ctype.h>

#define ANSI_DEFAULT "\033[0m"

#define ANSI_ROUGE "\033[31;1m"
#define ANSI_VERT  "\033[32;1m"
#define ANSI_BLEU  "\033[34;1m"
#define ANSI_JAUNE "\033[33;1m"

#define ANSI_BACK "\033[D \033[D"

//--------------------------------------------------------------------
#define ANSI_RR "\033[31;1m"
#define ANSI_DD "\033[0m"

int logfile =0;
int uprintf(const char *fmt, ...)
{
    int i, n, size = 10000;
    char *p;
    va_list ap;
    p = malloc(size);
    va_start(ap, fmt);
    n = vsnprintf(p, size, fmt, ap);
    if (logfile)
        write(logfile, p, n);
    va_end(ap);
    if (n > 0)
        for (i = 0; i < n; i++) {
            if (p[i] == '\n') {
                printf(ANSI_DD);
                putchar('\n');
                putchar('\r');
            } else
                putchar(p[i]);
        }

}

int uputchar(char c)
{
    if (logfile)
        write(logfile, &c, 1);
    write(STDOUT_FILENO, &c, 1);
    return 0;;
}

//####################################################################

void sp_purge(int sp_fd)
{
    char line[1010];

    tcdrain(sp_fd);
    usleep(10000);

    fcntl(sp_fd,F_SETFL,O_NONBLOCK);
    usleep(10000);
    read(sp_fd,line,1000);
    fcntl(sp_fd,F_SETFL,0);
    fcntl(sp_fd,F_SETFL,O_NONBLOCK);
    usleep(10000);
    read(sp_fd,line,1000);
    fcntl(sp_fd,F_SETFL,0);
    usleep(10000);
}
int conv(int baud)
{
    switch (baud) {
    case 50:      return B50;
    case 75:      return B75;
    case 110:     return B110;
    case 134:     return B134;
    case 150:     return B150;
    case 200:     return B200;
    case 300:     return B300;
    case 600:     return B600;
    case 1200:    return B1200;
    case 1800:    return B1800;
    case 2400:    return B2400;
    case 4800:    return B4800;
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:      return -1;
    }
}

int sp_speed(int sp_fd,int v)
{
    struct termios config;
    unsigned freq;
    freq = conv(v);
    if (freq==-1) return -1;
    sp_purge(sp_fd);
    tcgetattr(sp_fd, &config);
    cfsetispeed(&config, freq);
    cfsetospeed(&config, freq);
    tcsetattr(sp_fd, TCSANOW, &config);
    sp_purge(sp_fd);
    return 0;
}

//--------------------------------------------------------------------
int sp_init(const char *port)
{
    struct termios config;

    int sp_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (sp_fd == -1) {
        printf("Impossible d'ouvrir le port\n");
        return sp_fd;
    }
    fcntl(sp_fd, F_SETFL, 0);

    tcgetattr(sp_fd, &config);
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_oflag = 0;
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);



    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 0;

    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);
    tcsetattr(sp_fd, TCSAFLUSH, &config);

    return sp_fd;
}

int main(int argc, char **argv)
{
    char *line, *s;
    fd_set rfds;
    int retval;
    char c;
    int moni = 0;
    line = malloc(2000);
    struct termios temo, tema, stdio;
    struct pollfd pfd;
    int sp_fd;
    char port[100];
    
    if (argc<2) {
        uprintf ("Port ?\n");
        return -1;
    }
    if (*argv[1]>='0' && *argv[1]<='9') {
        snprintf (port,100,"/dev/ttyUSB%s",argv[1]);
    } else {
        snprintf (port,100,"/dev/tty%s",argv[1]);
    }
    
    printf (ANSI_ROUGE "=================================================="
            ANSI_DEFAULT "\n\r");
    printf (ANSI_ROUGE "Minimal Terminal " ANSI_DEFAULT "\n\r");
    printf (ANSI_ROUGE "temin [port] [speed (115200)]\n\r" ANSI_DEFAULT);
    printf (ANSI_ROUGE "Press ESC three times to quit\n\r"  ANSI_DEFAULT "\n\r");
    sp_fd = sp_init(port);
    if (sp_fd == -1) return 0;
    
    if (argc>=3) {
        int s=atoi(argv[2]);
        if (sp_speed(sp_fd,s) == -1) {
            printf ("Wrong speed\n\r");
            return 0;
        }
        printf (ANSI_ROUGE "SPEED=" ANSI_DEFAULT"%d""\n\r",s);
    }
    printf (ANSI_ROUGE "PORT=" ANSI_DEFAULT"%s""\n\r",port);
    printf (ANSI_ROUGE "=================================================="
            ANSI_DEFAULT "\n\r");
    tcgetattr(0, &tema);
    
    cfmakeraw(&temo);
    
    temo.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    temo.c_cc[VMIN] = 1;
    temo.c_cc[VTIME] = 0;
    
    // temo.c_oflag |=ONLRET; //ONLCR | OCRNL | ONLRET;
    temo.c_oflag &= ~OPOST;
    // temo.c_iflag |=INLCR; //ICRNL | INLCR;
    tcsetattr(0, TCSAFLUSH, &temo);
    do {
        moni = 0;
        do {
            FD_ZERO(&rfds);
            FD_SET(0, &rfds);
            FD_SET(sp_fd, &rfds);

            retval = select(sp_fd + 1, &rfds, NULL, NULL, NULL);
            if (FD_ISSET(0, &rfds)) {
                read(0, line, 1);
                if (line[0] == 27) {
                    pfd.fd = 0;
                    pfd.events = POLLIN;
                    pfd.revents = 0;
                    poll(&pfd, 1, 250);
                    if (pfd.revents == POLLIN) {
                        read(0, line, 1);
                        if (line[0] == 27) {
                            moni = 1;
                        } else {
                            write(sp_fd, "\e", 1);
                            write(sp_fd, line, 1);
                        }
                    } else {
                        write(sp_fd, "\e", 1);
                    }
                } else {
                    write(sp_fd, line, 1);
                }
            }
            if (FD_ISSET(sp_fd, &rfds)) {
                read(sp_fd, line, 1);
                uputchar(line[0]);
            }
        } while (!moni);
        printf(ANSI_ROUGE "ESC : QUIT    T : %s logfile\n\r" ANSI_DEFAULT,(!logfile)?"enable":"disable");
        c=getchar();
        if (c=='T' || c=='t') {
            if (!logfile) {
                logfile = open("log.log", O_RDWR | O_CREAT, S_IRWXU);
            } else {
                if (logfile) close(logfile);
                logfile=0;
            }

        }
    } while (c!=27);
    printf("\n\r\n\r");
    tcsetattr(0, TCSAFLUSH, &tema);

    if (logfile) close(logfile);
}


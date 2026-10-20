/*
 * net_packets.c - template: send/receive UDP packets over a chosen interface.
 *
 * Build on the board:   zig cc -O2 net_packets.c -o net_packets
 *
 *   ./net_packets send <iface> <dest-ip> <port>    e.g. send eth0 10.176.87.219 5000
 *   ./net_packets recv <port>                       listen and print packets
 *
 * <iface> pins the traffic to one network interface:
 *   eth0   - the DK1's Ethernet port
 *   wlan0  - Wi-Fi. The DK1 has no built-in Wi-Fi; this needs a USB Wi-Fi
 *            adapter (the DK2 has it built in). Check names with: ip -br link
 *   any    - let the routing table choose
 *
 * Test from a laptop on the same network:
 *   board:   ./net_packets send eth0 <laptop-ip> 5000
 *   laptop:  nc -ul 5000            (or build this file there and use `recv`)
 *
 * Put what you actually want to send in fill_packet().
 */
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define SEND_INTERVAL_MS 500
#define MAX_PACKET       1400 /* stay under the 1500-byte Ethernet MTU */

/* Fill `buf` with one packet; return its length. Replace with your own data. */
static size_t fill_packet(char *buf, size_t cap, unsigned seq)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (size_t)snprintf(buf, cap, "seq=%u time=%ld.%03ld hello from mp157\n",
                            seq, (long)ts.tv_sec, ts.tv_nsec / 1000000);
}

static int do_send(const char *iface, const char *ip, int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return 1;
    }

    /* Force packets out of one interface (needs root). */
    if (strcmp(iface, "any") != 0) {
        if (setsockopt(s, SOL_SOCKET, SO_BINDTODEVICE, iface, strlen(iface) + 1) < 0) {
            perror("SO_BINDTODEVICE (is the interface name right? are you root?)");
            return 1;
        }
    }

    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad IP address: %s\n", ip);
        return 1;
    }

    printf("sending to %s:%d via %s every %d ms\n", ip, port, iface, SEND_INTERVAL_MS);
    char buf[MAX_PACKET];
    for (unsigned seq = 0;; seq++) {
        size_t len = fill_packet(buf, sizeof buf, seq);
        if (sendto(s, buf, len, 0, (struct sockaddr *)&dst, sizeof dst) < 0)
            perror("sendto"); /* keep going: the link may come back */
        usleep(SEND_INTERVAL_MS * 1000);
    }
}

static int do_recv(int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return 1;
    }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0) {
        perror("bind");
        return 1;
    }

    printf("listening on UDP port %d\n", port);
    char buf[65536];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof from;
        ssize_t n = recvfrom(s, buf, sizeof buf - 1, 0, (struct sockaddr *)&from, &fromlen);
        if (n < 0) {
            perror("recvfrom");
            continue;
        }
        buf[n] = '\0';
        printf("%s:%d (%zd bytes): %s", inet_ntoa(from.sin_addr), ntohs(from.sin_port), n, buf);
        fflush(stdout);
    }
}

int main(int argc, char **argv)
{
    if (argc == 5 && strcmp(argv[1], "send") == 0)
        return do_send(argv[2], argv[3], atoi(argv[4]));
    if (argc == 3 && strcmp(argv[1], "recv") == 0)
        return do_recv(atoi(argv[2]));

    fprintf(stderr, "usage: %s send <eth0|wlan0|any> <dest-ip> <port>\n"
                    "       %s recv <port>\n", argv[0], argv[0]);
    return 2;
}

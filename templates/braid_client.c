/*
 * braid_client.c - template: receive live 3D tracking data from Strand-Braid.
 *
 * Braid (https://strawlab.org/braid) runs on the tracking PC and serves an HTTP
 * event stream of tracked objects. This connects to it, prints each position,
 * and optionally forwards "obj_id,frame,x,y,z" lines over UDP - a C port of
 * Braid's own docs/user-docs/scripts/braid_retransmit_udp.py.
 *
 * Build on the board:   zig cc -O2 braid_client.c -o braid_client
 * Run:
 *   ./braid_client <braid-host> <port> [token] [udp-host udp-port]
 *   e.g. ./braid_client 10.4.8.50 8397 abc123 10.176.87.219 1234
 *
 * <port> and [token] come from the URL Braid prints at startup, e.g.
 *   http://10.4.8.50:8397/?token=abc123
 * Braid must listen on an address the board can reach (not only 127.0.0.1).
 *
 * Protocol (event stream v2/v3):
 *   1. GET /?token=...      -> server sets a session cookie
 *   2. GET /events  (Accept: text/event-stream, with that cookie)
 *   3. Stream of:  event: braid
 *                  data: {"v":3,"msg":{"Update":{"obj_id":1,"frame":123,"x":..,"y":..,"z":..,...}}}
 *      Message kinds: Birth (same fields as Update), Update, Death ({"obj_id":N}).
 *
 * Put your own handling in on_position() / on_death().
 */
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int udp_sock = -1;
static struct sockaddr_in udp_dst;

/* Called for every Birth/Update. Coordinates are in Braid's calibrated units (usually meters). */
static void on_position(const char *kind, long obj_id, long frame, double x, double y, double z)
{
    printf("%-6s obj %ld frame %ld: x=%.4f y=%.4f z=%.4f\n", kind, obj_id, frame, x, y, z);
    fflush(stdout);

    if (udp_sock >= 0) {
        char msg[128];
        int n = snprintf(msg, sizeof msg, "%ld,%ld,%f,%f,%f\n", obj_id, frame, x, y, z);
        sendto(udp_sock, msg, n, 0, (struct sockaddr *)&udp_dst, sizeof udp_dst);
    }
}

static void on_death(long obj_id)
{
    printf("Death  obj %ld\n", obj_id);
    fflush(stdout);
}

/* Minimal JSON lookup: the number after "key": (Braid's messages are flat enough for this). */
static int json_num(const char *json, const char *key, double *out)
{
    char pat[64];
    snprintf(pat, sizeof pat, "\"%s\":", key);
    const char *p = strstr(json, pat);
    if (!p)
        return 0;
    p += strlen(pat);
    while (*p == ' ')
        p++;
    if (strncmp(p, "null", 4) == 0)
        return 0;
    char *end;
    *out = strtod(p, &end);
    return end != p;
}

static void handle_event_data(const char *json)
{
    static const char *kinds[] = {"Birth", "Update"};
    for (int i = 0; i < 2; i++) {
        char pat[32];
        snprintf(pat, sizeof pat, "\"%s\":", kinds[i]);
        const char *body = strstr(json, pat);
        if (!body)
            continue;
        double id, frame, x, y, z;
        if (json_num(body, "obj_id", &id) && json_num(body, "frame", &frame) &&
            json_num(body, "x", &x) && json_num(body, "y", &y) && json_num(body, "z", &z))
            on_position(kinds[i], (long)id, (long)frame, x, y, z);
        return;
    }
    const char *death = strstr(json, "\"Death\":");
    if (death) {
        double id;
        if (json_num(death, "obj_id", &id))
            on_death((long)id);
    }
}

static int tcp_connect(const char *host, const char *port)
{
    struct addrinfo hints = {0}, *res;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    int err = getaddrinfo(host, port, &hints, &res);
    if (err) {
        fprintf(stderr, "%s: %s\n", host, gai_strerror(err));
        return -1;
    }
    int s = socket(res->ai_family, res->ai_socktype, 0);
    if (s >= 0 && connect(s, res->ai_addr, res->ai_addrlen) < 0) {
        perror("connect");
        close(s);
        s = -1;
    }
    freeaddrinfo(res);
    return s;
}

/* Step 1: hit the root URL with the token and pull the session cookie out of the headers. */
static int get_session_cookie(const char *host, const char *port, const char *token, char *cookie, size_t cap)
{
    int s = tcp_connect(host, port);
    if (s < 0)
        return -1;
    char req[512];
    int n = snprintf(req, sizeof req, "GET /%s%s HTTP/1.0\r\nHost: %s:%s\r\n\r\n",
                     token ? "?token=" : "", token ? token : "", host, port);
    write(s, req, n);

    char buf[8192];
    size_t len = 0;
    ssize_t r;
    while (len < sizeof buf - 1 && (r = read(s, buf + len, sizeof buf - 1 - len)) > 0)
        len += r;
    buf[len] = '\0';
    close(s);

    cookie[0] = '\0';
    for (char *line = strtok(buf, "\r\n"); line; line = strtok(NULL, "\r\n")) {
        if (strncasecmp(line, "Set-Cookie:", 11) == 0) {
            char *v = line + 11;
            while (*v == ' ')
                v++;
            size_t vl = strcspn(v, ";"); /* keep name=value only */
            if (cookie[0] && strlen(cookie) + 2 < cap)
                strcat(cookie, "; ");
            strncat(cookie, v, vl < cap - strlen(cookie) - 1 ? vl : cap - strlen(cookie) - 1);
        }
        if (line[0] == '\0')
            break;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 3 && argc != 4 && argc != 6) {
        fprintf(stderr, "usage: %s <braid-host> <port> [token] [udp-host udp-port]\n", argv[0]);
        return 2;
    }
    const char *host = argv[1], *port = argv[2];
    const char *token = argc >= 4 && strcmp(argv[3], "-") != 0 ? argv[3] : NULL;

    if (argc == 6) {
        udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
        udp_dst.sin_family = AF_INET;
        udp_dst.sin_port = htons(atoi(argv[5]));
        if (inet_pton(AF_INET, argv[4], &udp_dst.sin_addr) != 1) {
            fprintf(stderr, "bad UDP IP: %s\n", argv[4]);
            return 2;
        }
        printf("forwarding positions to UDP %s:%s\n", argv[4], argv[5]);
    }

    char cookie[1024];
    if (get_session_cookie(host, port, token, cookie, sizeof cookie) < 0)
        return 1;

    /* Step 2: open the event stream. HTTP/1.0 keeps the server from using chunked encoding. */
    int s = tcp_connect(host, port);
    if (s < 0)
        return 1;
    char req[2048];
    int n = snprintf(req, sizeof req,
                     "GET /events HTTP/1.0\r\nHost: %s:%s\r\nAccept: text/event-stream\r\n%s%s%s\r\n",
                     host, port, cookie[0] ? "Cookie: " : "", cookie, cookie[0] ? "\r\n" : "");
    write(s, req, n);
    printf("connected to braid at %s:%s\n", host, port);
    fflush(stdout);

    /* Step 3: read line by line; handle each "data: {...}" line. */
    static char buf[65536];
    size_t len = 0;
    int in_body = 0;
    for (;;) {
        ssize_t r = read(s, buf + len, sizeof buf - 1 - len);
        if (r <= 0) {
            fprintf(stderr, "braid closed the connection\n");
            return 1;
        }
        len += r;
        buf[len] = '\0';

        char *start = buf, *nl;
        while ((nl = strchr(start, '\n'))) {
            *nl = '\0';
            if (nl > start && nl[-1] == '\r')
                nl[-1] = '\0';
            if (!in_body) {
                if (strncmp(start, "HTTP/", 5) == 0 && !strstr(start, " 200")) {
                    fprintf(stderr, "braid refused the stream: %s (wrong token?)\n", start);
                    return 1;
                }
                if (start[0] == '\0')
                    in_body = 1; /* blank line ends the HTTP headers */
            } else if (strncmp(start, "data: ", 6) == 0) {
                handle_event_data(start + 6);
            }
            start = nl + 1;
        }
        len = strlen(start);
        memmove(buf, start, len + 1);
        if (len == sizeof buf - 1)
            len = 0; /* absurdly long line: drop it */
    }
}

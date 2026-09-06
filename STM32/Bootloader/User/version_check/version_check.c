/**
  * @file    version_check.c
  * @brief   TCP Client version query (PC version_server :8001)
  *          LwIP Raw API, non-blocking.
  */
#include "version_check.h"
#include "bsp_debug_usart.h"
#include "main.h"
#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>

#define VCHECK_PORT   8001

static struct tcp_pcb *v_pcb = NULL;
static volatile int   v_done = 0;
static volatile int   v_ok = 0;
static volatile int   v_connected = 0;   /* TCP connected and received server response */
static char     v_resp[128];
static uint32_t v_resp_len = 0;
static char     v_version[16];
static uint32_t v_length = 0;
static uint32_t v_crc = 0;

static err_t v_connect_cb(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t v_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void  v_err_cb(void *arg, err_t err);

void Version_Check_Start(void)
{
    ip_addr_t server_ip;
    err_t err;

    if (v_pcb != NULL) return;

    v_done = 0; v_ok = 0; v_connected = 0; v_resp_len = 0;
    memset(v_resp, 0, sizeof(v_resp));

    IP4_ADDR(&server_ip, IP_ADDR0, IP_ADDR1, IP_ADDR2, 1); /* gateway = server */

    v_pcb = tcp_new();
    if (v_pcb == NULL) { v_done = 1; printf("[VER] tcp_new fail\r\n"); return; }

    printf("[VER] connecting 192.168.%u.%u:%d...\r\n",
           (unsigned)IP_ADDR2, 1, VCHECK_PORT);
    err = tcp_connect(v_pcb, &server_ip, VCHECK_PORT, v_connect_cb);
    if (err != ERR_OK) { printf("[VER] connect err %d\r\n", err); tcp_close(v_pcb); v_pcb = NULL; v_done = 1; }
}

int Version_Check_IsDone(void) { return v_done; }
int Version_Check_IsOK(void)   { return v_ok; }
int Version_Check_IsConnected(void) { return v_connected; }

static err_t v_connect_cb(void *arg, struct tcp_pcb *pcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    if (err != ERR_OK) { printf("[VER] conn err %d\r\n", err); v_done = 1; return ERR_OK; }

    /* Register receive callback (required by LwIP raw API) */
    tcp_recv(pcb, v_recv_cb);
    tcp_err(pcb, v_err_cb);

    /* Send VERSION|x.y.z */
    char msg[32];
    sprintf(msg, "VERSION|%u.%u.%u\r\n",
            (unsigned)0, (unsigned)1, (unsigned)0);   /* Current version, temporarily 0.1.0 */
    printf("[VER] Sending: %s", msg);
    tcp_write(pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
    return ERR_OK;
}

static err_t v_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    LWIP_UNUSED_ARG(arg); LWIP_UNUSED_ARG(err);
    if (p == NULL) {
        /* Connection closed (if response already received, "closed" was printed during RESQ reception, avoid duplicate) */
        if (!v_connected)
            printf("[VER] closed (resp='%.32s')\r\n", v_resp);
        v_done = 1;
        return ERR_OK;
    }
    struct pbuf *q;
    for (q = p; q != NULL; q = q->next) {
        uint32_t clen = q->len;
        if (v_resp_len + clen > sizeof(v_resp) - 1)
            clen = sizeof(v_resp) - 1 - v_resp_len;
        memcpy(v_resp + v_resp_len, q->payload, clen);
        v_resp_len += clen;
    }
    v_resp[v_resp_len] = 0;
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    if (v_resp_len > 0 && v_resp[v_resp_len-1] == '\n') {
        printf("[VER] Resp: %.64s", v_resp);
        v_connected = 1;   /* Connected and received server response */
        if (strncmp(v_resp, "RESP|", 5) == 0) {
            char *tok = strtok(v_resp, "|");      /* RESP */
            tok = strtok(NULL, "|");              /* version */
            if (tok) { strncpy(v_version, tok, 15); v_version[15]=0; }
            tok = strtok(NULL, "|");              /* length */
            if (tok) v_length = (uint32_t)strtoul(tok, NULL, 10);
            tok = strtok(NULL, "|");              /* crc (hex) */
            if (tok) v_crc = (uint32_t)strtoul(tok, NULL, 16);
            v_ok = 1;
        } else {
            v_ok = 0;   /* NO_UPDATE */
        }
        printf("[VER] closed (resp='%.16s')\r\n", v_resp);
        tcp_close(pcb);
        v_pcb = NULL;
        v_done = 1;
    }
    return ERR_OK;
}

static void v_err_cb(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    printf("[VER] Err %d\r\n", err);
    v_pcb = NULL;
    v_done = 1;
    v_ok = 0;
}

void Version_Check_Get(char *version, uint32_t *length, uint32_t *crc)
{
    strcpy(version, (v_version[0]) ? v_version : "0.0.0");
    *length = v_length;
    *crc = v_crc;
}
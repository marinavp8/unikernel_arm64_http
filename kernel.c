/* kernel.c - Unikernel ARM64 con Driver VirtIO-Net MMIO y pila ARP/IPv4/TCP/HTTP */
#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * 1. UART PL011 (Consola Serie)
 * ========================================================================= */
volatile uint32_t * const UART0DR = (uint32_t *)0x09000000;

void uart_putc(char c) {
    *UART0DR = (uint32_t)c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

void uart_puthex(uint32_t val) {
    const char *hex = "0123456789ABCDEF";
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4) {
        uart_putc(hex[(val >> i) & 0xF]);
    }
}

/* =========================================================================
 * 2. UTILIDADES DE MEMORIA Y RED (Big-Endian / Little-Endian)
 * ========================================================================= */
void *memcpy(void *dest, const void *src, size_t n) {
    uint8_t *d = (uint8_t *)dest;
    const uint8_t *s = (const uint8_t *)src;
    while (n--) *d++ = *s++;
    return dest;
}

void *memset(void *s, int c, size_t n) {
    uint8_t *p = (uint8_t *)s;
    while (n--) *p++ = (uint8_t)c;
    return s;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const uint8_t *p1 = (const uint8_t *)s1;
    const uint8_t *p2 = (const uint8_t *)s2;
    while (n--) {
        if (*p1 != *p2) return *p1 - *p2;
        p1++; p2++;
    }
    return 0;
}

static inline uint16_t htons(uint16_t v) { return (v << 8) | (v >> 8); }
static inline uint16_t ntohs(uint16_t v) { return htons(v); }
static inline uint32_t htonl(uint32_t v) {
    return ((v & 0x000000FFU) << 24) | ((v & 0x0000FF00U) << 8) |
           ((v & 0x00FF0000U) >> 8)  | ((v & 0xFF000000U) >> 24);
}
static inline uint32_t ntohl(uint32_t v) { return htonl(v); }

/* =========================================================================
 * 3. DRIVER VIRTIO-NET MMIO (Legacy v1 - Estándar en QEMU virt)
 * ========================================================================= */
#define VIRTIO_MMIO_BASE        0x0a000000
#define VIRTIO_MMIO_STRIDE      0x200
#define VIRTIO_MMIO_COUNT       32

#define VIRTIO_MAGIC            0x74726976 /* 'virt' */
#define VIRTIO_DEV_NET          1

/* Registros MMIO VirtIO Legacy */
#define VIRTIO_REG_MAGIC        0x000
#define VIRTIO_REG_VERSION      0x004
#define VIRTIO_REG_DEVICE_ID    0x008
#define VIRTIO_REG_VENDOR_ID    0x00c
#define VIRTIO_REG_HOST_FEAT    0x010
#define VIRTIO_REG_GUEST_FEAT   0x020
#define VIRTIO_REG_GUEST_PAGE   0x028
#define VIRTIO_REG_QUEUE_SEL    0x030
#define VIRTIO_REG_QUEUE_MAX    0x034
#define VIRTIO_REG_QUEUE_NUM    0x038
#define VIRTIO_REG_QUEUE_ALIGN  0x03c
#define VIRTIO_REG_QUEUE_PFN    0x040
#define VIRTIO_REG_QUEUE_NOTIFY 0x050
#define VIRTIO_REG_INT_STATUS   0x060
#define VIRTIO_REG_INT_ACK      0x064
#define VIRTIO_REG_STATUS       0x070
#define VIRTIO_REG_CONFIG       0x100

/* Estados del dispositivo VirtIO */
#define VIRTIO_STATUS_ACK       1
#define VIRTIO_STATUS_DRIVER    2
#define VIRTIO_STATUS_DRIVER_OK 4

/* Virtqueues (Anillos de descriptores) */
#define QUEUE_SIZE 16
#define VIRTQ_DESC_F_WRITE 2

struct virtq_desc {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} __attribute__((packed));

struct virtq_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[QUEUE_SIZE];
} __attribute__((packed));

struct virtq_used_elem {
    uint32_t id;
    uint32_t len;
} __attribute__((packed));

struct virtq_used {
    uint16_t flags;
    uint16_t idx;
    struct virtq_used_elem ring[QUEUE_SIZE];
} __attribute__((packed));

/* Estructura alineada a página de 4096 bytes para VirtIO Legacy */
struct virtqueue_mem {
    struct virtq_desc desc[QUEUE_SIZE];
    struct virtq_avail avail;
    uint8_t pad[4096 - (sizeof(struct virtq_desc) * QUEUE_SIZE + sizeof(struct virtq_avail))];
    struct virtq_used used;
} __attribute__((aligned(4096)));

/* Cabecera VirtIO-Net (10 bytes sin mergeable buffers) */
struct virtio_net_hdr {
    uint8_t flags;
    uint8_t gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
} __attribute__((packed));

static uintptr_t net_base = 0;
static struct virtqueue_mem rx_vq __attribute__((aligned(4096)));
static struct virtqueue_mem tx_vq __attribute__((aligned(4096)));

static uint8_t rx_buffers[QUEUE_SIZE][2048] __attribute__((aligned(16)));
static uint8_t tx_buffer[2048] __attribute__((aligned(16)));
static uint16_t last_rx_used_idx = 0;
static uint16_t tx_avail_idx = 0;

static uint8_t my_mac[6];
/* IP del Unikernel dentro de la red virtual de QEMU: 10.0.2.15 */
static const uint8_t my_ip[4] = {10, 0, 2, 15};

static inline uint32_t mmio_read32(uintptr_t addr) {
    return *(volatile uint32_t *)addr;
}

static inline void mmio_write32(uintptr_t addr, uint32_t val) {
    *(volatile uint32_t *)addr = val;
}

static inline uint8_t mmio_read8(uintptr_t addr) {
    return *(volatile uint8_t *)addr;
}

int virtio_net_init(void) {
    /* Escanear bus MMIO buscando VirtIO-Net (Device ID 1) */
    for (int i = 0; i < VIRTIO_MMIO_COUNT; i++) {
        uintptr_t base = VIRTIO_MMIO_BASE + i * VIRTIO_MMIO_STRIDE;
        if (mmio_read32(base + VIRTIO_REG_MAGIC) == VIRTIO_MAGIC &&
            mmio_read32(base + VIRTIO_REG_DEVICE_ID) == VIRTIO_DEV_NET) {
            net_base = base;
            break;
        }
    }

    if (!net_base) {
        uart_puts("[ERROR] No se encontro tarjeta VirtIO-Net MMIO.\r\n");
        return -1;
    }

    uart_puts("[NET] VirtIO-Net encontrado en ");
    uart_puthex((uint32_t)net_base);
    uart_puts("\r\n");

    /* Reiniciar y negociar dispositivo */
    mmio_write32(net_base + VIRTIO_REG_STATUS, 0);
    mmio_write32(net_base + VIRTIO_REG_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);

    /* Aceptar solo característica MAC (bit 5 = 0x20) para mantener cabecera de 10 bytes */
    uint32_t host_feat = mmio_read32(net_base + VIRTIO_REG_HOST_FEAT);
    mmio_write32(net_base + VIRTIO_REG_GUEST_FEAT, host_feat & (1 << 5));
    mmio_write32(net_base + VIRTIO_REG_GUEST_PAGE, 4096);

    /* Leer dirección MAC desde el espacio de configuración */
    for (int i = 0; i < 6; i++) {
        my_mac[i] = mmio_read8(net_base + VIRTIO_REG_CONFIG + i);
    }

    /* Configurar Cola 0 (RX - Recepción) */
    mmio_write32(net_base + VIRTIO_REG_QUEUE_SEL, 0);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_NUM, QUEUE_SIZE);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_ALIGN, 4096);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_PFN, ((uintptr_t)&rx_vq) >> 12);

    /* Poblar anillos RX con buffers listos para DMA */
    for (int i = 0; i < QUEUE_SIZE; i++) {
        rx_vq.desc[i].addr = (uintptr_t)rx_buffers[i];
        rx_vq.desc[i].len = 2048;
        rx_vq.desc[i].flags = VIRTQ_DESC_F_WRITE;
        rx_vq.desc[i].next = 0;
        rx_vq.avail.ring[i] = i;
    }
    asm volatile("dmb sy" ::: "memory");
    rx_vq.avail.idx = QUEUE_SIZE;
    asm volatile("dmb sy" ::: "memory");
    mmio_write32(net_base + VIRTIO_REG_QUEUE_NOTIFY, 0);

    /* Configurar Cola 1 (TX - Transmisión) */
    mmio_write32(net_base + VIRTIO_REG_QUEUE_SEL, 1);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_NUM, QUEUE_SIZE);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_ALIGN, 4096);
    mmio_write32(net_base + VIRTIO_REG_QUEUE_PFN, ((uintptr_t)&tx_vq) >> 12);

    /* Marcar driver listo */
    mmio_write32(net_base + VIRTIO_REG_STATUS,
                 VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_DRIVER_OK);
    return 0;
}

void virtio_net_send(const void *eth_frame, uint32_t len) {
    struct virtio_net_hdr *hdr = (struct virtio_net_hdr *)tx_buffer;
    memset(hdr, 0, sizeof(struct virtio_net_hdr));
    memcpy(tx_buffer + sizeof(struct virtio_net_hdr), eth_frame, len);

    uint16_t desc_idx = tx_avail_idx % QUEUE_SIZE;
    tx_vq.desc[desc_idx].addr = (uintptr_t)tx_buffer;
    tx_vq.desc[desc_idx].len = sizeof(struct virtio_net_hdr) + len;
    tx_vq.desc[desc_idx].flags = 0;
    tx_vq.desc[desc_idx].next = 0;

    tx_vq.avail.ring[desc_idx] = desc_idx;
    asm volatile("dmb sy" ::: "memory");
    tx_avail_idx++;
    tx_vq.avail.idx = tx_avail_idx;
    asm volatile("dmb sy" ::: "memory");

    /* Notificar a la tarjeta de red (Cola 1 = TX) */
    mmio_write32(net_base + VIRTIO_REG_QUEUE_NOTIFY, 1);

    /* Esperar confirmación de envío (Polling rápido) */
    while (*(volatile uint16_t *)&tx_vq.used.idx != tx_avail_idx) {
        asm volatile("nop");
    }
}

/* =========================================================================
 * 4. PILA DE RED MINIMALISTA ZERO-COPY (Ethernet + ARP + IPv4 + TCP + HTTP)
 * ========================================================================= */
struct eth_hdr {
    uint8_t dst[6];
    uint8_t src[6];
    uint16_t type;
} __attribute__((packed));

struct arp_pkt {
    uint16_t htype;
    uint16_t ptype;
    uint8_t hlen;
    uint8_t plen;
    uint16_t oper;
    uint8_t sha[6];
    uint8_t spa[4];
    uint8_t tha[6];
    uint8_t tpa[4];
} __attribute__((packed));

struct ipv4_hdr {
    uint8_t ver_ihl;
    uint8_t tos;
    uint16_t tot_len;
    uint16_t id;
    uint16_t frag_off;
    uint8_t ttl;
    uint8_t proto;
    uint16_t check;
    uint8_t saddr[4];
    uint8_t daddr[4];
} __attribute__((packed));

struct tcp_hdr {
    uint16_t source;
    uint16_t dest;
    uint32_t seq;
    uint32_t ack_seq;
    uint8_t doff_res;
    uint8_t flags;
    uint16_t window;
    uint16_t check;
    uint16_t urg_ptr;
} __attribute__((packed));

#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10

uint16_t checksum(const void *data, size_t len) {
    const uint16_t *p = (const uint16_t *)data;
    uint32_t sum = 0;
    while (len > 1) {
        sum += *p++;
        len -= 2;
    }
    if (len == 1) {
        sum += *(const uint8_t *)p;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

uint16_t tcp_checksum(const struct ipv4_hdr *ip, const struct tcp_hdr *tcp, uint16_t tcp_len) {
    uint32_t sum = 0;
    const uint16_t *saddr = (const uint16_t *)ip->saddr;
    const uint16_t *daddr = (const uint16_t *)ip->daddr;
    sum += saddr[0] + saddr[1];
    sum += daddr[0] + daddr[1];
    sum += htons(6); /* Protocolo TCP = 6 */
    sum += htons(tcp_len);

    const uint16_t *p = (const uint16_t *)tcp;
    size_t len = tcp_len;
    while (len > 1) {
        sum += *p++;
        len -= 2;
    }
    if (len == 1) {
        sum += *(const uint8_t *)p;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

/* Respuesta HTTP pre-construida en memoria estática */
static const char http_payload[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Content-Length: 175\r\n\r\n"
    "<html><body style='font-family:sans-serif;background:#111;color:#0f0;padding:2rem'>"
    "<h1>Unikernel ARM64 Bare-Metal</h1>"
    "<p>Servido directo desde Ring 0 sin SO.</p></body></html>";

static uint8_t reply_frame[1514];

void send_tcp_packet(const struct eth_hdr *in_eth, const struct ipv4_hdr *in_ip,
                     const struct tcp_hdr *in_tcp, uint8_t flags,
                     uint32_t seq, uint32_t ack, const void *payload, uint16_t payload_len) {
    struct eth_hdr *eth = (struct eth_hdr *)reply_frame;
    memcpy(eth->dst, in_eth->src, 6);
    memcpy(eth->src, my_mac, 6);
    eth->type = htons(0x0800);

    struct ipv4_hdr *ip = (struct ipv4_hdr *)(reply_frame + sizeof(struct eth_hdr));
    uint16_t tcp_total_len = sizeof(struct tcp_hdr) + payload_len;
    uint16_t ip_total_len = sizeof(struct ipv4_hdr) + tcp_total_len;

    ip->ver_ihl = 0x45;
    ip->tos = 0;
    ip->tot_len = htons(ip_total_len);
    ip->id = htons(0x1234);
    ip->frag_off = 0;
    ip->ttl = 64;
    ip->proto = 6;
    ip->check = 0;
    memcpy(ip->saddr, my_ip, 4);
    memcpy(ip->daddr, in_ip->saddr, 4);
    ip->check = checksum(ip, sizeof(struct ipv4_hdr));

    struct tcp_hdr *tcp = (struct tcp_hdr *)((uint8_t *)ip + sizeof(struct ipv4_hdr));
    tcp->source = in_tcp->dest;
    tcp->dest = in_tcp->source;
    tcp->seq = htonl(seq);
    tcp->ack_seq = htonl(ack);
    tcp->doff_res = (5 << 4);
    tcp->flags = flags;
    tcp->window = htons(65535);
    tcp->check = 0;
    tcp->urg_ptr = 0;

    if (payload_len > 0) {
        memcpy((uint8_t *)tcp + sizeof(struct tcp_hdr), payload, payload_len);
    }

    tcp->check = tcp_checksum(ip, tcp, tcp_total_len);
    virtio_net_send(reply_frame, sizeof(struct eth_hdr) + ip_total_len);
}

/* =========================================================================
 * 4b. CLIENTE HTTP SALIENTE: GET /remote -> http://10.0.2.2:9000/
 * 10.0.2.2 es el Mac (SLIRP). Sin DNS: el destino se da por IP.
 * El resultado se devuelve al cliente que hizo GET /remote.
 * ========================================================================= */
#define OUT_LOCAL_BASE     40000   /* puerto local = base + nº de conexión */
#define UPSTREAM_PORT      9000
#define OUT_ISN_BASE       5000    /* ISN = base + nº de conexión * 1000 */
#define OUT_CHUNK          1200
#define OUT_TIMEOUT_TICKS  50000000u

static const uint8_t gw_ip[4] = {10, 0, 2, 2};
static const char upstream_req[] =
    "GET / HTTP/1.1\r\n"
    "Host: 10.0.2.2:9000\r\n"
    "Connection: close\r\n\r\n";

enum { OUT_IDLE, OUT_ARP, OUT_SYN, OUT_WAIT_DATA };
static int out_state = OUT_IDLE;
static uint8_t gw_mac[6];
static int gw_mac_known = 0;
static uint32_t out_seq = 0;      /* siguiente número de secuencia que enviamos */
static uint32_t out_expect = 0;   /* siguiente secuencia que esperamos del upstream */
static uint32_t out_ticks = 0;
static uint8_t cli_mac[6];        /* cliente original que espera la respuesta */
static uint8_t cli_ip[4];
static uint16_t cli_port = 0;
static uint32_t cli_ack = 0;
static uint8_t up_buf[4096];
static uint32_t up_len = 0;
/* Cada conexión saliente usa puerto e ISN nuevos: SLIRP conserva el estado de la anterior */
static uint16_t out_lport = OUT_LOCAL_BASE;
static uint32_t out_isn = OUT_ISN_BASE;
static uint32_t out_conn = 0;

void uart_putdec(uint32_t v) {
    char b[12];
    int i = 0;
    do {
        b[i++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (i) uart_putc(b[--i]);
}

/* Construye y envía un segmento TCP/IP genérico (origen = my_ip) */
static void tcp_tx(const uint8_t dst_mac[6], const uint8_t dst_ip[4],
                   uint16_t sport, uint16_t dport, uint32_t seq, uint32_t ack,
                   uint8_t flags, const void *payload, uint16_t plen) {
    struct eth_hdr *eth = (struct eth_hdr *)reply_frame;
    memcpy(eth->dst, dst_mac, 6);
    memcpy(eth->src, my_mac, 6);
    eth->type = htons(0x0800);

    struct ipv4_hdr *ip = (struct ipv4_hdr *)(reply_frame + sizeof(struct eth_hdr));
    uint16_t tcp_total_len = sizeof(struct tcp_hdr) + plen;
    uint16_t ip_total_len = sizeof(struct ipv4_hdr) + tcp_total_len;
    ip->ver_ihl = 0x45;
    ip->tos = 0;
    ip->tot_len = htons(ip_total_len);
    ip->id = htons(0x4321);
    ip->frag_off = 0;
    ip->ttl = 64;
    ip->proto = 6;
    ip->check = 0;
    memcpy(ip->saddr, my_ip, 4);
    memcpy(ip->daddr, dst_ip, 4);
    ip->check = checksum(ip, sizeof(struct ipv4_hdr));

    struct tcp_hdr *tcp = (struct tcp_hdr *)((uint8_t *)ip + sizeof(struct ipv4_hdr));
    tcp->source = htons(sport);
    tcp->dest = htons(dport);
    tcp->seq = htonl(seq);
    tcp->ack_seq = htonl(ack);
    tcp->doff_res = (5 << 4);
    tcp->flags = flags;
    tcp->window = htons(65535);
    tcp->check = 0;
    tcp->urg_ptr = 0;
    if (plen > 0) {
        memcpy((uint8_t *)tcp + sizeof(struct tcp_hdr), payload, plen);
    }
    tcp->check = tcp_checksum(ip, tcp, tcp_total_len);
    virtio_net_send(reply_frame, sizeof(struct eth_hdr) + ip_total_len);
}

/* Envía la respuesta al cliente original en trozos de OUT_CHUNK; el último lleva FIN */
static void send_client_response(const uint8_t *body, uint32_t len) {
    uint32_t off = 0;
    do {
        uint32_t n = len - off;
        if (n > OUT_CHUNK) n = OUT_CHUNK;
        uint8_t fl = TCP_PSH | TCP_ACK;
        if (off + n >= len) fl |= TCP_FIN;
        tcp_tx(cli_mac, cli_ip, 18080, cli_port, 1001 + off, cli_ack,
               fl, body + off, (uint16_t)n);
        off += n;
    } while (off < len);
}

static void send_arp_request(void) {
    struct eth_hdr *eth = (struct eth_hdr *)reply_frame;
    memset(eth->dst, 0xFF, 6);
    memcpy(eth->src, my_mac, 6);
    eth->type = htons(0x0806);

    struct arp_pkt *arp = (struct arp_pkt *)(reply_frame + sizeof(struct eth_hdr));
    arp->htype = htons(1);
    arp->ptype = htons(0x0800);
    arp->hlen = 6;
    arp->plen = 4;
    arp->oper = htons(1); /* ARP Request */
    memcpy(arp->sha, my_mac, 6);
    memcpy(arp->spa, my_ip, 4);
    memset(arp->tha, 0, 6);
    memcpy(arp->tpa, gw_ip, 4);
    virtio_net_send(reply_frame, sizeof(struct eth_hdr) + sizeof(struct arp_pkt));
}

static void outbound_send_syn(void) {
    tcp_tx(gw_mac, gw_ip, out_lport, UPSTREAM_PORT, out_isn, 0, TCP_SYN, NULL, 0);
    out_seq = out_isn + 1;
    out_state = OUT_SYN;
}

static void outbound_start(void) {
    out_ticks = 0;
    out_conn++;
    out_lport = (uint16_t)(OUT_LOCAL_BASE + (out_conn % 1000));
    out_isn = OUT_ISN_BASE + out_conn * 1000;
    uart_puts("[OUT] Conectando a http://10.0.2.2:9000/\r\n");
    if (gw_mac_known) {
        outbound_send_syn();
    } else {
        send_arp_request();
        out_state = OUT_ARP;
    }
}

static void outbound_finish(void) {
    tcp_tx(gw_mac, gw_ip, out_lport, UPSTREAM_PORT, out_seq, out_expect,
           TCP_FIN | TCP_ACK, NULL, 0);
    out_seq++;
    out_state = OUT_IDLE;
    uart_puts("[OUT] Respuesta recibida: ");
    uart_putdec(up_len);
    uart_puts(" bytes -> reenviando al cliente\r\n");
    send_client_response(up_buf, up_len);
}

/* Procesa un segmento TCP recibido del upstream (puerto local out_lport) */
static void outbound_rx(const struct tcp_hdr *tcp, const uint8_t *data, uint32_t dlen) {
    uint32_t seq = ntohl(tcp->seq);
    uint8_t flags = tcp->flags;
    out_ticks = 0;

    if (out_state == OUT_SYN) {
        if ((flags & (TCP_SYN | TCP_ACK)) == (TCP_SYN | TCP_ACK) &&
            ntohl(tcp->ack_seq) == out_isn + 1) {
            out_expect = seq + 1;
            /* ACK del SYN-ACK y petición HTTP en el mismo segmento */
            tcp_tx(gw_mac, gw_ip, out_lport, UPSTREAM_PORT, out_seq, out_expect,
                   TCP_PSH | TCP_ACK, upstream_req, sizeof(upstream_req) - 1);
            out_seq += sizeof(upstream_req) - 1;
            up_len = 0;
            out_state = OUT_WAIT_DATA;
        }
        return;
    }

    if (out_state != OUT_WAIT_DATA) return;

    /* Segmento con datos fuera de orden o duplicado: confirmar lo que tenemos */
    if (dlen > 0 && seq != out_expect) {
        tcp_tx(gw_mac, gw_ip, out_lport, UPSTREAM_PORT, out_seq, out_expect,
               TCP_ACK, NULL, 0);
        return;
    }

    if (dlen > 0) {
        uint32_t room = sizeof(up_buf) - up_len;
        uint32_t n = dlen < room ? dlen : room;
        memcpy(up_buf + up_len, data, n);
        up_len += n;
        out_expect += dlen;
    }

    int fin = (flags & TCP_FIN) && (seq + dlen == out_expect);
    if (fin) out_expect++;
    if (dlen > 0 || (flags & TCP_FIN)) {
        tcp_tx(gw_mac, gw_ip, out_lport, UPSTREAM_PORT, out_seq, out_expect,
               TCP_ACK, NULL, 0);
    }
    if (fin) outbound_finish();
}

void handle_packet(uint8_t *pkt, uint32_t len) {
    if (len < sizeof(struct eth_hdr)) return;
    struct eth_hdr *eth = (struct eth_hdr *)pkt;
    uint16_t eth_type = ntohs(eth->type);

    /* 1. Responder ARP (¿Quién tiene la IP 10.0.2.15?) */
    if (eth_type == 0x0806 && len >= sizeof(struct eth_hdr) + sizeof(struct arp_pkt)) {
        struct arp_pkt *arp = (struct arp_pkt *)(pkt + sizeof(struct eth_hdr));
        /* ARP Reply de la puerta de enlace: guardamos su MAC */
        if (ntohs(arp->oper) == 2 && memcmp(arp->spa, gw_ip, 4) == 0) {
            memcpy(gw_mac, arp->sha, 6);
            gw_mac_known = 1;
            uart_puts("[OUT] Gateway 10.0.2.2 resuelto por ARP\r\n");
            if (out_state == OUT_ARP) outbound_send_syn();
            return;
        }
        if (ntohs(arp->oper) == 1 && memcmp(arp->tpa, my_ip, 4) == 0) {
            struct eth_hdr *reth = (struct eth_hdr *)reply_frame;
            memcpy(reth->dst, eth->src, 6);
            memcpy(reth->src, my_mac, 6);
            reth->type = htons(0x0806);

            struct arp_pkt *rarp = (struct arp_pkt *)(reply_frame + sizeof(struct eth_hdr));
            rarp->htype = htons(1);
            rarp->ptype = htons(0x0800);
            rarp->hlen = 6;
            rarp->plen = 4;
            rarp->oper = htons(2); /* ARP Reply */
            memcpy(rarp->sha, my_mac, 6);
            memcpy(rarp->spa, my_ip, 4);
            memcpy(rarp->tha, arp->sha, 6);
            memcpy(rarp->tpa, arp->spa, 4);

            virtio_net_send(reply_frame, sizeof(struct eth_hdr) + sizeof(struct arp_pkt));
        }
        return;
    }

    /* 2. Procesar IPv4 + TCP en puerto 18080 */
    if (eth_type == 0x0800 && len >= sizeof(struct eth_hdr) + sizeof(struct ipv4_hdr)) {
        struct ipv4_hdr *ip = (struct ipv4_hdr *)(pkt + sizeof(struct eth_hdr));
        if (ip->proto != 6 || memcmp(ip->daddr, my_ip, 4) != 0) return;

        uint8_t ihl = (ip->ver_ihl & 0x0F) * 4;
        struct tcp_hdr *tcp = (struct tcp_hdr *)((uint8_t *)ip + ihl);
        /* Respuestas del upstream (10.0.2.2:9000) hacia nuestro puerto local */
        if (ntohs(tcp->dest) == out_lport && out_state != OUT_IDLE &&
            memcmp(ip->saddr, gw_ip, 4) == 0) {
            uint8_t ohl = ((tcp->doff_res >> 4) & 0x0F) * 4;
            uint16_t odl = ntohs(ip->tot_len) - ihl - ohl;
            outbound_rx(tcp, (uint8_t *)tcp + ohl, odl);
            return;
        }
        if (ntohs(tcp->dest) != 18080) return;

        uint8_t tcp_hdr_len = ((tcp->doff_res >> 4) & 0x0F) * 4;
        uint16_t ip_tot_len = ntohs(ip->tot_len);
        uint16_t tcp_data_len = ip_tot_len - ihl - tcp_hdr_len;
        uint32_t client_seq = ntohl(tcp->seq);

        /* Handshake TCP: Cliente envía SYN -> Respondemos SYN+ACK */
        if (tcp->flags & TCP_SYN) {
            send_tcp_packet(eth, ip, tcp, TCP_SYN | TCP_ACK, 1000, client_seq + 1, NULL, 0);
        }
        /* Cliente envía petición HTTP (PSH/ACK con datos) -> Respondemos HTTP 200 + FIN */
        /* GET /remote: el unikernel consulta 10.0.2.2:9000 y devuelve su respuesta */
        else if (tcp_data_len >= 12 && out_state == OUT_IDLE &&
                 memcmp((uint8_t *)tcp + tcp_hdr_len, "GET /remote ", 12) == 0) {
            uart_puts("[HTTP] GET /remote -> consultando upstream\r\n");
            memcpy(cli_mac, eth->src, 6);
            memcpy(cli_ip, ip->saddr, 4);
            cli_port = ntohs(tcp->source);
            cli_ack = client_seq + tcp_data_len;
            /* ACK a la petición del cliente; la respuesta llega cuando termine el upstream */
            tcp_tx(cli_mac, cli_ip, 18080, cli_port, 1001, cli_ack, TCP_ACK, NULL, 0);
            outbound_start();
        }
        else if (tcp_data_len > 0) {
            uart_puts("[HTTP] Peticion recibida -> Enviando respuesta 200 OK\r\n");
            uint16_t resp_len = sizeof(http_payload) - 1;
            send_tcp_packet(eth, ip, tcp, TCP_PSH | TCP_ACK | TCP_FIN,
                            1001, client_seq + tcp_data_len, http_payload, resp_len);
        }
        /* Cliente cierra conexión con FIN -> Respondemos ACK */
        else if (tcp->flags & TCP_FIN) {
            send_tcp_packet(eth, ip, tcp, TCP_ACK, 1001 + sizeof(http_payload) - 1, client_seq + 1, NULL, 0);
        }
    }
}

/* =========================================================================
 * 5. PUNTO DE ENTRADA DEL UNIKERNEL
 * ========================================================================= */
void kernel_main(void) {
    uart_puts("\r\n==================================================\r\n");
    uart_puts("  UNIKERNEL ARM64 HTTP SERVER (BARE-METAL / MMIO) \r\n");
    uart_puts("==================================================\r\n");

    if (virtio_net_init() != 0) {
        while (1) asm volatile("wfe");
    }

    uart_puts("[HTTP] Servidor escuchando en http://localhost:18080\r\n");
    uart_puts("[INFO] Modo Poll-Mode activo (0 interrupciones).\r\n\r\n");

    /* Bucle infinito de sondeo (Polling extremo sin interrupciones) */
    while (1) {
        asm volatile("dmb sy" ::: "memory");
        uint16_t used_idx = *(volatile uint16_t *)&rx_vq.used.idx;

        if (last_rx_used_idx != used_idx) {
            uint16_t ring_slot = last_rx_used_idx % QUEUE_SIZE;
            uint32_t desc_id = rx_vq.used.ring[ring_slot].id;
            uint32_t total_len = rx_vq.used.ring[ring_slot].len;

            if (total_len > sizeof(struct virtio_net_hdr)) {
                uint8_t *eth_pkt = rx_buffers[desc_id] + sizeof(struct virtio_net_hdr);
                uint32_t eth_len = total_len - sizeof(struct virtio_net_hdr);
                handle_packet(eth_pkt, eth_len);
            }

            /* Devolver el buffer al anillo de recepción */
            rx_vq.avail.ring[rx_vq.avail.idx % QUEUE_SIZE] = desc_id;
            asm volatile("dmb sy" ::: "memory");
            rx_vq.avail.idx++;
            asm volatile("dmb sy" ::: "memory");
            mmio_write32(net_base + VIRTIO_REG_QUEUE_NOTIFY, 0);

            last_rx_used_idx++;
        } else if (out_state != OUT_IDLE && ++out_ticks > OUT_TIMEOUT_TICKS) {
            /* Sin respuesta del upstream: devolver 504 al cliente */
            uart_puts("[OUT] Timeout esperando al upstream -> 504\r\n");
            out_state = OUT_IDLE;
            static const char t504[] =
                "HTTP/1.1 504 Gateway Timeout\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
            send_client_response((const uint8_t *)t504, sizeof(t504) - 1);
            out_ticks = 0;
        }
    }
}

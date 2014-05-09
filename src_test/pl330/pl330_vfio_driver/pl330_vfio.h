#ifndef PL330_VFIO_H
#define PL330_VFIO_H

#include <stdbool.h>
#include <glib.h>
#include <pthread.h>
#include <linux/types.h>
#include <sys/select.h>

/*
 * Register offset
 * */
#define DSR			0x000
#define DSR_STATUS_SHIFT	0
#define DSR_STATUS_MASK		0x00F
#define DPC			0x004
#define INTEN			0x020
#define INT_EVENT_RIS		0x024
#define INTMIS			0x028
#define INTCLR			0x02C
#define CSR_BASE		0x100
#define CSR(n)			(CSR_BASE + (n)*0x8) // n = 0:7
#define CSR_CHANNEL_STATUS_SH	0
#define CSR_CHANNEL_STATUS_MK	0x00F
#define CPC_BASE		0x104
#define CPC(n)			(CPC_BASE + (n)*0x8) // n = 0:7
#define SAR_BASE		0x400
#define SAR(n)			(SAR_BASE + (n)*0x8) // n = 0:7
#define DAR_BASE		0x404
#define DAR(n)			(DAR_BASE + (n)*0x8) // n = 0:7
#define CCR_BASE		0x408
#define CCR(n)			(CCR_BASE + (n)*0x8) // n = 0:7
#define LC0_BASE		0x40C
#define LC0(n)			(LC0_BASE + (n)*0x8) // n = 0:7
#define LC1_BASE		0x410
#define LC1(n)			(LC1_BASE + (n)*0x8) // n = 0:7
#define CR_BASE			0xE00
#define CR(n)			(CR_BASE  + (n)*0x4) // n = 0:4
#define CR0_PERIF_REQ_SUPP	(1 << 0)
#define CR0_NUM_CHANNELS_SH	4
#define CR0_NUM_CHANNELS_MK	0x007
#define CR0_NUM_PERIF_REQ_SH	12
#define CR0_NUM_PERIF_REQ_MK	0x01F
#define CR0_NUM_EVENT_SHIFT	17
#define CR0_NUM_EVENT_MASK	0x01F
#define CRD			0xE14
#define CRD_BUS_WIDTH_SHIFT	0
#define CRD_BUS_WIDTH_MASK	0x007
#define CRD_BUF_DEPTH_SHIFT	20
#define CRD_BUF_DEPTH_MASK	0x3FF
#define DBGSTATUS		0xD00
#define DBG_BUSY_MASK		(1 << 0)
#define DBGCMD			0xD04
#define DBGINST0		0xD08
#define DBGINST1		0xD0C

#define	shift_and_mask(a, x, y)	(((a) >> (x)) & (y))

/*
 * Available states encoding
 * */
#define STOPPED			0x000
#define EXECUTING		0x001
#define CACHE_MISS		0x002
#define UPDATING_PC		0x003
#define WAIT_EVENT		0x004
#define BARRIER			0x005
#define WAIT_PERIPH		0x007
#define KILLING			0x008
#define COMPLETING		0x009
#define FAULT_COMPLETING	0x00E
#define FAULTING		0x00F

#define INVALID_STATE		0x010

/*
 * Commands encoding
 */

/*
 * DMAMOV
 */
#define DMAMOV			0x0BC
#define _SAR			0x0 // 0b000
#define _CCR			0x1 // 0b001
#define _DAR			0x2 // 0b010
#define DMAMOV_SIZE		6 // command + type + address = 1 + 1 + 4

/*
 * DMAEND
 */
#define DMAEND			0x000
#define DMAEND_SIZE		1

/*
 * DMALP
 */
#define DMALP			0x020
#define DMALP_SIZE		2

/*
 * DMALPEND
 */
#define DMALPEND		0x028
#define DMALPEND_SIZE		2

/*
 * DMALD
 * */
#define DMALD			0x004
#define DMALD_SIZE		1

/*
 * DMAST
 * */
#define DMAST			0x008
#define DMAST_SIZE		1

/*
 * DMARMB
 * */
#define DMARMB			0x012
#define DMARMB_SIZE		1

/*
 * DMAWMB
 * */
#define DMAWMB			0x013
#define DMAWMB_SIZE		1

/*
 * DMAGO
 */
#define DMAGO			0x0A0
#define DMAGO_SIZE		6

/*
 * DMASEV
 * */
#define DMASEV			0x034
#define DMASEV_SIZE		2

/*
 * DMAKILL
 * */
#define DMAKILL			0x001
#define DMAKILL_SIZE		1

/*
 * Channel Control Register - CCR
 */
#define DST_SHIFT		14

#define CCR_SRCINC_SHIFT	0		// source control
#define CCR_SRCBURSTSIZE_SHIFT	1
#define CCR_SRCBURSTLEN_SHIFT	4
#define CCR_SRCPROTCTRL_SHIFT	8
#define CCR_SRCCACHECTRL_SHIFT	11

#define CCR_DSTINC_SHIFT	0  + DST_SHIFT	// destination control
#define CCR_DSTBURSTSIZE_SHIFT	1  + DST_SHIFT
#define CCR_DSTBURSTLEN_SHIFT	4  + DST_SHIFT
#define CCR_DSTPROTCTRL_SHIFT	8  + DST_SHIFT
#define CCR_DSTCACHECTRL_SHIFT	11 + DST_SHIFT

#define CCR_ENDIANSWAPSZ_SHIFT	28

// default values
#define INC_DEF_VAL		1
#define CCR_PROTCTRL_DEF_VAL	(0 << 0) | (1 << 1) | (0 << 2)
#define CCR_BURSTLEN_DEF_VAL	1
#define CCR_BURSTSIZE_DEF_VAL	2
#define CCR_CACHECTRL_DEF_VAL	0

// CCR limit val
#define CCR_BURSTSIZE_MAX	16 // bytes
#define CCR_BURSTLEN_MAX	16 // data transfers

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_MSG(fmt, ...)				\
	do {						\
		printf("debug: "fmt, ##__VA_ARGS__);	\
	} while(0)
#else
#define DEBUG_MSG(fmt, ...) do {} while(0)
#endif

#define NUM_OF_BURST(tot, b_len, b_size)	((tot) / (b_len) / (b_size))

/*
 * IDs
 *
 * The channels IDs are 0,1,...,status->channels
 * Since there could be 8 channels maximum, the manager
 * thread ID is 8
 * */
#define MANAGER_ID		8

typedef __u8 uchar;
typedef __u32 uint;
typedef __u64 u64;

enum DMAMOV_type {
	SAR = 0,
	CCR,
	DAR,
};

enum DMA_LOOP_REGISTER {
	LOOP_CNT_0_REG = 0,
	LOOP_CNT_1_REG = 1,
};

enum LOOP_START_TYPE {
	BY_DMALP = 0,
	BY_DMALPFE,
};

enum request_type {
	SINGLE,
	BURST,
	ALWAYS,
};

enum transfer_type {
	MEM2MEM,
	MEM2DEV,
	DEV2MEM,
};

struct controller_config {
	bool non_secure_mode;
};

/*
 * move to another file
 * CCR configuration
 * */
enum dst_src {
	DST,
	SRC,
};

struct req_config_ops {
	int (*set_inc)(uint val, enum dst_src type, uint *reg);
	int (*set_burst_size)(uint val, enum dst_src type, uint *reg); // in byte, power of 2, <= 16
	int (*set_burst_length)(uint val, enum dst_src type, uint *reg); // # of data transfer 1:16
	int (*set_prot_control)(uint val, enum dst_src type, uint *reg);
};

struct req_config {
	// source and destination
	__u64 iova_src;
	__u64 iova_dst;

	// bytes to transfer
	int size;

	// type of the transfer (mem to mem, mem to dev, dev to mem)
	enum transfer_type t_type;

	/*
	 * channel configuration
	 */
	// increment value
	unsigned int src_inc;
	unsigned int dst_inc;

	// burst size
	unsigned int src_burst_size;
	unsigned int dst_burst_size;

	// burst length
	unsigned int src_burst_len;
	unsigned int dst_burst_len;

	// channel to which submit the request
	unsigned int chan_id;

	// prot control value, for now we set directly the value
	/*
	bool nonsecure;
	bool privileged;
	bool insnaccess;
	*/
	unsigned int src_prot_ctrl;
	unsigned int dst_prot_ctrl;

	// cache control value
	unsigned int src_cache_ctrl;
	unsigned int dst_cache_ctrl;

	// arise an interrupt when the transfer is completed
	bool int_fin;

	// callback to be called when the request has been served
	void (*callback)(void *user_data);
	void *user_data;

	struct req_config_ops config_ops;
};

enum channel_thread_state {
	FREE,
	ALLOCATED,
};

struct channel_thread {
	enum channel_thread_state state;
	/*
	 * event to fire when the transfer completes
	 * */
	int event_id;

	// callback when finished
	void (* callback)(void *user_data);
	void *user_data;
};

/*
 * init the controller
 * */
void pl330_vfio_init(uchar *base_regs);

/*
 * fill config with default value for a mem2mem transfer:
 * It will set:
 * 	config_ops
 * 	src_inc, dst_inc at def val
 * 	src_cache_ctrl, dst_cache_ctrl at def val
 * 	src_burst_size, dst_burst_size at max
 * 	src_burst_len, dst_burst_len at max
 *	t_type = MEM2MEM
 *
 *	Remember that iova_src, iova_dst and size
 *	are still to be set
 * */
int pl330_vfio_mem2mem_defconfig(struct req_config *config);

/*
 * fill the buffer with the instructions needed to realize
 * the transfer configured by config
 * */
int generate_cmds_from_request(uchar *cmds_buf, struct req_config *config);

/*
 * TODO describe
 * */
int pl330_vfio_request_channel();
void pl330_vfio_release_channel(uint id);

/*
 * handy function to test mem to mem transactions
 *
 * cmds has to point to a memory area accessible by the device and
 * wide enough to store all the commands generated by the controller.
 * For such a transaction only, 1KB is more than enough.
 *
 * */
int pl330_vfio_mem2mem_int(uchar *cmds, u64 iova_cmds, u64 iova_src, u64 iova_dst);

/*
 * Tell to the controller where the instructions are
 * and instruct it to go
 * */
int pl330_vfio_submit_req(uchar *cmds, u64 iova_cmds, struct req_config *conf);

void pl330_vfio_start_irq_handler();
int pl330_vfio_add_irq(int eventfd_irq, int vfio_irq_index);

/*
 * Clear interrupt number num
 * */
void pl330_vfio_clear_irq(int irq_num);

/*
 * For every available channel, check if it's in stopped state; if it's not,
 * force it to move into it. Stop also the manager thread.
 * */
void pl330_vfio_reset();

/*
 * Unload driver
 * */
void pl330_vfio_remove();
#endif

#include "pl330_vfio.h"

#include <linux/types.h>
#include <errno.h>
#include <linux/vfio.h>
#include <poll.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/eventfd.h>
#include <sys/fcntl.h>
#include <sys/select.h>

struct pl330_status {
	uint channels; // # of channels available
	struct channel_thread *ch_threads;

	/*
	 * the controller supports 32 events.
	 * The envent i is allocated if
	 * allocated_events[i] == 1
	 * */
	uint allocated_events;

	uchar * regs; // pointer to the first pl330 register
	fd_set set_irq_efd;
	int highest_irq_num;
	pthread_t irq_handler;
	/*
	 * key is eventfd number
	 * value is irqnumber
	 * */
	GHashTable *efdnum_irqnum;
};

struct pl330_status *status = NULL;

static int pl330_set_burst_size(uint val, enum dst_src type, uint *reg)
{
	if(val & (val - 1) || val > CCR_BURSTSIZE_MAX) {
		// error, not power of 2
		return -1;
	}

	uchar ret = 0;
	while(val >>= 1) {
		ret++;
	} // see page 3-27

	switch(type){
	case SRC:
		*reg |= ret << CCR_SRCBURSTSIZE_SHIFT;
	case DST:
		*reg |= ret << CCR_DSTBURSTSIZE_SHIFT;
	default:
		return -1;
	}

	return 0;
}

static int pl330_set_burst_length(uint val, enum dst_src type, uint *reg)
{
	if(!val || val > CCR_BURSTLEN_MAX) {
		return -1;
	}

	switch(type) {
	case SRC:
		*reg |= (val - 1) << CCR_SRCBURSTLEN_SHIFT;
	case DST:
		*reg |= (val - 1) << CCR_DSTBURSTLEN_SHIFT;
	default:
		return -1;
	}

	return 0;
}

static void pl330_vfio_req_config_init(struct req_config *config)
{
	config->config_ops.set_burst_size = pl330_set_burst_size;
	config->config_ops.set_burst_length = pl330_set_burst_length;
}

struct CR0_conf {
	bool perif_req_support;
	uint num_channels;
	uint num_perif_req;
	uint num_events;
};

static void CR0_read_conf(struct CR0_conf *conf)
{
	uint cr0_reg;

	cr0_reg = *((uint *)(status->regs + CR(0)));

	conf->perif_req_support = (cr0_reg & CR0_PERIF_REQ_SUPP) ? true : false;

	conf->num_channels =  shift_and_mask(cr0_reg,
			CR0_NUM_CHANNELS_SH, CR0_NUM_CHANNELS_MK) + 1;

	conf->num_perif_req = shift_and_mask(cr0_reg,
			CR0_NUM_PERIF_REQ_SH, CR0_NUM_PERIF_REQ_MK) + 1;

	conf->num_events = shift_and_mask(cr0_reg,
			CR0_NUM_EVENT_SHIFT, CR0_NUM_EVENT_MASK) + 1;
}

struct CRD_conf {
	uint bus_width;
	uint buf_depth;
};

static void CRD_read_conf(struct CRD_conf *conf)
{
	uint crd_reg, tmp;

	crd_reg = *((uint *)(status->regs + CRD));

	tmp = shift_and_mask(crd_reg,
			CRD_BUS_WIDTH_SHIFT, CRD_BUS_WIDTH_MASK);
	conf->bus_width = 8 * (1 << tmp);

	conf->buf_depth = shift_and_mask(crd_reg,
			CRD_BUF_DEPTH_SHIFT, CRD_BUF_DEPTH_MASK) + 1;
}

/*
 * end CCR  configuration
 * */

static inline uchar ith_uchar(void *buf, int pos)
{
	uchar *ptr = buf + pos;

	return *ptr;
}

static inline uint insert_DMAEND(uchar *buffer)
{
	buffer[0] = DMAEND;

	DEBUG_MSG("DMAEND\n");

	return DMAEND_SIZE;
}

static inline uint insert_DMAMOV(uchar *buffer, enum DMAMOV_type type,
		uint dst)
{
	buffer[0] = DMAMOV;
	switch(type) {
		case SAR:
			buffer[1] = _SAR;
			break;
		case CCR:
			buffer[1] = _CCR;
			break;
		case DAR:
			buffer[1] = _DAR;
			break;
		default:
			error(-1, EINVAL, "insert_DMAMOV()");
	}
	uint *ptr = (uint *)&buffer[2];
	*ptr = dst;

	DEBUG_MSG("DMAMOV %x %x\n", buffer[1], *((uint *)&buffer[2]));

	return DMAMOV_SIZE;
}

static inline uint insert_DMALP(uchar *buffer, enum DMA_LOOP_REGISTER type,
		uchar count)
{
	switch(type) {
		case LOOP_CNT_0_REG:
			buffer[0] = DMALP;
			break;
		case LOOP_CNT_1_REG:
			buffer[0] = DMALP | (1 << 1);
			break;
		default:
			error(-1, EINVAL, "insert_DMALP()");
	}

	buffer[1] = count - 1;

	DEBUG_MSG("DMALP LOOP_CNT_%d_REG, count: %d\n", type, count);

	return DMALP_SIZE;
}

struct args_DMALPEND {
	enum request_type type;  // burst, single
	enum DMA_LOOP_REGISTER loop_cnt_num;
	uchar backflip_jump;
};

static inline uint insert_DMALPEND(uchar *buffer, enum LOOP_START_TYPE type,
		struct args_DMALPEND *args)
{
	buffer[0] = DMALPEND;

	switch(type) {
		case BY_DMALP:
			// TODO add "forever" support
			// set by dmalp
			buffer[0] |= 1 << 4;
			// set dma loop register
			buffer[0] |= type << 2;

			switch(args->type) {
			case SINGLE:
				buffer[0] |= (0 << 1) | (1 << 0);
			case BURST:
				buffer[0] |= (1 << 1) | (1 << 0);
			case ALWAYS:
				break;
			default:
				error(-1, EINVAL, "insert_DMALPEND(), type error");
			}

			buffer[1] = args->backflip_jump;
			break;
		case BY_DMALPFE:
			/*TODO*/
			break;
		default:
			error(-1, EINVAL, "insert_DMALPEND()");
	}

	return DMALPEND_SIZE;
}

// TODO SINGLE and BURST cases
static inline uint insert_DMALD(uchar *buf)
{
	buf[0] = DMALD;
	return DMALD_SIZE;
}

// TODO SINGLE and BURST cases
static inline uint insert_DMAST(uchar *buf)
{
	buf[0] = DMAST;
	return DMAST_SIZE;
}

static inline uint insert_DMARMB(uchar *buf)
{
	buf[0] = DMARMB;
	return DMARMB_SIZE;
}

static inline uint insert_DMAWMB(uchar *buf)
{
	buf[0] = DMAWMB;
	return DMAWMB_SIZE;
}

static inline uint insert_DMAGO(uchar *buf, uchar channel,
		uint address, bool nonsecure)
{
	buf[0] = DMAGO;
	buf[0] |= (nonsecure) ? (1 << 1) : (0 << 1);
	buf[1] = channel & 0x7;

	*((uint *)&buf[2]) = address;

	return DMAGO_SIZE;
}

static inline uint insert_DMASEV(uchar *buf, uchar event)
{
	buf[0] = DMASEV;

	event &= 0x1f;
	buf[1] = (event << 3); // see page 4-15

	DEBUG_MSG("DMASEV event: %d\n", event);

	return DMASEV_SIZE;
}

static inline uint insert_DMAKILL(uchar *buf)
{
	buf[0] = DMAKILL;

	return DMAKILL_SIZE;
}

static inline void submit_to_DBGINST(uchar *dbg_instrs, uint thread_id)
{
	uint val;

	val = (dbg_instrs[0] << 16) | (dbg_instrs[1] << 24);
	if(thread_id < MANAGER_ID) {
		val |= (1 << 0);
		val |= (thread_id << 8);
	}

	*((uint *)(status->regs + DBGINST0)) = val;

	*((uint *)(status->regs + DBGINST1)) = *((uint *)&dbg_instrs[2]);

	// GO
	*((uint *)(status->regs + DBGCMD)) = 0;
}

static bool is_dmac_idle()
{
	if (*((uint *)(status->regs + DBGSTATUS)) & DBG_BUSY_MASK) {
		return false;
	} else {
		return true;
	}
}

void pl330_vfio_init(uchar *base_regs)
{
	struct CRD_conf *crd_conf;
	struct CR0_conf *cr0_conf;
	int i;

	crd_conf = malloc(sizeof(struct CRD_conf));
	cr0_conf = malloc(sizeof(struct CR0_conf));
	memset(crd_conf, 0, sizeof(struct CRD_conf));
	memset(cr0_conf, 0, sizeof(struct CR0_conf));

	status = malloc(sizeof(struct pl330_status));

	if(status) {
		memset(status, 0, sizeof(struct pl330_status));
	}
	else {
		error(-1, EINVAL, "error during init");
	}

	status->regs = base_regs;

	FD_ZERO(&status->set_irq_efd);
	status->highest_irq_num = 0;
	status->efdnum_irqnum = g_hash_table_new_full(g_int_hash, g_int_equal,
								free, free);

	status->allocated_events = 0;

	// grab number of channels available
	CRD_read_conf(crd_conf);
	CR0_read_conf(cr0_conf);

	status->channels = cr0_conf->num_channels;
	printf("device init, num channel: %d\n", status->channels);

	status->ch_threads = malloc(status->channels*sizeof(struct channel_thread));
	struct channel_thread free_state = {FREE, -1};
	for(i = 0; i < status->channels; i++) {
		status->ch_threads[i] = free_state;
	}
	// grab AXI master interface bus configuration TODO
	// grab device buffer length TODO
}

/*
 * add new irq to the triggering set.
 * vfio_irq_index is not the irq hw number
 * */
int pl330_vfio_add_irq(int eventfd_irq, int vfio_irq_index)
{
	int *efd_ptr = NULL;
	int *irq_idx_ptr = NULL;

	if(FD_ISSET(eventfd_irq, &(status->set_irq_efd))) {
		return -1;
	}
	FD_SET(eventfd_irq, &status->set_irq_efd);

	efd_ptr = malloc(sizeof(*efd_ptr));
	*efd_ptr = eventfd_irq;
	irq_idx_ptr = malloc(sizeof(*irq_idx_ptr));
	*irq_idx_ptr = vfio_irq_index;

	g_hash_table_insert(status->efdnum_irqnum, efd_ptr,
						irq_idx_ptr);

	if(eventfd_irq > status->highest_irq_num) {
		status->highest_irq_num = eventfd_irq;
	}
}

static inline void pl330_vfio_build_CCR(uint * ccr, struct req_config *config)
{
	config->config_ops.set_burst_size(config->src_burst_size, SRC, ccr);
	config->config_ops.set_burst_length(config->src_burst_len, SRC, ccr);
	config->config_ops.set_burst_size(config->dst_burst_size, DST, ccr);
	config->config_ops.set_burst_length(config->dst_burst_len, DST, ccr);

	*ccr |= (config->src_inc & 0x1);
	*ccr |= ((config->dst_inc & 0x1) << CCR_DSTINC_SHIFT);

	*ccr |= ((config->src_prot_ctrl & 0x7) << CCR_SRCPROTCTRL_SHIFT);
	*ccr |= ((config->dst_prot_ctrl & 0x7) << CCR_DSTPROTCTRL_SHIFT);

	*ccr |= ((config->src_cache_ctrl & 0x7) << CCR_SRCCACHECTRL_SHIFT);
	*ccr |= ((config->dst_cache_ctrl & 0x7) << CCR_DSTCACHECTRL_SHIFT);

}

static void pl330_vfio_dma_map_init(struct vfio_iommu_type1_dma_map *map,
						u64 iova, u64 size)
{
	map->argsz = sizeof(map);
	map->vaddr = (uintptr_t)mmap(NULL, size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
	map->size = size;
	map->iova = iova;
	map->flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
}

static void setup_load_store(uchar *buf, uint *offset, enum transfer_type t_type)
{
	switch(t_type) {
	case MEM2MEM: // TODO handle different design revision
		*offset += insert_DMALD(&buf[*offset]);
		*offset += insert_DMARMB(&buf[*offset]);
		*offset += insert_DMAST(&buf[*offset]);
		*offset += insert_DMAWMB(&buf[*offset]);
		break;
	case MEM2DEV: // TODO
	case DEV2MEM: // TODO
	default:
		error(-1, EINVAL, "setup_load_store");
	}
}

static uint add_inner_outer_loops(uchar *buf, uint *offset, uint in_cnt,
					uint out_cnt, enum transfer_type t_type)
{
	int out_off, in_off = 0;
	struct args_DMALPEND args;

	if(out_cnt > 1) {
		// outer loop : LOOP_CNT_1_REG
		*offset += insert_DMALP(&buf[*offset], LOOP_CNT_1_REG, out_cnt);
		out_off = *offset;
		DEBUG_MSG("        outer_loop_off:%u, cnt: %u\n", out_off, out_cnt);
	}
	// inner loop : LOOP_CNT_0_REG
	*offset += insert_DMALP(&buf[*offset], LOOP_CNT_0_REG, in_cnt);
	in_off = *offset;
	DEBUG_MSG("        inner_loop_off:%u, cnt: %u\n", in_off, in_cnt);

	/*
	 * Load&Store operations
	 * */
	setup_load_store(buf, offset, t_type);

	// insert end of inner loop
	args.type = ALWAYS;
	args.loop_cnt_num = LOOP_CNT_0_REG;
	args.backflip_jump = *offset - in_off;
	*offset += insert_DMALPEND(&buf[*offset], LOOP_CNT_0_REG, &args);
	DEBUG_MSG("        inner_loop_end:%u, backjmp: %d\n", *offset, args.backflip_jump);

	if(out_cnt > 1) {
		args.type = ALWAYS;
		args.loop_cnt_num = LOOP_CNT_1_REG;
		args.backflip_jump = *offset - out_off;
		*offset += insert_DMALPEND(&buf[*offset], BY_DMALP, &args);
		DEBUG_MSG("        outer_loop_end:%u, backjmp: %d\n", *offset, args.backflip_jump);
	}
}

static int setup_req_loops(uchar *buf_cmds, uint *offset, uint burst_size, uint burst_len, uint size,
								enum transfer_type t_type)
{
	/*
	 * full loop means two nested loop of
	 * 256 iterations each
	 * */
	int full_loop_cnt = 0;
	int full_loop_len = 65536; // 256*256

	unsigned long remaining_burst = 0;

	/*
	 * size has to be aligned with the amount of data
	 * moved every burst
	 * */
	if(size % (burst_size * burst_len)) {
		return -1;
	}
	unsigned long burst_count = NUM_OF_BURST(size,
				burst_len, burst_size);

	DEBUG_MSG("set up loops:\n");
	DEBUG_MSG("    burst_cnt:%lu\n", burst_count);

	full_loop_cnt = burst_count / full_loop_len;
	remaining_burst = burst_count % full_loop_len;

	DEBUG_MSG("    full_loop_cnt:%u\n", full_loop_cnt);
	DEBUG_MSG("    remaining_burst:%lu\n", remaining_burst);

	while(full_loop_cnt--) {
		add_inner_outer_loops(buf_cmds, offset, 256, 256, t_type);
	}

	// there could be n < 256*256 bursts left
	// TODO add loop to handle more than one full loop
	if(remaining_burst >= 256) {
		add_inner_outer_loops(buf_cmds, offset, 256, remaining_burst/256, t_type);
		remaining_burst %= 256;
		DEBUG_MSG("    remaining_burst_1:%lu\n", remaining_burst);
	}

	// there could be n < 256 bursts left
	if(remaining_burst) {
		add_inner_outer_loops(buf_cmds, offset, remaining_burst, 0, t_type);
	}

	return 0;
}

/*
 * insert required commands to set up the request.
 * */
int generate_cmds_from_request(uchar *cmds_buf, struct req_config *config)
{
	uint offset = 0;

	uint ccr_conf = 0;
	pl330_vfio_build_CCR(&ccr_conf, config);

	// add instructions to configure CCR, SAR and DAR
	offset += insert_DMAMOV(cmds_buf, CCR, ccr_conf);
	offset += insert_DMAMOV(&cmds_buf[offset], SAR, config->iova_src);
	offset += insert_DMAMOV(&cmds_buf[offset], DAR, config->iova_dst);

	// set up loops, if any TODO handle src and dst burst size/length
	if (setup_req_loops(cmds_buf, &offset, config->src_burst_size,
			config->src_burst_len, config->size, config->t_type)) {
		return -1;
	}

	if(config->int_fin) {
		// see the event enabled in enable_int_for_req()
		offset += insert_DMASEV(&cmds_buf[offset], config->chan_id);
	}

	// terminate transaction
	offset += insert_DMAEND(&cmds_buf[offset]);
}


int pl330_vfio_mem2mem_defconfig(struct req_config *config)
{
	pl330_vfio_req_config_init(config);

	config->src_inc = config->dst_inc = INC_DEF_VAL;
	config->src_prot_ctrl = config->dst_prot_ctrl = CCR_PROTCTRL_DEF_VAL;
	config->src_cache_ctrl = config->dst_cache_ctrl = CCR_CACHECTRL_DEF_VAL;

	config->src_burst_size = config->dst_burst_size = CCR_BURSTSIZE_MAX;
	config->src_burst_len = config->dst_burst_len = CCR_BURSTLEN_MAX;

	config->t_type = MEM2MEM;

	config->callback = NULL;
	config->user_data = NULL;
}

/*
 * For the channel num. i, we activate the event i
 * */
void enable_int_for_req(struct req_config *config)
{
	if(config->int_fin) {
		uint int_reg;
		int_reg = *((uint *)(status->regs + INTEN)) | (1 << config->chan_id);
		(*((uint *)(status->regs + INTEN))) |= int_reg;
	}
}

int pl330_vfio_submit_req(uchar *cmds, u64 iova_cmds, struct req_config *conf)
{
	if(!is_dmac_idle()) {
		return -1;
	}

	// enable interrupt
	enable_int_for_req(conf);

	uchar ins_debug[6] = {0, 0, 0, 0, 0, 0};

	bool non_secure = true;

	insert_DMAGO(ins_debug, conf->chan_id, iova_cmds,
			non_secure);

	if(conf->int_fin) {
		status->ch_threads[conf->chan_id].callback =
					conf->callback;
		status->ch_threads[conf->chan_id].user_data =
					conf->user_data;
	}

	submit_to_DBGINST(ins_debug, MANAGER_ID);

	return 0;
}

int pl330_vfio_mem2mem_int(uchar *cmds, u64 iova_cmds,
					u64 iova_src, u64 iova_dst)
{
	int offset = 0;
	struct req_config config;
	pl330_vfio_req_config_init(&config);

	config.src_inc = config.dst_inc = INC_DEF_VAL;
	config.src_prot_ctrl = config.dst_prot_ctrl = CCR_PROTCTRL_DEF_VAL;
	config.src_cache_ctrl = config.dst_cache_ctrl = CCR_CACHECTRL_DEF_VAL;
	config.t_type = MEM2MEM;

	config.size = 4; // int size
	config.src_burst_size = config.dst_burst_size = 4;
	config.src_burst_len = config.dst_burst_len = 1;

	uint ccr_conf = 0;
	pl330_vfio_build_CCR(&ccr_conf, &config);

	offset += insert_DMAMOV(cmds, CCR, ccr_conf);
	offset += insert_DMAMOV(&cmds[offset], SAR, iova_src);
	offset += insert_DMAMOV(&cmds[offset], DAR, iova_dst);
	offset += insert_DMALP(&cmds[offset], LOOP_CNT_0_REG, 0);
	offset += insert_DMALD(&cmds[offset]);
	offset += insert_DMARMB(&cmds[offset]);
	offset += insert_DMAST(&cmds[offset]);
	offset += insert_DMAWMB(&cmds[offset]);

	struct args_DMALPEND args = {ALWAYS, LOOP_CNT_0_REG, 4};
	offset += insert_DMALPEND(&cmds[offset], BY_DMALP, &args);

	offset += insert_DMAEND(&cmds[offset]);

	if (!is_dmac_idle()) {
		return -1;
	}

	uchar ins_debug[6] = {0, 0, 0, 0, 0, 0};

	uchar channel_id = 0;
	bool non_secure = true;

	insert_DMAGO(ins_debug, channel_id, iova_cmds,
			non_secure);

	submit_to_DBGINST(ins_debug, MANAGER_ID);

	return 0;
}

static void fdset_insert(gpointer key, gpointer val, gpointer nodata)
{
	// the key is the eventfd number
	FD_SET(*((int *)key), &status->set_irq_efd);
}

static void restore_fdset()
{
	FD_ZERO(&status->set_irq_efd);

	g_hash_table_foreach(status->efdnum_irqnum, fdset_insert, NULL);
}

static void handle_trigger_fdset(gpointer key, gpointer val, gpointer mask)
{
	if(FD_ISSET(*(int *)key, &status->set_irq_efd)) {
		// clear irq
		pl330_vfio_clear_irq(*(int *)val);
		// restore eventfd
		eventfd_t eval;
		if(eventfd_read(*(int *)key, &eval)) {
			error(-1, errno, "error while reading from eventfd");
		}
		// trigger callback
		struct channel_thread *ch = &status->ch_threads[*(int *)val];
		if(ch->callback != NULL) {
			ch->callback(ch->user_data);
		}
	}
}

static void *irq_handler_func(void *arg)
{
	int *irq_num;
	int efd_num;
	int irq_triggered_mask = 0;

	while (1) {
		/*
		 * waiting for I/O, in this case for the notification
		 * of an interrupt
		 * */
		efd_num = select(status->highest_irq_num + 1, &status->set_irq_efd,
								NULL, NULL, NULL);
		printf("TRIGGER!\n");

		g_hash_table_foreach(status->efdnum_irqnum, handle_trigger_fdset,
							&irq_triggered_mask);

		restore_fdset();
	}
}

void pl330_vfio_start_irq_handler()
{
	int ret;

	ret = pthread_create(&status->irq_handler, NULL, irq_handler_func, NULL);

	if(ret) {
		error(-1, "unable to create irq thread");
	}
}

void pl330_vfio_clear_irq(int irq_num)
{
	uint *int_reg = (uint *)(status->regs + INTEN);
	uint *cl_irq_reg = (uint *)(status->regs + INTCLR);

	if(*int_reg & (1 << irq_num)) {
		// clear it
		*cl_irq_reg = (1 << irq_num);
	}
}

static uint thread_state(uint id)
{
	uint state_reg, state;
	if(id == MANAGER_ID) {
		state_reg = *((uint *)(status->regs + DSR));
		state = shift_and_mask(state_reg,
				DSR_STATUS_SHIFT, DSR_STATUS_MASK);
		switch(state) {
		case STOPPED:
		case EXECUTING:
		case CACHE_MISS:
		case UPDATING_PC:
		case WAIT_EVENT:
		case FAULTING:
			return state;
		default:
			return INVALID_STATE;
		}
	} else {
		state_reg = *((uint *)(status->regs + CSR(id)));
		state = shift_and_mask(state_reg,
				CSR_CHANNEL_STATUS_SH, CSR_CHANNEL_STATUS_MK);
		switch(state) {
		case STOPPED:
		case EXECUTING:
		case CACHE_MISS:
		case UPDATING_PC:
		case WAIT_EVENT:
		case BARRIER:
		case WAIT_PERIPH:
		case KILLING:
		case COMPLETING:
		case FAULT_COMPLETING:
		case FAULTING:
			return state;
		default:
			return INVALID_STATE;
		}
	}
}

static void stop_thread(uint id)
{
	uchar ins_debug[6] = {0, 0, 0, 0, 0, 0};
	uint int_reg, state;

	if(id > MANAGER_ID) {
		error(-1, "invalid channel id");
	}

	state = thread_state(id);
	if(state == INVALID_STATE) {
		error(-1, "invalid state");
	}

	switch(state) {
		case STOPPED:
		case KILLING:
		case COMPLETING:
			// nothing to do
			return;
		default:
			break;
	}

	// stop interrupt for channel id
	int_reg = *((uint *)(status->regs + INTEN));
	*((uint *)(status->regs + INTEN)) = int_reg
		& ~(1 << status->ch_threads[id].event_id);
	printf("closing event %d for thread %d", status->ch_threads[id].event_id, id);

	insert_DMAKILL(ins_debug);

	submit_to_DBGINST(ins_debug, id);
}

int pl330_vfio_request_channel()
{
	int i;
	int ret = -1;

	for(i = 0; i <= status->channels; i++) {
		if(status->ch_threads[i].state == FREE) {
			ret = i;
			if(status->allocated_events & (1 << i)) {
				// the event is already allocated
				error(-1, "event already allocated");
			}
			status->ch_threads[i].state = ALLOCATED;
			status->ch_threads[i].event_id = i;
			status->allocated_events |= (1 << i);
			break;
		}
	}
	printf("allocated thread %d\n", ret);

	return ret;
}

void pl330_vfio_release_channel(uint id)
{
	status->allocated_events &= ~(1 << id);
	status->ch_threads[id].state = FREE;
	status->ch_threads[id].event_id = -1;
}

void pl330_vfio_reset()
{
	int i;

	// stop the manager
	stop_thread(MANAGER_ID);

	// stop all the channels
	for(i = 0; i < status->channels; i++) {
		stop_thread(i);
	}
}

void pl330_vfio_remove()
{
	// clear all interrups
	uint *int_reg = (uint *)(status->regs + INTCLR);
	*int_reg = 0;

	pthread_cancel(status->irq_handler);

	free(status);
}

#include "pl330_vfio_driver/pl330_vfio.h"

#include "../vfio_utils.h"

static bool completed = false;

void done_callback(void *user_data)
{
	completed = true;

	char *ptr = (char *)user_data;
	/*printf("the message is: %s\n", ptr);*/
}

int main(int argc, char **argv)
{
	int container, ret;
	unsigned int i;
	uchar *base_regs;

	struct vfio_info dev_vfio_info;
	init_vfio_info(&dev_vfio_info);

	struct vfio_dev_spec dev;
	dev.name = argv[2];

	// source memory area the DMA controller will read from
	struct vfio_iommu_type1_dma_map dma_map_src;
	// destination memory area the DMA controller will read to
	struct vfio_iommu_type1_dma_map dma_map_dst;
	/*
	 * memory area where the DMA controller will grub the instructions
	 * to execute. We will tell to the controller how to reach these
	 * instructions through the DEBUG registers.
	 */
	struct vfio_iommu_type1_dma_map dma_map_inst;

	if (argc != 3) {
		printf("Usage: %s /dev/vfio/${group_id} device_id\n", argv[0]);

		goto error;
	}

	/* Create a new container */
	dev_vfio_info.container = open(VFIO_CONTAINER_PATH, O_RDWR);

	if (check_vfio_version(&dev_vfio_info)
		|| check_iommu_extension(&dev_vfio_info)) {
		printf("IOMMU Type1 not supported or unknown API\n");

		goto error;
	}

	/* Open the group */
	dev_vfio_info.group = open(argv[1], O_RDWR);

	if (!is_group_viable(&dev_vfio_info)) {
		printf("the group is not viable\n");

		goto error;
	}

	if (set_group_to_container(&dev_vfio_info)
			|| set_iommu_type(&dev_vfio_info)) {
		printf("something went wrong\n");

		goto error;
	}

	// easy and safer map
	int size_to_map = getpagesize();
	int cmds_len = size_to_map;

	ret = dma_do_map(&dev_vfio_info, &dma_map_src, 0, size_to_map);
	ret |= dma_do_map(&dev_vfio_info, &dma_map_dst, dma_map_src.size,
							size_to_map);
	ret |= dma_do_map(&dev_vfio_info, &dma_map_inst, dma_map_src.size +
					 dma_map_dst.size, cmds_len);

	if(ret) {
		printf("Could not map DMA memory\n");

		goto error;
	}

	/* Get a file descriptor for the device */
	dev.device_fd = ioctl(dev_vfio_info.group, VFIO_GROUP_GET_DEVICE_FD, dev.name);
	if(dev.device_fd < 0) {
		printf("Could not get VFIO device\n");

		goto error;
	}

	get_vfio_device_info(dev.device_fd, &dev.vfio_device_info);
	populate_device_regions(&dev);

	base_regs = (uchar *)mmap(NULL, dev.regions[0].size, PROT_READ | PROT_WRITE, MAP_SHARED,
			dev.device_fd, dev.regions[0].offset);

	if (base_regs != MAP_FAILED)
		printf("register area mapped successfully\n");

	populate_device_irqs(&dev);

	int irqfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
	if (irqfd < 0) {
		printf("error while allocating eventfd\n");

		goto error;
	}

	struct vfio_irq_info *irq = &dev.irqs[0];
	if (vfio_irqfd_init(dev.device_fd, irq->index, irqfd)) {
		printf("error while section IRQ num.%d\n",
					      irq->index);

		goto error;
	}

	// init the controller before adding irq
	pl330_vfio_init(base_regs);
	// add the irq to the pl330 controller
	pl330_vfio_add_irq(irqfd, irq->index);

	int *src_ptr = (int *)((uintptr_t)dma_map_src.vaddr);
	int *dst_ptr = (int *)((uintptr_t)dma_map_dst.vaddr);

	// fill with random data
	int c;
	int tot = dma_map_src.size/sizeof(*src_ptr);
	srand(time(NULL));
	for(c = 0; c < tot; c++) {
		src_ptr[c] = rand();
	}

	// irq handler after setting up irqs
	pl330_vfio_start_irq_handler();

	struct req_config config;
	pl330_vfio_mem2mem_defconfig(&config);

	config.iova_src = dma_map_src.iova;
	config.iova_dst = dma_map_dst.iova;
	config.size	= dma_map_src.size;
	config.int_fin  = true;

	int channel_id;
	channel_id = pl330_vfio_request_channel();
	if(channel_id < 0) {
		printf("fail! No channels available!\n");
		return -1;
	} else {
		printf("channel %d allocated\n", channel_id);
		config.chan_id = channel_id;
	}

	config.callback = done_callback;
	char msg[] = "transfer completed";
	config.user_data = msg;

	generate_cmds_from_request((uchar *)((uintptr_t)dma_map_inst.vaddr), &config);
	pl330_vfio_submit_req((uchar *)((uintptr_t)dma_map_inst.vaddr), dma_map_inst.iova,
								&config);

	/* sleep waiting for the copy being completed */
	sleep(3);

	if (completed) {
		for(c = 0; c < tot; c++) {
			if(src_ptr[c] != dst_ptr[c]) {
				printf("test failed! Source and destination don't match.\n");
				goto error;
			}
		}
	} else {
		printf("Error, the controller hasn't notified yet the completion"
							       "of the copy.\n");
		goto error;
	}

	printf("------------------------------\n");
	printf("| test completed successfully |\n");
	printf("------------------------------\n");

	pl330_vfio_reset();

	vfio_irqfd_clean(dev.device_fd, irq->index);

	close(irqfd);

	pl330_vfio_release_channel(channel_id);

	return 0;

error:
	exit(1);
}


#include "vfio_utils.h"

static int parse_arguments(int argc, const char **arguments,
			    struct vfio_dev_spec *dev, bool *test_irq)
{
	int i, iommu_group;
	char *dev_name, *err, *dev_bus;

	if (argc != 5) {
		printf("Usage: %s device_name iommu_group test_irq bus_name\n",
								arguments[0]);
		printf("	example: %s 2c0a0000.dma 0 1 amba\n",
								arguments[0]);

		goto error;
	}

	iommu_group = strtol(arguments[2], &err, 10);
	if (*err != '\0' || iommu_group < 0) {
		printf("error while parsing iommu group\n");

		goto error;
	}

	*test_irq = strtol(arguments[3], &err, 10);
	if (*err != '\0' || (*test_irq != 0 && *test_irq != 1)) {
		printf("error while parsing irq test flag: %s\n", arguments[3]);

		goto error;
	}

	dev_name = g_strdup(arguments[1]);
	dev_bus  = g_strdup(arguments[4]);

	dev->name = dev_name;
	dev->iommu_group = iommu_group;
	dev->bus = dev_bus;

	return 0;

error:
	return -1;
}

int main(int argc, const char **argv)
{
	int group, i, ret;

	struct vfio_info dev_vfio_info;
	init_vfio_info(&dev_vfio_info);

	struct vfio_iommu_type1_dma_map dma_map;
	struct vfio_dev_spec dev;
	init_vfio_dev_spec(&dev);

	char *chr_group = NULL, *group_addr = NULL;
	char vfio_base[] = VFIO_BASE_PATH;
	bool test_irq;

	if (parse_arguments(argc, argv, &dev, &test_irq)) {
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
	chr_group = g_strdup_printf("%i", dev.iommu_group);
	group_addr = g_strdup_printf("%s%s", vfio_base, chr_group);
	dev_vfio_info.group = open(group_addr, O_RDWR);

	if (!is_group_viable(&dev_vfio_info)) {
		printf("the group is not viable\n");

		goto error;
	}

	if (set_group_to_container(&dev_vfio_info)
			|| set_iommu_type(&dev_vfio_info)) {
		printf("something went wrong\n");

		goto error;
	}

	/* try to map 1MB of memory to the device */
	printf("dma-mapping some memory to the device\n");
	if (dma_do_map(&dev_vfio_info, &dma_map, 0, 1024*1024)) {
		printf("error while dma-mapping\n");
	} else {
		printf("dma-map successful\n");
		dma_do_unmap(&dev_vfio_info, &dma_map);
	}

	/* Get a file descriptor for the device */
	dev.device_fd = ioctl(dev_vfio_info.group, VFIO_GROUP_GET_DEVICE_FD, dev.name);
	if (dev.device_fd < 0) {
		printf("unable to get device fd\n");

		goto error;
	}

	get_vfio_device_info(dev.device_fd, &dev.vfio_device_info);
	populate_device_regions(&dev);

	printf("\nNum regions: %d\n", dev.vfio_device_info.num_regions);

	for (i = 0; i < dev.vfio_device_info.num_regions; i++) {
		struct vfio_region_info *reg = &dev.regions[i];
		uint32_t *mem;

		printf("    region #%d:\n", reg->index);
		printf("        size: %llu\n", reg->size);
		printf("        offset: 0x%llx\n", reg->offset);
		printf("        flags: 0x%llx\n", reg->offset);

		mem = (uint32_t *)mmap(NULL, reg->size, PROT_READ | PROT_WRITE,
					      MAP_SHARED, dev.device_fd, reg->offset);

		if (mem != MAP_FAILED) {
			printf("        Successful MMAP to address %p\n", mem);
		}

		/* test if is an AMBA device and compat string matches */
		if (vfio_is_amba_device(mem, reg->size)
		   ^ !g_strcmp0(dev.bus, AMBA_NAME)) {
			printf(
	"        *** The device seems to be an AMBA device, but it's not ***\n"
	"        ***          attached to an AMBA bus (or viceversa)     ***\n");
		}

		/* unmap */
		if (munmap(mem, reg->size)) {
			printf("error while unmapping region %d\n", i);
		}
	}

	if (test_irq) {
		populate_device_irqs(&dev);

		printf("\nNum irqs: %d\n", dev.vfio_device_info.num_irqs);

		for (i = 0; i < dev.vfio_device_info.num_irqs; i++) {
			unsigned long long int e;
			struct vfio_irq_info *irq = &dev.irqs[i];

			printf("    irq #%d:\n", irq->index);
			printf("        flags: 0x%x\n", irq->flags);
			printf("        count: %d\n", irq->count);

			int irqfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
			if (irqfd < 0) {
				printf("error while allocating eventfd\n");

				goto error;
			}

			if (vfio_irqfd_init(dev.device_fd, irq->index, irqfd)) {
				printf("error while settion IRQ num.%d\n",
							      irq->index);

				goto error;
			}

			ret = read(irqfd, &e, sizeof(e));
			if (ret != -1 || errno != EAGAIN) {
				printf("IRQ %d shouldn't trigger yet.\n", irq->index);

				goto error;
			}

			if (vfio_irqfd_clean(dev.device_fd, irq->index)) {
				printf("error while cleaning IRQ num.%d\n",
								irq->index);

				exit(1);
			}
			close(irqfd);
		}
	}

	g_free(chr_group);
	g_free(group_addr);
	g_free(dev.name);
	g_free(dev.bus);

	free_vfio_dev(&dev);

	return 0;

error:
	exit(1);
}

#include "vfio_utils.h"

int vfio_irqfd_clean(int device, unsigned int index)
{
	struct vfio_irq_set irq_set = {
		.argsz = sizeof(irq_set),
		.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER,
		.index = index,
		.start = 0,
		.count = 0,
	};

	int ret = ioctl(device, VFIO_DEVICE_SET_IRQS, &irq_set);

	if (ret) {
		return 1;
	}

	return 0;
}

int vfio_irqfd_init(int device, unsigned int index, int fd)
{
	struct vfio_irq_set *irq_set;
	int32_t *pfd;
	int ret, argsz;

	argsz = sizeof(*irq_set) + sizeof(*pfd);
	irq_set = malloc(argsz);

	if (!irq_set) {
		printf("Failure in %s allocating memory\n", __func__);

		return 1;
	}

	irq_set->argsz = argsz;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = index;
	irq_set->start = 0;
	irq_set->count = 1;
	pfd = (int32_t *)&irq_set->data;
	*pfd = fd;

	ret = ioctl(device, VFIO_DEVICE_SET_IRQS, irq_set);
	free(irq_set);

	if (ret) {
		return 1;
	}

	return 0;
}

void free_vfio_dev(struct vfio_dev_spec *dev)
{
	if (dev->regions) {
		free(dev->regions);
	}

	if (dev->irqs) {
		free(dev->irqs);
	}
}

bool vfio_is_amba_device(uint32_t *base, uint32_t size)
{
	uint32_t res;
	uint32_t pcell[4];
	int i;

	for (i = 0; i < 4; i++) {
		pcell[i] = base[(size - 0x10 + 4*i) / sizeof(uint32_t)];
		res |= (pcell[i] & 255) << i*8;
	}

	if (res == AMBA_CID) {
		return true;
	}

	return false;
}

void init_vfio_info(struct vfio_info *info)
{
	info->group_status.argsz = sizeof(struct vfio_group_status);
	info->iommu_info.argsz = sizeof(struct vfio_iommu_type1_info);
}

void init_vfio_dev_spec(struct vfio_dev_spec *dev)
{
	dev->regions = NULL;
	dev->irqs = NULL;
}

int is_group_viable(struct vfio_info *info)
{
	ioctl(info->group, VFIO_GROUP_GET_STATUS, &info->group_status);

	return info->group_status.flags & VFIO_GROUP_FLAGS_VIABLE;
}

/* return 1 on fail */
int check_vfio_version(struct vfio_info *info)
{
	return ioctl(info->container, VFIO_GET_API_VERSION) != VFIO_API_VERSION;
}

/* return 1 on fail */
int check_iommu_extension(struct vfio_info *info)
{
	return !ioctl(info->container, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU);
}

/* return 1 on fail */
int set_group_to_container(struct vfio_info *info)
{
	return ioctl(info->group, VFIO_GROUP_SET_CONTAINER, &info->container);
}

/* return 1 on fail */
int set_iommu_type(struct vfio_info *info)
{
	return ioctl(info->container, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
}

int dma_do_map(struct vfio_info *info, struct vfio_iommu_type1_dma_map *map,
						    uint64_t iova, int size)
{
	map->argsz = sizeof(*map);
	map->vaddr = (uintptr_t)mmap(0, size, PROT_READ | PROT_WRITE,
			     MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
	map->iova = iova;
	map->flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
	map->size = size;

	return ioctl(info->container, VFIO_IOMMU_MAP_DMA, map);
}

int dma_do_unmap(struct vfio_info *info, struct vfio_iommu_type1_dma_map *map)
{
	int ret;
	struct vfio_iommu_type1_dma_unmap unmap;

	unmap.argsz = sizeof(struct vfio_iommu_type1_dma_unmap);
	unmap.iova = map->iova;
	unmap.size = map->size;
	unmap.flags = map->flags;

	ret = ioctl(info->container, VFIO_IOMMU_UNMAP_DMA, &unmap);

	return ret;
}

void get_vfio_device_info(int dev_fd, struct vfio_device_info *info)
{
	info->argsz = sizeof(*info);
	ioctl(dev_fd, VFIO_DEVICE_GET_INFO, info);
}

void populate_device_regions(struct vfio_dev_spec *dev)
{
	int i;
	int num_regs = dev->vfio_device_info.num_regions;

	dev->regions = malloc(num_regs * sizeof(struct vfio_region_info));

	for (i = 0; i < num_regs; i++) {
		struct vfio_region_info *reg = &dev->regions[i];

		reg->argsz = sizeof(*reg);
		reg->index = i;

		ioctl(dev->device_fd, VFIO_DEVICE_GET_REGION_INFO, reg);
	}
}

void populate_device_irqs(struct vfio_dev_spec *dev)
{
	int i;
	int num_irqs = dev->vfio_device_info.num_irqs;

	dev->irqs = malloc(num_irqs * sizeof(struct vfio_irq_info));

	for (i = 0; i < num_irqs; i++) {
		struct vfio_irq_info *irq = &dev->irqs[i];

		irq->argsz = sizeof(*irq);
		irq->index = i;

		ioctl(dev->device_fd, VFIO_DEVICE_GET_IRQ_INFO, irq);
	}
}

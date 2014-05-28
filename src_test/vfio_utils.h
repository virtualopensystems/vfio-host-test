#ifndef VFIO_UTILS_PLAT
#define VFIO_UTILS_PLAT

#include <linux/vfio.h>
#include <linux/types.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <glib.h>
#include <errno.h>

#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>

#include <time.h>

#define VFIO_BASE_PATH "/dev/vfio/"
#define VFIO_CONTAINER_PATH  "/dev/vfio/vfio"

#define VFIO_DMA_MAP_FLAG_NOEXEC (1 << 2)

struct vfio_dev_spec {
	char *name;
	char *bus;
	int iommu_group;

	struct vfio_device_info vfio_device_info;
	int device_fd;

	struct vfio_region_info *regions;
	struct vfio_irq_info *irqs;
};

struct vfio_info {
	struct vfio_group_status group_status;
	struct vfio_iommu_type1_info iommu_info;
	struct vfio_dev_spec dev;

	int container;
	int group;
};

int vfio_irqfd_clean(int device, unsigned int index);
int vfio_irqfd_init(int device, unsigned int index, int fd);

void free_vfio_dev(struct vfio_dev_spec *dev);

#define AMBA_CID 0xb105f00d
#define AMBA_NAME "amba"
bool vfio_is_amba_device(uint32_t *base, uint32_t size);

void init_vfio_info(struct vfio_info *info);

void init_vfio_dev_spec(struct vfio_dev_spec *dev);

int is_group_viable(struct vfio_info *info);

/* return 1 on fail */
int check_vfio_version(struct vfio_info *info);

/* return 1 on fail */
int check_iommu_extension(struct vfio_info *info);

/* return 1 on fail */
int set_group_to_container(struct vfio_info *info);

/* return 1 on fail */
int set_iommu_type(struct vfio_info *info);

int dma_do_map(struct vfio_info *info, struct vfio_iommu_type1_dma_map *map,
						    uint64_t iova, int size);

int dma_do_unmap(struct vfio_info *info, struct vfio_iommu_type1_dma_map *map);

void get_vfio_device_info(int dev_fd, struct vfio_device_info *info);

void populate_device_regions(struct vfio_dev_spec *dev);

void populate_device_irqs(struct vfio_dev_spec *dev);

#endif

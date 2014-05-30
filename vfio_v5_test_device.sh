#!/bin/bash

# bind device $1 to VFIO and unbind the current attached driver, if any.
vfio_probe_device() {

    local device=$1
    local bus=$2
    local has_driver=$3

    if [ ! -e /sys/bus/$bus/devices/$device/driver_override ]; then
        return 1;
    else
        echo "vfio-platform" > /sys/bus/$bus/devices/$device/driver_override
    fi

    if [ -d $has_driver ]; then
        echo the device has already a driver, unloading it
        echo $device > /sys/bus/$bus/devices/$device/driver/unbind
    fi
    echo $device > /sys/bus/$bus/drivers_probe

    return $?
}

# unbind device $1
vfio_unbind_device() {

    local device=$1
    local bus=$2

    echo $device > /sys/bus/$bus/drivers/vfio-platform/unbind

    return $?
}

# Get the IOMMU group of the device at $1
vfio_get_iommu_group() {

    local dev_path=$1

    local _iommu=$(readlink $dev_path)
    local _iommu_group="${_iommu##*iommu_groups/}"

    # check if it's integer
    if [ -n "$_iommu_group" ] && [ "$_iommu_group" -eq "$_iommu_group" ] 2> /dev/null; then
        echo "$_iommu_group"
    else
        echo -1
    fi
}

BASE_DIR=$(pwd)
DEVS_DIR=( "/sys/bus/platform/devices/" "/sys/bus/amba/devices/" )

SOURCE_DIR="./src_test"
SOURCE_FILES=( $SOURCE_DIR"/vfio_device_test.c" )
test_bin=$SOURCE_DIR"/vfio_dev_test"

MAKE="make"

if [ ! -f $test_bin ]; then

    echo Compiling VFIO test binary

    # Check for all needed source files
    for src_file in "${SOURCE_FILES[@]}"; do
        if [ ! -f $src_file ]; then
            echo missing source file $src_file
            exit 1
        fi
    done

    # Look for Makefile
    if [ ! -f $SOURCE_DIR"/Makefile" ]; then
        echo Makefile is missing!
        exit 1
    fi

    # Compile the test
    cd $SOURCE_DIR && $MAKE

    if [ $? -ne 0 ]; then
        echo Some error occurred during the compilation
        exit 1
    fi

    cd $BASE_DIR
fi

TEST_IRQ=0
ONLY_SPEC_TEST=0
for key in "$@"
do
    case $key in
        -i|--test-irq)
            TEST_IRQ=1
            shift
            ;;
        -s|--only-specific)
            ONLY_SPEC_TEST=1
            shift
            ;;
        --default)
            shift
            ;;
        *)
            echo `basename $0` "[-i|--test-irq][-s|--only-specific]"
            exit 0
            ;;
    esac
done

i=0
idx=0

echo Reading available devices to test...
for dir in "${DEVS_DIR[@]}"; do
    for line in $(find $dir); do

        has_iommu="$line/iommu_group"
        device="${line##$dir}"

        if [ -d $has_iommu ]; then

            DEVS[$idx]="$line"
            DEVS[$(($idx+1))]="$dir"

            echo $i\) "$device"
            i=$((i+1))
            idx=$(($i*2))

        fi
    done
done

echo Choose the device index to test \[0 ... "$((i-1))"\]:
read dev_idx

# Check if it's an int inside the range
regex='^[0-9]+$'
if ! [[ $dev_idx =~ $regex  ]] || [ "$dev_idx" -lt 0 ] || [ "$dev_idx" -ge "$i" ]; then
    echo Please select a value between 0 and $((i-1)) >&2; exit 1
fi

dev_line="${DEVS[$(($dev_idx*2))]}"
dev_dir="${DEVS[$(($dev_idx*2+1))]}"

# read bus of the device
subsystem="$dev_line/subsystem"
busdir=$(readlink $subsystem)
bus="${busdir##*/}"

has_driver="${dev_line}/driver"

device="${dev_line##$dev_dir}"

# path to the device IOMMU
has_iommu="$dev_line/iommu_group"

# Only if the device is attached to a IOMMU we can perform the test
iommu_group=$(vfio_get_iommu_group $has_iommu)
if [ $iommu_group -eq -1 ]; then
    echo error reading IOMMU group of the device
    exit 1
fi

vfio_probe_device "$device" "$bus" "$has_driver"
if [ $? -ne 0 ]; then
    echo VFIO: error while probing device $device
    exit 1
fi

echo -e '\n'

if [ $ONLY_SPEC_TEST -ne 1 ]; then
    echo ==================== begin test =====================
    echo testing device: $device\; IOMMU group: $iommu_group\; bus: $bus

    echo ------------------ C code ---------------------------
    $test_bin "$device" "$iommu_group" "$TEST_IRQ" "$bus"

    if [ $? -ne 0 ]; then
        echo ----------------- end C code -----------------------
        echo some errors occurred
    else
        echo -e '\n'
        echo ----------------- end C code -----------------------
        echo test completed successfully
    fi
fi

echo ============== device specifid test =================
case $device in
        2c0a0000.dma | 2c0a1000.dma | 2c0a2000.dma | 2c0a3000.dma)
            # Look for Makefile
            if [ ! -f $SOURCE_DIR"/pl330/Makefile" ]; then
                echo Makefile is missing!
                exit 1
            fi

            echo compiling DMA330 test code
            # Compile the test
            cd $SOURCE_DIR"/pl330" && $MAKE

            if [ $? -ne 0 ]; then
                echo Some error occurred during the compilation
                exit 1
            fi

            cd $BASE_DIR

            $SOURCE_DIR/pl330/test_pl330_vfio_driver /dev/vfio/$iommu_group $device

            shift
            ;;
        *)
            echo no specific test available
            ;;
esac

vfio_unbind_device "$device" "$bus"
if [ $? -ne 0 ]; then
    echo VFIO: error while unbinding device $device
    exit 1
else
    echo VFIO: device $device unbound successfully
fi
echo ===================== end test ======================
echo -e '\n'

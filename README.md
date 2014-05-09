 VFIO host test case

This is a test suite to try all the VFIO features on a selected device.
To run the test simply:

./vfio_v5_test_device.sh [-i|--test-irq]

If the the FastModels DMA330 controller is tested then, after the generic test,
the device specific code will be executed.

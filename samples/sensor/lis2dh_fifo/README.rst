.. zephyr:code-sample:: lis2dh_fifo
   :name: LIS2DH FIFO streaming
   :relevant-api: sensor_interface

   Drain LIS2DH hardware FIFO batches using the standard Sensor trigger API.

Overview
********

This sample starts an RTIO ``sensor_stream()`` for the LIS2DH hardware FIFO.
It requests both ``SENSOR_TRIG_FIFO_WATERMARK`` and ``SENSOR_TRIG_FIFO_FULL``
with ``SENSOR_STREAM_DATA_INCLUDE``. The driver drains each FIFO batch and
returns it through RTIO. The standard Sensor Async API decoder converts the
raw frames into accelerometer samples with timestamps.

The supplied HOLYIOT-25008 overlay sets the FIFO watermark to 16 samples. The
board routes LIS2DH INT1 to GPIO2.0 and uses SPI mode 3, which is selected by
the LIS2DH driver.

Building and Running
********************

Build and flash the sample for HOLYIOT-25008:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/lis2dh_fifo
   :board: holyiot_25008/nrf54l15/cpuapp
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   LIS2DH FIFO streaming started
   123456789 ns: (0.010763, -0.004785, 9.801000) m/s^2
   133456789 ns: (0.015548, -0.009570, 9.796215) m/s^2

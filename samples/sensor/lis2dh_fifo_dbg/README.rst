LIS2DH FIFO debug firmware
==========================

Test-only application. It is **not** intended for upstream Zephyr and lives
only on the ``test/dev`` branch of this fork.

It combines the LIS2DH hardware FIFO stream with the tooling needed to drive
the sensor remotely on a HOLYIOT-25008:

* **MCUboot** bootloader with signed, slot-based images (DFU capable),
* **MCUmgr over BLE** (SMP) with the IMG, OS, STAT and SHELL groups,
* local **console and shell on UART20** as a wired fallback,
* the LIS2DH FIFO stream from the upstream sample, reporting batch and frame
  counters through the MCUmgr ``stat`` group.

Build
*****

.. code-block:: console

   west build -b holyiot_25008/nrf54l15/cpuapp --sysbuild \
       samples/sensor/lis2dh_fifo_dbg

Resulting images:

``build/merged_holyiot_25008_nrf54l15_cpuapp.hex``
   Bootloader plus signed application, for a first flash.
``build/lis2dh_fifo_dbg/zephyr/zephyr.signed.bin``
   Application image for upload through MCUmgr.

Flashing
********

The board is normally flashed over its SWD header:

.. code-block:: console

   west flash -d build --erase

The board's default runner uses a system reset because the SWD header does not
expose the reset pin.

Remote access
*************

The firmware advertises as ``LIS2DH FIFO debug`` and exposes the standard SMP
service, so `nRF Connect Device Manager`_ can reach it:

* **IMG** - upload a new ``zephyr.signed.bin`` to the secondary slot, test and
  confirm the image; MCUboot applies it on the next reset.
* **OS** - reset the device, read device information and task statistics.
* **STAT** - read the ``lis2dh_fifo_stats`` group (``batches``, ``frames``).
* **SHELL** - run Zephyr shell commands and read back the output.

The wired console on UART20 (115200 8N1) stays available as a fallback.

Security
********

* The build uses the **default MCUboot signing key**, which is for debug use
  only. Provide your own key before any deployment.
* SMP access requires an encrypted link (``MCUMGR_TRANSPORT_BT_PERM_RW_ENCRYPT``)
  and Bluetooth bonding.
* The SHELL group grants broad register and reset access. Keep it out of
  production builds; ship only statistics and DFU.

.. _nRF Connect Device Manager: https://github.com/nordicsemi/IOS-nRF-Connect-Device-Manager

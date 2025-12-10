.. zephyr:code-sample:: pdm_i2s_loopback
   :name: PDM to I2S Audio Loopback
   :relevant-api: i2s_interface audio_dmic_interface

   Real-time audio loopback from PDM microphones to I2S codec.

Overview
********

This sample demonstrates real-time audio loopback using PDM (Pulse Density Modulation)
for audio capture and I2S (Inter-IC Sound) for audio playback through an external codec.

The sample:

* Captures audio from PDM digital microphones
* Applies FIR/IIR filtering in hardware
* Converts mono to stereo
* Plays audio through I2S codec to speakers/headphones

Requirements
************

* Board with PDM (LPPDM) peripheral
* I2S peripheral
* External audio codec (e.g., WM8904)
* Digital microphones connected to PDM pins
* Speakers or headphones connected to codec output

Building and Running
********************

This sample has been tested on Alif B1 Development Kit.

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/pdm_i2s
   :board: alif_b1_dk/ab1c1f4m51820ph0/rtss_he
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

   *** Booting Zephyr OS ***
   PDM to I2S Audio Loopback Sample
   Initializing PDM...
   Initializing I2S...
   Initializing Codec...
   Audio loopback started
   Speak into the microphone to hear your voice!

You should hear your voice in the headphones/speakers with minimal latency.

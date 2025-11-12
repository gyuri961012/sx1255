diff --git a/SoapySX/SoapySX.cpp b/SoapySX/SoapySX.cpp
index 54c2bfd6c3de2045af61718713e508f8fc4de019..bc7f118e1c4cdaeba8fd2ff9eac8a7682e39e1f8 100644
--- a/SoapySX/SoapySX.cpp
+++ b/SoapySX/SoapySX.cpp
@@ -1,38 +1,40 @@
 // SPDX-License-Identifier: MIT
 
 #include <SoapySDR/Device.hpp>
 #include <SoapySDR/Registry.hpp>
 #include <SoapySDR/Logger.hpp>
 #include <SoapySDR/Time.hpp>
 
 #include <string.h>
 #include <cassert>
 #include <chrono>
 #include <fstream>
 #include <thread>
 #include <mutex>
+#include <type_traits>
+#include <utility>
 
 #include <fcntl.h>
 #include <unistd.h>
 #include <sys/ioctl.h>
 #include <linux/types.h>
 #include <linux/spi/spidev.h>
 
 #include <alsa/asoundlib.h>
 #include <alsa/control.h>
 #include <gpiod.hpp>
 
 // Streaming mode, affecting how starting, stopping, overruns and underruns
 // are handled.
 enum stream_mode {
     // Behave like most SDRs: RX overrun or TX underrun
     // may cause samples to be dropped, but streams will keep running.
     // Application can use timestamps to maintain correct timing.
     STREAM_MODE_NORMAL,
     // Behave like linked ALSA PCMs with default software parameters.
     // Overrun or underrun causes both RX and TX streams to stop.
     // TX buffer must be always kept filled to avoid underrun.
     // First write to TX stream starts both streams.
     // This was a stop-gap solution before implementation of timestamps
     // and is primarily kept here for compatibility with applications
     // that have not been modified to use timestamps yet.
@@ -381,52 +383,110 @@ public:
     alsa_error:
         if (hwp != NULL)
             snd_pcm_hw_params_free(hwp);
         if (swp != NULL)
             snd_pcm_sw_params_free(swp);
         // TODO more detailed error messages?
         throw std::runtime_error("Error configuring ALSA device");
     }
 };
 
 
 /***********************************************************************
  * Device interface
  **********************************************************************/
 class SoapySX : public SoapySDR::Device
 {
 private:
     double masterClock;
     double sampleRate;
 
     // Mutex for anything involving SX1255 registers
     // to avoid problems if an application calls methods from multiple threads.
     mutable std::recursive_mutex reg_mutex;
 
     Spi spi;
+    // Helper wrapper to support both libgpiod 1.x and 2.x C++ APIs
+    using GpiodLineRequestReturn = decltype(std::declval<gpiod::line>().request(std::declval<gpiod::line_request>(), 0));
+    static constexpr bool hasGpiodRequestHandle = !std::is_void<GpiodLineRequestReturn>::value;
+
+    template <bool HasRequestHandle, typename RequestReturn = GpiodLineRequestReturn>
+    class GpiodLine;
+
+    template <typename RequestReturn>
+    class GpiodLine<false, RequestReturn>
+    {
+    public:
+        void init(gpiod::chip &chip, unsigned int offset, const char *consumer, int flags, int default_value)
+        {
+            line = chip.get_line(offset);
+            line.request({
+                .consumer = consumer,
+                .request_type = gpiod::line_request::DIRECTION_OUTPUT,
+                .flags = flags
+            }, default_value);
+        }
+
+        void set_value(int value)
+        {
+            line.set_value(value);
+        }
+
+    private:
+        gpiod::line line;
+    };
+
+    template <typename RequestReturn>
+    class GpiodLine<true, RequestReturn>
+    {
+    public:
+        static_assert(!std::is_void<RequestReturn>::value, "libgpiod 2.x should return a request handle");
+
+        void init(gpiod::chip &chip, unsigned int offset, const char *consumer, int flags, int default_value)
+        {
+            line = chip.get_line(offset);
+            request = line.request({
+                .consumer = consumer,
+                .request_type = gpiod::line_request::DIRECTION_OUTPUT,
+                .flags = flags
+            }, default_value);
+        }
+
+        void set_value(int value)
+        {
+            request.set_value(value);
+        }
+
+    private:
+        gpiod::line line;
+        RequestReturn request;
+    };
+
+    using GpiodLineType = GpiodLine<hasGpiodRequestHandle>;
+
     gpiod::chip gpio;
-    gpiod::line gpio_reset, gpio_rx, gpio_tx;
+    GpiodLineType gpio_reset, gpio_rx, gpio_tx;
     AlsaPcm alsa_rx;
     AlsaPcm alsa_tx;
 
     // Transmitter is turned on when squared magnitude of a TX sample
     // exceeds this threshold.
     float tx_threshold2;
     // If true, RX and TX streams have been linked using snd_pcm_link
     bool linked;
 
     // Values of registers (to be) written to the chip.
     // Storing them here makes it easier and faster to change
     // specific bits since they do not need to be read
     // from the chip every time.
     uint8_t regs[MAX_REGS];
 
     // Convert a SoapySDR nanosecond timestamp to a sample counter.
     int64_t timestamp_to_samples(long long timestamp) const
     {
         return SoapySDR::timeNsToTicks(timestamp, sampleRate);
     }
 
     // Convert a sample counter to a SoapySDR nanosecond timestamp.
     long long samples_to_timestamp(int64_t samples) const
     {
         return SoapySDR::ticksToTimeNs(samples, sampleRate);
@@ -447,68 +507,56 @@ private:
     unsigned get_cached_register_bits(size_t address, unsigned lowestbit, unsigned nbits) const
     {
         if (address >= MAX_REGS)
             throw std::runtime_error("Invalid register address");
         unsigned mask = ((1 << nbits) - 1) << lowestbit;
         return (regs[address] & mask) >> lowestbit;
     }
 
     // Write a range of registers to the chip.
     void write_registers_to_chip(size_t firstreg, size_t nregs)
     {
         if ((firstreg >= MAX_REGS) || (nregs > MAX_REGS) || (firstreg > MAX_REGS - nregs))
             throw std::runtime_error("Invalid register address");
 
         size_t transfer_len = nregs + 1;
         // Buffer for SPI transfer
         std::vector<uint8_t> buf(transfer_len, 0);
 
         buf[0] = firstreg | 0x80;
         for (size_t i = 1; i < transfer_len; i++)
             buf[i] = regs[firstreg + i - 1];
 
         spi.transfer(buf, buf);
     }
 
-    void init_gpio(void)
+    void init_gpio(unsigned int reset_offset, unsigned int rx_offset, unsigned int tx_offset)
     {
         SoapySDR_logf(SOAPY_SDR_DEBUG, "Requesting GPIO lines");
-        gpio_reset.request({
-            .consumer = "SX reset",
-            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
-            .flags = gpiod::line_request::FLAG_OPEN_SOURCE
-        }, 0);
-        gpio_rx.request({
-            .consumer = "SX RX",
-            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
-            .flags = 0
-        }, 1);
-        gpio_tx.request({
-            .consumer = "SX TX",
-            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
-            .flags = 0
-        }, 1);
+        gpio_reset.init(gpio, reset_offset, "SX reset", gpiod::line_request::FLAG_OPEN_SOURCE, 0);
+        gpio_rx.init(gpio, rx_offset, "SX RX", 0, 1);
+        gpio_tx.init(gpio, tx_offset, "SX TX", 0, 1);
     }
 
     void reset_chip(void)
     {
         SoapySDR_logf(SOAPY_SDR_DEBUG, "Resetting chip");
         // Timing from datasheet Figure 6-2: Manual Reset Timing Diagram
         gpio_reset.set_value(1);
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
         gpio_reset.set_value(0);
         std::this_thread::sleep_for(std::chrono::milliseconds(5));
     }
 
     void init_chip(void)
     {
         for (size_t i = 0; i < N_INIT_REGISTERS; i++)
             set_register_bits(i, 0, 8, init_registers[i]);
         // Enable RX and TX, just for initial testing. This should be done somewhere else.
         set_register_bits(0, 1, 3, 0b111);
         write_registers_to_chip(0, N_INIT_REGISTERS);
     }
 
     bool does_synth_tune(double frequency)
     {
         setFrequency(SOAPY_SDR_RX, 0, frequency, {});
         setFrequency(SOAPY_SDR_TX, 0, frequency, {});
@@ -534,64 +582,65 @@ private:
             SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 38.4 MHz");
             masterClock = 38.4e6;
         } else if (tunes_high && (!tunes_low)) {
             SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 32.0 MHz");
         } else {
             SoapySDR_logf(SOAPY_SDR_INFO, "Clock detection failed, assuming 32.0 MHz");
         }
 
         // Update default values for new masterClock value
         sampleRate = masterClock / 256.0;
         setFrequency(SOAPY_SDR_RX, 0, 433.92e6, {});
         setFrequency(SOAPY_SDR_TX, 0, 433.92e6, {});
     }
 
 /***********************************************************************
  * Initialization and destruction
  **********************************************************************/
 
 public:
     SoapySX(const SoapySDR::Kwargs &args, uint16_t hwversion):
         masterClock(32.0e6),
         sampleRate(125.0e3),
         // TODO: support custom SPIDEV path as an argument
         spi("/dev/spidev0.0"),
         gpio("gpiochip0"),
-        gpio_reset(gpio.get_line(5)),
-        gpio_rx(gpio.get_line(hwversion == 0x0100 ? 13 : 23)),
-        gpio_tx(gpio.get_line(hwversion == 0x0100 ? 12 : 22)),
         alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
         alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
         tx_threshold2(0.0f),
         linked(false),
         regs{0}
     {
         (void)args;
 
         SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");
 
-        init_gpio();
+        const unsigned int reset_offset = 5;
+        const unsigned int rx_offset = (hwversion == 0x0100 ? 13U : 23U);
+        const unsigned int tx_offset = (hwversion == 0x0100 ? 12U : 22U);
+
+        init_gpio(reset_offset, rx_offset, tx_offset);
         reset_chip();
         init_chip();
         detect_clock();
         // Open ALSA devices now when I2S clocks are already running.
         // I am not sure if this makes any difference but just in case.
         alsa_rx.open();
         alsa_tx.open();
     }
 
     ~SoapySX(void)
     {
         SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");
 
         // Put SX1255 to sleep
         set_register_bits(0, 0, 4, 0);
         write_registers_to_chip(0, 1);
 
         // Make sure PA is turned off
         writeSetting("PA", "OFF");
     }
 
 /***********************************************************************
  * Sample streams
  **********************************************************************/
 

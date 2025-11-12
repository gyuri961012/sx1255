// SPDX-License-Identifier: MIT

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <string.h>
#include <string>
#include <cassert>
#include <chrono>
#include <fstream>
#include <cmath>
#include <thread>
#include <mutex>
#include <utility>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>
#include <gpiod.h>

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
    STREAM_MODE_LINK,
};

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int32_t scale_from_range(SoapySDR::Range range, double value)
{
    return (int)std::round(
@@ -381,52 +388,138 @@ public:
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

    class GpiodLine
    {
    public:
        GpiodLine() = default;
        GpiodLine(const GpiodLine &) = delete;
        GpiodLine &operator=(const GpiodLine &) = delete;
        GpiodLine(GpiodLine &&other) noexcept : line(other.line)
        {
            other.line = nullptr;
        }

        GpiodLine &operator=(GpiodLine &&other) noexcept
        {
            if (this != &other)
            {
                release();
                line = other.line;
                other.line = nullptr;
            }
            return *this;
        }

        ~GpiodLine()
        {
            release();
        }

        void init(gpiod_chip *chip, unsigned int offset, const char *consumer, int flags, int default_value)
        {
            release();
            line = gpiod_chip_get_line(chip, offset);
            if (!line)
            {
                int err = errno;
                throw std::system_error(err, std::generic_category(), "Failed to get GPIO line");
            }

            gpiod_line_request_config config;
            memset(&config, 0, sizeof(config));
            config.consumer = consumer;
            config.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
            config.flags = flags;

            int ret = gpiod_line_request(line, &config, default_value);

            if (ret < 0)
            {
                int err = errno;
                line = nullptr;
                throw std::system_error(err, std::generic_category(), "Failed to request GPIO line");
            }
        }

        void release()
        {
            if (line != nullptr)
            {
                gpiod_line_release(line);
                line = nullptr;
            }
        }

        void set_value(int value)
        {
            if (line == nullptr)
                throw std::runtime_error("GPIO line not initialized");

            if (gpiod_line_set_value(line, value) < 0)
                throw std::system_error(errno, std::generic_category(), "Failed to set GPIO line value");
        }

    private:
        gpiod_line *line = nullptr;
    };

    using GpiodChipPtr = std::unique_ptr<gpiod_chip, decltype(&gpiod_chip_close)>;

    static GpiodChipPtr open_gpio_chip(const char *name)
    {
        gpiod_chip *chip = gpiod_chip_open_by_name(name);
        if (chip == nullptr)
            throw std::system_error(errno, std::generic_category(), "Failed to open GPIO chip");
        return GpiodChipPtr(chip, &gpiod_chip_close);
    }

    GpiodChipPtr gpio;
    GpiodLine gpio_reset, gpio_rx, gpio_tx;
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
@@ -447,68 +540,56 @@ private:
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

    void init_gpio(unsigned int reset_offset, unsigned int rx_offset, unsigned int tx_offset)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Requesting GPIO lines");
        gpio_reset.init(gpio.get(), reset_offset, "SX reset", GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE, 0);
        gpio_rx.init(gpio.get(), rx_offset, "SX RX", 0, 1);
        gpio_tx.init(gpio.get(), tx_offset, "SX TX", 0, 1);
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
@@ -533,84 +614,89 @@ private:
        if (tunes_low && (!tunes_high)) {
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
        gpio(open_gpio_chip("gpiochip0")),
        alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
        alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
        tx_threshold2(0.0f),
        linked(false),
        regs{0}
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        const unsigned int reset_offset = 5;
        const unsigned int rx_offset = (hwversion == 0x0100 ? 13U : 23U);
        const unsigned int tx_offset = (hwversion == 0x0100 ? 12U : 22U);

        init_gpio(reset_offset, rx_offset, tx_offset);
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

        gpio_tx.release();
        gpio_rx.release();
        gpio_reset.release();
    }

/***********************************************************************
 * Sample streams
 **********************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string & format,
        const std::vector<size_t> & channels,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channels; // Only one channel
        (void)args; // Unused for now

        std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);

        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        if (
            (snd_pcm_state(alsa_rx.pcm) == SND_PCM_STATE_RUNNING) ||
            (snd_pcm_state(alsa_tx.pcm) == SND_PCM_STATE_RUNNING)
        )
            throw std::runtime_error("Streams can be setup only if none of the streams are running");

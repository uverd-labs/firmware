#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include("Adafruit_PM25AQI.h")

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "Adafruit_PM25AQI.h"
#include "AirQualityTelemetry.h"
#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "main.h"
#include <Throttle.h>

#ifndef SENSOR_POWER_EN_PIN
#define SENSOR_POWER_EN_PIN 17
#endif

#ifndef BOOST_EN_ACTIVE_HIGH
#define BOOST_EN_ACTIVE_HIGH 1
#endif

#ifndef PMSA003I_WARMUP_MS
// From the PMSA003I datasheet:
// "Stable data should be got at least 30 seconds after the sensor wakeup
// from the sleep mode because of the fan’s performance."
#define PMSA003I_WARMUP_MS 15000
#endif

#ifndef AQI_FIXED_INTERVAL_MS
#define AQI_FIXED_INTERVAL_MS 20000UL // wildfire debug: 1 min
// #define AQI_FIXED_INTERVAL_MS 300000UL   // battery-saver: 5 min
// #define AQI_FIXED_INTERVAL_MS 1800000UL  // background logging: 30 min
#endif

#ifndef AQI_FIRST_CYCLE_DELAY_MS
#define AQI_FIRST_CYCLE_DELAY_MS 5000
#endif

#ifndef AQI_MIN_INTERVAL_MS
#define AQI_MIN_INTERVAL_MS 5000
#endif

static void logI2CProbe(TwoWire *bus, uint8_t addr, const char *name)
{
    bus->beginTransmission(addr);
    uint8_t err = bus->endTransmission();

    if (err == 0) {
        LOG_INFO("AirQualityTelemetry: I2C probe OK for %s at 0x%02X", name, addr);
    } else {
        LOG_WARN("AirQualityTelemetry: I2C probe FAIL for %s at 0x%02X (err=%u)", name, addr, err);
    }
}

static bool boostStateCached = false;

static inline bool boostIsCommandedOn()
{
    return boostStateCached;
}

static inline void boostEnable(bool on)
{
    boostStateCached = on;

#ifdef SENSOR_POWER_EN_PIN
    pinMode(SENSOR_POWER_EN_PIN, OUTPUT);
#if BOOST_EN_ACTIVE_HIGH
    digitalWrite(SENSOR_POWER_EN_PIN, on ? HIGH : LOW);
#else
    digitalWrite(SENSOR_POWER_EN_PIN, on ? LOW : HIGH);
#endif
    delay(2);
    LOG_INFO("AirQualityTelemetry: boostEnable(%d) pin=%d readback=%d", on ? 1 : 0, SENSOR_POWER_EN_PIN,
             digitalRead(SENSOR_POWER_EN_PIN));
#else
    LOG_WARN("AirQualityTelemetry: SENSOR_POWER_EN_PIN not defined");
    (void)on;
#endif
}

int32_t AirQualityTelemetryModule::runOnce()
{
    LOG_INFO("AirQualityTelemetry: runOnce entered, enabled=%d, pin17=%d, EN=%d",
             moduleConfig.telemetry.air_quality_enabled ? 1 : 0, digitalRead(17), digitalRead(SENSOR_POWER_EN_PIN));

    enum AqiState {
        AQI_IDLE = 0,
        AQI_POWERING = 1,
        AQI_WARMING = 2,
    };

    static AqiState aqiState = AQI_IDLE;
    static uint32_t powerOnStartMs = 0;
    static uint32_t warmupStartMs = 0;
    static uint32_t nextCycleMs = 0;
    static uint32_t lastIdleProbeMs = 0;

    const uint32_t PMSA_READY_TIMEOUT_MS = 10000; // was 5000
    const uint32_t PMSA_READY_POLL_MS = 500;      // slower, safer polling
    const uint32_t AQI_RETRY_DELAY_MS = 3000;     // short retry if startup fails

    if (!moduleConfig.telemetry.air_quality_enabled) {
        LOG_INFO("AirQualityTelemetry: disabled");
        boostEnable(false);
        aqiState = AQI_IDLE;
        nextCycleMs = 0;
        return 30000;
    }

    uint32_t now = millis();
    uint32_t intervalMs = AQI_FIXED_INTERVAL_MS;

#if defined(AQI_DEBUG_FAST_INTERVAL_MS)
    if (AQI_DEBUG_FAST_INTERVAL_MS > 0) {
        intervalMs = AQI_DEBUG_FAST_INTERVAL_MS;
    }
#endif

    if (intervalMs < AQI_MIN_INTERVAL_MS) {
        intervalMs = AQI_MIN_INTERVAL_MS;
    }

    sendToPhoneIntervalMs = intervalMs;

    if (nextCycleMs == 0) {
        LOG_INFO("AirQualityTelemetry: forcing booster OFF during first-cycle setup");
        boostEnable(false);
        delay(20);

        TwoWire *bus = &Wire;
        bus->begin();
        bus->setClock(100000);

        LOG_INFO("AirQualityTelemetry: boot-time I2C probe before first AQ cycle");
        logI2CProbe(bus, 0x12, "PMSA003I");
        logI2CProbe(bus, 0x77, "BME68x");

        nextCycleMs = now + AQI_FIRST_CYCLE_DELAY_MS;
    }

    LOG_INFO("AirQualityTelemetry: runOnce state=%d now=%u nextCycleMs=%u intervalMs=%u", (int)aqiState, now, nextCycleMs,
             intervalMs);

    if ((now - lastIdleProbeMs) > 10000) {
        lastIdleProbeMs = now;

        LOG_INFO("AirQualityTelemetry: idle heartbeat now=%u nextCycleMs=%u BOOST_CMD=%d", now, nextCycleMs,
                 boostIsCommandedOn() ? 1 : 0);

        TwoWire *bus = &Wire;
        bus->begin();
        bus->setClock(100000);

        // Keep BME probe always.
        logI2CProbe(bus, 0x77, "BME68x idle");

        // PMSA probe only when power is commanded on.
        if (boostIsCommandedOn()) {
            logI2CProbe(bus, 0x12, "PMSA003I idle");
        }
    }

    if (aqiState == AQI_IDLE) {
        int32_t untilNext = (int32_t)(nextCycleMs - now);
        LOG_INFO("AirQualityTelemetry: idle wait, %ld ms until next cycle", (long)untilNext);

        if ((int32_t)(now - nextCycleMs) < 0) {
            return nextCycleMs - now;
        }

        LOG_INFO("AirQualityTelemetry: starting AQ cycle, fixed interval=%u ms", intervalMs);
        LOG_INFO("AirQualityTelemetry: enabling booster");

        boostEnable(true);
        powerOnStartMs = millis();
        aqiState = AQI_POWERING;

        return PMSA_READY_POLL_MS;
    }

    if (aqiState == AQI_POWERING) {
        TwoWire *bus = &Wire;
        bus->begin();
        bus->setClock(100000);

        uint32_t elapsed = millis() - powerOnStartMs;

        LOG_INFO("AirQualityTelemetry: POWERING elapsed=%u ms EN=%d", elapsed, digitalRead(SENSOR_POWER_EN_PIN));

        bus->beginTransmission(0x12);
        uint8_t err = bus->endTransmission();

        if (err == 0) {
            LOG_INFO("AirQualityTelemetry: PMSA003I is ready after %u ms", elapsed);
            logI2CProbe(bus, 0x12, "PMSA003I after boost ON");
            logI2CProbe(bus, 0x77, "BME68x after boost ON");

            // Fresh driver instance every power cycle
            aqi = Adafruit_PM25AQI();

            LOG_INFO("AirQualityTelemetry: calling aqi.begin_I2C()");
            if (!aqi.begin_I2C(bus)) {
                LOG_ERROR("AirQualityTelemetry: aqi.begin_I2C() failed, shutting booster OFF and retrying soon");
                boostEnable(false);
                aqiState = AQI_IDLE;
                nextCycleMs = millis() + AQI_RETRY_DELAY_MS;
                return AQI_RETRY_DELAY_MS;
            }

            warmupStartMs = millis();
            aqiState = AQI_WARMING;

            LOG_INFO("AirQualityTelemetry: sensor found, starting warm-up timer (%d ms)", PMSA003I_WARMUP_MS);
            return PMSA003I_WARMUP_MS;
        }

        LOG_WARN("AirQualityTelemetry: PMSA003I not ready yet (err=%u) after %u ms", err, elapsed);

        if (elapsed >= PMSA_READY_TIMEOUT_MS) {
            LOG_WARN("AirQualityTelemetry: PMSA did not appear after timeout, shutting booster OFF and retrying soon");
            boostEnable(false);
            aqiState = AQI_IDLE;
            nextCycleMs = millis() + AQI_RETRY_DELAY_MS;
            return AQI_RETRY_DELAY_MS;
        }

        return PMSA_READY_POLL_MS;
    }

    if (aqiState == AQI_WARMING) {
        uint32_t elapsed = now - warmupStartMs;
        if (elapsed < PMSA003I_WARMUP_MS) {
            uint32_t remaining = PMSA003I_WARMUP_MS - elapsed;
            LOG_DEBUG("AirQualityTelemetry: still warming up, %u ms remaining", remaining);
            return remaining;
        }

        LOG_INFO("AirQualityTelemetry: warm-up complete, attempting read/send");

        bool sent = false;

        if (((lastSentToMesh == 0) || !Throttle::isWithinTimespanMs(lastSentToMesh, intervalMs)) &&
            airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
            airTime->isTxAllowedAirUtil()) {

            sent = sendTelemetry();
            if (sent) {
                lastSentToMesh = now;
                LOG_INFO("AirQualityTelemetry: sent packet to mesh");
            } else {
                LOG_WARN("AirQualityTelemetry: sendTelemetry() failed");
            }

        } else if (service->isToPhoneQueueEmpty()) {
            sent = sendTelemetry(NODENUM_BROADCAST, true);
            if (sent) {
                LOG_INFO("AirQualityTelemetry: sent packet to phone only");
            } else {
                LOG_WARN("AirQualityTelemetry: phone-only sendTelemetry() failed");
            }
        } else {
            LOG_WARN("AirQualityTelemetry: send deferred by airtime/queue constraints");
        }

        {
            TwoWire *bus = &Wire;
            bus->begin();
            bus->setClock(100000);
            logI2CProbe(bus, 0x77, "BME68x before leaving AQ cycle");
        }

        LOG_INFO("AirQualityTelemetry: disabling booster after AQ cycle");
        boostEnable(false);
        delay(100);
        LOG_INFO("AirQualityTelemetry: EN pin readback after OFF = %d", digitalRead(SENSOR_POWER_EN_PIN));

        {
            TwoWire *bus = &Wire;
            bus->begin();
            bus->setClock(100000);
            logI2CProbe(bus, 0x12, "PMSA003I after boost OFF");
            logI2CProbe(bus, 0x77, "BME68x after boost OFF");
        }

        aqiState = AQI_IDLE;
        nextCycleMs = millis() + intervalMs;

        LOG_INFO("AirQualityTelemetry: cycle complete, nextCycleMs=%u intervalMs=%u", nextCycleMs, intervalMs);
        return intervalMs;
    }

    LOG_WARN("AirQualityTelemetry: unexpected state, resetting to IDLE");
    LOG_WARN("AirQualityTelemetry: fallback path, forcing booster OFF");
    boostEnable(false);
    aqiState = AQI_IDLE;
    nextCycleMs = millis() + AQI_RETRY_DELAY_MS;
    return AQI_RETRY_DELAY_MS;
}

bool AirQualityTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
{
    if (t->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);

        LOG_INFO("(Received from %s): pm10_standard=%i, pm25_standard=%i, pm100_standard=%i", sender,
                 t->variant.air_quality_metrics.pm10_standard, t->variant.air_quality_metrics.pm25_standard,
                 t->variant.air_quality_metrics.pm100_standard);

        LOG_INFO("                  | pm10_env=%i, pm25_env=%i, pm100_env=%i", t->variant.air_quality_metrics.pm10_environmental,
                 t->variant.air_quality_metrics.pm25_environmental, t->variant.air_quality_metrics.pm100_environmental);
#endif
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);
    }

    return false; // Let others also process this message
}

bool AirQualityTelemetryModule::getAirQualityTelemetry(meshtastic_Telemetry *m)
{
    LOG_INFO("AirQualityTelemetry: AQI read attempt after warmup");

    if (!aqi.read(&data)) {
        LOG_WARN("AirQualityTelemetry: skip send, AQI read failed");
        return false;
    }

    m->time = getTime();
    m->which_variant = meshtastic_Telemetry_air_quality_metrics_tag;

    m->variant.air_quality_metrics.has_pm10_standard = true;
    m->variant.air_quality_metrics.pm10_standard = data.pm10_standard;

    m->variant.air_quality_metrics.has_pm25_standard = true;
    m->variant.air_quality_metrics.pm25_standard = data.pm25_standard;

    m->variant.air_quality_metrics.has_pm100_standard = true;
    m->variant.air_quality_metrics.pm100_standard = data.pm100_standard;

    m->variant.air_quality_metrics.has_pm10_environmental = true;
    m->variant.air_quality_metrics.pm10_environmental = data.pm10_env;

    m->variant.air_quality_metrics.has_pm25_environmental = true;
    m->variant.air_quality_metrics.pm25_environmental = data.pm25_env;

    m->variant.air_quality_metrics.has_pm100_environmental = true;
    m->variant.air_quality_metrics.pm100_environmental = data.pm100_env;

    LOG_INFO("Send: PM1.0(Standard)=%i, PM2.5(Standard)=%i, PM10(Standard)=%i", m->variant.air_quality_metrics.pm10_standard,
             m->variant.air_quality_metrics.pm25_standard, m->variant.air_quality_metrics.pm100_standard);

    LOG_INFO("     | PM1.0(Env)=%i, PM2.5(Env)=%i, PM10(Env)=%i", m->variant.air_quality_metrics.pm10_environmental,
             m->variant.air_quality_metrics.pm25_environmental, m->variant.air_quality_metrics.pm100_environmental);

    return true;
}

meshtastic_MeshPacket *AirQualityTelemetryModule::allocReply()
{
    if (currentRequest) {
        auto req = *currentRequest;
        const auto &p = req.decoded;

        meshtastic_Telemetry scratch;
        meshtastic_Telemetry *decoded = NULL;
        memset(&scratch, 0, sizeof(scratch));

        if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
            decoded = &scratch;
        } else {
            LOG_ERROR("Error decoding AirQualityTelemetry module!");
            return NULL;
        }

        if (decoded->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
            meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
            if (getAirQualityTelemetry(&m)) {
                LOG_INFO("Air quality telemetry reply to request");
                return allocDataProtobuf(m);
            }
        }
    }

    return NULL;
}

bool AirQualityTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;

    if (!getAirQualityTelemetry(&m))
        return false;

    meshtastic_MeshPacket *p = allocDataProtobuf(m);
    p->to = dest;
    p->decoded.want_response = false;

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    else
        p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    if (lastMeasurementPacket != nullptr)
        packetPool.release(lastMeasurementPacket);

    lastMeasurementPacket = packetPool.allocCopy(*p);

    if (phoneOnly) {
        LOG_INFO("Send packet to phone");
        service->sendToPhone(p);
    } else {
        LOG_INFO("Send packet to mesh");
        service->sendToMesh(p, RX_SRC_LOCAL, true);
    }

    return true;
}

#endif
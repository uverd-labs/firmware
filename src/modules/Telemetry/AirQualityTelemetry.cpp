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
#include "detect/ScanI2CTwoWire.h"
#include "main.h"
#include <Throttle.h>

#ifndef PMSA003I_WARMUP_MS
// From the PMSA003I datasheet:
// "Stable data should be got at least 30 seconds after the sensor wakeup
// from the sleep mode because of the fan’s performance."
#define PMSA003I_WARMUP_MS 30000
#endif

// PMSA003I default I2C address (same as Adafruit_PM25AQI.h)
static constexpr uint8_t PMSA003I_ADDR = 0x12;

// Optional dev override (OFF unless you define it in build flags)
// Example: add -DAQI_DEBUG_FAST_INTERVAL_MS=5000 in platformio.ini
// #define AQI_DEBUG_FAST_INTERVAL_MS 30000

static inline void boostEnable(bool on)
{
#ifdef BOOST_EN_PIN
    pinMode(BOOST_EN_PIN, OUTPUT);
#if BOOST_EN_ACTIVE_HIGH
    digitalWrite(BOOST_EN_PIN, on ? HIGH : LOW);
#else
    digitalWrite(BOOST_EN_PIN, on ? LOW : HIGH);
#endif
#else
    (void)on; // placeholder: no pin configured
#endif
}

int32_t AirQualityTelemetryModule::runOnce()
{
    moduleConfig.telemetry.air_quality_enabled = 1;
    LOG_INFO("AirQualityTelemetry: enabled");
    // if (!moduleConfig.telemetry.air_quality_enabled) {
    //     return disable();
    // }
    //  Local init state for this module
    static bool aqi_inited = false;
    static uint32_t warmup_start_ms = 0;

    // --- FIRST PHASE: one-time init + warmup --------------------------------
    if (!aqi_inited) {
        LOG_INFO("Air quality Telemetry: init");

        // --- VEXT HANDLING (DISABLED FOR NOW) --------------------
        /*
        #ifdef PMSA003I_ENABLE_PIN
            pinMode(PMSA003I_ENABLE_PIN, OUTPUT);
            // POLARITY: HIGH = VEXT OFF (sensor unpowered)
            digitalWrite(PMSA003I_ENABLE_PIN, HIGH);
        #endif
        */
        // ---------------------------------------------------------
        LOG_INFO("AirQualityTelemetry: enabling booster");
        boostEnable(true);
        delay(50); // tiny settle time; safe placeholder

        LOG_INFO("AirQualityTelemetry: init I2C (no reinit). Selecting external bus.");

        // Prefer the “external peripherals” I2C bus if it exists on this build.
        TwoWire *bus = &Wire;
        LOG_INFO("AirQualityTelemetry: forcing Wire (RAK4631)");

        LOG_INFO("AirQualityTelemetry: using Wire for PMSA003I");

        bus->begin();
        bus->setClock(100000);

        // Optional: one-time I2C scan to confirm devices are visible on this bus.
        for (uint8_t addr = 8; addr < 120; addr++) {
            bus->beginTransmission(addr);
            uint8_t err = bus->endTransmission();
            if (err == 0) {
                LOG_INFO("I2C device ACK at 0x%02X", addr);
            }
        }

        LOG_INFO("AirQualityTelemetry: calling aqi.begin_I2C()");
        if (!aqi.begin_I2C(bus)) {
            LOG_ERROR("AQI begin_I2C() failed. Disabling module.");
            boostEnable(false);
            return disable();
        }

        LOG_INFO("AQI sensor found, starting warm-up timer (%d ms)", PMSA003I_WARMUP_MS);
        warmup_start_ms = millis();
        aqi_inited = true;

        return PMSA003I_WARMUP_MS;
    }

    // --- WARMUP GUARD -------------------------------------------------------
    uint32_t now = millis();
    if (now - warmup_start_ms < PMSA003I_WARMUP_MS) {
        uint32_t remaining = PMSA003I_WARMUP_MS - (now - warmup_start_ms);
        LOG_DEBUG("AirQualityTelemetry: still warming up, %u ms remaining", remaining);
        return remaining;
    }

    // --- MAIN ACTIVE STATE: send telemetry periodically ---------------------
    LOG_DEBUG("AirQualityTelemetry: ACTIVE");

    // Base interval from runtime config.
    // “Dev default” only applies if config is unset/0.
    uint32_t minIntervalMs = Default::getConfiguredOrDefaultMsScaled(moduleConfig.telemetry.air_quality_interval,
#if defined(DEBUG_BUILD) || defined(ARDUINO_USB_CDC_ON_BOOT)
                                                                     10, // 10s default ONLY if user didn’t set it
#else
                                                                     default_telemetry_broadcast_interval_secs,
#endif
                                                                     numOnlineNodes);

    // Optional explicit dev override (ONLY if you define it via build flags)
#if defined(AQI_DEBUG_FAST_INTERVAL_MS)
    if (AQI_DEBUG_FAST_INTERVAL_MS > 0 && AQI_DEBUG_FAST_INTERVAL_MS < minIntervalMs) {
        minIntervalMs = AQI_DEBUG_FAST_INTERVAL_MS;
    }
#endif

    // Use same interval for phone updates so UI refresh aligns
    sendToPhoneIntervalMs = minIntervalMs;

    // Throttle + channel util guards stay the same
    if (((lastSentToMesh == 0) || !Throttle::isWithinTimespanMs(lastSentToMesh, minIntervalMs)) &&
        airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
        airTime->isTxAllowedAirUtil()) {

        sendTelemetry();
        lastSentToMesh = now;

    } else if (service->isToPhoneQueueEmpty()) {
        sendTelemetry(NODENUM_BROADCAST, true);
    }

    return sendToPhoneIntervalMs;
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
    if (!aqi.read(&data)) {
        LOG_WARN("Skip send measurements: AQI read failed");
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

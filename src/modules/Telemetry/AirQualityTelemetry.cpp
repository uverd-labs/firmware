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
// from the PMSA003I datasheet:
// "Stable data should be got at least 30 seconds after the sensor wakeup
// from the sleep mode because of the fan’s performance."
#define PMSA003I_WARMUP_MS 30000
#endif

// For lab testing: send AQ data very often.
// Set back to 60000 or remove this define for production.
#define AQI_DEBUG_FAST_INTERVAL_MS 5000 // 5 seconds

// PMSA003I default I2C address (same as Adafruit_PM25AQI.h)
static constexpr uint8_t PMSA003I_ADDR = 0x12;
// For debugging: send every 5 seconds instead of the usual minutes
#define AQI_DEBUG_FAST_INTERVAL_MS 5000

int32_t AirQualityTelemetryModule::runOnce()
{
    if (!moduleConfig.telemetry.air_quality_enabled) {
        return disable();
    }

    // Local init state for this module
    static bool aqi_inited = false;
    static uint32_t warmup_start_ms = 0;
    static bool wire_reinit_done = false;

    // --- FIRST PHASE: one-time init + warmup --------------------------------
    if (!aqi_inited) {
        LOG_INFO("Air quality Telemetry: init");

        // --- VEXT HANDLING (DISABLED FOR NOW) --------------------
        // We'll re-enable this later once PMSA003I is working reliably.
        /*
        #ifdef PMSA003I_ENABLE_PIN
            pinMode(PMSA003I_ENABLE_PIN, OUTPUT);
            // POLARITY: HIGH = VEXT OFF (sensor unpowered)
            digitalWrite(PMSA003I_ENABLE_PIN, HIGH);
        #endif
        */
        // ---------------------------------------------------------

        // Re-init I2C like in the Arduino test (SDA=41, SCL=42, 100kHz)
        if (!wire_reinit_done) {
            LOG_INFO("AirQualityTelemetry: reinitializing I2C bus on SDA=41, SCL=42 at 100kHz");

            Wire.end();            // Meshtastic had it running already
            Wire.begin(41, 42);    // Heltec V3 pins from your test sketch
            Wire.setClock(100000); // PMSA003I expects 100kHz

            // Give sensor MCU time to boot (same as test sketch)
            delay(3000);

            wire_reinit_done = true;
        }

        LOG_INFO("AirQualityTelemetry: calling aqi.begin_I2C() after bus re-init");
        if (!aqi.begin_I2C(&Wire)) {
            LOG_ERROR("AQI begin_I2C() failed after I2C re-init (addr 0x%02X). Disabling module.", PMSA003I_ADDR);
            return disable();
        }

        LOG_INFO("AQI sensor found, starting warm-up timer (%d ms)", PMSA003I_WARMUP_MS);
        warmup_start_ms = millis();
        aqi_inited = true;

        // Schedule next run after warmup time
        return PMSA003I_WARMUP_MS;
    }

    // If somehow init was skipped or failed, bail out safely
    if (!aqi_inited) {
        LOG_WARN("AirQualityTelemetry: aqi_inited == false after init. Disabling module.");
        return disable();
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

    // 1. Base interval from config
    uint32_t minIntervalMs = Default::getConfiguredOrDefaultMsScaled(moduleConfig.telemetry.air_quality_interval,
                                                                     default_telemetry_broadcast_interval_secs, numOnlineNodes);

    // 2. Debug override: allow faster interval for testing
#ifdef AQI_DEBUG_FAST_INTERVAL_MS
    if (AQI_DEBUG_FAST_INTERVAL_MS < minIntervalMs) {
        minIntervalMs = AQI_DEBUG_FAST_INTERVAL_MS;
    }
#endif

    // Also use this as phone update interval, so screen refresh matches
    sendToPhoneIntervalMs = minIntervalMs;

    // 3. Same throttle logic, but using minIntervalMs instead of the big default
    if (((lastSentToMesh == 0) || !Throttle::isWithinTimespanMs(lastSentToMesh, minIntervalMs)) &&
        airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
        airTime->isTxAllowedAirUtil()) {

        // Normal send to mesh
        sendTelemetry();
        lastSentToMesh = now;

    } else if (service->isToPhoneQueueEmpty()) {
        // Lower priority: push latest measurement to phone
        sendTelemetry(NODENUM_BROADCAST, true);
    }

    // How soon runOnce() is called again
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

        LOG_INFO("                  | PM1.0(Environmental)=%i, PM2.5(Environmental)=%i, PM10.0(Environmental)=%i",
                 t->variant.air_quality_metrics.pm10_environmental, t->variant.air_quality_metrics.pm25_environmental,
                 t->variant.air_quality_metrics.pm100_environmental);
#endif
        // release previous packet before occupying a new spot
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);
    }

    return false; // Let others look at this message also if they want
}

bool AirQualityTelemetryModule::getAirQualityTelemetry(meshtastic_Telemetry *m)
{
    if (!aqi.read(&data)) {
        LOG_WARN("Skip send measurements. Could not read AQIn");
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

    LOG_INFO("Send: PM1.0(Standard)=%i, PM2.5(Standard)=%i, PM10.0(Standard)=%i", m->variant.air_quality_metrics.pm10_standard,
             m->variant.air_quality_metrics.pm25_standard, m->variant.air_quality_metrics.pm100_standard);

    LOG_INFO("         | PM1.0(Environmental)=%i, PM2.5(Environmental)=%i, PM10.0(Environmental)=%i",
             m->variant.air_quality_metrics.pm10_environmental, m->variant.air_quality_metrics.pm25_environmental,
             m->variant.air_quality_metrics.pm100_environmental);

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
        // Check for a request for air quality metrics
        if (decoded->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
            meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
            if (getAirQualityTelemetry(&m)) {
                LOG_INFO("Air quality telemetry reply to request");
                return allocDataProtobuf(m);
            } else {
                return NULL;
            }
        }
    }
    return NULL;
}

bool AirQualityTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    if (getAirQualityTelemetry(&m)) {
        meshtastic_MeshPacket *p = allocDataProtobuf(m);
        p->to = dest;
        p->decoded.want_response = false;
        if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
            p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
        else
            p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

        // release previous packet before occupying a new spot
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

    return false;
}

#endif
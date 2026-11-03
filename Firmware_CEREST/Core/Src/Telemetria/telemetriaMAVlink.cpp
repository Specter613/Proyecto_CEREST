/*
 * telemetriaMAVlink.cpp
 *
 *  Created on: 8 ago 2026
 *      Author: specter0163
 */
#include "Telemetria/telemetriaMAVlink.hpp"
#include <string.h>
#include "usbd_cdc_if.h"
#include <stdio.h>

TelemetriaMavlink::TelemetriaMavlink(MPU6500 &imu, QMC5883P &mag, BMP180 &baro, MahonyOrientacion &mahony, GananciasPID &pid, EkfPosicion &ekf,const Gnss &gps)
    : imu_(imu), mag_(mag), baro_(baro), mahony_(mahony), pid_(pid), ekf_(ekf), gps_(gps){

    // --- Bucle EXTERIOR (ángulo) — roll y pitch únicamente, yaw no tiene ---
    paramTable_[0]  = {"PID_AR_P",  &pid_.kp_angulo_roll};
    paramTable_[1]  = {"PID_AR_I",  &pid_.ki_angulo_roll};
    paramTable_[2]  = {"PID_AR_D",  &pid_.kd_angulo_roll};
    paramTable_[3]  = {"PID_AP_P",  &pid_.kp_angulo_pitch};
    paramTable_[4]  = {"PID_AP_I",  &pid_.ki_angulo_pitch};
    paramTable_[5]  = {"PID_AP_D",  &pid_.kd_angulo_pitch};

    // --- Bucle INTERIOR (velocidad / rate) — los 3 ejes ---
    paramTable_[6]  = {"PID_RR_P",  &pid_.kp_velocidad_roll};
    paramTable_[7]  = {"PID_RR_I",  &pid_.ki_velocidad_roll};
    paramTable_[8]  = {"PID_RR_D",  &pid_.kd_velocidad_roll};
    paramTable_[9]  = {"PID_RP_P",  &pid_.kp_velocidad_pitch};
    paramTable_[10] = {"PID_RP_I",  &pid_.ki_velocidad_pitch};
    paramTable_[11] = {"PID_RP_D",  &pid_.kd_velocidad_pitch};
    paramTable_[12] = {"PID_RY_P",  &pid_.kp_velocidad_yaw};
    paramTable_[13] = {"PID_RY_I",  &pid_.ki_velocidad_yaw};
    paramTable_[14] = {"PID_RY_D",  &pid_.kd_velocidad_yaw};
}

void TelemetriaMavlink::Init() {
    pendingPidIndex_ = 0;
    sendingPidList_ = false;
}

void TelemetriaMavlink::SetTxCallback(MavlinkTxCallback callback, void *userCtx) {
    txCallback_ = callback;
    txCtx_      = userCtx;
}

bool TelemetriaMavlink::SendMessage(mavlink_message_t *msg) {
    if (txCallback_ == nullptr) return false;
    uint8_t txBuf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(txBuf, msg);
    return txCallback_(txBuf, len, txCtx_);
}

void TelemetriaMavlink::Update() {
    // 1. PRIORIDAD: Enviar PIDs pendientes si aplica
    if (sendingPidList_) {
        uint32_t now = HAL_GetTick();
        if(now - lastPidSendTick_ >= 20) {
            if (SendParamValue(pendingPidIndex_)) {
                lastPidSendTick_ = now;
                pendingPidIndex_++;
                if (pendingPidIndex_ >= NUM_PARAMS) {
                    sendingPidList_ = false;
                }
            }
        }
        return;
    }

    uint32_t now = HAL_GetTick();

    // Un solo mensaje por llamada: si el transmisor sigue ocupado con el anterior,
    // el siguiente se perdería. Lo que no sale ahora, sale en la próxima llamada.

    // 2. Heartbeat (siempre activo)
    if (intervalos_.heartbeatMs > 0 && (now - ticks_.heartbeat >= intervalos_.heartbeatMs)) {
        ticks_.heartbeat = now;
        SendHeartbeat();
        return;
    }

    // 3. Batería: voltaje, corriente y % (siempre activo)
    if (intervalos_.bateriaMs > 0 && (now - ticks_.bateria >= intervalos_.bateriaMs)) {
        ticks_.bateria = now;
        SendSysStatus();
        return;
    }

    // 4. Batería: mAh consumidos (siempre activo)
    if (intervalos_.bateriaStatusMs > 0 && (now - ticks_.bateriaStatus >= intervalos_.bateriaStatusMs)) {
        ticks_.bateriaStatus = now;
        SendBatteryStatus();
        return;
    }

    // 5. VFR HUD (solo si el GCS lo pidió)
    if (intervalos_.vfrHudMs > 0 && (now - ticks_.vfrHud >= intervalos_.vfrHudMs)) {
        ticks_.vfrHud = now;
        SendVfrHud();
        return;
    }

    // 6. IMU (solo si el GCS lo pidió)
    if (intervalos_.imuMs > 0 && (now - ticks_.imu >= intervalos_.imuMs)) {
        ticks_.imu = now;
        SendAttitude();
        return;
    }

    // 7. Posición del estimador (solo si el GCS lo pidió)
    if (intervalos_.posicionMs > 0 && (now - ticks_.posicion >= intervalos_.posicionMs)) {
        ticks_.posicion = now;
        SendPosicion();
        return;
    }

    // 8. GPS (solo si el GCS lo pidió)
    if (intervalos_.gpsMs > 0 && (now - ticks_.gps >= intervalos_.gpsMs)) {
        ticks_.gps = now;
        SendGpsRaw();
        return;
    }
}


void TelemetriaMavlink::ProcessRxByte(uint8_t byte) {
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        OnMavlinkMessageReceived(msg);
    }
}

void TelemetriaMavlink::SendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        systemId_, componentId_, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );
    SendMessage(&msg);
}

void TelemetriaMavlink::SendSysStatus() {
    // SYS_STATUS: voltaje en mV (UINT16_MAX = desconocido),
    // corriente en cA (-1 = desconocida), restante en % (-1 = desconocido)
    mavlink_sys_status_t s{};

    if (batVoltaje_ >= 0.0f) {
        float mv = batVoltaje_ * 1000.0f;
        if (mv > 65534.0f) mv = 65534.0f;
        s.voltage_battery = static_cast<uint16_t>(mv);
    } else {
        s.voltage_battery = UINT16_MAX;
    }

    if (batCorriente_ >= 0.0f) {
        float ca = batCorriente_ * 100.0f;
        if (ca > 32767.0f) ca = 32767.0f;
        s.current_battery = static_cast<int16_t>(ca);
    } else {
        s.current_battery = -1;
    }

    s.battery_remaining = (batRestante_ >= 0 && batRestante_ <= 100) ? batRestante_ : -1;

    mavlink_message_t msg;
    mavlink_msg_sys_status_encode(systemId_, componentId_, &msg, &s);
    SendMessage(&msg);
}

void TelemetriaMavlink::SendBatteryStatus() {
    // BATTERY_STATUS: igual que SYS_STATUS, más los mAh consumidos.
    //   voltages[0]: voltaje total en mV si no se miden celdas (resto = UINT16_MAX)
    //   current_consumed: mAh (-1 = desconocido)
    mavlink_battery_status_t b{};

    b.id = 0;
    b.battery_function = MAV_BATTERY_FUNCTION_ALL;
    b.type = MAV_BATTERY_TYPE_LIPO;
    b.temperature = INT16_MAX;   // desconocida

    for (int i = 0; i < 10; i++) b.voltages[i] = UINT16_MAX;
    if (batVoltaje_ >= 0.0f) {
        float mv = batVoltaje_ * 1000.0f;
        if (mv > 65534.0f) mv = 65534.0f;
        b.voltages[0] = static_cast<uint16_t>(mv);
    }

    if (batCorriente_ >= 0.0f) {
        float ca = batCorriente_ * 100.0f;
        if (ca > 32767.0f) ca = 32767.0f;
        b.current_battery = static_cast<int16_t>(ca);
    } else {
        b.current_battery = -1;
    }

    b.current_consumed  = (batMah_ >= 0.0f) ? static_cast<int32_t>(batMah_ + 0.5f) : -1;
    b.energy_consumed   = -1;
    b.battery_remaining = (batRestante_ >= 0 && batRestante_ <= 100) ? batRestante_ : -1;

    mavlink_message_t msg;
    mavlink_msg_battery_status_encode(systemId_, componentId_, &msg, &b);
    SendMessage(&msg);
}


void TelemetriaMavlink::SendAttitude() {
    mavlink_message_t msg;

    float rollDeg, pitchDeg, yawDeg;
    mahony_.GetEuler(rollDeg, pitchDeg, yawDeg);   // <-- filtrado, no crudo

    float rollRad  = rollDeg  * 0.01745329f;
    float pitchRad = pitchDeg * 0.01745329f;

    // GetEuler da yaw en convencion brujula 0-360; MAVLink ATTITUDE
    // espera -pi a +pi. Hay que envolver, si no, headings >180 se
    // mandan como angulos mayores a pi (fuera de rango esperado).
    float yawRad = yawDeg * 0.01745329f;
    if (yawRad > 3.14159265f) yawRad -= 2.0f * 3.14159265f;

    mavlink_msg_attitude_pack(
        systemId_, componentId_, &msg,
        0, rollRad, pitchRad, yawRad, 0.0f, 0.0f, 0.0f
    );
    SendMessage(&msg);
}

void TelemetriaMavlink::SendGpsRaw() {
    // GPS_RAW_INT a partir de NAV-PVT (u-blox). Unidades MAVLink:
    //   lat/lon: grados*1e7   alt: mm (MSL)   vel: cm/s   cog: grados*100
    //   eph/epv: DOP*100 (UINT16_MAX = desconocido)   h_acc/v_acc: mm
    const GnssData &d = gps_.GetData();
    mavlink_gps_raw_int_t g{};

    g.time_usec = static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;

    // Tipo de fix: u-blox (0 sin fix, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 solo hora)
    //   -> MAVLink (1 sin fix, 2 2D, 3 3D)
    switch (d.fixType) {
        case 2:  g.fix_type = GPS_FIX_TYPE_2D_FIX; break;
        case 3:
        case 4:  g.fix_type = GPS_FIX_TYPE_3D_FIX; break;
        default: g.fix_type = GPS_FIX_TYPE_NO_FIX; break;
    }

    g.lat = d.lat;
    g.lon = d.lon;
    g.alt = d.hMSL;
    g.alt_ellipsoid = d.height;
    g.satellites_visible = d.numSV;

    g.eph = UINT16_MAX;   // NAV-PVT no trae HDOP en lo que guardamos: desconocido
    g.epv = UINT16_MAX;
    g.h_acc = d.hAcc;
    g.v_acc = d.vAcc;

    // Velocidad: mm/s -> cm/s
    int32_t velCm = d.gSpeed / 10;
    if (velCm < 0) velCm = 0;
    if (velCm > 65534) velCm = 65534;
    g.vel = static_cast<uint16_t>(velCm);

    // Rumbo de movimiento: grados*1e5 -> grados*100, en 0..35999
    int32_t cog = d.headMot / 1000;
    cog %= 36000;
    if (cog < 0) cog += 36000;
    g.cog = static_cast<uint16_t>(cog);

    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(systemId_, componentId_, &msg, &g);
    SendMessage(&msg);
}

void TelemetriaMavlink::SendPosicion() {
    // LOCAL_POSITION_NED: x, y en m; z = altura en m, POSITIVA hacia arriba;
    // vx, vy en m/s del estimador
    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(
        systemId_, componentId_, &msg,
        HAL_GetTick(),
        ekf_.PosX(), ekf_.PosY(), ekf_.AlturaUsada(),
        ekf_.VelX(), ekf_.VelY(), 0.0f
    );
    SendMessage(&msg);
}

void TelemetriaMavlink::SendVfrHud() {
    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(
        systemId_, componentId_, &msg,
        0.0f, 0.0f, static_cast<int16_t>(mag_.ObtenerProcesado().heading),
        0, baro_.GetAltitude(), 0.0f
    );
    SendMessage(&msg);
}

void TelemetriaMavlink::OnMavlinkMessageReceived(const mavlink_message_t &msg) {
    // =================================================================
    // 1. SOLICITUD DE LISTA COMPLETA DE PIDs (PARAM_REQUEST_LIST)
    // =================================================================
    if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
    	sendingPidList_ = true;
    	pendingPidIndex_ = 0;
    }
    // =================================================================
    // 2. SOLICITUD DE UN PID ESPECÍFICO (PARAM_REQUEST_READ)
    // =================================================================
    else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
        mavlink_param_request_read_t readReq;
        mavlink_msg_param_request_read_decode(&msg, &readReq);

        if (readReq.param_index >= 0 && readReq.param_index < static_cast<int16_t>(NUM_PARAMS)) {
            SendParamValue(static_cast<uint16_t>(readReq.param_index));
        }
        else {
            char reqId[17] = {0};
            memcpy(reqId, readReq.param_id, 16);
            for (uint16_t i = 0; i < NUM_PARAMS; i++) {
                if (strcmp(reqId, paramTable_[i].id) == 0) {
                    SendParamValue(i);
                    break;
                }
            }
        }
    }
    // =================================================================
    // 3. CAMBIO DE PID DESDE PYTHON (PARAM_SET)
    // =================================================================
    else if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
        mavlink_param_set_t setReq;
        mavlink_msg_param_set_decode(&msg, &setReq);

        char setKey[17] = {0};
        memcpy(setKey, setReq.param_id, 16);

        for (int k = 15; k >= 0; k--) {
            if (setKey[k] == ' ' || setKey[k] == '\0') setKey[k] = '\0';
            else break;
        }

        for (uint16_t i = 0; i < NUM_PARAMS; i++) {
             if (strcmp(setKey, paramTable_[i].id) == 0) {
                *(paramTable_[i].valuePtr) = setReq.param_value;
                 contadorCambios_++;
                 SendParamValue(i);
                 break;
             }
        }
    }
    // =================================================================
    // 4. SOLICITUD DE STREAM (REQUEST_DATA_STREAM)
    //    La batería NO depende de esto: está siempre activa, como el heartbeat.
    // =================================================================
    else if (msg.msgid == MAVLINK_MSG_ID_REQUEST_DATA_STREAM) {
        mavlink_request_data_stream_t req;
        mavlink_msg_request_data_stream_decode(&msg, &req);
        const uint16_t hz = req.start_stop ? req.req_message_rate : 0;

        if (req.req_stream_id == MAV_DATA_STREAM_RAW_SENSORS) {
            SetImuHz(hz);
        }
        else if (req.req_stream_id == MAV_DATA_STREAM_POSITION) {
            SetPosicionHz(hz);
        }
        else if (req.req_stream_id == MAV_DATA_STREAM_EXTENDED_STATUS) {
            SetGpsHz(hz);
        }
        else if (req.req_stream_id == MAV_DATA_STREAM_ALL) {
            SetImuHz(hz);
            SetPosicionHz(hz);
            SetGpsHz(hz);
        }
    }
}

bool TelemetriaMavlink::SendParamValue(uint16_t index) {
    if (index >= NUM_PARAMS) return false;
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        systemId_, componentId_, &msg,
        paramTable_[index].id, *(paramTable_[index].valuePtr),
        MAV_PARAM_TYPE_REAL32, NUM_PARAMS, index
    );
    return SendMessage(&msg);
}

void TelemetriaMavlink::SetImuHz(uint16_t hz) {
    intervalos_.imuMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetHeartbeatHz(uint16_t hz) {
    intervalos_.heartbeatMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetVfrHudHz(uint16_t hz) {
    intervalos_.vfrHudMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetPosicionHz(uint16_t hz) {
    intervalos_.posicionMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetGpsHz(uint16_t hz) {
    intervalos_.gpsMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetBateriaHz(uint16_t hz) {
    intervalos_.bateriaMs = (hz > 0) ? (1000 / hz) : 0;
}

void TelemetriaMavlink::SetBateriaStatusHz(uint16_t hz) {
    intervalos_.bateriaStatusMs = (hz > 0) ? (1000 / hz) : 0;
}
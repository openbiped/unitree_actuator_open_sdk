#include "unitreeMotor/unitreeMotor.h"

// CRC helpers used by the Unitree RS-485 protocols.
#include "unitreeMotor/include/crc32.h"
#include "unitreeMotor/include/crc_ccitt.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
// The official examples use 6.2832 instead of a higher-precision constant.
// Keep the same value here to avoid tiny numeric differences when converting.
constexpr float kTwoPi = 6.2832f;

template <typename T>
inline T clamp_cast(long long v, long long lo, long long hi) {
    v = std::max(lo, std::min(v, hi));
    return static_cast<T>(v);
}
}  // namespace

// ------------------------------- MotorCmd ---------------------------------

uint8_t* MotorCmd::get_motor_send_data() {
    switch (motorType) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2:
            return reinterpret_cast<uint8_t*>(&GO_M8010_6_motor_send_data);
        case MotorType::A1:
        case MotorType::B1:
        default:
            return reinterpret_cast<uint8_t*>(&A1B1_motor_send_data);
    }
}

void MotorCmd::modify_data(MotorCmd* motor_s) {
    if (motor_s == nullptr) {
        return;
    }

    switch (motor_s->motorType) {
        case MotorType::GO_M8010_6: {
            // Full packed frame size.
            static_assert(sizeof(ControlData_t) == 17, "Unexpected GO control frame size");
            motor_s->hex_len = static_cast<int>(sizeof(ControlData_t));

            // Frame header (command)
            GO_M8010_6_motor_send_data.head[0] = 0xFE;
            GO_M8010_6_motor_send_data.head[1] = 0xEE;

            // Mode/id
            GO_M8010_6_motor_send_data.mode.id = static_cast<uint8_t>(motor_s->id);
            GO_M8010_6_motor_send_data.mode.status = static_cast<uint8_t>(motor_s->mode);
            GO_M8010_6_motor_send_data.mode.none = 0;

            // Command encoding (matches the public GO-M8010-6 examples)
            const long long pos_des = llround(static_cast<double>(motor_s->q) / kTwoPi * 32768.0);
            const long long spd_des = llround(static_cast<double>(motor_s->dq) / kTwoPi * 256.0);
            const long long tor_des = llround(static_cast<double>(motor_s->tau) * 256.0);

            // k_pos/k_spd are normalized in the protocol (Q15). The public examples
            // map a user gain in [0, 25.6] to [0, 1.0].
            const long long k_pos = llround(static_cast<double>(motor_s->kp) / 25.6 * 32768.0);
            const long long k_spd = llround(static_cast<double>(motor_s->kd) / 25.6 * 32768.0);

            GO_M8010_6_motor_send_data.comd.pos_des = static_cast<int32_t>(pos_des);
            GO_M8010_6_motor_send_data.comd.spd_des = clamp_cast<int16_t>(spd_des, -32768, 32767);
            GO_M8010_6_motor_send_data.comd.tor_des = clamp_cast<int16_t>(tor_des, -32768, 32767);
            GO_M8010_6_motor_send_data.comd.k_pos = clamp_cast<uint16_t>(k_pos, 0, 65535);
            GO_M8010_6_motor_send_data.comd.k_spd = clamp_cast<uint16_t>(k_spd, 0, 65535);

            GO_M8010_6_motor_send_data.CRC16 =
                crc_ccitt(0, reinterpret_cast<const uint8_t*>(&GO_M8010_6_motor_send_data),
                          sizeof(ControlData_t) - 2);
        } break;

        case MotorType::GO_2: {
            // Full packed frame size.
            static_assert(sizeof(ControlData_t) == 17, "Unexpected GO2 control frame size");
            motor_s->hex_len = static_cast<int>(sizeof(ControlData_t));

            // Frame header (command)
            GO_M8010_6_motor_send_data.head[0] = 0xFE;
            GO_M8010_6_motor_send_data.head[1] = 0xEE;

            // Mode/id
            GO_M8010_6_motor_send_data.mode.id = static_cast<uint8_t>(motor_s->id);
            GO_M8010_6_motor_send_data.mode.status = static_cast<uint8_t>(motor_s->mode);
            GO_M8010_6_motor_send_data.mode.none = 0;

            // Command encoding (matches the public GO-M8010-6 examples)
            const long long pos_des = llround(static_cast<double>(motor_s->q) / kTwoPi * 32768.0);
            const long long spd_des = llround(static_cast<double>(motor_s->dq) / kTwoPi * 256.0);
            const long long tor_des = llround(static_cast<double>(motor_s->tau) * 256.0);

            // k_pos/k_spd are normalized in the protocol (Q15). The public examples
            // map a user gain in [0, 25.6] to [0, 1.0].
            const long long k_pos = llround(static_cast<double>(motor_s->kp) / 25.6 * 32768.0);
            const long long k_spd = llround(static_cast<double>(motor_s->kd) / 25.6 * 32768.0);

            GO_M8010_6_motor_send_data.comd.pos_des = static_cast<int32_t>(pos_des);
            GO_M8010_6_motor_send_data.comd.spd_des = clamp_cast<int16_t>(spd_des, -32768, 32767);
            GO_M8010_6_motor_send_data.comd.tor_des = clamp_cast<int16_t>(tor_des, -32768, 32767);
            GO_M8010_6_motor_send_data.comd.k_pos = clamp_cast<uint16_t>(k_pos, 0, 65535);
            GO_M8010_6_motor_send_data.comd.k_spd = clamp_cast<uint16_t>(k_spd, 0, 65535);

            // GO2 CRC16 variant over the frame excluding the CRC field.
            GO_M8010_6_motor_send_data.CRC16 =
                crc_ccitt(0x2CBB, reinterpret_cast<const uint8_t*>(&GO_M8010_6_motor_send_data),
                          sizeof(ControlData_t) - 2);
        } break;

        case MotorType::A1:
        case MotorType::B1:
        default: {
            motor_s->hex_len = 34;

            // Frame header
            A1B1_motor_send_data.head.start[0] = 0xFE;
            A1B1_motor_send_data.head.start[1] = 0xEE;
            A1B1_motor_send_data.head.motorID = static_cast<uint8_t>(motor_s->id);
            A1B1_motor_send_data.head.reserved = 0;

            // Command body
            A1B1_motor_send_data.Mdata.mode = static_cast<uint8_t>(motor_s->mode);
            A1B1_motor_send_data.Mdata.ModifyBit = 0;
            A1B1_motor_send_data.Mdata.ReadBit = 0;
            A1B1_motor_send_data.Mdata.reserved = 0;
            A1B1_motor_send_data.Mdata.Modify.u32 = 0;

            const long long t = llround(static_cast<double>(motor_s->tau) * 256.0);
            const long long w = llround(static_cast<double>(motor_s->dq) * 128.0);
            const long long pos = llround(static_cast<double>(motor_s->q) * (16384.0 / kTwoPi));
            const long long kp = llround(static_cast<double>(motor_s->kp) * 2048.0);
            const long long kd = llround(static_cast<double>(motor_s->kd) * 1024.0);

            A1B1_motor_send_data.Mdata.T = clamp_cast<int16_t>(t, -32768, 32767);
            A1B1_motor_send_data.Mdata.W = clamp_cast<int16_t>(w, -32768, 32767);
            A1B1_motor_send_data.Mdata.Pos = static_cast<int32_t>(pos);
            A1B1_motor_send_data.Mdata.K_P = clamp_cast<int16_t>(kp, -32768, 32767);
            A1B1_motor_send_data.Mdata.K_W = clamp_cast<int16_t>(kd, -32768, 32767);
            A1B1_motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
            A1B1_motor_send_data.Mdata.LowHzMotorCmdByte = 0;
            A1B1_motor_send_data.Mdata.Res[0].u32 = motor_s->Res.u32;

            // CRC32 (Unitree uses a word-based CRC32 implementation).
            // Command packet without CRC is 30 bytes; the implementation processes
            // floor(30/4)=7 words (28 bytes) and ignores the 2-byte remainder.
            A1B1_motor_send_data.CRCdata.u32 =
                crc32_core(reinterpret_cast<uint32_t*>(&A1B1_motor_send_data), 7);
        } break;
    }
}

// ------------------------------- MotorData --------------------------------

uint8_t* MotorData::get_motor_recv_data() {
    switch (motorType) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2:
            return reinterpret_cast<uint8_t*>(&GO_M8010_6_motor_recv_data);
        case MotorType::A1:
        case MotorType::B1:
        default:
            return reinterpret_cast<uint8_t*>(&A1B1_motor_recv_data);
    }
}

bool MotorData::extract_data(MotorData* motor_r) {
    if (motor_r == nullptr) {
        return false;
    }

    motor_r->correct = 0;

    switch (motor_r->motorType) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2: {
            // Full packed frame size.
            static_assert(sizeof(MotorData_t) == 16, "Unexpected GO feedback frame size");
            motor_r->hex_len = static_cast<int>(sizeof(MotorData_t));

            // Validate header and CRC.
            const uint8_t h0 = GO_M8010_6_motor_recv_data.head[0];
            const uint8_t h1 = GO_M8010_6_motor_recv_data.head[1];

            const bool header_ok = ((h0 == 0xFD || h0 == 0xFE) && h1 == 0xEE);

            const size_t crc_len = sizeof(MotorData_t) - 2;
            const uint16_t crc_expect =
                (motor_r->motorType == MotorType::GO_2)
                    ? crc_ccitt(0x2CBB, reinterpret_cast<const uint8_t*>(&GO_M8010_6_motor_recv_data),
                                crc_len)
                    : crc_ccitt(0, reinterpret_cast<const uint8_t*>(&GO_M8010_6_motor_recv_data),
                                crc_len);
            const bool crc_ok = (GO_M8010_6_motor_recv_data.CRC16 == crc_expect);

            if (!header_ok || !crc_ok) {
                return false;
            }

            motor_r->correct = 1;
            motor_r->motor_id = GO_M8010_6_motor_recv_data.mode.id;
            motor_r->mode = GO_M8010_6_motor_recv_data.mode.status;
            motor_r->temp = static_cast<int>(GO_M8010_6_motor_recv_data.fbk.temp);
            motor_r->merror = static_cast<int>(GO_M8010_6_motor_recv_data.fbk.MError);
            motor_r->footForce = static_cast<int>(GO_M8010_6_motor_recv_data.fbk.force);

            // Decode feedback
            motor_r->q = static_cast<float>(GO_M8010_6_motor_recv_data.fbk.pos) * kTwoPi / 32768.0f;
            motor_r->dq = static_cast<float>(GO_M8010_6_motor_recv_data.fbk.speed) * kTwoPi / 256.0f;
            motor_r->tau = static_cast<float>(GO_M8010_6_motor_recv_data.fbk.torque) / 256.0f;

            // The GO feedback struct does not include IMU data.
            std::fill(std::begin(motor_r->gyro), std::end(motor_r->gyro), 0.0f);
            std::fill(std::begin(motor_r->acc), std::end(motor_r->acc), 0.0f);
            motor_r->Acc = 0;
            motor_r->LW = 0;
        } break;

        case MotorType::A1:
        case MotorType::B1:
        default: {
            motor_r->hex_len = 78;

            // CRC32 over the packet without CRC (74 bytes). The Unitree word-based
            // implementation processes floor(74/4)=18 words (72 bytes), ignoring
            // the final 2 bytes.
            const uint32_t crc_expect =
                crc32_core(reinterpret_cast<uint32_t*>(&A1B1_motor_recv_data), 18);
            if (crc_expect != A1B1_motor_recv_data.CRCdata.u32) {
                return false;
            }

            motor_r->correct = 1;
            motor_r->motor_id = A1B1_motor_recv_data.head.motorID;
            motor_r->mode = A1B1_motor_recv_data.Mdata.mode;
            motor_r->temp = static_cast<int>(A1B1_motor_recv_data.Mdata.Temp);
            motor_r->merror = static_cast<int>(A1B1_motor_recv_data.Mdata.MError);
            motor_r->footForce = static_cast<int>(A1B1_motor_recv_data.Mdata.Force16);

            motor_r->q = static_cast<float>(A1B1_motor_recv_data.Mdata.Pos) * kTwoPi / 16384.0f;
            motor_r->dq = static_cast<float>(A1B1_motor_recv_data.Mdata.W) / 128.0f;
            motor_r->tau = static_cast<float>(A1B1_motor_recv_data.Mdata.T) / 256.0f;

            for (int i = 0; i < 3; ++i) {
                motor_r->gyro[i] = static_cast<float>(A1B1_motor_recv_data.Mdata.gyro[i]);
                motor_r->acc[i] = static_cast<float>(A1B1_motor_recv_data.Mdata.acc[i]);
            }

            motor_r->Acc = static_cast<int>(A1B1_motor_recv_data.Mdata.Acc);
            motor_r->LW = static_cast<int>(A1B1_motor_recv_data.Mdata.LW);
        } break;
    }

    return motor_r->correct != 0;
}

// ------------------------------ Utilities ---------------------------------

int queryMotorMode(MotorType type, MotorMode mode) {
    switch (type) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2:
            switch (mode) {
                case MotorMode::BRAKE:
                    return 0;
                case MotorMode::FOC:
                    return 1;
                case MotorMode::CALIBRATE:
                    return 2;
                default:
                    return 0;
            }
        case MotorType::A1:
        case MotorType::B1:
        default:
            switch (mode) {
                case MotorMode::BRAKE:
                    return 0;   // stop/lock
                case MotorMode::FOC:
                    return 10;  // closed-loop servo
                case MotorMode::CALIBRATE:
                    return 12;  // matches upstream SDK (0x0C)
                default:
                    return 0;
            }
    }
}

float queryGearRatio(MotorType type) {
    switch (type) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2:
            return 6.33f;
        case MotorType::A1:
        case MotorType::B1:
            return 9.1f;
        default:
            return 1.0f;
    }
}

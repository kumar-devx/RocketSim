#pragma once

#include "../sim/world.cuh"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define closesocket ::close
#endif

#include <cstdint>
#include <cstdio>
#include <string>

namespace vis {

constexpr uint16_t TARGET_PORT = 9273;

class StateSender {
private:
    SOCKET sock;
    sockaddr_in target_addr;
    bool initialised;
    std::string j;

    void jf(float v) { char b[32]; snprintf(b, sizeof(b), "%.6g", v); j += b; }

    void jv(const Vec3& v) {
        j += "["; jf(v.x); j += ","; jf(v.y); j += ","; jf(v.z); j += "]";
    }

    void j_phys(const Vec3& pos, const Mat3& rot, const Vec3& vel, const Vec3& ang) {
        j += "\"pos\":";   jv(pos);
        j += ",\"forward\":"; jv(rot.x_axis);
        j += ",\"up\":";   jv(rot.z_axis);
        j += ",\"vel\":";  jv(vel);
        j += ",\"ang_vel\":"; jv(ang);
    }

public:
    StateSender() : sock(INVALID_SOCKET), initialised(false) {}
    ~StateSender() { close(); }

    bool init() {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
#endif
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) return false;

        target_addr = {};
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(TARGET_PORT);
        inet_pton(AF_INET, "127.0.0.1", &target_addr.sin_addr);

        initialised = true;
        j.reserve(8192);
        return true;
    }

    void close() {
        if (sock != INVALID_SOCKET) {
            closesocket(sock);
            sock = INVALID_SOCKET;
        }
#ifdef _WIN32
        WSACleanup();
#endif
        initialised = false;
    }

    bool is_initialised() const { return initialised; }

    void send(const Arena& arena) {
        if (!initialised) return;

        j.clear();
        j += "{\"gamemode\":\"soccar\",\"ball_phys\":{";

        const RigidBody& ball_body = arena.world.bodies[arena.ball_body_idx];
        Vec3 ball_pos = ball_body.collision.transform.translation * physics::INV_SCALE;
        Mat3 ball_rot = ball_body.collision.transform.matrix3;
        Vec3 ball_vel = ball_body.linear_vel * physics::INV_SCALE;
        Vec3 ball_ang = ball_body.angular_vel;

        j_phys(ball_pos, ball_rot, ball_vel, ball_ang);

        j += "},\"cars\":[";

        for (int c = 0; c < arena.num_cars; c++) {
            if (c > 0) j += ",";
            j += "{";

            const Car& car = arena.cars[c];
            const CarState& s = car.state;

            j += "\"team_num\":";
            j += (car.team == 0) ? "0" : "1";

            j += ",\"phys\":{";
            j_phys(s.pos, s.rot, s.vel, s.ang_vel);
            j += "}";

            j += ",\"boost_amount\":";
            jf(s.boost_amount);

            j += ",\"on_ground\":";
            j += s.is_on_ground ? "true" : "false";

            j += ",\"is_demoed\":";
            j += s.is_demoed ? "true" : "false";

            j += ",\"has_flipped_or_double_jumped\":";
            j += (s.has_flipped || s.has_double_jumped) ? "true" : "false";

            j += ",\"controls\":{";
            j += "\"throttle\":"; jf(s.controls.throttle);
            j += ",\"steer\":";   jf(s.controls.steer);
            j += ",\"pitch\":";   jf(s.controls.pitch);
            j += ",\"yaw\":";     jf(s.controls.yaw);
            j += ",\"roll\":";    jf(s.controls.roll);
            j += ",\"boost\":";   j += s.controls.boost ? "true" : "false";
            j += ",\"jump\":";    j += s.controls.jump ? "true" : "false";
            j += ",\"handbrake\":"; j += s.controls.handbrake ? "true" : "false";
            j += "}";

            j += "}";
        }

        j += "],\"boost_pad_states\":[";
        for (int p = 0; p < TOTAL_PADS; p++) {
            if (p > 0) j += ",";
            j += arena.pads[p].state.is_active ? "true" : "false";
        }
        j += "]}";

        sendto(sock, j.c_str(), static_cast<int>(j.size()), 0,
               reinterpret_cast<sockaddr*>(&target_addr), sizeof(target_addr));
    }
};

}

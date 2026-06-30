// simulate_bot.cpp -- C++ bot simulator for lidar-slam-dashboard.
//
// Stands in for the real stm32-lidar-firmware + esp32-raw-mac-radio chain:
// it ray-casts a sweeping LIDAR head against a predefined map (see
// ../maps/room_with_pillar.map) and speaks the exact same wire format as
// app/contract.py over UDP, so the dashboard cannot tell this apart from
// real hardware. It also answers control_command frames (start/stop/
// set_sweep_range/ping) the same way base-radio would, by acking on the
// telemetry socket -- see DATA_CONTRACT.md's "control_ack flows in the
// telemetry direction" rule.
//
// This file deliberately re-implements CRC16/frame pack/unpack from
// scratch rather than sharing code with app/contract.py -- that's the
// established pattern across this whole repo (every language boundary
// gets its own independent implementation of the same byte-for-byte
// contract, so a parity bug in one implementation can't hide inside a
// shared library). See DATA_CONTRACT.md for the canonical spec this must
// match.
//
// Build (from this directory, in WSL or any Linux box with g++):
//   make
//   ./simulate_bot                       # uses ../maps/room_with_pillar.map
//   ./simulate_bot --host=127.0.0.1 --port=5005 --listen-port=5006
//
// See ../README.md for full build/run instructions.

#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

// ===================== data contract (mirrors app/contract.py) =====================

namespace contract {

constexpr int FRAME_LEN = 14;     // sof + type + seq(2) + payload(10)
constexpr int WIRE_LEN = 16;      // FRAME_LEN + crc16(2)
constexpr int PAYLOAD_LEN = 10;

constexpr uint8_t SOF_TELEMETRY = 0xAA;
constexpr uint8_t SOF_CONTROL = 0xAB;

constexpr uint16_t OUT_OF_RANGE = 0xFFFF;

enum Type : uint8_t {
    SCAN_SAMPLE = 0x01,
    SCAN_COMPLETE = 0x02,
    HEALTH_STATUS = 0x03,
    CONTROL_COMMAND = 0x10,
    CONTROL_ACK = 0x11,
};

enum CmdId : uint8_t {
    START_SCAN = 0x01,
    STOP_SCAN = 0x02,
    SET_SWEEP_RANGE = 0x03,
    PING = 0x04,
};

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no input/output reflection,
// xorout=0x0000. Catalogue check value for ASCII "123456789" is 0x29B1 --
// verified against app/contract.py's crc16() during development.
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

// Explicit byte-level put/get helpers rather than struct+memcpy, so this
// is correct regardless of host endianness (the wire format fixes field
// byte order independent of the host CPU).
inline void put_u16le(uint8_t *buf, uint16_t v) {
    buf[0] = static_cast<uint8_t>(v & 0xFF);
    buf[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline uint16_t get_u16le(const uint8_t *buf) {
    return static_cast<uint16_t>(buf[0] | (buf[1] << 8));
}
inline void put_u32le(uint8_t *buf, uint32_t v) {
    buf[0] = static_cast<uint8_t>(v & 0xFF);
    buf[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
inline uint32_t get_u32le(const uint8_t *buf) {
    return static_cast<uint32_t>(buf[0]) | (static_cast<uint32_t>(buf[1]) << 8) |
           (static_cast<uint32_t>(buf[2]) << 16) | (static_cast<uint32_t>(buf[3]) << 24);
}
// crc16 trailer is the one big-endian field in the wire format (see
// DATA_CONTRACT.md's wire format diagram).
inline void put_u16be(uint8_t *buf, uint16_t v) {
    buf[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(v & 0xFF);
}

std::vector<uint8_t> pack_frame(uint8_t sof, uint8_t type, uint16_t seq,
                                 const uint8_t payload[PAYLOAD_LEN]) {
    std::vector<uint8_t> wire(WIRE_LEN, 0);
    wire[0] = sof;
    wire[1] = type;
    put_u16le(&wire[2], seq);
    std::memcpy(&wire[4], payload, PAYLOAD_LEN);
    uint16_t crc = crc16(wire.data(), FRAME_LEN);
    put_u16be(&wire[FRAME_LEN], crc);
    return wire;
}

std::vector<uint8_t> encode_scan_sample(uint16_t angle_cdeg, uint16_t distance_mm,
                                         uint32_t timestamp_ms, uint16_t seq) {
    uint8_t payload[PAYLOAD_LEN] = {0};
    put_u16le(&payload[0], angle_cdeg);
    put_u16le(&payload[2], distance_mm);
    put_u32le(&payload[4], timestamp_ms);
    return pack_frame(SOF_TELEMETRY, Type::SCAN_SAMPLE, seq, payload);
}

std::vector<uint8_t> encode_scan_complete(uint8_t sweep_dir, uint32_t timestamp_ms, uint16_t seq) {
    uint8_t payload[PAYLOAD_LEN] = {0};
    payload[0] = sweep_dir;
    put_u32le(&payload[2], timestamp_ms);
    return pack_frame(SOF_TELEMETRY, Type::SCAN_COMPLETE, seq, payload);
}

std::vector<uint8_t> encode_health_status(uint16_t fault_flags, uint16_t battery_mv,
                                           uint32_t timestamp_ms, uint16_t seq) {
    uint8_t payload[PAYLOAD_LEN] = {0};
    put_u16le(&payload[0], fault_flags);
    put_u16le(&payload[2], battery_mv);
    put_u32le(&payload[4], timestamp_ms);
    return pack_frame(SOF_TELEMETRY, Type::HEALTH_STATUS, seq, payload);
}

std::vector<uint8_t> encode_control_ack(uint8_t cmd_id, uint8_t status, uint32_t timestamp_ms,
                                         uint16_t seq) {
    uint8_t payload[PAYLOAD_LEN] = {0};
    payload[0] = cmd_id;
    payload[1] = status;
    put_u32le(&payload[6], timestamp_ms);
    return pack_frame(SOF_TELEMETRY, Type::CONTROL_ACK, seq, payload);
}

struct ControlCommand {
    bool valid = false;
    uint8_t cmd_id = 0;
    uint16_t param1 = 0;
    uint16_t param2 = 0;
    uint32_t timestamp_ms = 0;
};

// Decodes an inbound control_command datagram. Returns valid=false on any
// length/CRC/sof/type mismatch -- callers must not ack a frame that fails
// this check (there's no well-formed cmd_id to ack).
ControlCommand decode_control_command(const uint8_t *wire, size_t len) {
    ControlCommand cc;
    if (len != WIRE_LEN) return cc;
    uint16_t expected_crc = crc16(wire, FRAME_LEN);
    uint16_t received_crc = static_cast<uint16_t>((wire[FRAME_LEN] << 8) | wire[FRAME_LEN + 1]);
    if (expected_crc != received_crc) return cc;
    if (wire[0] != SOF_CONTROL || wire[1] != Type::CONTROL_COMMAND) return cc;
    const uint8_t *payload = wire + 4;
    cc.cmd_id = payload[0];
    cc.param1 = get_u16le(&payload[2]);
    cc.param2 = get_u16le(&payload[4]);
    cc.timestamp_ms = get_u32le(&payload[6]);
    cc.valid = true;
    return cc;
}

} // namespace contract

// ===================== predefined map (../maps/room_with_pillar.map) =====================

namespace simmap {

struct Segment {
    double x1, y1, x2, y2;
};

struct Map {
    double max_range_mm = 2000.0;
    std::vector<Segment> walls;
};

static std::string strip_comment_and_trim(const std::string &line) {
    std::string s = line;
    size_t hash = s.find('#');
    if (hash != std::string::npos) s = s.substr(0, hash);
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

Map load(const std::string &path) {
    Map m;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "error: cannot open map file '%s'\n", path.c_str());
        std::exit(1);
    }
    std::string raw_line;
    while (std::getline(f, raw_line)) {
        std::string line = strip_comment_and_trim(raw_line);
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        if (keyword == "SENSOR_MAX_RANGE_MM") {
            iss >> m.max_range_mm;
        } else if (keyword == "WALL") {
            Segment s{};
            iss >> s.x1 >> s.y1 >> s.x2 >> s.y2;
            m.walls.push_back(s);
        } else {
            std::fprintf(stderr, "warning: unrecognized map directive '%s', ignored\n", keyword.c_str());
        }
    }
    return m;
}

// Ray (from origin, unit direction (dx,dy)) vs. segment (a..b) intersection.
// Returns true and sets t_out (distance along the ray, in the segment's
// units) if they intersect with t>=0 and the segment parameter in [0,1].
//
// Derivation: solve origin + t*(dx,dy) = a + u*(b-a) via Cramer's rule.
// See simulator design notes for the full derivation; this is the
// standard ray/segment intersection formula.
bool ray_intersect(double dx, double dy, const Segment &seg, double &t_out) {
    double sx = seg.x2 - seg.x1;
    double sy = seg.y2 - seg.y1;
    double denom = dx * sy - dy * sx;
    if (std::fabs(denom) < 1e-12) return false; // parallel (or degenerate segment)
    double qx = seg.x1;
    double qy = seg.y1;
    double t = (qx * sy - qy * sx) / denom;
    double u = (qx * dy - qy * dx) / denom;
    if (t < 0.0) return false;
    if (u < -1e-9 || u > 1.0 + 1e-9) return false;
    t_out = t;
    return true;
}

// Casts one ray at angle_deg (0=+x/right, 90=+y/forward, 180=-x/left -- see
// the map file's header comment and app/occupancy_grid.py's
// integrate_point(), which this must match) and returns the distance to
// the nearest wall in millimeters, or contract::OUT_OF_RANGE if nothing is
// hit within max_range_mm.
uint16_t raycast_mm(const Map &map, double angle_deg) {
    double rad = angle_deg * M_PI / 180.0;
    double dx = std::cos(rad);
    double dy = std::sin(rad);
    double best_t = std::numeric_limits<double>::infinity();
    for (const auto &seg : map.walls) {
        double t;
        if (ray_intersect(dx, dy, seg, t) && t < best_t) {
            best_t = t;
        }
    }
    if (!std::isfinite(best_t)) return contract::OUT_OF_RANGE;
    double mm = best_t * 1000.0;
    if (mm > map.max_range_mm) return contract::OUT_OF_RANGE;
    return static_cast<uint16_t>(std::lround(mm));
}

} // namespace simmap

// ===================== UDP plumbing =====================

namespace net {

int make_send_socket() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::perror("socket (send)");
        std::exit(1);
    }
    return fd;
}

int make_recv_socket(int port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::perror("socket (recv)");
        std::exit(1);
    }
    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::perror("bind (recv)");
        std::exit(1);
    }
    return fd;
}

void send_to(int fd, const std::string &host, int port, const std::vector<uint8_t> &wire) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr, "error: invalid dashboard host '%s' (must be a dotted IPv4 address, "
                              "e.g. 127.0.0.1)\n", host.c_str());
        std::exit(1);
    }
    ::sendto(fd, wire.data(), wire.size(), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
}

} // namespace net

// ===================== current time helper =====================

static uint32_t now_ms() {
    timeval tv;
    ::gettimeofday(&tv, nullptr);
    return static_cast<uint32_t>(tv.tv_sec * 1000ull + tv.tv_usec / 1000ull);
}

// ===================== sweep state =====================

struct SweepState {
    double min_deg = 0.0;
    double max_deg = 180.0;
    double step_deg = 0.5;
    double current_deg = 0.0;
    int direction = +1; // +1 = increasing (toward max_deg), -1 = decreasing
    bool running = true;
    uint16_t seq = 0;
};

// ===================== CLI options =====================

struct Options {
    std::string map_path = "../maps/room_with_pillar.map";
    std::string dashboard_host = "127.0.0.1";
    int dashboard_port = 5005;
    int listen_port = 5006;
    double step_deg = 0.5;
    int sample_interval_ms = 20;
    int health_interval_ms = 2000;
};

static bool starts_with(const std::string &s, const std::string &prefix) {
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

Options parse_args(int argc, char **argv) {
    Options opt;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (starts_with(arg, "--host=")) {
            opt.dashboard_host = arg.substr(7);
        } else if (starts_with(arg, "--port=")) {
            opt.dashboard_port = std::atoi(arg.substr(7).c_str());
        } else if (starts_with(arg, "--listen-port=")) {
            opt.listen_port = std::atoi(arg.substr(14).c_str());
        } else if (starts_with(arg, "--step-deg=")) {
            opt.step_deg = std::atof(arg.substr(11).c_str());
        } else if (starts_with(arg, "--sample-interval-ms=")) {
            opt.sample_interval_ms = std::atoi(arg.substr(21).c_str());
        } else if (starts_with(arg, "--health-interval-ms=")) {
            opt.health_interval_ms = std::atoi(arg.substr(21).c_str());
        } else if (arg == "--help" || arg == "-h") {
            std::printf(
                "usage: simulate_bot [map_file] [--host=127.0.0.1] [--port=5005]\n"
                "                    [--listen-port=5006] [--step-deg=0.5]\n"
                "                    [--sample-interval-ms=20] [--health-interval-ms=2000]\n");
            std::exit(0);
        } else if (!starts_with(arg, "--")) {
            opt.map_path = arg;
        } else {
            std::fprintf(stderr, "warning: unrecognized option '%s', ignored\n", arg.c_str());
        }
    }
    opt.step_deg = std::max(0.05, opt.step_deg);
    return opt;
}

// ===================== main =====================

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    simmap::Map map = simmap::load(opt.map_path);
    std::printf("[simulate_bot/cpp] loaded map '%s' (%zu wall segments, max range %.0f mm)\n",
                opt.map_path.c_str(), map.walls.size(), map.max_range_mm);
    std::printf("[simulate_bot/cpp] telemetry -> %s:%d, control listen on :%d\n",
                opt.dashboard_host.c_str(), opt.dashboard_port, opt.listen_port);

    int send_fd = net::make_send_socket();
    int recv_fd = net::make_recv_socket(opt.listen_port);

    SweepState sweep;
    sweep.step_deg = opt.step_deg;

    uint16_t battery_mv = 8000;
    bool battery_discharging = true;

    auto send_wire = [&](const std::vector<uint8_t> &wire) {
        net::send_to(send_fd, opt.dashboard_host, opt.dashboard_port, wire);
    };

    auto handle_control_datagram = [&](const uint8_t *buf, size_t n) {
        contract::ControlCommand cc = contract::decode_control_command(buf, n);
        if (!cc.valid) {
            std::fprintf(stderr, "[simulate_bot/cpp] dropped invalid/CRC-failed control datagram (%zu bytes)\n", n);
            return;
        }
        uint8_t status = 0; // 0 = ok, 1 = rejected
        switch (cc.cmd_id) {
            case contract::CmdId::START_SCAN:
                sweep.running = true;
                std::printf("[simulate_bot/cpp] START_SCAN -> running\n");
                break;
            case contract::CmdId::STOP_SCAN:
                sweep.running = false;
                std::printf("[simulate_bot/cpp] STOP_SCAN -> paused\n");
                break;
            case contract::CmdId::SET_SWEEP_RANGE: {
                double new_min = std::max(0.0, cc.param1 / 100.0);
                double new_max = std::min(180.0, cc.param2 / 100.0);
                if (new_min >= new_max) {
                    status = 1;
                    std::fprintf(stderr, "[simulate_bot/cpp] SET_SWEEP_RANGE rejected: min(%.1f) >= max(%.1f)\n",
                                 new_min, new_max);
                } else {
                    sweep.min_deg = new_min;
                    sweep.max_deg = new_max;
                    sweep.current_deg = std::clamp(sweep.current_deg, new_min, new_max);
                    std::printf("[simulate_bot/cpp] SET_SWEEP_RANGE -> [%.1f, %.1f] deg\n", new_min, new_max);
                }
                break;
            }
            case contract::CmdId::PING:
                std::printf("[simulate_bot/cpp] PING\n");
                break;
            default:
                status = 1;
                std::fprintf(stderr, "[simulate_bot/cpp] unknown cmd_id 0x%02x\n", cc.cmd_id);
                break;
        }
        send_wire(contract::encode_control_ack(cc.cmd_id, status, now_ms(), sweep.seq++));
    };

    long long next_sample_ms = 0;
    long long next_health_ms = 0;
    auto monotonic_ms_now = []() -> long long {
        timeval tv;
        ::gettimeofday(&tv, nullptr);
        return static_cast<long long>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    };
    next_sample_ms = monotonic_ms_now();
    next_health_ms = monotonic_ms_now();

    while (true) {
        long long now = monotonic_ms_now();
        long long wait_ms = std::min(next_sample_ms, next_health_ms) - now;
        if (wait_ms < 0) wait_ms = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(recv_fd, &rfds);
        timeval tv;
        tv.tv_sec = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;
        int rv = ::select(recv_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (rv > 0 && FD_ISSET(recv_fd, &rfds)) {
            uint8_t buf[64];
            for (;;) {
                sockaddr_in src{};
                socklen_t srclen = sizeof(src);
                ssize_t n = ::recvfrom(recv_fd, buf, sizeof(buf), MSG_DONTWAIT,
                                        reinterpret_cast<sockaddr *>(&src), &srclen);
                if (n <= 0) break;
                handle_control_datagram(buf, static_cast<size_t>(n));
            }
        }

        now = monotonic_ms_now();

        if (now >= next_health_ms) {
            // Synthetic battery model purely for demo visuals: drifts down
            // from 8000mV to 6800mV then "recharges" back up, repeating --
            // not a real discharge curve, just something that visibly moves.
            if (battery_discharging) {
                battery_mv = static_cast<uint16_t>(battery_mv - 20);
                if (battery_mv <= 6800) battery_discharging = false;
            } else {
                battery_mv = static_cast<uint16_t>(battery_mv + 20);
                if (battery_mv >= 8000) battery_discharging = true;
            }
            send_wire(contract::encode_health_status(0 /* fault_flags: nominal */, battery_mv,
                                                       now_ms(), sweep.seq++));
            next_health_ms = now + opt.health_interval_ms;
        }

        if (now >= next_sample_ms) {
            if (sweep.running) {
                uint16_t distance_mm = simmap::raycast_mm(map, sweep.current_deg);
                uint16_t angle_cdeg = static_cast<uint16_t>(std::lround(sweep.current_deg * 100.0));
                send_wire(contract::encode_scan_sample(angle_cdeg, distance_mm, now_ms(), sweep.seq++));

                sweep.current_deg += sweep.step_deg * sweep.direction;
                if (sweep.current_deg >= sweep.max_deg) {
                    sweep.current_deg = sweep.max_deg;
                    send_wire(contract::encode_scan_complete(0 /* forward */, now_ms(), sweep.seq++));
                    sweep.direction = -1;
                } else if (sweep.current_deg <= sweep.min_deg) {
                    sweep.current_deg = sweep.min_deg;
                    send_wire(contract::encode_scan_complete(1 /* reverse */, now_ms(), sweep.seq++));
                    sweep.direction = +1;
                }
            }
            next_sample_ms = now + opt.sample_interval_ms;
        }
    }

    return 0;
}

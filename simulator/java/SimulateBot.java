import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

/**
 * SimulateBot -- Java bot simulator for lidar-slam-dashboard.
 *
 * Independent re-implementation of the same job as ../cpp/simulate_bot.cpp:
 * ray-casts a sweeping LIDAR head against the predefined map
 * (../maps/room_with_pillar.map) and speaks app/contract.py's exact wire
 * format over UDP, including answering control_command frames
 * (start/stop/set_sweep_range/ping) by acking on the telemetry socket --
 * see DATA_CONTRACT.md's "control_ack flows in the telemetry direction"
 * rule.
 *
 * Deliberately does NOT share code with the C++ simulator or the Python
 * dashboard -- every language boundary in this repo gets its own
 * from-scratch implementation of the same byte-for-byte contract, so a
 * parity bug in one can't hide behind a shared library. See
 * DATA_CONTRACT.md for the canonical spec this must match.
 *
 * Build & run (from this directory, in WSL or any box with a JDK):
 *   javac SimulateBot.java
 *   java SimulateBot
 *   java SimulateBot --host=127.0.0.1 --port=5005 --listen-port=5006
 *
 * See ../README.md for full instructions.
 */
public final class SimulateBot {

    // ===================== data contract (mirrors app/contract.py) =====================

    static final int FRAME_LEN = 14;   // sof + type + seq(2) + payload(10)
    static final int WIRE_LEN = 16;    // FRAME_LEN + crc16(2)
    static final int PAYLOAD_LEN = 10;

    static final int SOF_TELEMETRY = 0xAA;
    static final int SOF_CONTROL = 0xAB;

    static final int OUT_OF_RANGE = 0xFFFF;

    static final int TYPE_SCAN_SAMPLE = 0x01;
    static final int TYPE_SCAN_COMPLETE = 0x02;
    static final int TYPE_HEALTH_STATUS = 0x03;
    static final int TYPE_CONTROL_COMMAND = 0x10;
    static final int TYPE_CONTROL_ACK = 0x11;

    static final int CMD_START_SCAN = 0x01;
    static final int CMD_STOP_SCAN = 0x02;
    static final int CMD_SET_SWEEP_RANGE = 0x03;
    static final int CMD_PING = 0x04;

    /**
     * CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no input/output
     * reflection, xorout=0x0000. Catalogue check value for ASCII
     * "123456789" is 0x29B1 -- verified against app/contract.py's crc16()
     * during development.
     */
    static int crc16(byte[] data, int len) {
        int crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= (data[i] & 0xFF) << 8;
            for (int bit = 0; bit < 8; bit++) {
                if ((crc & 0x8000) != 0) {
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
                } else {
                    crc = (crc << 1) & 0xFFFF;
                }
            }
        }
        return crc & 0xFFFF;
    }

    // Explicit byte-level put/get helpers (rather than relying on
    // ByteBuffer's order() defaults everywhere) so each field's endianness
    // is visible at the call site and matches the wire format exactly:
    // every multi-byte payload field is little-endian; only the trailing
    // crc16 is big-endian.
    static void putU16LE(byte[] buf, int off, int v) {
        buf[off] = (byte) (v & 0xFF);
        buf[off + 1] = (byte) ((v >> 8) & 0xFF);
    }

    static int getU16LE(byte[] buf, int off) {
        return (buf[off] & 0xFF) | ((buf[off + 1] & 0xFF) << 8);
    }

    static void putU32LE(byte[] buf, int off, long v) {
        buf[off] = (byte) (v & 0xFF);
        buf[off + 1] = (byte) ((v >> 8) & 0xFF);
        buf[off + 2] = (byte) ((v >> 16) & 0xFF);
        buf[off + 3] = (byte) ((v >> 24) & 0xFF);
    }

    static long getU32LE(byte[] buf, int off) {
        return (buf[off] & 0xFFL) | ((buf[off + 1] & 0xFFL) << 8)
                | ((buf[off + 2] & 0xFFL) << 16) | ((buf[off + 3] & 0xFFL) << 24);
    }

    static void putU16BE(byte[] buf, int off, int v) {
        buf[off] = (byte) ((v >> 8) & 0xFF);
        buf[off + 1] = (byte) (v & 0xFF);
    }

    static byte[] packFrame(int sof, int type, int seq, byte[] payload) {
        byte[] wire = new byte[WIRE_LEN];
        wire[0] = (byte) sof;
        wire[1] = (byte) type;
        putU16LE(wire, 2, seq);
        System.arraycopy(payload, 0, wire, 4, PAYLOAD_LEN);
        int crc = crc16(wire, FRAME_LEN);
        putU16BE(wire, FRAME_LEN, crc);
        return wire;
    }

    static byte[] encodeScanSample(int angleCdeg, int distanceMm, long timestampMs, int seq) {
        byte[] payload = new byte[PAYLOAD_LEN];
        putU16LE(payload, 0, angleCdeg);
        putU16LE(payload, 2, distanceMm);
        putU32LE(payload, 4, timestampMs);
        return packFrame(SOF_TELEMETRY, TYPE_SCAN_SAMPLE, seq, payload);
    }

    static byte[] encodeScanComplete(int sweepDir, long timestampMs, int seq) {
        byte[] payload = new byte[PAYLOAD_LEN];
        payload[0] = (byte) sweepDir;
        putU32LE(payload, 2, timestampMs);
        return packFrame(SOF_TELEMETRY, TYPE_SCAN_COMPLETE, seq, payload);
    }

    static byte[] encodeHealthStatus(int faultFlags, int batteryMv, long timestampMs, int seq) {
        byte[] payload = new byte[PAYLOAD_LEN];
        putU16LE(payload, 0, faultFlags);
        putU16LE(payload, 2, batteryMv);
        putU32LE(payload, 4, timestampMs);
        return packFrame(SOF_TELEMETRY, TYPE_HEALTH_STATUS, seq, payload);
    }

    static byte[] encodeControlAck(int cmdId, int status, long timestampMs, int seq) {
        byte[] payload = new byte[PAYLOAD_LEN];
        payload[0] = (byte) cmdId;
        payload[1] = (byte) status;
        putU32LE(payload, 6, timestampMs);
        return packFrame(SOF_TELEMETRY, TYPE_CONTROL_ACK, seq, payload);
    }

    /** Decoded control_command, or {@code valid=false} if the datagram failed length/CRC/sof/type checks. */
    static final class ControlCommand {
        boolean valid = false;
        int cmdId;
        int param1;
        int param2;
        long timestampMs;
    }

    static ControlCommand decodeControlCommand(byte[] buf, int len) {
        ControlCommand cc = new ControlCommand();
        if (len != WIRE_LEN) return cc;
        int expectedCrc = crc16(buf, FRAME_LEN);
        int receivedCrc = ((buf[FRAME_LEN] & 0xFF) << 8) | (buf[FRAME_LEN + 1] & 0xFF);
        if (expectedCrc != receivedCrc) return cc;
        if ((buf[0] & 0xFF) != SOF_CONTROL || (buf[1] & 0xFF) != TYPE_CONTROL_COMMAND) return cc;
        cc.cmdId = buf[4] & 0xFF;
        cc.param1 = getU16LE(buf, 6);
        cc.param2 = getU16LE(buf, 8);
        cc.timestampMs = getU32LE(buf, 10);
        cc.valid = true;
        return cc;
    }

    // ===================== predefined map =====================

    static final class Segment {
        final double x1, y1, x2, y2;

        Segment(double x1, double y1, double x2, double y2) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }
    }

    static final class SimMap {
        double maxRangeMm = 2000.0;
        final List<Segment> walls = new ArrayList<>();
    }

    static SimMap loadMap(String path) {
        SimMap map = new SimMap();
        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            String rawLine;
            while ((rawLine = reader.readLine()) != null) {
                String line = rawLine;
                int hash = line.indexOf('#');
                if (hash >= 0) line = line.substring(0, hash);
                line = line.trim();
                if (line.isEmpty()) continue;
                String[] parts = line.split("\\s+");
                switch (parts[0]) {
                    case "SENSOR_MAX_RANGE_MM":
                        map.maxRangeMm = Double.parseDouble(parts[1]);
                        break;
                    case "WALL":
                        map.walls.add(new Segment(
                                Double.parseDouble(parts[1]), Double.parseDouble(parts[2]),
                                Double.parseDouble(parts[3]), Double.parseDouble(parts[4])));
                        break;
                    default:
                        System.err.println("warning: unrecognized map directive '" + parts[0] + "', ignored");
                }
            }
        } catch (IOException e) {
            System.err.println("error: cannot open map file '" + path + "': " + e.getMessage());
            System.exit(1);
        }
        return map;
    }

    /**
     * Ray (from origin, unit direction (dx,dy)) vs. segment intersection,
     * solved via Cramer's rule -- same derivation as the C++ simulator's
     * ray_intersect(). Returns NaN if there's no valid intersection
     * (t&lt;0, or the segment parameter falls outside [0,1]).
     */
    static double rayIntersectT(double dx, double dy, Segment seg) {
        double sx = seg.x2 - seg.x1;
        double sy = seg.y2 - seg.y1;
        double denom = dx * sy - dy * sx;
        if (Math.abs(denom) < 1e-12) return Double.NaN;
        double qx = seg.x1;
        double qy = seg.y1;
        double t = (qx * sy - qy * sx) / denom;
        double u = (qx * dy - qy * dx) / denom;
        if (t < 0.0) return Double.NaN;
        if (u < -1e-9 || u > 1.0 + 1e-9) return Double.NaN;
        return t;
    }

    /**
     * Casts one ray at angleDeg (0=+x/right, 90=+y/forward, 180=-x/left --
     * see the map file's header comment and app/occupancy_grid.py's
     * integrate_point(), which this must match) and returns the distance
     * to the nearest wall in millimeters, or OUT_OF_RANGE if nothing is
     * hit within maxRangeMm.
     */
    static int raycastMm(SimMap map, double angleDeg) {
        double rad = Math.toRadians(angleDeg);
        double dx = Math.cos(rad);
        double dy = Math.sin(rad);
        double bestT = Double.POSITIVE_INFINITY;
        for (Segment seg : map.walls) {
            double t = rayIntersectT(dx, dy, seg);
            if (!Double.isNaN(t) && t < bestT) bestT = t;
        }
        if (!Double.isFinite(bestT)) return OUT_OF_RANGE;
        double mm = bestT * 1000.0;
        if (mm > map.maxRangeMm) return OUT_OF_RANGE;
        return (int) Math.round(mm);
    }

    // ===================== sweep state (shared between main + listener threads) =====================

    static final class SweepState {
        double minDeg = 0.0;
        double maxDeg = 180.0;
        double stepDeg = 0.5;
        double currentDeg = 0.0;
        int direction = +1; // +1 = increasing (toward maxDeg), -1 = decreasing
        volatile boolean running = true;
    }

    /** Thread-safe monotonically increasing 16-bit sequence counter, shared by telemetry and control_ack sends. */
    static final class SeqCounter {
        private int value = 0;

        synchronized int next() {
            int v = value;
            value = (value + 1) & 0xFFFF;
            return v;
        }
    }

    // ===================== CLI options =====================

    static final class Options {
        String mapPath = "../maps/room_with_pillar.map";
        String dashboardHost = "127.0.0.1";
        int dashboardPort = 5005;
        int listenPort = 5006;
        double stepDeg = 0.5;
        int sampleIntervalMs = 20;
        int healthIntervalMs = 2000;
    }

    static Options parseArgs(String[] args) {
        Options opt = new Options();
        for (String arg : args) {
            if (arg.startsWith("--host=")) {
                opt.dashboardHost = arg.substring(7);
            } else if (arg.startsWith("--port=")) {
                opt.dashboardPort = Integer.parseInt(arg.substring(7));
            } else if (arg.startsWith("--listen-port=")) {
                opt.listenPort = Integer.parseInt(arg.substring(14));
            } else if (arg.startsWith("--step-deg=")) {
                opt.stepDeg = Double.parseDouble(arg.substring(11));
            } else if (arg.startsWith("--sample-interval-ms=")) {
                opt.sampleIntervalMs = Integer.parseInt(arg.substring(21));
            } else if (arg.startsWith("--health-interval-ms=")) {
                opt.healthIntervalMs = Integer.parseInt(arg.substring(21));
            } else if (arg.equals("--help") || arg.equals("-h")) {
                System.out.println(
                        "usage: java SimulateBot [map_file] [--host=127.0.0.1] [--port=5005]\n"
                                + "                         [--listen-port=5006] [--step-deg=0.5]\n"
                                + "                         [--sample-interval-ms=20] [--health-interval-ms=2000]");
                System.exit(0);
            } else if (!arg.startsWith("--")) {
                opt.mapPath = arg;
            } else {
                System.err.println("warning: unrecognized option '" + arg + "', ignored");
            }
        }
        opt.stepDeg = Math.max(0.05, opt.stepDeg);
        return opt;
    }

    // ===================== main =====================

    public static void main(String[] args) throws Exception {
        Options opt = parseArgs(args);

        SimMap map = loadMap(opt.mapPath);
        System.out.printf("[simulate_bot/java] loaded map '%s' (%d wall segments, max range %.0f mm)%n",
                opt.mapPath, map.walls.size(), map.maxRangeMm);
        System.out.printf("[simulate_bot/java] telemetry -> %s:%d, control listen on :%d%n",
                opt.dashboardHost, opt.dashboardPort, opt.listenPort);

        final InetAddress dashboardAddr = InetAddress.getByName(opt.dashboardHost);
        final DatagramSocket sendSocket = new DatagramSocket();
        final DatagramSocket recvSocket = new DatagramSocket(opt.listenPort);

        final SweepState sweep = new SweepState();
        sweep.stepDeg = opt.stepDeg;
        final SeqCounter seq = new SeqCounter();
        final Object sweepLock = new Object();

        final java.util.concurrent.atomic.AtomicInteger batteryMv = new java.util.concurrent.atomic.AtomicInteger(8000);
        final java.util.concurrent.atomic.AtomicBoolean batteryDischarging = new java.util.concurrent.atomic.AtomicBoolean(true);

        SendFn send = (byte[] wire) -> {
            DatagramPacket pkt = new DatagramPacket(wire, wire.length, dashboardAddr, opt.dashboardPort);
            try {
                sendSocket.send(pkt);
            } catch (IOException e) {
                System.err.println("[simulate_bot/java] send failed: " + e.getMessage());
            }
        };

        // Listener thread: blocks on recvSocket.receive(), handles each
        // control_command as it arrives, and acks on the telemetry socket
        // -- independent of the main thread's sweep/health timing loop.
        Thread listenerThread = new Thread(() -> {
            byte[] buf = new byte[64];
            while (true) {
                DatagramPacket pkt = new DatagramPacket(buf, buf.length);
                try {
                    recvSocket.receive(pkt);
                } catch (IOException e) {
                    System.err.println("[simulate_bot/java] recv failed: " + e.getMessage());
                    continue;
                }
                ControlCommand cc = decodeControlCommand(pkt.getData(), pkt.getLength());
                if (!cc.valid) {
                    System.err.println("[simulate_bot/java] dropped invalid/CRC-failed control datagram ("
                            + pkt.getLength() + " bytes)");
                    continue;
                }
                int status = 0; // 0 = ok, 1 = rejected
                synchronized (sweepLock) {
                    switch (cc.cmdId) {
                        case CMD_START_SCAN:
                            sweep.running = true;
                            System.out.println("[simulate_bot/java] START_SCAN -> running");
                            break;
                        case CMD_STOP_SCAN:
                            sweep.running = false;
                            System.out.println("[simulate_bot/java] STOP_SCAN -> paused");
                            break;
                        case CMD_SET_SWEEP_RANGE: {
                            double newMin = Math.max(0.0, cc.param1 / 100.0);
                            double newMax = Math.min(180.0, cc.param2 / 100.0);
                            if (newMin >= newMax) {
                                status = 1;
                                System.err.printf("[simulate_bot/java] SET_SWEEP_RANGE rejected: min(%.1f) >= max(%.1f)%n",
                                        newMin, newMax);
                            } else {
                                sweep.minDeg = newMin;
                                sweep.maxDeg = newMax;
                                sweep.currentDeg = Math.min(newMax, Math.max(newMin, sweep.currentDeg));
                                System.out.printf("[simulate_bot/java] SET_SWEEP_RANGE -> [%.1f, %.1f] deg%n",
                                        newMin, newMax);
                            }
                            break;
                        }
                        case CMD_PING:
                            System.out.println("[simulate_bot/java] PING");
                            break;
                        default:
                            status = 1;
                            System.err.printf("[simulate_bot/java] unknown cmd_id 0x%02x%n", cc.cmdId);
                            break;
                    }
                }
                send.send(encodeControlAck(cc.cmdId, status, System.currentTimeMillis(), seq.next()));
            }
        }, "control-listener");
        listenerThread.setDaemon(true);
        listenerThread.start();

        // Main thread: drives the sweep + health timing loop. Uses
        // wall-clock deadlines (not Thread.sleep(interval) in a tight loop)
        // so jitter from control-command handling doesn't accumulate drift.
        long nextSampleMs = System.currentTimeMillis();
        long nextHealthMs = System.currentTimeMillis();

        while (true) {
            long now = System.currentTimeMillis();

            if (now >= nextHealthMs) {
                // Synthetic battery model purely for demo visuals: drifts
                // down from 8000mV to 6800mV then "recharges" back up,
                // repeating -- not a real discharge curve, just something
                // that visibly moves on the dashboard.
                int mv = batteryMv.get();
                if (batteryDischarging.get()) {
                    mv -= 20;
                    if (mv <= 6800) batteryDischarging.set(false);
                } else {
                    mv += 20;
                    if (mv >= 8000) batteryDischarging.set(true);
                }
                batteryMv.set(mv);
                send.send(encodeHealthStatus(0 /* fault_flags: nominal */, mv, System.currentTimeMillis(), seq.next()));
                nextHealthMs = now + opt.healthIntervalMs;
            }

            if (now >= nextSampleMs) {
                synchronized (sweepLock) {
                    if (sweep.running) {
                        int distanceMm = raycastMm(map, sweep.currentDeg);
                        int angleCdeg = (int) Math.round(sweep.currentDeg * 100.0);
                        send.send(encodeScanSample(angleCdeg, distanceMm, System.currentTimeMillis(), seq.next()));

                        sweep.currentDeg += sweep.stepDeg * sweep.direction;
                        if (sweep.currentDeg >= sweep.maxDeg) {
                            sweep.currentDeg = sweep.maxDeg;
                            send.send(encodeScanComplete(0 /* forward */, System.currentTimeMillis(), seq.next()));
                            sweep.direction = -1;
                        } else if (sweep.currentDeg <= sweep.minDeg) {
                            sweep.currentDeg = sweep.minDeg;
                            send.send(encodeScanComplete(1 /* reverse */, System.currentTimeMillis(), seq.next()));
                            sweep.direction = +1;
                        }
                    }
                }
                nextSampleMs = now + opt.sampleIntervalMs;
            }

            long sleepMs = Math.min(nextSampleMs, nextHealthMs) - System.currentTimeMillis();
            if (sleepMs > 0) {
                Thread.sleep(sleepMs);
            }
        }
    }

    @FunctionalInterface
    interface SendFn {
        void send(byte[] wire);
    }
}

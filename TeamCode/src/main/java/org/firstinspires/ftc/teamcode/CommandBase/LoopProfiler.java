package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * Zero-allocation circular-buffer profiler for FTC OpMode loop phases.
 * <p>
 * Stores per-phase nanosecond timings in pre-allocated ring buffers,
 * computes rolling statistics (avg, p50, p95, p99, max) over the last
 * 1000 samples, prints stats every 30 seconds via toString(), and
 * feeds FTC Dashboard telemetry at ≤5 Hz.
 */
public class LoopProfiler {

    // ── Phase indices ──────────────────────────────────────────
    public static final int BULK_CACHE_CLEAR = 0;
    public static final int GAMEPAD_COPY = 1;
    public static final int SCHEDULER_EXECUTE = 2;
    public static final int LOOP_EX = 3;
    public static final int TELEMETRY = 4;
    public static final int TOTAL_LOOP = 5;

    // --- Subsystem Sub-Phases ---
    public static final int DRIVE_BASE = 6;
    public static final int ODOMETRY = 7;
    public static final int TURRET = 8;
    public static final int INTAKE = 9;
    public static final int APRIL_TAG = 10;

    private static final int PHASE_COUNT = 11;



    // ── Configuration ──────────────────────────────────────────
    private static final int BUFFER_SIZE = 1000;
    private static final long PRINT_INTERVAL_NS = 30_000_000_000L; // 30 s
    private static final long TELEM_INTERVAL_NS = 200_000_000L; // 5 Hz

    // ── Pre-allocated ring buffers (one per phase) ─────────────
    private final long[][] buffers = new long[PHASE_COUNT][BUFFER_SIZE];
    private final int[] writeIndex = new int[PHASE_COUNT];
    private final int[] sampleCount = new int[PHASE_COUNT];

    // ── Scratch array for percentile sorting (avoids allocation) ─
    private final long[] sortScratch = new long[BUFFER_SIZE];

    // ── Timing state ───────────────────────────────────────────
    private long phaseStartNs;
    private long totalLoopStartNs;
    private long lastPrintNs;
    private long lastTelemNs;

    // ── Stats cache (updated at print/telem interval) ──────────
    private final double[] cachedAvg = new double[PHASE_COUNT];
    private final double[] cachedP50 = new double[PHASE_COUNT];
    private final double[] cachedP95 = new double[PHASE_COUNT];
    private final double[] cachedP99 = new double[PHASE_COUNT];
    private final double[] cachedMax = new double[PHASE_COUNT];

    // ── Reusable StringBuilder ─────────────────────────────────
    private final StringBuilder sb = new StringBuilder(512);

    public LoopProfiler() {
        long now = System.nanoTime();
        lastPrintNs = now;
        lastTelemNs = now;
    }

    /** Call at the very start of loop() to begin totalLoop measurement. */
    public void startTotalLoop() {
        totalLoopStartNs = System.nanoTime();
    }

    /** Call at the very end of loop() to record totalLoop elapsed time. */
    public void endTotalLoop() {
        long elapsed = System.nanoTime() - totalLoopStartNs;
        int idx = writeIndex[TOTAL_LOOP];
        buffers[TOTAL_LOOP][idx] = elapsed;
        writeIndex[TOTAL_LOOP] = (idx + 1) % BUFFER_SIZE;
        if (sampleCount[TOTAL_LOOP] < BUFFER_SIZE) {
            sampleCount[TOTAL_LOOP]++;
        }
    }

    /** Call immediately before a phase begins. */
    public void startPhase() {
        phaseStartNs = System.nanoTime();
    }

    /** Call immediately after a phase ends; records the elapsed time. */
    public void endPhase(int phaseIndex) {
        recordDuration(phaseIndex, System.nanoTime() - phaseStartNs);
    }

    /** Manually record a duration for a phase (useful for nested/sub-profiling). */
    public void recordDuration(int phaseIndex, long durationNs) {
        int idx = writeIndex[phaseIndex];
        buffers[phaseIndex][idx] = durationNs;
        writeIndex[phaseIndex] = (idx + 1) % BUFFER_SIZE;
        if (sampleCount[phaseIndex] < BUFFER_SIZE) {
            sampleCount[phaseIndex]++;
        }
    }

    /**
     * Call once per loop, after all phases are recorded.
     * Pushes profiler stats to telemetry at ≤5 Hz and
     * logs a stats summary every 30 s.
     */


    // ── Internal statistics computation ────────────────────────

    private void computeAllStats() {
        for (int i = 0; i < PHASE_COUNT; i++) {
            computeStats(i);
        }
    }

    private void computeStats(int phaseIndex) {
        int n = sampleCount[phaseIndex];
        if (n == 0) {
            cachedAvg[phaseIndex] = 0;
            cachedP50[phaseIndex] = 0;
            cachedP95[phaseIndex] = 0;
            cachedP99[phaseIndex] = 0;
            cachedMax[phaseIndex] = 0;
            return;
        }

        // Copy into scratch for sorting
        System.arraycopy(buffers[phaseIndex], 0, sortScratch, 0, n);
        Arrays.sort(sortScratch, 0, n);

        // Compute stats
        long sum = 0;
        for (int i = 0; i < n; i++) {
            sum += sortScratch[i];
        }

        cachedAvg[phaseIndex] = (double) sum / n;
        cachedP50[phaseIndex] = sortScratch[percentileIndex(n, 50)];
        cachedP95[phaseIndex] = sortScratch[percentileIndex(n, 95)];
        cachedP99[phaseIndex] = sortScratch[percentileIndex(n, 99)];
        cachedMax[phaseIndex] = sortScratch[n - 1];
    }

    private static int percentileIndex(int n, int percentile) {
        int idx = (int) Math.ceil(n * percentile / 100.0) - 1;
        return Math.max(0, Math.min(idx, n - 1));
    }


}

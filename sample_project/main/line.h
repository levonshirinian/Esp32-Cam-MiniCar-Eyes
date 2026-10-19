// ===========================================================================
// line.h — find a white tape line in one GRAYSCALE frame.
//
// The job is deliberately small: three horizontal scan rows across the frame,
// and on each one, where is the bright stripe. No blob labelling, no filtering
// across frames, nothing that needs the previous answer to produce this one —
// so a bad frame costs one frame and never poisons the next.
//
// WHY LONGEST-RUN AND NOT A CENTROID OF THE WHOLE ROW: a plain intensity
// centroid is dragged by anything bright — a desk lamp's reflection, the edge
// of a sheet of paper, the robot's own shadow boundary. The longest contiguous
// run above threshold is the tape and only the tape, and its WIDTH is a free
// sanity check: real 19 mm tape at a known distance subtends a known band of
// pixels, so a run that is 3 px or 200 px wide is rejected rather than steered
// on.
//
// The threshold is per-row and RELATIVE (min + frac*(max-min)), not absolute,
// because the near row and the far row see very different amounts of light
// through the same lens, and because "black desk" is a different grey under
// every lamp in the house. What is fixed is the CONTRAST GATE: if the
// brightest and darkest pixel on a row differ by less than minContrast, there
// is no line there, whatever the arithmetic says.
// ===========================================================================
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
// HOST BUILD (../../sim). The detector is pure arithmetic over a byte buffer,
// so it runs off-target unchanged and the simulator tests the SAME code the
// robot flashes rather than a paraphrase of it.
#include <cstdint>
template <class T> static inline T constrain(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#endif

struct LineCfg {
    float   thrFrac     = 0.55f;   // threshold between the row's min and max
    uint8_t minContrast = 35;      // below this the row has no line at all
    // A FRACTION OF THE FRAME, not a pixel count, and that is the whole point
    // of the change: this was the only resolution-dependent constant left in
    // the detector -- rowIndex is fractional and maxWidthFrac is already a
    // fraction -- so halving the frame size to buy frame rate would silently
    // have doubled it in real terms and started rejecting the line. 0.05 is
    // exactly the 16 px it used to be, over a 320-wide frame.
    //
    // 16 px, not 3, was itself MEASURED at the end of a tape strip on a wood
    // floor: the far and mid rows returned runs of 1-8 px of floor grain,
    // which at 3 px sailed through as "the line" -- conf sat at 25-38 instead
    // of 0, haveLine stayed true, and the robot drove straight off the end at
    // full speed without ever reaching the recovery states. Contrast cannot
    // tell these apart (81-105 for grain against 113-160 for tape,
    // overlapping); width can, by 5-10x. Real 19 mm tape subtends 53 px at the
    // far row, 68 at the mid and 79 at the near, and the narrowest ever
    // measured was 28 -- so at QQVGA that is still 14-40 px against a floor of
    // 8, and the margin survives the halving.
    float   minWidthFrac = 0.05f;
    float   maxWidthFrac= 0.45f;   // of the across-axis, as a fraction
    bool    swapAxes    = false;   // scan COLUMNS instead of rows. The one
                                   // geometric unknown in the whole build: the
                                   // sensor's rows may run along either board
                                   // axis, and the debug view settles it in a
                                   // second instead of a rebuild.
    bool    nearAtHigh  = true;    // the NEAR ground is at the high depth index
    float   turnSpread  = 1.10f;   // weight on far-minus-near row offset
    float   turnSkew    = 1.00f;   // weight on the second-run skew (see below).
                                   // Below turnSpread on purpose: when both far
                                   // and near see tape, spread IS the geometry and
                                   // skew must not be able to override it.
};

struct RowHit {
    bool  ok   = false;
    float pos  = 0;      // -1 (far left) .. +1 (far right)
    int   width = 0;
    int   contrast = 0;
    int   centrePx = 0;  // for the debug view
    int   runStart = 0;  // ditto — so the view can show WHICH bright run won
    int   depthIdx = 0;
    float skew  = 0;     // -1..1, where the row's bright MASS sits relative to
                         // the run that won. 0 on a straight line; signed
                         // toward the bend at a corner. See scan().
    uint8_t runs = 0;    // separate bright runs crossing this row. 2+ means the
                         // row is cutting both legs of a corner.
    bool  wide  = false; // widest run exceeded maxWidthFrac
};

struct LineResult {
    RowHit near_, mid, far_;
    float   err  = 0;    // -1..1, the error to null
    float   aim  = 0;    // -1..1, lookahead
    uint8_t conf = 0;    // 0..100
    float   turn = 0;    // -1..1, which way the line BENDS ahead. Survives the
                         // far row going blind, which err and aim do not.
    uint8_t rowsOk = 0;  // bit0 near, bit1 mid, bit2 far
};

// Depth fractions of the three scan rows, measured from the FAR edge of frame.
//
// WIDENED 2026-09-02, and this is what makes a sharp corner detectable at all.
// The old {0.50, 0.70, 0.86} put all three rows in the near half of a frame
// that reaches 152 mm: they read the ground 54, 37 and 26 mm ahead of the lens,
// spanning 28 mm. A corner is noticed in the window between the FAR row losing
// the tape and the NEAR row losing it, so that window was 28 mm long — 130 ms,
// or 1.4 frames at 11 fps, and at any real speed the robot drove through the
// whole event between two frames and simply went blind.
//
// {0.15, 0.50, 0.85} reads 108, 54 and 27 mm ahead instead. Same three scans,
// same cost, but the window is 81 mm — 4.2 frames — which is enough to see a
// corner coming, decide, and commit. The top half of the image had been paid
// for and thrown away.
// PULLED BACK 2026-09-03 after the first real run. {0.15, ...} put the far row
// 155 mm (6.1 in) ahead of the pivot centre, which reads lovely in simulation
// and is too far on actual tape: the zigzag course's legs are 533-927 mm, so
// the far row spends a large fraction of EVERY leg looking past the corner at
// bare floor. Verified from the recording — the far row's view held no tape at
// all for ~20 s (contrast 70, i.e. carpet) while near and mid tracked it fine
// at contrast 254. That mattered more than it sounds, because `farOk` gates
// both the corner trip and the approach speed: the robot crawled the entire
// course at the corner-approach speed and never once ran at its base speed.
// 0.35 is 119 mm (4.7 in) ahead — still more warning than the 101 mm this
// started life with, but inside the range where tape is actually present.
static const float SCAN_FRAC[3] = {0.35f, 0.60f, 0.85f};   // far, mid, near

class LineFinder {
public:
    static const int kMaxRuns = 8;   // more bright runs than this on one row is
                                     // glare, not tape
    LineCfg cfg;

    void setFrame(const uint8_t *buf, int w, int h) {
        buf_ = buf; w_ = w; h_ = h;
        across_ = cfg.swapAxes ? h : w;
        depth_  = cfg.swapAxes ? w : h;
    }

    int across() const { return across_; }
    int depth()  const { return depth_;  }

    // depth index of scan row i (0 = far, 2 = near)
    int rowIndex(int i) const {
        float f = SCAN_FRAC[i];
        int   d = (int)(f * (depth_ - 1));
        return cfg.nearAtHigh ? d : (depth_ - 1 - d);
    }

    inline uint8_t px(int a, int d) const {
        return cfg.swapAxes ? buf_[a * w_ + d] : buf_[d * w_ + a];
    }

    // hintPx: where the row BELOW this one (nearer the robot) found the line, or
    // -1 for none. See run() — this is what stops a corner's outgoing leg from
    // stealing the row.
    RowHit scan(int i, int hintPx = -1) const {
        RowHit r;
        r.depthIdx = rowIndex(i);
        const int d = r.depthIdx;
        int lo = 255, hi = 0;
        for (int a = 0; a < across_; a++) {
            uint8_t v = px(a, d);
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        r.contrast = hi - lo;
        if (r.contrast < cfg.minContrast) return r;

        const int thr = lo + (int)(cfg.thrFrac * r.contrast);
        const int maxW = (int)(cfg.maxWidthFrac * across_);
        int minW = (int)(cfg.minWidthFrac * across_); if (minW < 2) minW = 2;

        // ONE PASS, THREE JOBS: the longest run (the tape), how many separate
        // runs there are, and the centroid of ALL bright pixels on the row.
        // The last two are what a corner looks like from here — see skew below.
        int bestS = -1, bestL = 0, s = -1;
        long anum = 0, aden = 0;
        int runS[kMaxRuns], runL[kMaxRuns], nrun = 0;
        for (int a = 0; a <= across_; a++) {
            const bool on = (a < across_) && (px(a, d) >= thr);
            if (on) {
                const int wgt = px(a, d) - thr + 1;
                anum += (long)a * wgt; aden += wgt;
            }
            if (on && s < 0) s = a;
            if (!on && s >= 0) {
                r.runs++;
                const int len = a - s;
                if (nrun < kMaxRuns) { runS[nrun] = s; runL[nrun] = len; nrun++; }
                if (len > bestL) { bestL = len; bestS = s; }
                s = -1;
            }
        }
        r.wide = (bestL > maxW);

        // CONTINUITY, NOT LENGTH, once a nearer row has already found the line.
        // A corner puts TWO runs on the same row: the leg the robot is on, and
        // the leg it is about to turn onto. The outgoing one is crossed
        // obliquely, so it is the WIDER of the two and wins a longest-run
        // contest outright — which is how the far row came to report the corner
        // as the line's position from 155 mm away, and how the robot came to
        // start its turn a body-length and a half early. The run nearest to
        // where the row below found the line is the one that is actually
        // continuous with it. The rejected run is not discarded: it is the
        // bright mass that gives skew its sign, below.
        if (hintPx >= 0 && nrun > 1) {
            int pick = -1, bestD = 1 << 30;
            for (int k = 0; k < nrun; k++) {
                if (runL[k] < minW || runL[k] > maxW) continue;
                const int d2 = runS[k] + runL[k] / 2 - hintPx;
                const int ad = d2 < 0 ? -d2 : d2;
                if (ad < bestD) { bestD = ad; pick = k; }
            }
            if (pick >= 0) { bestS = runS[pick]; bestL = runL[pick]; r.wide = false; }
        }
        if (bestL < minW) return r;

        // intensity-weighted centroid INSIDE the run, so a partly-lit edge of
        // the tape does not pull the answer a whole pixel
        long num = 0, den = 0;
        for (int a = bestS; a < bestS + bestL; a++) {
            int wgt = px(a, d) - thr + 1;
            num += (long)a * wgt;
            den += wgt;
        }
        float c = den ? (float)num / den : (bestS + bestL * 0.5f);

        // SKEW — the corner signal, and the reason a sharp turn is survivable.
        // Where the line bends, the row also cuts the OUTGOING leg — as a second
        // run further out, or, right at the vertex, merged into one fat run
        // whose mass hangs to one side. That extra mass is the evidence, and it
        // does not move `pos`, so it survives after the far row has lost the
        // line.
        //
        // MEASURED AGAINST THE MASS THAT IS NOT THE TRACKED RUN. The first
        // version compared the ALL-bright centroid against the run's, which
        // silently conflated two different things: "there is extra tape over
        // there" and "the run I am tracking sits off-centre". Whenever the run
        // is well off to one side, everything else in the row is necessarily on
        // the other side, so that version reported a large skew with no second
        // leg involved at all — and since turnSkew (1.60) outweighs turnSpread
        // (1.10), it could flip `turn` outright. Caught on real tape: with the
        // far row reporting the line hard LEFT at pos -0.641 into a 131 deg left
        // corner, `turn` came out +0.673, confidently right, and would have sent
        // both the corner arc and any memory of it the wrong way.
        //
        // Requiring the extra mass to be a real fraction of the run's own mass
        // also stops floor texture and lighting gradients from registering.
        //
        // FROM OTHER TAPE-LIKE RUNS ONLY, not from every pixel over threshold.
        // The pixel version failed on real tape the moment contrast dropped:
        // at conf 23-32 (a 131 deg corner in flat light) the relative threshold
        // sits low enough that floor texture clears it as scattered pixels,
        // and their centroid — weighted 1.6 against spread's 1.1 — swamped the
        // correct geometric evidence. `turn` flipped sign nearly every frame
        // (+0.43, +0.17, -0.16, +0.46, -1.00, +0.70 ... into a LEFT corner) and
        // the corner arced right, away from the outgoing leg. A second leg of
        // tape is a contiguous run of tape-like width; noise is not.
        const float half = (across_ - 1) * 0.5f;
        long sNum = 0, sDen = 0;
        for (int k = 0; k < nrun; k++) {
            if (runS[k] == bestS || runL[k] < minW) continue;
            sNum += (long)(runS[k] * 2 + runL[k]) * runL[k];   // 2*centre * len
            sDen += runL[k];
        }
        if (sDen > bestL / 4)   // a real second stripe, not a stray blob
            r.skew = constrain((((float)sNum / (2.0f * sDen)) - c) / half, -1.0f, 1.0f);

        r.runStart = bestS;
        r.width = bestL;
        r.centrePx = (int)(c + 0.5f);
        r.pos = (c - half) / half;
        // A run wider than maxWidthFrac is not steerable and never was. But it
        // is now REPORTED rather than thrown away: at a corner vertex the two
        // legs merge into exactly that, and discarding the row is how the robot
        // went blind at the one moment it most needed to know which way to go.
        r.ok = !r.wide;
        return r;
    }

    LineResult run() {
        LineResult res;
        // NEAR FIRST, then outward. The near row is the one the robot is
        // straddling, so it is the only row that can be trusted without a
        // reference, and each row further out is resolved against the one
        // behind it. Order matters: run the far row first and the chain has
        // nothing to anchor on.
        res.near_ = scan(2);
        res.mid   = scan(1, res.near_.ok ? res.near_.centrePx : -1);
        res.far_  = scan(0, res.mid.ok   ? res.mid.centrePx
                          : (res.near_.ok ? res.near_.centrePx : -1));

        // err prefers the near row; if the near row is blind (the line has run
        // out from under the nose) the mid row stands in rather than the robot
        // going blind at exactly the moment it is closest to leaving the tape.
        // ---- TURN: which way the line bends ahead ---------------------------
        // COMPUTED BEFORE THE BAIL-OUT BELOW, on purpose. At a corner vertex
        // both legs can merge into a single over-wide run, which leaves every
        // row un-steerable and sends run() home early — and that is precisely
        // the frame whose skew knows which way to go.
        //
        // Two pieces of evidence, because as the robot closes on a sharp corner
        // they become available in this order:
        //
        //   1. ROW SPREAD. The far row is offset toward the turn before the near
        //      row has moved at all. Good early warning, but it dies the moment
        //      the far row loses the line — which on a 120 deg corner happens
        //      while the robot is still a body-length short of the vertex.
        //   2. SKEW. The extra bright mass from the outgoing leg. This one
        //      arrives exactly when spread stops being available, because both
        //      facts have the same cause: the vertex entering the row.
        //
        // Past about 90 deg the outgoing leg heads BACK past the robot, so no
        // amount of lookahead makes it visible from the incoming leg. There is
        // no steering answer at that point, only a pivot — and `turn` is how the
        // driver knows which way to pivot. It is deliberately NOT a curvature
        // estimate; it is a signed direction with a magnitude worth trusting.
        // The outermost row that found ANY tape is the witness — `width`, not
        // `ok`, because a row REJECTED as over-wide is the most informative row
        // there is. Between about 60 and 100 degrees the outgoing leg never
        // comes back into a scan row as a second run, so there is no skew to
        // read; instead the row that straddles the vertex runs along the
        // outgoing leg and returns one enormous run whose POSITION is pinned to
        // the side the line went. Reading only `ok` rows threw that away and
        // left `turn` at exactly 0.000 through a 90 degree corner — detectable
        // in the trace as a robot that saw the tape end and had no opinion about
        // which way it went.
        //
        // The NEAR row can be the witness too, but only when it is WIDE. An
        // image row maps to a straight line on the ground PERPENDICULAR to the
        // heading, so a 90 degree outgoing leg lies parallel to the scan rows:
        // it is either entirely inside one row or invisible, and there is no
        // in-between. Approached square-on it therefore shows up as the near row
        // going one solid colour, with the far and mid rows seeing nothing at
        // all — no spread, no skew, no witness, `turn` exactly 0.000, and a
        // robot that watched the tape end and had no opinion. Requiring `wide`
        // is what keeps this from firing on a line the robot is merely off
        // centre of: a near row that is wide is a stripe running ACROSS the
        // robot's path, which is a corner and not an error.
        float t = 0;
        const RowHit *out = res.far_.width ? &res.far_
                          : (res.mid.width ? &res.mid
                          : ((res.near_.width && res.near_.wide) ? &res.near_ : nullptr));
        if (out) {
            // Referenced to the near row where there is one to reference, and to
            // frame centre when the near row IS the witness.
            const float ref = (out != &res.near_ && res.near_.width) ? res.near_.pos : 0.0f;
            t += (out->pos - ref) * cfg.turnSpread;
            t += out->skew * cfg.turnSkew;
        }
        res.turn = constrain(t, -1.0f, 1.0f);
        res.rowsOk = (uint8_t)((res.near_.ok ? 1 : 0) | (res.mid.ok ? 2 : 0)
                             | (res.far_.ok ? 4 : 0));

        const RowHit *e = res.near_.ok ? &res.near_ : (res.mid.ok ? &res.mid : nullptr);
        const RowHit *a = res.far_.ok  ? &res.far_  : (res.mid.ok ? &res.mid : nullptr);
        if (!e) return res;

        res.err = e->pos;
        res.aim = a ? a->pos : e->pos;

        // Confidence: contrast is most of it, but a hit on all three rows is
        // worth more than a hit on one — a real line is continuous, a glare
        // spot is not.
        int hits = (res.near_.ok ? 1 : 0) + (res.mid.ok ? 1 : 0) + (res.far_.ok ? 1 : 0);
        int c = e->contrast * 100 / 160;               // 160 counts = full marks
        if (c > 100) c = 100;
        c = c * (55 + 15 * hits) / 100;                // 1 row 70%, 3 rows 100%
        res.conf = (uint8_t)constrain(c, 0, 100);

        return res;
    }

private:
    const uint8_t *buf_ = nullptr;
    int w_ = 0, h_ = 0, across_ = 0, depth_ = 0;
};

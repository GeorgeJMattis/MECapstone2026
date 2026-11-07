# Three-Tendon Spiral Continuum Arm — Control Architecture

> Design record ported from an architecture conversation so work can continue on any machine.
> This is a living design doc — **not yet approved for firmware/coding**; hand geometry + kinematics
> math comes first. Last updated 2026-11-07.

## Status — resume here
**Decided:**
- Robot = 3-tendon spiral continuum arm; shape set by **sequential base→tip hard-stop curling**
  (deterministic, 1-DOF in reel `δ_eff` + 1-DOF in bend direction `φ`).
- **Position-only** control with mechanical pretension (no force/tension sensing).
- **Standalone ESP32-S3** brain running `FastAccelStepper`.
- **CL57Y(V20)** closed-loop stepper drivers (5 V inputs via a 74HCT125 buffer, **ALM-based
  hard-stop homing**, MF clears alarms).
- Control via one **transport-agnostic protocol**: laptop/USB-serial now → ESP32-hosted Wi-Fi page
  (identical on iPhone) → optional native Swift app.

**Immediate next step (before any firmware):**
1. **Task 1** — extract `{ℓ_j, θ_max,j, d_j}` per joint from the CAD (`θ_max,j` = the joint angles
   in the fully-curled pose).
2. **Task 2** — the sequential-curl math (below).

**Open decision:** map source — hybrid vs full SolidWorks sampling vs analytic-only (see
*Pre-coding work*).

## Context
Capstone (MECapstone2026). Build the control system for a three-tendon continuum "spiral" arm
(log-spiral shape, `r ≈ a·e^(kθ)`). Three **closed-loop stepper motors** (StepperOnline
**CL57Y(V20)** drivers — loop closed *in the driver*, no servo loop to write) reel three tendons
spaced ~120° around a **single continuum section**; pulling the right combination curls the arm in
any plane. Motion has **2 bending DOF** (bend-plane direction `φ` + curvature `κ`) plus a
third-tendon **tautness-management redundancy**.

The old test rig (Arduino + Tkinter + USB serial) proved motor comms but was laptop-tethered and
treated the motors as *independent* axes — unsuitable for a *coupled, standalone* tendon robot. This
is effectively a greenfield build (repo is a stub README).

**Goal:** a cheap, **standalone, on-robot** controller that (a) moves all three tendons in
**coordinated absolute-position** moves so tendons never slack or over-tension, (b) is driven first
from the laptop and trivially transitions to iPhone later, and (c) grows a task-space kinematics
layer for intuitive pose control.

## Key physics / control decisions (locked)
- **Shape is ~length-determined** (rigid-ish links, small play), not force-balance → control tendons
  by **absolute reeled length**; friction is second-order.
- **No tension/force sensing.** Position-only + mechanical pretension keeps tendons taut.
- **Coupling is purely geometric** → the three axes must move **synchronized through
  kinematically-consistent intermediate lengths** (coordinated moves), never independent per-axis
  endpoint moves (which can transiently slack a tendon).
- **Dominant residual error is mechanical play/backlash**, not friction → mitigate by approaching
  each pose from a consistent direction (anti-backlash) + firm pretension; quantify with a
  two-direction hysteresis test.
- **Absolute-position accuracy limiters to design out:** tendon stretch (low-stretch line,
  Dyneema/steel), spool effective-radius drift (**constant-radius capstan**, not a stacking spool),
  and a repeatable **home** reference.

## Architecture (locked)
**Single ESP32 (recommend ESP32-S3 devkit) on the robot = entire brain.** No laptop at runtime
(laptop only flashes firmware + runs calibration).

- **Motion:** `FastAccelStepper` (hardware-timed step generation via RMT/MCPWM) for all 3 steppers +
  a thin coordination layer that scales per-axis speeds so all axes finish together. Pin motion to
  one core, comms/UI to the other → Wi-Fi never disturbs step timing. **ESP32 step count = motor
  position** (loop closed in the CL57Y); ALM is the safety net.
- **Control input — transport-agnostic command protocol** (one internal "command intent" = bend
  direction + curvature rate, and/or per-tendon jog) with swappable clients, so laptop-now →
  iPhone-later is a non-event:
  - Bring-up: **USB-serial dev console** from the laptop (Python CLI / keyboard teleop).
  - Primary UI: **ESP32-hosted Wi-Fi control page** (ESP32 as Wi-Fi AP). Laptop browser drives it
    now; **iPhone browser drives the identical page later** — same URL, zero porting. Robot stays
    standalone (controller is just a browser client).
  - Optional later: **native Swift app** speaking the same protocol (Wi-Fi or BLE) for polish.
- **Safety / homing:** hardware **e-stop** drops the motor bus (contactor) + signals the ESP32;
  per-axis **software limits**; monitor each driver's **ALM** line. **Homing = hard-stop + driver
  following-error alarm:** gently drive each tendon into a mechanical hard stop → the CL57Y "drive
  overtolerance" **ALM asserts** → zero there → **clear the alarm by toggling MF** (no power cycle).
  *(Confirmed from manual — fully automatable, no home switches needed.)*

## Driver interface — CL57Y(V20) (confirmed from manual)
- **Inputs:** opto-isolated PU (pulse) / DR (dir) / MF (enable), nominal **5 V**, 1–7.5 mA. → Drive
  via a **74HCT125/74AHCT125 buffer @5 V** (3.3 V→5 V), **common-negative** wiring (PU−/DR−/MF− →
  GND, buffer drives PU+/DR+/MF+); set the driver's **5/24 V switch to 5 V**. Honor ≥2.5 µs pulse
  width and a ~150 µs MF-enable-to-step settle (easily met; ≤200 kHz).
- **DIP (per driver):** SW1 OFF = PU&DR (step/dir); SW2 = direction (per wiring); **SW3 OFF =
  closed-loop**; **SW4 ON = ALM normally-closed** (fail-safe); SW5–SW8 = microstep, start
  **1600–3200 steps/rev** (pick vs capstan size; speed headroom is ample).
- **ALM output → ESP32:** opto open-collector (max 50 mA); **pull up to 3.3 V** + series R into an
  ESP32 input (no level shift this direction). Trips on overtolerance/following-error (homing),
  over/under-voltage, phase loss, encoder fault. (PEND/in-position may also be exposed — verify on
  unit.)
- **MF (enable) from ESP32** (via buffer): frees motors + **clears alarms** → homing recovery +
  soft-stop.
- **Power:** motor PSU **24–50 V** (size for 3 × ~2.3 A typ / 5.6 A peak); buck to 5 V (buffer) +
  3.3 V (ESP32).
- **No runtime position readback** (RS232 port is tuning-only) — by design; step count is truth.

## Pre-coding work: geometry + kinematics (before firmware)
Source of truth = the **static CAD model** (as-built, not articulable). Shape is set by **sequential
base→tip hard-stop curling**: as a tendon reels in, joints lock at their hard-stop angle one at a
time. This removes the many-joint ambiguity, so the arm is effectively **1-DOF in curl**
(parametrized by along-plane reel `δ_eff`) + **1-DOF in bend direction `φ`** (set by the 3-tendon
split). Shape is a true function of length; the spiral comes from the graded per-joint geometry in
the CAD.

### Task 1 — extract from CAD
Per joint `j = 1..N`, base→tip: link length `ℓ_j`; hard-stop angle `θ_max,j` (read straight off the
fully-curled CAD pose); tendon offset radius `d_j` (all 3 tendons at ~120°); pivot axis; confirm
lock order is base→tip; neutral/zero pose; total reel travel.

### Task 2 — the math (sequential-curl model)
- Per-joint full-bend reel: `c_j ≈ d_j·θ_max,j` (use CAD-exact guide geometry if not small-angle).
- Lock breakpoints: `C_m = Σ_{j≤m} c_j`.
- State at along-`φ` reel `δ_eff`: active joint `m` with `C_{m-1} ≤ δ_eff < C_m`;
  `θ_j = θ_max,j` (j<m), `θ_m ≈ (δ_eff − C_{m-1})/d_m`, `θ_j = 0` (j>m).
- Forward (tip pose): `ψ_j = Σ_{k≤j} θ_k`; `lateral = Σ ℓ_j·sin ψ_j`, `axial = Σ ℓ_j·cos ψ_j`;
  `X = lateral·cosφ`, `Y = lateral·sinφ`, `Z = axial`.
- Physical tendon targets (120° split): `ΔL_i = −δ_eff·cos(β_i − φ)`, `β = {0,120,240}°`.
  Automatically satisfies `ΔL₁ + ΔL₂ + ΔL₃ = 0` (taut/coupling constraint).
- Hold `φ` fixed during a curl (locked joints keep their plane); changing `φ` mid-curl → 3-D twist,
  not a planar spiral (a control rule, not a math problem).
- Build this from extracted params and **validate/tune against the real arm in Phase 2** (measure
  actual curl vs `δ_eff`; adjust `c_j` / `θ_max,j`).

### Getting the map from SolidWorks (if the model is made moveable)
A constrained moveable assembly is a *data generator*, not a symbolic solver — drive the curl,
export `(δ_eff, tip XYZ, joint angles)` to CSV (Design Study / Motion / API macro), fit offline;
LimitAngle mates = the hard stops. **Gotcha:** a single tendon-length drive is kinematically
underdetermined across several limited joints → SolidWorks snaps to *a* curl, not the physical
base-first one; reproduce the real order by either driving joints in sequence, or a Motion dynamics
sim (joint return-springs + tendon force + stop contacts). Fit = piecewise-linear schedule (exact
for sequential) or poly/spline/table.

**Map-source decision (OPEN):** hybrid (model pins `c_j` + validates, analytic schedule on-board —
recommended) vs full articulated sampling vs analytic-only.

### One cable → all three → in-between directions (superposition)
- **Characterize one cable once.** The 3 tendons are identical, rotated 120° (`β₁=0°, β₂=120°,
  β₃=240°`), so the single-cable map `pull δ → planar curl θ(δ)/κ(δ)` is the **same function for all
  three**, only rotated in azimuth. No per-cable re-derivation — pure symmetry.
- **In-between azimuths via cosine projection** (the plan's relation): `ΔL_i = −δ_eff·cos(β_i − φ)`.
  Nearest cable pulls hardest, far side pays out.
  - Consistency check: `Σcos(β_i−φ) = 0` ⇒ `ΣΔL_i = 0` (total tendon length conserved).
  - Inverse: `φ = atan2(√3(ΔL₃−ΔL₂), ΔL₂+ΔL₃−2ΔL₁)`, magnitude from the `ΔL` combination.
- **Assumption to VALIDATE on the real arm** (it's not a smooth constant-curvature arm — it curls by
  sequential hard stops, so superposition can break between cables):
  1. *Direction snapping* — command `φ=60°` (between cables 1 & 2); does the tip hit 60°, or snap
     toward 0°/120° because of the discrete stops?
  2. *Slack side can't push* — far-side tendons get negative `ΔL` and must pay out; pretension must
     keep them from going fully slack or control authority is lost.
  3. *Large-deflection / friction* — cosine linearity drifts at big curl; friction adds hysteresis.
- **Phase-2 calibration:** sweep commanded `φ` between cables, measure actual tip azimuth, fit a
  small correction (gain or short lookup on `φ`) only for the between-cable region. Model stays; just
  trim it.

## Phased plan
### Phase 1 — Hardware bring-up + per-tendon jog (laptop over USB serial)
- Wire ESP32 → **74HCT125 (3.3→5 V)** → 3 × CL57Y PU/DR/MF (common-negative); set DIPs above. Wire
  each **ALM → ESP32 input** (3.3 V pull-up); wire **e-stop** (motor-bus contactor + ESP32 sense).
- Firmware: `FastAccelStepper` setup; **USB-serial command console** for per-tendon jog; soft limits;
  e-stop; ALM handling + MF alarm-clear. Calibrate steps→mm (constant-radius capstan).
- Verify each tendon reels accurately to a commanded absolute length.

### Phase 2 — Coordinated moves + hard-stop homing + Wi-Fi UI
- Coordination layer: synchronized multi-axis moves; segment paths so intermediate length triples
  stay taut (use the tautness redundancy to hold a minimum tension).
- **Hard-stop homing routine** (drive to stop → ALM → zero → MF-clear) for absolute reference.
- **ESP32-hosted Wi-Fi control page** (identical on laptop + iPhone) → coordinated in-plane curl.
  Run the **two-direction hysteresis test** to quantify play.

### Phase 3 — Task-space kinematics (sequential hard-stop model)
- Implement the sequential-curl map from *Pre-coding work* above: `δ_eff` ↔ joint-lock schedule ↔
  tip pose (forward), and inverse `(φ, curl) → ΔL_i = −δ_eff·cos(β_i−φ)`. Piecewise-linear, tiny,
  exact on the ESP32 — no large lookup table needed.
- UI commands a pose/curl; firmware solves → coordinated move. Validate & tune `c_j`/`θ_max,j`
  against the physical arm using the Phase 2 measurements.

## Open items (resolve during build)
- ~~Driver manual~~ **RESOLVED** (CL57Y specifics above: 5 V buffer, ALM homing, MF clear).
- **Phase-3 calibration measurement:** camera/vision vs manual (protractor/grid) vs joint sensors —
  decide after the Phase 2 hysteresis test shows how much correction is actually needed.
- **Microstepping + capstan geometry:** fix steps/mm once capstan diameter is chosen.
- **Power / e-stop hardware:** final PSU voltage + current; e-stop contactor in the motor bus.
- **Mechanical** (user's domain): pretension mechanism, constant-radius capstan, hard-stop features
  for homing.

## Verification
- **Phase 1:** command each tendon to absolute lengths; measure reeled length vs commanded; confirm
  e-stop + ALM halt motion and MF clears the alarm.
- **Phase 2:** drive coordinated curls; confirm no tendon slacks mid-move (visual/tension check);
  confirm hard-stop homing zeroes repeatably; run the two-direction test and record hysteresis.
- **Phase 3:** command set poses; measure tip/shape error vs commanded across the workspace; check
  repeatability with a one-direction approach.

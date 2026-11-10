# Kalman Filter 3D Visualizer

An interactive learning aid I vibe-coded to help myself understand Kalman filters.

[Try the live demo](https://jackeown.github.io/KalmanFilters/)

Open `index.html` directly in a modern browser. Keep `vendor/` beside it; all scripts, math fonts, and styles are local, so no network connection is required.

Alternatively, run `python -m http.server 8765 --bind 127.0.0.1` in this directory and visit http://127.0.0.1:8765.

## Driving

- **Race track** follows an asymmetric closed circuit with true straights, rounded corners and an infield bend. The road is visible before any trails accumulate. Trees, rocks, hills, cabins sit beyond the circuit, with a generous clear buffer. Placement reserves clear space around the cabin roofs so trees and rocks cannot cut through them. Scenery is decorative only: it has no interaction or collision handling.
- Select **Arrow keys** to drive: ↑ accelerates, ↓ brakes and then reverses if held, ←/→ steer. Signed speed is limited to −6 m/s in reverse and +15 m/s forward; steering yaw changes sign when reversing. Click the scene to give it keyboard focus. On-screen arrow buttons also support touch. Sliders and the mode selector retain their normal keyboard controls.
- **Record new drive** starts a fresh manual simulation. Each prediction records throttle/steer `u`, applied world acceleration, noise settings, the edited Q shape, and GPS samples. **Stop recording** pauses and completes a pending correction.
- **Replay recording** restarts the saved drive and plays those exact samples, including GPS noise, through the filter. The u vector, applied acceleration, timestamps and recent input rows update during replay. Replay stops after the last correction. Pause and Step also work during replay.
- Recordings remain in memory while the page is open. Reloading clears them; starting a new recording replaces the previous one. Reset keeps the saved recording.

## Filter and controls

The state is `[px, py, vx, vy, ax, ay]`. F is shown as a compact 3 × 3 block for one axis; the same model applies to both axes.

Track mode predicts constant acceleration without control inputs. Manual/replay mode converts throttle and steering to commanded world acceleration and uses `x⁻ = F x⁺ + B a_command`. Per axis, `B = [dt²/2, dt, 1]ᵀ`; F in this mode retains position and velocity, and the input sets acceleration. The raw `u = [throttle, steer]ᵀ` is nonlinear in world coordinates, so its converted acceleration is shown separately.

Q defaults to a simple diagonal model covariance. The green slider is an overall scale q: `Q = q Q_shape`. Initially each axis adds position variance `q dt⁴/4`, velocity variance `q dt²`, and acceleration variance `q`, with zero cross terms. This is a tuning assumption, not a measurement or a continuous white jerk model.

**Edit Q…** opens a paused editor with x/y variances and correlation for position, velocity, and acceleration. These are actual covariance values at the current slider scale. The editor also accepts a full 6 × 6 JSON matrix in state order `[px, py, vx, vy, ax, ay]`, including cross-stage terms. Apply validates finite values, symmetry, nonnegative variances and positive semidefiniteness. Invalid matrices leave Q unchanged. The green slider subsequently scales the entire edited matrix. Simple default and Elliptical example presets are available; presets do not apply until Apply Q. Cancel/Escape/backdrop close discards the draft and restores the prior playback state. Q edits take effect on the next prediction; simulation reset preserves the chosen Q.

Equal x/y process and GPS noise, identical per-axis dynamics, and an isotropic initial covariance keep `Pxx = Pyy` and `Pxy = 0`, so the map level sets really are circles. Turning does not change covariance in this linear model because F, Q, R and H determine it, independently of measured position values. Edit Q with unequal variances to stretch the regions, and x/y covariance to tilt them as P evolves. Q is model noise; the regions are based on the saved prediction covariance P, not directly on Q.

Slider maxima are Q scale **1.0**, GPS variance **10 m²**, and playback **6×** (double their previous limits). R is GPS position variance in m², not standard deviation. The amber slider changes the next GPS sample and its matching covariance. Q editing and Q/R sliders are disabled during replay because their recorded values are restored. The cyan slider controls playback speed.

**Pause / Play** controls playback. **Step** pauses and completes one prediction or correction phase. A prediction advances the car and filter by 0.1 simulated seconds and samples GPS; correction uses the same timestamp. At 1×, each phase takes 0.25 wall-clock seconds for inspection.

**Reset** clears the simulation and trails while preserving noise settings, playback state and the saved recording; it stops recording. **Center view** restores the camera. **Drag orbits by default.** Right-drag also orbits; Shift-drag pans, and scroll zooms. The visible Pan/Orbit buttons choose what left-drag does. Touch uses one finger for the selected mode and two fingers for zoom plus the other mode. Camera buttons provide zoom in/out, Center on car, Top view, and Reset view. Center on car is a toggle: its indented state follows the moving car by translating the camera and its target together, with frame-rate-independent exponential easing (0.18 s time constant) between simulation updates. The viewing angle stays fixed relative to the world rather than turning with the car. Orbit and zoom adjust the view while following stays on; clicking the toggle again, panning, or resetting the view releases follow. Simulation reset keeps following enabled. Panning stays on the ground plane, and camera limits prevent going underground. Full section headings are clickable. Every collapsible heading also has a ? button that opens a modal explaining the section, its matrix/vector, and its place in the filter cycle. Reading help pauses playback and closing it restores the prior playback state. Escape, the Close button, or a click outside the modal dismisses it.

The covariance correction uses Joseph form. Three filled, translucent uncertainty disks are centered on the latest predicted position (before GPS correction), with crisp boundaries at Mahalanobis distances **2 (red), 4 (orange), and 6 (yellow)**. A boundary satisfies `(p−predicted_mean)ᵀ predicted_P_pos⁻¹ (p−predicted_mean) = d²`. These are generally ellipses in world coordinates and circles in whitened coordinates, as shown in the HUD. The nested disks use fill opacities of 20%, 14%, and 10% respectively.

All state markers are dots in track mode. In manual/replay mode, a moving marker becomes a triangle pointing along its own velocity; near zero speed (≤0.05 m/s) it becomes a dot because velocity has no meaningful direction. No marker uses simulator body heading or a ground-truth reverse flag to orient an estimate. The simulator still keeps an internal heading to generate steering dynamics; that is ground truth, not an estimated state.

The single purple prediction marker sits exactly at the prediction mean, at the center of all three disks, with 60% surface opacity and sharp outlines. The prediction and its covariance are saved before correction and remain visible afterward; the cyan marker shows the current corrected state. The prediction marker is hidden until the first prediction. The covariance panel shows the saved prediction covariance, matching the regions. Camera easing affects only the view, never state values, marker positions, timestamps, or covariance.

Every overlay on the car scene (camera toolbar, phase/time badges, legend, and noise controls) has 70% opacity, increasing to 90% on hover or keyboard focus. Header and sidebar controls remain fully opaque. The outer page stays within the viewport; the desktop dashboard scrolls independently, while the stacked mobile content scrolls inside the app.

## Verification

Run `node tests/simulation.cjs` (no npm install needed). It checks phase timestamps, 6,000 cycles at noise bounds, positive definite covariance, reset, track closure and straights/turns, exact replay of 120 recorded samples, reverse motion and steering, constant Mahalanobis distance for rotated covariances, Q validation, elliptical covariance and exact custom-Q replay.

Vendored dependencies: Three.js r128 (including OrbitControls) and KaTeX 0.16.8. Their MIT licenses are in `vendor/`.

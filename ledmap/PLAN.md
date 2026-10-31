# LED Strand Self-Identifying 3D Mapping: Project Plan

## 1. Goal

Recover the 3D position of every LED in an installation of 12 to 16 strands of roughly 300 individually addressable LEDs each (about 3,600 to 4,800 LEDs total). Each LED blinks a unique color-coded ID. Short iPhone video clips from several stationary viewpoints are decoded to label each visible LED, and the labeled 2D observations across views are triangulated into 3D positions.

Key properties of the approach:

- Every LED carries its own ID, so cross-view correspondence is solved by decoding, not by feature matching.
- Not every LED needs to be visible or decodable in every clip. Occlusion is expected.
- Only LEDs with strong, independently validated decodes are kept. Anything ambiguous is discarded rather than guessed.
- Multiple short clips from different positions are combined at the end.

## 2. Hardware assumptions

- LEDs are individually addressable (WS2812/NeoPixel-class or similar), driven by a controller that can update all LEDs synchronously at a fixed frame period.
- The controller emits a fixed, repeatable frame clock so the pattern timing is known precisely.
- Camera: iPhone, mounted on a tripod, one stationary position per clip.

## 3. ID assignment and encoding

### 3.1 Global integer ID

Assign each LED a unique integer from 0 to N-1, where N is the total LED count across all strands. Strand number and position-in-strand can be folded into this single integer or kept as a lookup table. The decoder only needs to recover the integer and map it back through the table.

### 3.2 Randomized codes

Do not transmit sequential IDs directly. Instead:

1. Generate a fixed random permutation (seeded, stored in the repo) that maps each LED index to a code word.
2. Make code words longer than the minimum needed, so there is redundancy.
3. Choose code words so that any two valid codes differ in many symbol positions (minimum Hamming distance). A random code with enough length gets this naturally; it can be verified and re-rolled by the generator script.

Rationale: with sequential IDs, neighboring LEDs share most of their symbols, so blur or bleed between adjacent lights can produce a plausible-looking wrong ID. Scrambled, high-distance codes make a corrupted decode land outside the valid set, so it is detectable and can be rejected.

### 3.3 Symbol alphabet

The encoding mode is a tunable parameter. Three modes are supported, in order of preference:

- **Mode A (default): 8-color, 3 bits per symbol.** Each symbol is one of the eight on/off combinations of the red, green, and blue channels: off, red, green, blue, yellow, cyan, magenta, white. This matches how the sensor works, since a Bayer sensor measures R, G, and B separately, so each channel acts as its own on/off bit. There is no dark gap between symbols.
- **Mode B (fallback): 4-color, 2 bits per symbol.** Use if eight colors prove unreliable in test footage. Suggested set: red, green, blue, and white or off.
- **Mode C (fallback): 1 bit per symbol, Manchester coded.** White and off only. Each bit is sent as a transition within its slot: off-then-on for a one, on-then-off for a zero. Every bit has a mid-slot transition, so the decoder recovers the clock from the signal itself and no separate dark gap is needed. Most robust against color problems, but slowest.

Decoding rules for Mode A and B:

- Decode per channel, not per color. Threshold the red, green, and blue channels independently, each against its own level, then combine the three results into the symbol. This avoids misclassifying mixed colors when one channel bleeds or saturates.
- Mixed colors (yellow, cyan, magenta) depend on two channels being on together, so they are the likeliest to be misread. White can clip all three channels and off must stay clearly dark, so set exposure so full white is bright but not blown out.
- Hold each symbol for at least two camera frames so a clean frame exists between transitions, and decode from the middle of each symbol, never the edges, because rolling shutter can smear a transition across a frame.

Repeated symbols and clock recovery: Manchester coding is not needed in Modes A and B, but they have one weakness. A symbol boundary only shows up as a change when the next symbol differs, so two identical symbols in a row (for example red then red) look like one long symbol. Two ways to handle this:

- **Fixed clock (start here).** The controller sends symbols at a constant, known rate, and the preamble lets the decoder lock onto that rate. The decoder then samples at regular intervals and never depends on transitions.
- **Differential encoding (add only if needed).** Encode each symbol as the change from the previous one, with a repeat never sent, so every symbol boundary is a visible transition. Add this only if test footage shows the timing drifting.

### 3.4 Frame structure

Each transmission cycle:

1. Preamble: a known alternating pattern (for example, white and off alternating several times, then a distinctive start marker) so the decoder can find the start of the cycle, lock onto the symbol clock, and learn the timing. This replaces the dark gaps that a one-bit scheme would need between symbols.
2. Payload: the code word symbols, sent back to back with no gap in Modes A and B.
3. Check symbols: parity or a short CRC over the payload.
4. Gap: brief dark interval before the next cycle.

Repeat the full cycle several times within each clip (for example 3 to 5 repetitions). The decoder decodes each repetition independently and then takes a majority vote per symbol across repetitions. This handles dropped or corrupted frames.

### 3.5 Sizing estimate

With about 4,800 LEDs, the minimum is roughly 13 bits. In Mode A (3 bits per symbol) that is about 5 symbols, so plan for 10 to 12 payload symbols plus check symbols for real error margin. Mode B needs about twice as many symbols as Mode A for the same bits, and Mode C needs about three times as many, with Manchester coding doubling the slots per bit. For comparison, one bit per symbol with a dark gap after every bit would be about 6 times slower than Mode A. Exact length should be tuned from test footage.

## 4. Acceptance rules for a decode

An LED observation is accepted only if all of these hold:

- The blob was tracked continuously through the whole clip.
- Its brightness was above a minimum threshold and not saturated.
- Majority vote across repetitions produces a code word that passes the check symbols.
- The decoded code word is in the valid code set.
- The blob is well separated from neighboring blobs (no overlap within a pixel-radius threshold).

Anything failing these rules is dropped. Because many clips will be combined, losing a fraction of LEDs per clip is acceptable.

## 5. iPhone capture settings

Goal: no automatic adjustment during the clip, short exposures so LEDs stay point-like, and minimal compression damage to color.

### 5.1 App

Use a manual camera app such as Blackmagic Camera. The built-in Camera app only offers tap-and-hold AE/AF lock and cannot independently fix shutter, ISO, and white balance.

### 5.2 Settings

- Resolution: highest practical (4K). More pixels per LED improves separation of neighbors.
- Frame rate: 30 fps is a good starting point; 60 fps gives finer timing if the controller supports short symbol durations. Pick one and keep it fixed.
- Shutter speed: fixed and short, for example 1/120 s or faster. This prevents color smearing across symbol transitions and keeps LEDs from saturating. Note: this differs from the usual cinematic 180 degree rule on purpose.
- ISO: fixed and low enough to avoid noise, and exposure set so LED centers are bright but not clipped. Expose for the LEDs, not for the surrounding scene. A dim room is fine and helps.
- White balance: manually fixed to a constant color temperature. Never auto.
- Focus: manually locked on the strand plane. Avoid autofocus hunting.
- Stabilization: turn off video stabilization if the phone is on a tripod, since it can introduce small frame-to-frame warps.
- Codec and format: prefer the least-compressed option available (ProRes if the phone supports it, otherwise HEVC at the highest bitrate). Avoid Log and HDR color modes so colors stay simple and predictable.
- Lens: use the main wide lens, not digital zoom. Keep the same lens for every clip so calibration stays valid.
- Environment: minimize ambient light and avoid flickering sources. Dim or dark room is ideal.

### 5.3 Per-session checklist

1. Mount phone on tripod, frame as much of the installation as possible.
2. Lock focus, exposure, ISO, shutter, and white balance.
3. Run a test pattern with all LEDs in each of the symbol colors and confirm the colors are separable in a frame grab.
4. Record a clip containing at least 3 full repetitions of the pattern.
5. Move the tripod to the next viewpoint, repeat. Keep settings identical across clips.

## 6. Software pipeline (for Claude Code)

Suggested components, in order:

1. **Code generator.** Given the LED count, produces the seeded random permutation and code words, checks minimum pairwise distance, and exports both a lookup table and the controller firmware or pattern data.
2. **Controller firmware or script.** Plays the sync marker, payload, check symbols, and gap, with fixed timing, repeated continuously.
3. **Video decoder.** For each clip:
   - Extract frames and detect the sync marker to find cycle boundaries.
   - Detect LED blobs (for example, thresholded local maxima on a max-over-time image) and track them across frames.
   - Classify the color of each blob in each symbol window using the clean frame within the window.
   - Majority-vote across repetitions, verify check symbols, look up the ID.
   - Output a table of LED ID, pixel x, pixel y, confidence, for accepted LEDs only.
4. **Reconstruction.** Combine the per-clip tables. Because IDs are known, correspondence is given. Use camera calibration to run triangulation and bundle adjustment to recover 3D positions and camera poses. (Calibration plan is deferred; see below.)
5. **Validation.** Report reprojection error per LED, count of views per LED, and flag LEDs seen in too few views.

## 7. Deferred and open items

- Camera calibration: not yet planned. Likely intrinsic calibration with a checkerboard at the fixed lens and settings, with extrinsics recovered during bundle adjustment. Decide later.
- Exact symbol duration, code length, and color set should be tuned using real test footage before committing.
- Decide whether a scale reference is needed (a known distance in the scene) so the final 3D map has real-world units.
- Confirm the controller can update all LEDs synchronously at the needed rate, given strand length and data-line limits.

## 8. Suggested first milestone

Build a small test: one strand of 50 to 100 LEDs, one viewpoint, the full encode-record-decode loop, and measure what fraction of LEDs decode correctly. Use that to tune color set, symbol duration, and code length before scaling to the full installation.

## 9. Existing tools to review before building

Several existing tools already map LED positions with a camera, and Claude Code should inspect them first to reuse the triangulation and export steps. Most use flash-and-assess detection (light one LED at a time and watch where it appears), which is simple and robust but needs one flash per LED per viewpoint. At 3,600 to 4,800 LEDs that is slow, which is the main reason to use the parallel color-coded ID scheme in this plan instead. As far as the search showed, none of these offer parallel self-identifying codes as a standard feature.

| Tool | What it does | How it might help |
| --- | --- | --- |
| [Twinkly app mapping](https://help.twinkly.com/hc/en-gb/articles/18277831859741-how-do-i-map-my-twinkly-device) | Scans each LED with the phone camera and builds a 3D layout; fills in undetected LEDs by interpolation | Reference for the user experience; tied to Twinkly hardware, so not reusable directly |
| [xLights](https://www.youtube.com/watch?v=N-yEqznlyQ8) | Lighting sequencing software with an auto-mapping feature and pixel layout models | Possible export target for the final 3D coordinates |
| [pixel\_mapper](https://github.com/aaknitt/pixel_mapper) | Semi-automatic 3D pixel mapping with computer vision; triangulates from three camera positions and writes an xLights model file | Triangulation and xLights export code could be reused; replace its detection step with the ID decoder |
| [3d-wled-driver](https://github.com/gunicsba/3d-wled-driver) | Maps LEDs with a camera (MariMapper), then runs effects in 3D over WLED and DDP | Shows how mapped 3D positions can drive effects afterward |

Suggested approach: read the pixel\_mapper and MariMapper source, keep whatever triangulation, averaging, and file export logic fits, and swap in the self-identifying decoder from section 6 as the detection stage.

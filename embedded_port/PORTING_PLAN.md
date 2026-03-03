# BusCompressor strict-port notes (STM32H743, no-OS)

## 1) Dependency analysis

### BusCompressor direct dependencies (from original implementation)

- `multicomp.cpp` (`UniversalCompressor::BusCompressor`): compressor control, sidechain, envelope, auto-release, hysteresis, output saturation/limiter.
- `HardwareEmulation/TransformerEmulation.h`: input/output transformer models.
- `HardwareEmulation/ConvolutionEngine.h`: short IR convolution (initialized for Console_Bus IR).
- `HardwareEmulation/HardwareMeasurements.h`: Console Bus profile (transformer settings).
- `HardwareEmulation/WaveshaperCurves.h`: transformer saturation LUT/curve implementation.

### Classification per dependency

- `multicomp.cpp` BusCompressor class: **(A) ported as-is in math/order**, with JUCE/heap removed.
- `HardwareEmulation/TransformerEmulation.h`: **(B) rewritten in plain C++** (fixed arrays, no STL containers).
- `HardwareEmulation/ConvolutionEngine.h` (`ShortConvolution`): **(B) rewritten in plain C++** (fixed arrays).
- `HardwareEmulation/HardwareMeasurements.h` (Console Bus subset only): **(C) inlined subset** into `getConsoleBusProfile()`.
- `HardwareEmulation/WaveshaperCurves.h`: **(B) rewritten in plain C++** LUT class using C arrays.

## 2) Minimal file extraction subset

Required original files to reproduce ONLY Bus compressor behavior:

1. `multicomp.cpp` (lines containing `UniversalCompressor::BusCompressor`)
2. `HardwareEmulation/TransformerEmulation.h`
3. `HardwareEmulation/ConvolutionEngine.h`
4. `HardwareEmulation/HardwareMeasurements.h` (Console Bus profile values)
5. `HardwareEmulation/WaveshaperCurves.h`

## 3) Data structure conversion (no heap/no STL containers)

Fixed maxima in this port baseline:

- Channels: `MAX_CHANNELS = 2`
- FIR length: `MAX_IR_LENGTH = 256`
- Waveshaper table size: `TABLE_SIZE = 4096`

All runtime states are fixed-size C arrays:

- Detector states: `Detector detectors[2]`
- Transformer state arrays: `[2]` per state variable
- Convolution: `float impulseResponse[256]`, `reversedIR[256]`, `inputBuffer[256]`
- Waveshaper LUTs: `6 x float[4096]`

## 4) DSP chain preservation

Per-sample order preserved from original BusCompressor implementation:

1. Input transformer
2. Sidechain detection path (internal HP or external sidechain)
3. Threshold/ratio reduction computation
4. Attack/release update (including auto-release mode)
5. Envelope hysteresis
6. Gain application
7. Polynomial output saturation (`k2`, `k3`)
8. Makeup gain
9. Hard output limit

Note: In the original `BusCompressor::process`, output transformer + convolution are prepared but not called; strict behavior is preserved.

## 5) JUCE replacement map

- `juce::Decibels::*` -> `decibelsToGain()` / `gainToDecibels()`.
- `juce::jlimit/jmin/jmax` -> manual clamp/min/max helpers.
- `juce::dsp::ProcessorChain<IIR...>` -> explicit one-pole HP path exactly matching the in-process simplification used by original Bus compressor.
- Convolution remains direct FIR with static circular buffer.

## 6) Real-time safety analysis

### CPU-heavy sections

- Convolution FIR loop (up to 256 taps/sample) if enabled in processing path.
- Waveshaper table initialization (startup only, not per-sample).
- `powf/log10f/tanhf/sinf/cosf/sinhf` in control path and IR generation (mostly setup-time except dB conversion and saturation).

### Non-deterministic / unsafe patterns removed

- Removed heap allocation and `std::vector` growth from audio path.
- Removed `std::unique_ptr` and per-channel dynamic filter chains.
- No exceptions, no locks, no I/O in processing path.

### Remaining runtime caveats

- `powf/log10f` remain in sample path via dB conversion (deterministic but relatively expensive on MCU FPU).
- External sidechain toggling changes branch path but remains bounded and deterministic.

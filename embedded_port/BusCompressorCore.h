#pragma once

#include <stdint.h>

namespace EmbeddedPort {

struct HarmonicProfile {
    float h2;
    float h3;
    float h4;
    float h5;
    float h6;
    float h7;
    float evenOddRatio;
};

struct TransformerProfile {
    bool hasTransformer;
    float saturationThreshold;
    float saturationAmount;
    float lowFreqSaturation;
    float highFreqRolloff;
    float dcBlockingFreq;
    HarmonicProfile harmonics;
};

struct HardwareUnitProfile {
    TransformerProfile inputTransformer;
    TransformerProfile outputTransformer;
};

class WaveshaperCurvesCore {
public:
    enum CurveType {
        Opto_Tube,
        FET,
        Classic_VCA,
        Console_Bus,
        Transformer,
        Linear
    };

    static const int TABLE_SIZE = 4096;
    static const float TABLE_RANGE;

    WaveshaperCurvesCore();
    float process(float input, CurveType curve) const;

private:
    float optoCurve[TABLE_SIZE];
    float fetCurve[TABLE_SIZE];
    float vcaCurve[TABLE_SIZE];
    float consoleCurve[TABLE_SIZE];
    float transformerCurve[TABLE_SIZE];
    float linearCurve[TABLE_SIZE];

    static float clampFloat(float value, float lo, float hi);
    static float indexToInput(int index);
    const float* getTable(CurveType curve) const;
    void initialize();
    void initializeOptoCurve();
    void initializeFETCurve();
    void initializeVCACurve();
    void initializeConsoleCurve();
    void initializeTransformerCurve();
    void initializeLinearCurve();
};

class TransformerEmulationCore {
public:
    TransformerEmulationCore();
    void prepare(double sampleRate, int numChannels);
    void reset();
    void setProfile(const TransformerProfile& newProfile);
    void setEnabled(bool shouldBeEnabled);
    float processSample(float input, int channel);

private:
    TransformerProfile profile;
    double sampleRate;
    int numChannels;
    bool enabled;

    float dcBlockerCoeff;
    float dcBlockerX1[2];
    float dcBlockerY1[2];

    float hfRolloffCoeff;
    bool hfRolloffEnabled;
    float hfFilterState[2];

    float lastSample[2];
    float hfEstimate[2];

    WaveshaperCurvesCore waveshaper;

    static float clampFloat(float value, float lo, float hi);
    static int clampInt(int value, int lo, int hi);
    void updateDCBlocker(float cutoffFreq);
    void updateHFRolloff(float cutoffFreq);
    float estimateHighFrequencyContent(float input, int channel);
    float applyTransformerSaturation(float input);
    float addHarmonics(float input, const HarmonicProfile& harmonics);
    float applyHFRolloff(float input, int channel);
    float processDCBlocker(float input, int channel);
};

class ShortConvolutionCore {
public:
    enum TransformerType {
        Opto,
        FET_Type,
        Console_Bus_Type,
        Generic,
        Bypass
    };

    static const int MAX_IR_LENGTH = 256;

    ShortConvolutionCore();
    void prepare(double sampleRate);
    void reset();
    void loadTransformerIR(TransformerType type);
    float processSample(float input);

private:
    float impulseResponse[MAX_IR_LENGTH];
    float reversedIR[MAX_IR_LENGTH];
    float inputBuffer[MAX_IR_LENGTH];
    int irLength;
    int writePos;
    double sampleRate;

    static int minInt(int a, int b);
    static float absFloat(float x);
    void normalizeIR();
    void generateTransformerIR(float resonanceFreq, float resonanceAmount, float rolloffFreq, float rolloffDb, int length);
    void applyResonance(float freq, float amount);
    void applyLowpass(float freq, float db);
};

class BusCompressorCore {
public:
    static const int MAX_CHANNELS = 2;
    static constexpr float OUTPUT_HARD_LIMIT = 2.0f;
    static constexpr float EPSILON = 0.0001f;
    static constexpr float BUS_MAX_REDUCTION_DB = 20.0f;

    struct Detector {
        float envelope;
        float rms;
        float previousLevel;
        float hpState;
        float prevInput;
        float previousGR;
    };

    BusCompressorCore();
    void prepare(double sampleRate, int numChannels, int blockSize);
    float process(float input, int channel, float threshold, float ratio,
                  int attackIndex, int releaseIndex, float makeupGain,
                  float mixAmount, bool oversample, float sidechainSignal,
                  bool useExternalSidechain);

    float getGainReduction(int channel) const;

private:
    int numChannels;
    double sampleRate;
    Detector detectors[MAX_CHANNELS];

    TransformerEmulationCore inputTransformer;
    TransformerEmulationCore outputTransformer;
    ShortConvolutionCore convolution;

    static float clampFloat(float value, float lo, float hi);
    static int clampInt(int value, int lo, int hi);
    static float absFloat(float x);
    static float decibelsToGain(float dB);
    static float gainToDecibels(float gain);
};

HardwareUnitProfile getConsoleBusProfile();

} // namespace EmbeddedPort

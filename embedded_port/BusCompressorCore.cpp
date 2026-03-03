#include "BusCompressorCore.h"

#include <math.h>

namespace EmbeddedPort {

const float WaveshaperCurvesCore::TABLE_RANGE = 4.0f;

static HarmonicProfile makeHarmonic(float h2, float h3, float evenOdd, float h4 = 0.0f, float h5 = 0.0f, float h6 = 0.0f, float h7 = 0.0f)
{
    HarmonicProfile hp;
    hp.h2 = h2; hp.h3 = h3; hp.h4 = h4; hp.h5 = h5; hp.h6 = h6; hp.h7 = h7; hp.evenOddRatio = evenOdd;
    return hp;
}

HardwareUnitProfile getConsoleBusProfile()
{
    HardwareUnitProfile p;
    p.inputTransformer.hasTransformer = true;
    p.inputTransformer.saturationThreshold = 0.9f;
    p.inputTransformer.saturationAmount = 0.03f;
    p.inputTransformer.lowFreqSaturation = 1.05f;
    p.inputTransformer.highFreqRolloff = 22000.0f;
    p.inputTransformer.dcBlockingFreq = 10.0f;
    p.inputTransformer.harmonics = makeHarmonic(0.002f, 0.004f, 0.4f);

    p.outputTransformer.hasTransformer = true;
    p.outputTransformer.saturationThreshold = 0.92f;
    p.outputTransformer.saturationAmount = 0.02f;
    p.outputTransformer.lowFreqSaturation = 1.03f;
    p.outputTransformer.highFreqRolloff = 24000.0f;
    p.outputTransformer.dcBlockingFreq = 8.0f;
    p.outputTransformer.harmonics = makeHarmonic(0.002f, 0.003f, 0.45f);
    return p;
}

float WaveshaperCurvesCore::clampFloat(float value, float lo, float hi) { return value < lo ? lo : (value > hi ? hi : value); }
float WaveshaperCurvesCore::indexToInput(int index) { return (((float)index / (TABLE_SIZE - 1)) * TABLE_RANGE) - (TABLE_RANGE / 2.0f); }

WaveshaperCurvesCore::WaveshaperCurvesCore() { initialize(); }

void WaveshaperCurvesCore::initialize() { initializeOptoCurve(); initializeFETCurve(); initializeVCACurve(); initializeConsoleCurve(); initializeTransformerCurve(); initializeLinearCurve(); }

const float* WaveshaperCurvesCore::getTable(CurveType curve) const
{
    switch (curve) {
        case Opto_Tube: return optoCurve;
        case FET: return fetCurve;
        case Classic_VCA: return vcaCurve;
        case Console_Bus: return consoleCurve;
        case Transformer: return transformerCurve;
        case Linear: default: return linearCurve;
    }
}

float WaveshaperCurvesCore::process(float input, CurveType curve) const
{
    float normalized = (input + TABLE_RANGE / 2.0f) / TABLE_RANGE;
    normalized = clampFloat(normalized, 0.0f, 0.9999f);
    float indexFloat = normalized * (TABLE_SIZE - 1);
    int index0 = (int)indexFloat;
    int index1 = index0 + 1;
    if (index1 >= TABLE_SIZE) index1 = TABLE_SIZE - 1;
    float frac = indexFloat - (float)index0;
    const float* table = getTable(curve);
    return table[index0] * (1.0f - frac) + table[index1] * frac;
}

void WaveshaperCurvesCore::initializeOptoCurve(){for(int i=0;i<TABLE_SIZE;++i){float x=indexToInput(i);if(x>=0.0f){float soft=x/(1.0f+x*0.12f);float h2=soft*soft*0.025f;optoCurve[i]=soft-h2;}else{float ax=fabsf(x);optoCurve[i]=-ax/(1.0f+ax*0.08f);}}}
void WaveshaperCurvesCore::initializeFETCurve(){const float t=1.0f,h3c=0.18f,h5c=0.04f,sat=t+t*t*t*h3c+t*t*t*t*t*h5c;for(int i=0;i<TABLE_SIZE;++i){float x=indexToInput(i),ax=fabsf(x),s=x>=0?1.0f:-1.0f;float x3=x*x*x,x5=x3*x*x;float shaped=x+x3*h3c+x5*h5c;if(ax>t){float ex=ax-t;float lim=sat+tanhf(ex*1.5f)*0.15f;shaped=s*lim;}fetCurve[i]=shaped;}}
void WaveshaperCurvesCore::initializeVCACurve(){const float t=1.5f,h3=0.018f,sat=t+t*t*t*h3;for(int i=0;i<TABLE_SIZE;++i){float x=indexToInput(i),ax=fabsf(x),s=x>=0?1.0f:-1.0f;if(ax<t){vcaCurve[i]=x+x*x*x*h3;}else{float ex=ax-t;vcaCurve[i]=s*(sat+tanhf(ex*0.3f)*0.14f);}}}
void WaveshaperCurvesCore::initializeConsoleCurve(){const float tp=0.92f,tn=0.88f,h3=0.02f,sp=tp+tp*tp*tp*h3,sn=tn+tn*tn*tn*h3;for(int i=0;i<TABLE_SIZE;++i){float x=indexToInput(i),ax=fabsf(x),s=x>=0?1.0f:-1.0f;float t=x>=0?tp:tn,st=x>=0?sp:sn;if(ax<t){consoleCurve[i]=x+x*x*x*h3;}else{float ex=ax-t;consoleCurve[i]=s*(st+tanhf(ex*3.5f)*0.18f);}}}
void WaveshaperCurvesCore::initializeTransformerCurve(){for(int i=0;i<TABLE_SIZE;++i){float x=indexToInput(i),ax=fabsf(x),s=x>=0?1.0f:-1.0f;if(ax<0.7f){transformerCurve[i]=x+x*ax*0.05f;}else if(ax<1.2f){float ex=ax-0.7f;float c=0.7f+ex*(1.0f-ex*0.25f);float h2=(s*c)*c*0.08f;transformerCurve[i]=s*c+h2;}else{float ex=ax-1.2f;transformerCurve[i]=s*(1.05f+tanhf(ex*1.5f)*0.15f);}}}
void WaveshaperCurvesCore::initializeLinearCurve(){for(int i=0;i<TABLE_SIZE;++i) linearCurve[i]=indexToInput(i);}

TransformerEmulationCore::TransformerEmulationCore() : sampleRate(44100.0), numChannels(2), enabled(false), dcBlockerCoeff(0.999f), hfRolloffCoeff(0.99f), hfRolloffEnabled(true)
{ reset(); profile.hasTransformer = true; profile.dcBlockingFreq = 10.0f; profile.highFreqRolloff = 20000.0f; profile.lowFreqSaturation = 1.0f; profile.saturationAmount = 0.0f; }

float TransformerEmulationCore::clampFloat(float v, float lo, float hi){return v<lo?lo:(v>hi?hi:v);} int TransformerEmulationCore::clampInt(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);} 
void TransformerEmulationCore::prepare(double sr,int ch){sampleRate=sr;numChannels=ch;updateDCBlocker(profile.dcBlockingFreq);updateHFRolloff(profile.highFreqRolloff);reset();}
void TransformerEmulationCore::reset(){for(int ch=0;ch<2;++ch){dcBlockerX1[ch]=0;dcBlockerY1[ch]=0;hfFilterState[ch]=0;lastSample[ch]=0;hfEstimate[ch]=0;}}
void TransformerEmulationCore::setProfile(const TransformerProfile& np){profile=np;enabled=profile.hasTransformer;updateDCBlocker(profile.dcBlockingFreq);updateHFRolloff(profile.highFreqRolloff);} 
void TransformerEmulationCore::setEnabled(bool b){enabled=b&&profile.hasTransformer;}
void TransformerEmulationCore::updateDCBlocker(float cutoff){float dc=cutoff>0.5f?cutoff:0.5f;dcBlockerCoeff=1.0f-(6.283185f*dc/(float)sampleRate);} 
void TransformerEmulationCore::updateHFRolloff(float cutoff){if(cutoff<=0.0f){hfRolloffEnabled=false;hfRolloffCoeff=1.0f;return;}hfRolloffEnabled=true;float w=6.283185f*cutoff/(float)sampleRate;hfRolloffCoeff=w/(w+1.0f);} 
float TransformerEmulationCore::estimateHighFrequencyContent(float in,int ch){float diff=fabsf(in-lastSample[ch]);lastSample[ch]=in;hfEstimate[ch]=hfEstimate[ch]*0.95f+diff*0.05f;return clampFloat(hfEstimate[ch]*3.0f,0.0f,1.0f);} 
float TransformerEmulationCore::applyTransformerSaturation(float in){return waveshaper.process(in, WaveshaperCurvesCore::Transformer);} 
float TransformerEmulationCore::addHarmonics(float in,const HarmonicProfile& h){if(h.h2<=0&&h.h3<=0&&h.h4<=0) return in;float x=in,x2=x*x,x3=x2*x,o=x; o+=h.h2*x2; o+=h.h3*x3; if(h.h4>0)o+=h.h4*x2*x2; return o;} 
float TransformerEmulationCore::applyHFRolloff(float in,int ch){hfFilterState[ch]+=hfRolloffCoeff*(in-hfFilterState[ch]);return hfFilterState[ch];}
float TransformerEmulationCore::processDCBlocker(float in,int ch){float y=in-dcBlockerX1[ch]+dcBlockerCoeff*dcBlockerY1[ch];dcBlockerX1[ch]=in;dcBlockerY1[ch]=y;return y;}
float TransformerEmulationCore::processSample(float input,int channel){if(!enabled) return input;channel=clampInt(channel,0,1);float hf=estimateHighFrequencyContent(input,channel);float lf=profile.lowFreqSaturation*(1.0f-hf*0.5f);float driven=input*lf;float sat=applyTransformerSaturation(driven);float out=input+(sat-input)*profile.saturationAmount;out=addHarmonics(out,profile.harmonics);if(hfRolloffEnabled) out=applyHFRolloff(out,channel);out=processDCBlocker(out,channel);return out;}

ShortConvolutionCore::ShortConvolutionCore() : irLength(1), writePos(0), sampleRate(44100.0) { for(int i=0;i<MAX_IR_LENGTH;++i){impulseResponse[i]=0;reversedIR[i]=0;inputBuffer[i]=0;} impulseResponse[0]=1;reversedIR[0]=1; }
int ShortConvolutionCore::minInt(int a,int b){return a<b?a:b;} float ShortConvolutionCore::absFloat(float x){return x<0?-x:x;}
void ShortConvolutionCore::prepare(double sr){sampleRate=sr;reset();}
void ShortConvolutionCore::reset(){for(int i=0;i<MAX_IR_LENGTH;++i) inputBuffer[i]=0; writePos=0;}
void ShortConvolutionCore::normalizeIR(){float sum=0;for(int i=0;i<irLength;++i) sum+=absFloat(impulseResponse[i]); if(sum>0.001f){float s=1.0f/sum;for(int i=0;i<irLength;++i) impulseResponse[i]*=s;}}
void ShortConvolutionCore::applyResonance(float freq,float amount){float w0=2.0f*3.14159f*freq/(float)sampleRate;float cosw0=cosf(w0),sinw0=sinf(w0);if(fabsf(sinw0)<1e-6f) return;float bw=1.0f;float alpha=sinw0*sinhf((logf(2.0f)/2.0f)*bw*w0/sinw0);float A=powf(10.0f,amount*3.0f/40.0f);float b0=1+alpha*A,b1=-2*cosw0,b2=1-alpha*A,a0=1+alpha/A,a1=-2*cosw0,a2=1-alpha/A; b0/=a0;b1/=a0;b2/=a0;a1/=a0;a2/=a0; float temp[MAX_IR_LENGTH]; for(int i=0;i<MAX_IR_LENGTH;++i) temp[i]=0; float x1=0,x2=0,y1=0,y2=0;for(int i=0;i<irLength;++i){float x=impulseResponse[i];float y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;temp[i]=y;x2=x1;x1=x;y2=y1;y1=y;}for(int i=0;i<irLength;++i) impulseResponse[i]=temp[i];}
void ShortConvolutionCore::applyLowpass(float freq,float db){float w=2.0f*3.14159f*freq/(float)sampleRate;float coeff=w/(w+1.0f);int passes=(int)(absFloat(db)/0.75f+0.5f);if(passes<1) passes=1;for(int p=0;p<passes;++p){float state=0;for(int i=0;i<irLength;++i){state+=coeff*(impulseResponse[i]-state);impulseResponse[i]=state;}}}
void ShortConvolutionCore::generateTransformerIR(float resonanceFreq,float resonanceAmount,float rolloffFreq,float rolloffDb,int length){irLength=minInt(length,MAX_IR_LENGTH);for(int i=0;i<MAX_IR_LENGTH;++i) impulseResponse[i]=0;impulseResponse[0]=1.0f;if(resonanceAmount>0.0f&&resonanceFreq>0.0f) applyResonance(resonanceFreq,resonanceAmount);if(rolloffFreq>0.0f&&rolloffFreq<sampleRate/2.0f) applyLowpass(rolloffFreq,rolloffDb);normalizeIR();for(int i=0;i<irLength;++i) reversedIR[i]=impulseResponse[irLength-1-i];}
void ShortConvolutionCore::loadTransformerIR(TransformerType type){switch(type){case Opto:generateTransformerIR(80.0f,0.5f,16000.0f,-1.5f,64);break;case FET_Type:generateTransformerIR(100.0f,0.3f,22000.0f,-0.8f,48);break;case Console_Bus_Type:generateTransformerIR(2500.0f,0.4f,20000.0f,-0.5f,32);break;case Generic:generateTransformerIR(60.0f,0.3f,18000.0f,-1.0f,48);break;case Bypass:default:irLength=1;impulseResponse[0]=1;reversedIR[0]=1;break;}}
float ShortConvolutionCore::processSample(float input){if(irLength<=1) return input*impulseResponse[0];inputBuffer[writePos]=input;float out=0;int i=0,readPos=writePos;for(;i+4<=irLength;i+=4){int idx0=(readPos-i+MAX_IR_LENGTH)%MAX_IR_LENGTH;int idx1=(readPos-i-1+MAX_IR_LENGTH)%MAX_IR_LENGTH;int idx2=(readPos-i-2+MAX_IR_LENGTH)%MAX_IR_LENGTH;int idx3=(readPos-i-3+MAX_IR_LENGTH)%MAX_IR_LENGTH;out+=inputBuffer[idx0]*reversedIR[i];out+=inputBuffer[idx1]*reversedIR[i+1];out+=inputBuffer[idx2]*reversedIR[i+2];out+=inputBuffer[idx3]*reversedIR[i+3];}for(;i<irLength;++i){int idx=(readPos-i+MAX_IR_LENGTH)%MAX_IR_LENGTH;out+=inputBuffer[idx]*reversedIR[i];}writePos=(writePos+1)%MAX_IR_LENGTH;return out;}

BusCompressorCore::BusCompressorCore() : numChannels(0), sampleRate(0.0)
{
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        detectors[i].envelope = 1.0f;
        detectors[i].rms = 0.0f;
        detectors[i].previousLevel = 0.0f;
        detectors[i].hpState = 0.0f;
        detectors[i].prevInput = 0.0f;
        detectors[i].previousGR = 0.0f;
    }
}

float BusCompressorCore::clampFloat(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);} int BusCompressorCore::clampInt(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);} float BusCompressorCore::absFloat(float x){return x<0?-x:x;}
float BusCompressorCore::decibelsToGain(float dB){return powf(10.0f,dB/20.0f);} float BusCompressorCore::gainToDecibels(float g){if(g<=0.0f) return -1000.0f; return 20.0f*log10f(g);} 

void BusCompressorCore::prepare(double sr, int ch, int)
{
    if (sr <= 0.0 || ch <= 0) return;
    sampleRate = sr;
    numChannels = ch > MAX_CHANNELS ? MAX_CHANNELS : ch;
    for (int c = 0; c < numChannels; ++c) {
        detectors[c].envelope = 1.0f;
        detectors[c].rms = 0.0f;
        detectors[c].previousLevel = 0.0f;
        detectors[c].hpState = 0.0f;
        detectors[c].prevInput = 0.0f;
        detectors[c].previousGR = 0.0f;
    }

    HardwareUnitProfile p = getConsoleBusProfile();
    inputTransformer.prepare(sr, numChannels);
    inputTransformer.setProfile(p.inputTransformer);
    inputTransformer.setEnabled(true);

    outputTransformer.prepare(sr, numChannels);
    outputTransformer.setProfile(p.outputTransformer);
    outputTransformer.setEnabled(true);

    convolution.prepare(sr);
    convolution.loadTransformerIR(ShortConvolutionCore::Console_Bus_Type);
}

float BusCompressorCore::process(float input, int channel, float threshold, float ratio,
                                 int attackIndex, int releaseIndex, float makeupGain,
                                 float mixAmount, bool oversample, float sidechainSignal,
                                 bool useExternalSidechain)
{
    (void)mixAmount;
    (void)oversample;
    if (channel >= numChannels || channel < 0) return input;
    if (sampleRate <= 0.0) return input;

    Detector& detector = detectors[channel];
    float transformedInput = inputTransformer.processSample(input, channel);

    float detectionLevel;
    if (useExternalSidechain) {
        detectionLevel = absFloat(sidechainSignal);
    } else {
        float hpCutoff = 60.0f / (float)sampleRate;
        float hpAlpha = hpCutoff < 1.0f ? hpCutoff : 1.0f;
        detector.hpState = transformedInput - detector.prevInput + detector.hpState * (1.0f - hpAlpha);
        detector.prevInput = transformedInput;
        detectionLevel = absFloat(detector.hpState);
    }

    float thresholdLin = decibelsToGain(threshold);
    float reduction = 0.0f;
    if (detectionLevel > thresholdLin) {
        float overThreshDb = gainToDecibels(detectionLevel / thresholdLin);
        reduction = overThreshDb * (1.0f - 1.0f / ratio);
        if (reduction > BUS_MAX_REDUCTION_DB) reduction = BUS_MAX_REDUCTION_DB;
    }

    const float attackTimes[6] = {0.1f, 0.3f, 1.0f, 3.0f, 10.0f, 30.0f};
    const float releaseTimes[5] = {100.0f, 300.0f, 600.0f, 1200.0f, -1.0f};

    float attackTime = attackTimes[clampInt(attackIndex, 0, 5)] * 0.001f;
    float releaseTime = releaseTimes[clampInt(releaseIndex, 0, 4)] * 0.001f;

    if (releaseTime < 0.0f) {
        float signalDelta = absFloat(detectionLevel - detector.previousLevel);
        detector.previousLevel = detector.previousLevel * 0.95f + detectionLevel * 0.05f;
        float transientDensity = clampFloat(signalDelta * 20.0f, 0.0f, 1.0f);
        float compressionFactor = clampFloat(reduction / 12.0f, 0.0f, 1.0f);
        float sustainedFactor = (1.0f - transientDensity) * compressionFactor;
        releaseTime = 0.15f + sustainedFactor * (0.45f - 0.15f);
    }

    float targetGain = decibelsToGain(-reduction);
    if (targetGain < detector.envelope) {
        float divisor = attackTime * (float)sampleRate;
        if (divisor < EPSILON) divisor = EPSILON;
        float attackCoeff = 1.0f - 1.0f / divisor;
        attackCoeff = clampFloat(attackCoeff, 0.0f, 0.9999f);
        detector.envelope = targetGain + (detector.envelope - targetGain) * attackCoeff;
    } else {
        float divisor = releaseTime * (float)sampleRate;
        if (divisor < EPSILON) divisor = EPSILON;
        float releaseCoeff = 1.0f - 1.0f / divisor;
        releaseCoeff = clampFloat(releaseCoeff, 0.0f, 0.9999f);
        detector.envelope = targetGain + (detector.envelope - targetGain) * releaseCoeff;
    }

    float currentGR = 1.0f - detector.envelope;
    currentGR = 0.9f * currentGR + 0.1f * detector.previousGR;
    detector.previousGR = currentGR;
    detector.envelope = 1.0f - currentGR;

    if (!(detector.envelope == detector.envelope)) detector.envelope = 1.0f;

    float compressed = transformedInput * detector.envelope;
    float processed = compressed;
    const float k2 = 0.004f;
    const float k3 = 0.003f;
    float x2 = processed * processed;
    float x3 = x2 * processed;
    processed = processed + k2 * x2 + k3 * x3;

    // Strictly preserved from source: outputTransformer and convolution are prepared but not used in process().

    float output = processed * decibelsToGain(makeupGain);
    if (output < -OUTPUT_HARD_LIMIT) output = -OUTPUT_HARD_LIMIT;
    if (output > OUTPUT_HARD_LIMIT) output = OUTPUT_HARD_LIMIT;
    return output;
}

float BusCompressorCore::getGainReduction(int channel) const
{
    if (channel < 0 || channel >= numChannels) return 0.0f;
    return gainToDecibels(detectors[channel].envelope);
}

} // namespace EmbeddedPort

//
// Created by mr on 2/27/2024.
//

#include "audio.h"

#include "AudioTools.h"

AudioInfo info(8000, 1, 16);
SineWaveGenerator<int16_t> sineWave(32000);
GeneratedSoundStream<int16_t> sound(sineWave);
PWMAudioOutput pwm;
StreamCopy copier(pwm, sound);    // copy in to out

void setup() {
    Serial.begin(115200);
    AudioLogger::instance().begin(Serial, AudioLogger::Warning);

    // setup sine wave
    sineWave.begin(info, N_B4);

    // setup PWM output
    auto config = pwm.defaultConfig();
    config.copyFrom(info);
    pwm.begin(config);
}

void loop(){
    copier.copy();
}
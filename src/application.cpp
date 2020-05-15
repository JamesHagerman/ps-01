// #ifndef PARTICLE
// #define PARTICLE
// #endif
#if defined(PARTICLE)
#include "Particle.h"
// #include "Adafruit_Trellis.h"
// #include "PCF8574.h"
#include "synth/types.h"

SYSTEM_MODE(MANUAL);
SerialLogHandler dbg(LOG_LEVEL_NONE, { {"app", LOG_LEVEL_ALL} });

// Adafruit_Trellis matrix0 = Adafruit_Trellis();
// Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0);
// bool keypadUpdated = false;
// uint16_t keypadState = 0;
// void keypadInterrupt();
// Timer keyUpTimer(50, keypadInterrupt, true);

// void encoderInterrupt();
// PCF8574 encoders(0x20, ENCODER_INT_PIN, encoderInterrupt);
uint16_t state[4] = {10, 100, 100, 200}; // musical, pop-reduction
// uint16_t state[4] = {0, 0, 255, 0}; // "full" on
// int32_t position[4] = {200, 200, 1 << 6, 500};
// int32_t lastPosition[4] = {0, 0, 0, 0};
// bool encoderUpdated = false;
#else
#include <stdio.h>
#include <string.h>
#define delay(x) SDL_Delay(x)
#endif

#include "synth/synth.h"

// void keypadInterrupt() {
//     keypadUpdated = true;
// }

// void setupEncoders() {
//     encoders.pinMode(P0, INPUT);
//     encoders.pinMode(P1, INPUT);
//     encoders.pinMode(P2, INPUT);
//     encoders.pinMode(P3, INPUT);
//     encoders.pinMode(P4, INPUT);
//     encoders.pinMode(P5, INPUT);
//     encoders.pinMode(P6, INPUT);
//     encoders.pinMode(P7, INPUT);
//     encoders.begin();
// }

// void encoderInterrupt(){
//     encoderUpdated = true;
// }

// void calculateKnobPosition(uint8_t knob, uint8_t pinA, uint8_t pinB, uint8_t step=1) {
//     uint8_t newState = state[knob] & 3;
//     lastPosition[knob] = position[knob];
//     if (pinA)
//         newState |= 4;
//     if (pinB)
//         newState |= 8;
//     state[knob] = (newState >> 2);
//     switch (newState)
//     {
//         case 1:
//         case 7:
//         case 8:
//         case 14:
//             position[knob] -= step;
//             return;
//         case 2:
//         case 4:
//         case 11:
//         case 13:
//             position[knob] += step;
//             return;
//         case 3:
//         case 12:
//             position[knob] -= step * 2;
//             return;
//         case 6:
//         case 9:
//             position[knob] += step * 2;
//             return;
//     }
// }

// Wolfson stuff
#define W8731_ADDR_0 0x1A
#define W8731_ADDR_1 0x1B
#define W8731_NUM_REGS 10
// #define CODEC_ADDRESS           (W8731_ADDR_0 << 1)
// The Particle Wire protocol manages the write bit for us; we do not need the shift!
#define CODEC_ADDRESS  W8731_ADDR_0 
enum CodecRegister {
  CODEC_REG_LEFT_LINE_IN = 0x00,
  CODEC_REG_RIGHT_LINE_IN = 0x01,
  CODEC_REG_LEFT_HEADPHONES_OUT = 0x02,
  CODEC_REG_RIGHT_HEADPHONES_OUT = 0x03,
  CODEC_REG_ANALOGUE_ROUTING = 0x04,
  CODEC_REG_DIGITAL_ROUTING = 0x05,
  CODEC_REG_POWER_MANAGEMENT = 0x06,
  CODEC_REG_DIGITAL_FORMAT = 0x07,
  CODEC_REG_SAMPLE_RATE = 0x08,
  CODEC_REG_ACTIVE = 0x09,
  CODEC_REG_RESET = 0x0f,
};

enum CodecSettings {
  CODEC_INPUT_0_DB = 0x17,
  CODEC_INPUT_UPDATE_BOTH = 0x40,
  CODEC_HEADPHONES_MUTE = 0x00,
  CODEC_MIC_BOOST = 0x1,
  CODEC_MIC_MUTE = 0x2,
  CODEC_ADC_MIC = 0x4,
  CODEC_ADC_LINE = 0x0,
  CODEC_OUTPUT_DAC_ENABLE = 0x10,
  CODEC_OUTPUT_MONITOR = 0x20,
  CODEC_DEEMPHASIS_NONE = 0x00,
  CODEC_DEEMPHASIS_32K = 0x01,
  CODEC_DEEMPHASIS_44K = 0x02,
  CODEC_DEEMPHASIS_48K = 0x03,
  CODEC_SOFT_MUTE = 0x01,
  CODEC_ADC_HPF = 0x00,
  
  CODEC_POWER_DOWN_LINE_IN = 0x01,
  CODEC_POWER_DOWN_MIC = 0x02,
  CODEC_POWER_DOWN_ADC = 0x04,
  CODEC_POWER_DOWN_DAC = 0x08,
  CODEC_POWER_DOWN_LINE_OUT = 0x10,
  CODEC_POWER_DOWN_OSCILLATOR = 0x20,
  CODEC_POWER_DOWN_CLOCK_OUTPUT = 0x40,
  CODEC_POWER_DOWN_EVERYTHING = 0x80,
  
  CODEC_PROTOCOL_MASK_MSB_FIRST = 0x00,
  CODEC_PROTOCOL_MASK_LSB_FIRST = 0x01,
  CODEC_PROTOCOL_MASK_PHILIPS = 0x02,
  CODEC_PROTOCOL_MASK_DSP = 0x03,
  
  CODEC_FORMAT_MASK_16_BIT = 0x00 << 2,
  CODEC_FORMAT_MASK_20_BIT = 0x01 << 2,
  CODEC_FORMAT_MASK_24_BIT = 0x02 << 2,
  CODEC_FORMAT_MASK_32_BIT = 0x03 << 2,
  
  CODEC_FORMAT_LR_SWAP = 0x20,
  CODEC_FORMAT_MASTER = 0x40,
  CODEC_FORMAT_SLAVE = 0x00,
  CODEC_FORMAT_INVERT_CLOCK = 0x80,
  
  CODEC_RATE_48K_48K = 0x00 << 2,
  CODEC_RATE_8K_8K = 0x03 << 2,
  CODEC_RATE_96K_96K = 0x07 << 2,
  CODEC_RATE_32K_32K = 0x06 << 2,
  CODEC_RATE_44K_44K = 0x08 << 2,
};


uint8_t WriteControlRegister(uint8_t address, uint16_t data) {
    int status = 10;
    uint8_t byte_1 = ((address << 1) & 0xfe) | ((data >> 8) & 0x01);
    uint8_t byte_2 = data & 0xff;
    Serial.printlnf(" cmd - address: 0x%02x    data: 0x%02x", address, data);
    Serial.printlnf("        byte_1: 0x%02x  byte_2: 0x%02x", byte_1, byte_2);
    Wire.beginTransmission(CODEC_ADDRESS); // transmit to slave device #4
    Wire.write(byte_1);
    Wire.write(byte_2);
    status = Wire.endTransmission(true);
    Serial.printlnf("        status: 0x%02x\n", status);
    return status;
}

void setup() {
#if defined(PARTICLE)

    Serial.begin();
    pinMode(D7, OUTPUT);
    // digitalWrite(D7, HIGH);
    // waitUntil(Serial.isConnected);
    digitalWrite(D7, LOW);

    // For local step debugging, these need to be off:
    // Mesh.off();
    // BLE.off();

    // Wire.setSpeed(CLOCK_SPEED_400KHZ);
    // pinMode(KEYPAD_INT_PIN, INPUT_PULLUP);
    // attachInterrupt(KEYPAD_INT_PIN, keypadInterrupt, CHANGE);
    // trellis.begin(0x70);

    // setupEncoders();

    // Wolfson I2C configuration
    bool mcu_is_master = false;
    int32_t sample_rate = SAMPLERATE_HZ;
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();
    delay(1);

    Serial.printlnf("Writing to Wolfson address: 0x%02x", CODEC_ADDRESS);

    // Reset the device
    WriteControlRegister(CODEC_REG_RESET, 0);

    // Configure L&R input (mute L+R)
    WriteControlRegister(CODEC_REG_LEFT_LINE_IN, CODEC_INPUT_0_DB);
    WriteControlRegister(CODEC_REG_RIGHT_LINE_IN, CODEC_INPUT_0_DB);

    // Configure L&R headphone output volume (mute L+R)
    // WriteControlRegister(CODEC_REG_LEFT_HEADPHONES_OUT, CODEC_HEADPHONES_MUTE);
    // WriteControlRegister(CODEC_REG_RIGHT_HEADPHONES_OUT, CODEC_HEADPHONES_MUTE);

    // Configure analog routing (mute mic, connect line in to ADC, enable DAC)
    WriteControlRegister(CODEC_REG_ANALOGUE_ROUTING, 
                CODEC_MIC_MUTE | CODEC_ADC_LINE | CODEC_OUTPUT_DAC_ENABLE);

    // Configure digital routing (turn off deemphasis)
    WriteControlRegister(CODEC_REG_DIGITAL_ROUTING, CODEC_DEEMPHASIS_NONE);

    // Configure power management (power down mic, clock output, and internal oscillator)
    uint8_t power_down_reg = CODEC_POWER_DOWN_MIC | CODEC_POWER_DOWN_CLOCK_OUTPUT;
    if (mcu_is_master) {
        power_down_reg |= CODEC_POWER_DOWN_OSCILLATOR;
    }
    // WriteControlRegister(CODEC_REG_POWER_MANAGEMENT, (uint8_t)power_down_reg);
    
    // Configure the audio format
    uint8_t format_byte = CODEC_PROTOCOL_MASK_PHILIPS | CODEC_FORMAT_MASK_16_BIT;
    format_byte |= mcu_is_master ? CODEC_FORMAT_SLAVE : CODEC_FORMAT_MASTER;
    WriteControlRegister(CODEC_REG_DIGITAL_FORMAT, format_byte);
    
    // Configure the same rate
    uint8_t rate_byte = 0;
    if (mcu_is_master) {
        // According to the WM8731 datasheet, the 32kHz and 96kHz modes require the
        // master clock to be at 12.288 MHz (384 fs / 128 fs). The STM32F4 I2S clock
        // is always at 256 fs. So the 32kHz and 96kHz modes are achieved by
        // pretending that we are doing 48kHz, but with a slower or faster master
        // clock.
        rate_byte = sample_rate == 44100 ? CODEC_RATE_44K_44K : CODEC_RATE_48K_48K;
    } else {
        switch (sample_rate) {
        case 8000:
            rate_byte = CODEC_RATE_8K_8K;
            break;
        case 32000:
            rate_byte = CODEC_RATE_32K_32K;
            break;
        case 44100:
            rate_byte = CODEC_RATE_44K_44K;
            break;
        case 96000:
            rate_byte = CODEC_RATE_96K_96K;
            break;
        case 48000:
        default:
            rate_byte = CODEC_RATE_48K_48K;
            break;
        }
    }
    WriteControlRegister(CODEC_REG_SAMPLE_RATE, rate_byte);

    // For now codec is not active.
    // WriteControlRegister(CODEC_REG_ACTIVE, 0x00);

    // Enable the CODEC:
    WriteControlRegister(CODEC_REG_ACTIVE, 0x01);

    digitalWrite(D7, HIGH);

#endif
    Synth::instance()->voices[0].setWaveform(WF_TRIANGLE);
    Synth::instance()->voices[0].setPulseWidth(32536); // 50%
    Synth::instance()->voices[0].setFrequency(C4_HZ);
    Synth::instance()->voices[0].setADSR(state[0], state[1], state[2], state[3]);
    // Synth::instance()->voices[1].setWaveform(WF_SAWTOOTH);
    // Synth::instance()->voices[1].setFrequency(C4_HZ);
    Synth::instance()->begin();
}

// #define CYCLE
#define SCALE
// #define PWM
// #define TRELLIS

#ifdef CYCLE
void loop() {
    delay(1000);
    Synth::instance()->voices[0].setWaveform(WF_NOISE);
    delay(1000);
    Synth::instance()->voices[0].setWaveform(WF_PULSE);
    delay(1000);
    Synth::instance()->voices[0].setWaveform(WF_SAWTOOTH);
    delay(1000);
    Synth::instance()->voices[0].setWaveform(WF_TRIANGLE);
}
#endif

#ifdef PWM
void loop() {
    Synth::instance()->voices[0].setWaveform(WF_PULSE);
    Synth::instance()->voices[0].setFrequency(C4_HZ);
    Synth::instance()->voices[0].setADSR(state[0], state[1], state[2], state[3]);
    Synth::instance()->voices[0].setGate(true);
    uint16_t maxPW = (1 << 16) / 1 - 1;
    uint16_t steps = 16;
    for (uint16_t i = 0; i < maxPW; i+=maxPW / steps) {
        Log.trace("Fill %.0f%% %i", float(i) / maxPW * 100, i);
        // printf("Fill %.0f%% %i\n", float(i) / maxPW * 100, i);
        Synth::instance()->voices[0].setPulseWidth(i);
        delay(250);
    }
}
#endif

#ifdef NOISE
int del = 500;
void loop() {
    Synth::instance()->voices[0].setWaveform(WF_NOISE);
    Synth::instance()->voices[0].setFrequency(C4_HZ);
    delay(del);
    Synth::instance()->voices[0].setFrequency(E4_HZ);
    delay(del);
    Synth::instance()->voices[0].setFrequency(G4_HZ);
    delay(del);
}
#endif

#ifdef SCALE
// float scale[] = { C4_HZ, D4_HZ, E4_HZ, F4_HZ, G4_HZ, A4_HZ, B4_HZ, C5_HZ, B4_HZ, A4_HZ, G4_HZ, F4_HZ, E4_HZ, D4_HZ };
float scale[] = { C4_HZ, D4_HZ, E4_HZ, F4_HZ, G4_HZ, A4_HZ, B4_HZ, C5_HZ };

// All C's on a piano:
// float scale[] = { C0_HZ , C1_HZ, C2_HZ, C3_HZ, C4_HZ, C5_HZ, C6_HZ, C7_HZ, C8_HZ };

// Testing full range of this implementation; It doesn't work :(
// float scale[] = { 10.0f, 16000.0f };

void loop() {
    // Synth::instance()->voices[0].setWaveform(WF_SAWTOOTH);

    for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
        // trellis.setLED(i);
        // trellis.writeDisplay();
        Synth::instance()->voices[0].setFrequency(scale[i]);
        Synth::instance()->voices[0].setGate(true);
        delay(1000);
        Synth::instance()->voices[0].setGate(false);
        delay(500);
        // trellis.clrLED(i);
        // trellis.writeDisplay();
    }
    // Synth::instance()->voices[0].setWaveform(WF_TRIANGLE);
    // for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
    //     Synth::instance()->voices[0].setFrequency(scale[i]);
    //     delay(500);
    // }
}
#endif

#ifdef TRELLIS
float scale[] = { C4_HZ, C4S_HZ, D4_HZ, D4S_HZ, E4_HZ, F4_HZ, F4S_HZ, G4_HZ, G4S_HZ, A4_HZ, A4S_HZ, B4_HZ };

void loop() {
#if defined(PARTICLE)
    // delay(30);
    if (keypadUpdated) {
        if (trellis.readSwitches()) {
            bool keypadChanged = false;
            bool anyKeyPressed = false;
            for (uint8_t i=0; i<16; i++) {
                bool state = trellis.isKeyPressed(i);
                anyKeyPressed |= state;
                if (state != BIT_CHECK(keypadState, i)) {
                    // State changed
                    keypadChanged = true;

                    if (state) {
                        trellis.setLED(i);
                        BIT_SET(keypadState, i);
                        Synth::instance()->voices[0].setFrequency(scale[i]);
                        Synth::instance()->voices[0].setGate(true);
                    } else {
                        trellis.clrLED(i);
                        BIT_CLEAR(keypadState, i);
                        Synth::instance()->voices[0].setGate(false);
                    }
                }
            }
            
            if (keypadChanged) {
                trellis.writeDisplay();
            }

            // This is silly but necessary. HT16K33 doesn't trigger interrup
            // on key up so we need to check manually with a timer
            if (anyKeyPressed) {
                keyUpTimer.reset();
            }
        } else {
            keyUpTimer.reset();
        }
        keypadUpdated = false;
    }
    
    if (encoderUpdated) {
        PCF8574::DigitalInput input = encoders.digitalReadAll();

        calculateKnobPosition(0, input.p0, input.p1, 16);
        calculateKnobPosition(1, input.p2, input.p3, 8);
        calculateKnobPosition(2, input.p4, input.p5, 4);
        calculateKnobPosition(3, input.p6, input.p7, 8);

        Serial.printf(
            "Attack:%4d | Decay:%4d | Sustain:%4d | Release:%4d\r",
            position[0],
            position[1],
            position[2],
            position[3]
        );
        position[0] = max(min(position[0], 2 << 16), 0);
        position[1] = max(min(position[1], 2 << 16), 0);
        position[2] = max(min(position[2], 2 << 8), 0);
        position[3] = max(min(position[3], 2 << 16), 0);
        Synth::instance()->voices[0].setADSR(position[0], position[1], position[2], position[3]);
        encoderUpdated = false;
    }
#endif
}
#endif

#ifndef PARTICLE
int main(int argc, char **argv) {
    printf("ps-01\n");
    
    setup();

    while (1) {
        loop();
    }
}
#endif
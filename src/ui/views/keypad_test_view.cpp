#include "keypad_test_view.h"

KeypadTestView::KeypadTestView() : View() {}

void KeypadTestView::handleAction(uint8_t action, int16_t args[]) {
    float _upFreq;
    switch (action) {
        case ACTION_VIEW_INIT:
            init();
            break;
        case ACTION_KEY_DOWN:
            _frequency = _scale[args[0] - 1];
            Polyphony::instance()->startFrequency(_frequency * _octave);
            break;
        case ACTION_KEY_UP:
            _upFreq = _scale[args[0] - 1];
            Polyphony::instance()->stopFrequency(_upFreq * 0.25);
            Polyphony::instance()->stopFrequency(_upFreq * 0.5);
            Polyphony::instance()->stopFrequency(_upFreq);
            _playing = false;
            break;
        case ACTION_ENCODER_CHANGE:
            Polyphony::instance()->stopFrequency(_frequency * _octave);
            if (_octave == 1.0)
                _octave = 0.25;
            else if (_octave == 0.25)
                _octave = 0.5;
            else if (_octave == 0.5)
                _octave = 1.0;
            Polyphony::instance()->startFrequency(_frequency * _octave);
            break;
    }
}

void KeypadTestView::handleStoreUpdate(uint8_t storeKey) {}

void KeypadTestView::init() {
    Synth::instance()->setupAllVoices(WF_SAWTOOTH, 1, 1, (1 << 8) - 1, 1, (1 << 15));
    _octave = 1;

    // Init display
    _display.clearScreen();
    _display.drawDialog();
    _display.drawTabs(String("ADSR"), String("OCTA"), String("WAVE"), String("KEY"));

    _display.update();
}
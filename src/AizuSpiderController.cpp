#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <fmt/format.h>
#include <iostream>

using namespace std;
using namespace cnoid;
using fmt::format;

class AizuSpiderController : public SimpleController
{
    SharedJoystickPtr joystick;
    int targetMode;     // 0=ARM1, 1=ARM2
    bool prevPS;        // PSボタンの状態記憶
    double dt;
    SimpleControllerIO* io = nullptr;

public:
    virtual bool initialize(SimpleControllerIO* io) override {
        this->io = io;
        dt = io->timeStep();
        joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");

        // 2つのモードを作成（ARM1, ARM2）
        joystick->addMode(); // mode 0
        joystick->addMode(); // mode 1
        targetMode = 0;

        prevPS = false;

        io->os() << "AizuSpiderController initialized. Start with ARM1 active." << endl;
        return true;
    }

    virtual bool control() override {
        joystick->updateState(targetMode);

        // LOGO ボタン（PSボタンに相当）
        bool ps = joystick->getButtonState(0, Joystick::LOGO_BUTTON); // アームの状態に関わらず常にmode 0からボタン入力を取得

        if(ps && !prevPS){
            if(io) io->os() << "Button Press Detected. Current mode: " << targetMode;
            targetMode = 1 - targetMode; // 0と1を切り替える
            if(io) io->os() << ". New mode: " << targetMode << endl;
        }
        prevPS = ps;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuSpiderController)

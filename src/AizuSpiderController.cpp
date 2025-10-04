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
    int armState; // 0=OFF, 1=ARM1, 2=ARM2
    bool prevPS;
    SimpleControllerIO* io = nullptr;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override {
        this->io = io;
        dt = io->timeStep();
        joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");

        // モードを3つ作成 (0:AizuSpider, 1:ARM1, 2:ARM2)
        joystick->addMode(); // mode 0 for this controller
        joystick->addMode(); // mode 1 for ARM1
        joystick->addMode(); // mode 2 for ARM2
        
        armState = 0; // 0=OFF, 1=ARM1, 2=ARM2
        prevPS = false;

        io->os() << "AizuSpiderController initialized. Arms are OFF." << endl;
        return true;
    }

    virtual bool control() override {
        // 常に自身のモードを更新してPSボタンを検知
        joystick->updateState(0);
        bool ps = joystick->getButtonState(0, Joystick::LOGO_BUTTON);

        if(ps && !prevPS){
            armState = (armState + 1) % 3; // 0, 1, 2のサイクル
            if(io) {
                if(armState == 0) io->os() << "PS Button: All arms OFF" << endl;
                else if(armState == 1) io->os() << "PS Button: ARM1 ON" << endl;
                else if(armState == 2) io->os() << "PS Button: ARM2 ON" << endl;
            }
        }
        prevPS = ps;

        // アクティブなアームのモードだけを更新
        if(armState == 1){ // ARM1
            joystick->updateState(1);
        } else if(armState == 2){ // ARM2
            joystick->updateState(2);
        }
        // armStateが0 (OFF) の場合は何もしない

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuSpiderController)
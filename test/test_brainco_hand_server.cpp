#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

int main(int argc, char** argv)
{
    std::cout << "Usage: sudo " << argv[0] << " [left|right]" << std::endl;
    std::cout << "Default is 'right' if not specified." << std::endl;
    std::cout << "Uses combined ee topic (12 motors: right 0-5, left 6-11)." << std::endl;
    std::cout << "Convention: 1.0 = open, 0.0 = closed." << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(0, "");

    std::string side = argc > 1 ? argv[1] : "right";
    int offset = (side == "left") ? 6 : 0;

    auto lowcmd = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>("rt/ee/cmd");
    lowcmd->msg_.cmds().resize(12);
    for(auto & finger : lowcmd->msg_.cmds())
    {
        finger.dq() = 1.; // max speed
    }
    auto lowstate = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/ee/state");
    lowstate->wait_for_connection();

    auto hand_ctrl = [&](std::array<float, 6> & pos)
    {
        if (lowcmd->trylock()) {
            for(int i(0); i<6; i++)
            {
                lowcmd->msg_.cmds()[offset + i].q() = pos[i];
            }
            lowcmd->unlockAndPublish();
        }
    };

    std::array<float, 6> positions;

    while ( true)
    {
        // Open (1.0 = open in unified convention)
        positions = {1, 1, 1, 1, 1, 1};
        hand_ctrl(positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        // Thumb closed, others open
        positions = {0, 1, 1, 1, 1, 1};
        hand_ctrl(positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // All closed (0.0 = closed)
        positions = {0, 0, 0, 0, 0, 0};
        hand_ctrl(positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    return 0;
}

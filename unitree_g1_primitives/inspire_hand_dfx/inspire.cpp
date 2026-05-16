/**
 * @brief Unitree H1 Hand Controller
 * The user can subscribe to "rt/inspire/state" to get the current state of the hand and publish to "rt/inspire/cmd" to control the hand.
 * 
 *                  IDL Types
 * user ---(unitree_go::msg::dds_::MotorCmds_)---> "rt/inspire/cmd"
 * user <--(unitree_go::msg::dds_::MotorStates_)-- "rt/inspire/state"
 * 
 * @attention Currently the hand only supports position control, which means only the `q` field in idl is used.
 */
class H1HandController
{
public:
    H1HandController()
    {
        this->InitDDS_();
    }

    /**
     * @brief Control the hand to a specific label
     */
    void ctrl(std::string label)
    {
        if(labels.find(label) != labels.end())
        {
            this->ctrl(labels[label], labels[label]);
        }
        else
        {
            std::cout << "Invalid label: " << label << std::endl;
        }
    }

    /**
     * @brief Move the fingers to the specified angles
     * 
     * @note The angles should be in the range [0, 1]
     *       0: close  1: open
     */
    void ctrl(
        const Eigen::Matrix<float, 6, 1>& right_angles, 
        const Eigen::Matrix<float, 6, 1>& left_angles)
    {
        for(size_t i(0); i<6; i++)
        {
            cmd.cmds()[i].q() = right_angles(i);
            cmd.cmds()[i+6].q() = left_angles(i);
        }
        handcmd->Write(cmd);
    }

    /**
     * @brief Get the right hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getRightQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i].q();
        }
        return q;
    }

    /**
     * @brief Get the left hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getLeftQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i+6].q();
        }
        return q;
    }

    unitree_go::msg::dds_::MotorCmds_ cmd;
    unitree_go::msg::dds_::MotorStates_ state;
private:
    void InitDDS_()
    {
        handcmd = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/inspire/cmd");
        handcmd->InitChannel();
        cmd.cmds().resize(12);
        handstate = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
            "rt/inspire/state");
        handstate->InitChannel([this](const void *message){
            std::lock_guard<std::mutex> lock(mtx);
            state = *(unitree_go::msg::dds_::MotorStates_*)message;
        });
        state.states().resize(12);
    }

    // DDS parameters
    std::mutex mtx;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> handcmd;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> handstate;

    // Saved labels
    std::unordered_map<std::string, Eigen::Matrix<float, 6, 1>> labels = {
        {"open",   Eigen::Matrix<float, 6, 1>::Ones()},
        {"close",  Eigen::Matrix<float, 6, 1>::Zero()},
        {"half",   Eigen::Matrix<float, 6, 1>::Constant(0.5)},
    };
};
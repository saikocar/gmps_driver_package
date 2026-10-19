#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <can_msgs/msg/frame.hpp>                               //in out can
#include <gmps_msgs/msg/gmps_detect.hpp>                        //out 検知
#include <gmps_msgs/msg/gmps_error.hpp>                         //out エラー
//#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>  //in speed
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp> // in speed, autoware
#include <std_msgs/msg/bool.hpp>                                //in soft reset
#include <diagnostic_updater/diagnostic_updater.hpp>        //diagnostic updater

#define CMD_START 0x01
#define CMD_STOP 0x02
#define CMD_SELFTEST 0x03
#define CMD_OFFSETSEARCH 0x04
#define CMD_OFFSETWRITE 0x0C
#define CMD_SOFTRESET 0x0D

#define CANID_GMPS_INPUT 0x50           //to GMPS
#define CANID_GMPS_ACK 0x51             //from GMPS
#define CANID_GMPS_VER 0x5E             //from GMPS
#define CANID_GMPS_MARKER_DETECT1 0x10  //from GMPS
#define CANID_GMPS_INFO 0x12            //from GMPS
#define CANID_GMPS_SPEED 0x30           //to GMPS

#define SHOW_DEBUG_INFO 0 //DEBUG_INFO visible
#define DEBUG_INFO(...) {if (SHOW_DEBUG_INFO) {RCLCPP_INFO(__VA_ARGS__);}}

enum STATE{
    STEP1=1,
    STEP2=2,
    STEP3=3,
    STEP4=4,
    STEP101=101,
    STEP102=102,
    STEP103=103,
    STEP201=201,
    SYSTEMFAIL=91
};

class GMPSDriver : public rclcpp::Node
{
private://ros subscriber
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frame_;                       //From GMPS, can frame msg
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_; //車両からの速度情報
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_soft_reset_;                       //上位からのソフトリセット要求

private://ros publisher
    rclcpp::Publisher<gmps_msgs::msg::GmpsDetect>::SharedPtr pub_gmps_detect_;  //GMPSデバイスから取得した情報のpublisher
    rclcpp::Publisher<gmps_msgs::msg::GmpsError>::SharedPtr pub_gmps_error_;    //GMPSデバイスからのエラー通知
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frame_;          //To GMPS, can frame msg

private://ros timer
    rclcpp::TimerBase::SharedPtr state_timer_;              //状態遷移のタイマー。watchdogを兼ねる
    uint8_t state_;                                         //状態遷移
    rclcpp::Time last_cmd_send_time_ = get_clock()->now();  //GMPS_INPUTを最後に送信した時間。コマンドを連打しないため
    rclcpp::Time last_received_time_ = get_clock()->now();  //GMPS_INFOを最後に受信した時間。watchdogのため
    double elapsed_time_;                                   //最後に受信してからの経過時間

private://diagnostic updater
    diagnostic_updater::Updater diagnostic_updater_;

private://parameters
    const bool param_offset_search_mode_; //OFFSET_SEARCH実行モード
    const double param_delay_dist_;       //検知遅れ距離 [m]
    const double param_watchdog_timeout_; //GMPS_INFOによるウォッチドッグのタイムアウト時間[s]

private://GMPS用変数
    bool f_receive_ack_;    //GMPS_ACKを受信したフラグ
    bool f_receive_info_;   //GMPS_INFOを受信したフラグ
    uint8_t ack_command_;   //GMPS_INPUT コマンド
    uint8_t ack_result_;    //コマンド結果

private://arguments for publishCanFrame
    uint16_t out_can_id_;
    uint8_t out_dlc_;
    uint8_t out_data_[8];

private://arguments for publishDetect
    rclcpp::Time detect_stamp_;
    uint16_t out_counter_;
    double out_lateral_deviation_;
    uint8_t out_pole_;
    uint8_t out_mm_kind_;
    double out_delay_dist_;
    uint16_t detect_counter_; //検知回数のカウンタ 20241115

private://arguments for publishError
    uint8_t out_error_code_;

private:
    void callbackVelocity(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        uint16_t velocity_mps_100 = fabs(msg->longitudinal_velocity) * 100; //LSB 0.01 [m/s]

        out_can_id_ = CANID_GMPS_SPEED;
        out_dlc_ = 2;
        out_data_[0] = velocity_mps_100 & 0xFF;
        out_data_[1] = (velocity_mps_100 >> 8) & 0xFF;
        publishCanFrame();
    }

private:
    void callbackSoftReset(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "SOFTRESET is required.");
            RCLCPP_INFO(this->get_logger(), "transition STEP%d -> STEP201", state_);
            state_ = STEP201;
        }
    }

private:
    //decode can_msgs::Frame
    void callbackCanFrame(const can_msgs::msg::Frame::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        uint32_t can_id = msg->id;
        uint8_t data[8]={};
        for (int i=0;i<8;i++)
        {
            data[i] = msg->data[i];
        }

        switch (can_id)
        {
        case CANID_GMPS_ACK:
        {
            f_receive_ack_ = true;
            ack_command_ = data[0];
            ack_result_ = data[1];
            RCLCPP_INFO(this->get_logger(), "ACK received. CMD=%02x RET=%02x", ack_command_, ack_result_);
            break;
        }

        case CANID_GMPS_MARKER_DETECT1:
        {
            // for future use
//            uint16_t nDetect = data[0] & 0x03;
//            bool pole1 = data[0] & 0x04;
//            bool pole2 = data[0] & 0x08;
//            bool pole3 = data[0] & 0x10;
            detect_counter_ = detect_counter_ + (data[0] & 0x01); //検知回数のカウントアップ
            out_counter_ = detect_counter_;
            int16_t lateral_deviation = (data[2] << 8) | data[1];
            out_lateral_deviation_ = lateral_deviation * 0.001;
            out_pole_ = (data[0] & 0x04) ? 2 : 1; //N=1, S=2 //note /gmps_detectも1,2から0,1にしたい
            out_mm_kind_ = 0;
            out_delay_dist_ = data[7] * 0.01;
//            out_delay_dist_ = param_delay_dist_; //paramから定義する場合
            detect_stamp_ = stamp;
            publishDetect();
            break;
        }

        case CANID_GMPS_INFO:
        {
            //uint16_t speed = (data[1]<<8) | data[0];
            //double speed_mps = 0.01*speed; // [m/s]
            //uint32_t run_dist = (data[2] | data[3]<<4 | data[4]<<8 | data[5]<<12);
            //double run_dist_m = 0.01 * run_dist;
            out_error_code_ = data[6];
            publishError();
            f_receive_info_ = true;
            last_received_time_ = get_clock()->now();
            break;
        }

        case CANID_GMPS_INPUT:
            /* do nothing */
            break;

        case CANID_GMPS_SPEED:
            /* do nothing */
            break;

        default:
            DEBUG_INFO(this->get_logger(), "received undefined CAN_ID=%04x, data=%02x %02x %02x %02x %02x %02x %02x %02x", can_id, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
            break;
        }
    }

private:
    //測定開始に至るまでのシーケンス管理及びwatchdog
    void callbackTimer()
    {
        switch (state_)
        {
        case STEP1: //既に動いている可能性があるので停止要求
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 0.1)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_STOP;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish STOP= 5A %02x at STEP1", CMD_STOP);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_STOP ) &&
                    (ack_result_ == gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = STEP2;
                    RCLCPP_INFO(this->get_logger(), "ACK_OK. transition STEP1 -> STEP2");
                }
            }
            break;

        case STEP2: //測定開始前にセンサが正常であることを確認する
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 1.5)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_SELFTEST;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish SELFTEST= 5A %02x at STEP2", CMD_SELFTEST);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_SELFTEST ) &&
                    (ack_result_ == gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = STEP3;
                    RCLCPP_INFO(this->get_logger(), "ACK_OK. transition STEP2 -> STEP3");
                }
                else if ((ack_command_ == CMD_SELFTEST ) &&
                            ((ack_result_ == gmps_msgs::msg::GmpsError::ERR_GAIN_CHECK_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_NOISE_CHECK_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_VALUE_SATURATION) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_COMM_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_INIT_FAIL)  ))
                {
                    state_ = SYSTEMFAIL;
                    RCLCPP_ERROR(this->get_logger(), "SELFTEST fail at STEP2");
                    RCLCPP_INFO(this->get_logger(), "transition STEP2 -> SYSTEMFAIL");
                }
            }
            break;

        case STEP3: //測定開始指示
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 0.1)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_START;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish START= 5A %02x at STEP3", CMD_START);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_START ) &&
                    ((ack_result_ == gmps_msgs::msg::GmpsError::OK) ||
                        (ack_result_ == gmps_msgs::msg::GmpsError::ERR_DURING_MEASUREMENT)))
                {
                    state_ = STEP4;
                    last_received_time_ = get_clock()->now(); //watchdogのリセット
                    RCLCPP_INFO(this->get_logger(), "ACK_OK. transition STEP3 -> STEP4");
                    RCLCPP_INFO(this->get_logger(), "GMPS is ready to detect magnetic markers.");
                }
                else if ((ack_command_ == CMD_START ) &&
                            ((ack_result_ == gmps_msgs::msg::GmpsError::ERR_GAIN_CHECK_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_NOISE_CHECK_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_VALUE_SATURATION) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_COMM_FAIL) ||
                                (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SENSOR_INIT_FAIL)  ))
                {
                    state_ = SYSTEMFAIL;
                    RCLCPP_ERROR(this->get_logger(), "START fail at STEP3");
                    RCLCPP_INFO(this->get_logger(), "transition STEP3 -> SYSTEMFAIL");
                }
            }
            break;

        case STEP4: //測定中
            //watchdog
            elapsed_time_ = (get_clock()->now() - last_received_time_).seconds();
            if (elapsed_time_ > param_watchdog_timeout_)
            {
                out_error_code_ = gmps_msgs::msg::GmpsError::ERR_WATCHDOG_TIMEOUT;
                publishError();
                RCLCPP_ERROR(this->get_logger(), "ERR: watchdog timeout. elapsed time from last receive is %f", elapsed_time_);

                state_ = SYSTEMFAIL;
                RCLCPP_INFO(this->get_logger(), "transition STEP4 -> SYSTEMFAIL");
            }

            //GMPS_INFOのエラーコード
            if (f_receive_info_ == true)
            {
                f_receive_info_ = false;
                if (out_error_code_ != gmps_msgs::msg::GmpsError::OK)
                {
                    state_ = SYSTEMFAIL;
                    RCLCPP_ERROR(this->get_logger(), "ERR: some error has occurred during measuring.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP4 -> SYSTEMFAIL");
                }
            }

            break;

        case STEP101: //オフセットサーチ要求
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 5.0)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_OFFSETSEARCH;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish OFFSET_SEARCH= 5A %02x at STEP101", CMD_OFFSETSEARCH);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_OFFSETSEARCH ) &&
                    (ack_result_ == gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = STEP102;
                    RCLCPP_INFO(this->get_logger(), "OFFSET_SEARCH was acknowledged.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP101 -> STEP102");
                }
                else if ((ack_command_ == CMD_OFFSETSEARCH ) &&
                            (ack_result_ == gmps_msgs::msg::GmpsError::ERR_SEARCH_OFFSET_FAIL))
                {
                    state_ = SYSTEMFAIL;
                    RCLCPP_ERROR(this->get_logger(), "ERR: OFFSET_SEARCH fails. there must be significant magnetic noise around here.");
                    RCLCPP_ERROR(this->get_logger(), "Please move the vehicle to the place where there is less magnetic noise.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP101 -> SYSTEMFAIL");
                }
            }
            break;

        case STEP102: //EEPROM書き込み要求
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 1.5)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_OFFSETWRITE;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish OFFSET_WRITE_EEP= 5A %02x at STEP102", CMD_OFFSETWRITE);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_OFFSETWRITE ) &&
                    (ack_result_ == gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = STEP103;
                    RCLCPP_INFO(this->get_logger(), "OFFSET_WRITE_EEP was acknowledged.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP102 -> STEP103");
                }
                else if ((ack_command_ == CMD_OFFSETWRITE ) &&
                            (ack_result_ != gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = SYSTEMFAIL;
                    RCLCPP_ERROR(this->get_logger(), "OFFSET_WRITE_EEP fails. this indicates that some FATAL ERROR has occurred to GMPS.");
                    RCLCPP_ERROR(this->get_logger(), "Please contact your support.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP102 -> SYSTEMFAIL");
                }
            }
            break;

        case STEP103: //require reboot
            elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
            if (elapsed_time_ > 5.0)
            {
                RCLCPP_INFO(this->get_logger(), "Please change [offset_search_mode] to [false], then reboot.");
                last_cmd_send_time_ = get_clock()->now();
            }
            break;

        case STEP201: //soft reset
            if (f_receive_ack_ == false)
            {
                elapsed_time_ = (get_clock()->now() - last_cmd_send_time_).seconds();
                if (elapsed_time_ > 1.5)
                {
                    out_can_id_ = CANID_GMPS_INPUT;
                    out_dlc_ = 2;
                    out_data_[0] = 0x5A;
                    out_data_[1] = CMD_SOFTRESET;
                    publishCanFrame();
                    last_cmd_send_time_ = get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "publish SOFTRESET= 5A %02x at STEP201", CMD_SOFTRESET);
                }
            }
            else if (f_receive_ack_ == true)
            {
                f_receive_ack_ = false;
                if ((ack_command_ == CMD_SOFTRESET ) &&
                    (ack_result_ == gmps_msgs::msg::GmpsError::OK))
                {
                    state_ = STEP1;
                    RCLCPP_INFO(this->get_logger(), "SOFTRESET was acknowledged.");
                    RCLCPP_INFO(this->get_logger(), "transition STEP201 -> STEP1");
                }
                else
                {
                    //do nothing
                }
            }
            break;

        case SYSTEMFAIL:
            //RCLCPP_ERROR(this->get_logger(), "GMPS SYSTEM FAIL. Please reboot system and this node");
            out_error_code_ = gmps_msgs::msg::GmpsError::ERR_SYSTEM_FAIL;
            publishError();
            //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "undefined STATE %d. transition here -> STEP1", state_);
            state_ = STEP1;
            break;
        }
    }

private:
    //publish to_can_bus
    void publishCanFrame()
    {
        can_msgs::msg::Frame can_frame;
        can_frame.header.frame_id = "gmps";
        can_frame.header.stamp = get_clock()->now();
        can_frame.id = out_can_id_;
        can_frame.is_rtr = false;
        can_frame.is_extended = false;
        can_frame.is_error = false;
        can_frame.dlc = out_dlc_;
        for (int i=0;i<out_dlc_;i++)
        {
            can_frame.data[i] = out_data_[i];
        }
        pub_can_frame_->publish(can_frame);
    }

private:
    //publish gmps_detect
    void publishDetect()
    {
        gmps_msgs::msg::GmpsDetect msg_detect;
        msg_detect.header.frame_id = "gmps";
        msg_detect.header.stamp = detect_stamp_;
        msg_detect.counter = out_counter_;
        msg_detect.lateral_deviation = out_lateral_deviation_;
        msg_detect.pole = out_pole_;
        msg_detect.mm_kind = out_mm_kind_; //not used
        msg_detect.delay_dist = out_delay_dist_;
        pub_gmps_detect_->publish(msg_detect);
    }

private:
    //publish gmps_error
    void publishError()
    {
        gmps_msgs::msg::GmpsError error;
        error.header.frame_id = "gmps";
        error.header.stamp = get_clock()->now();
        error.error_code = out_error_code_;
        pub_gmps_error_->publish(error);
    }

private:
    //diagnostic updater
    void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        unsigned char lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;

        if (state_ == SYSTEMFAIL)
            lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        if(out_error_code_ != gmps_msgs::msg::GmpsError::OK)
            lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        stat.add<uint8_t>("SYSTEMFAIL", state_);
        stat.add<uint8_t>("error_code", out_error_code_);
        stat.summary(lvl, (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) ? "OK" : "Error");
    }
public:
    GMPSDriver(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("gmps_driver", node_option)
        , diagnostic_updater_(this, 0.2)//diagnosticを0.1秒間隔でpublish
        , param_offset_search_mode_(declare_parameter<bool>("offset_search_mode", false))
        , param_delay_dist_(declare_parameter<double>("delay_dist", 0.1))
        , param_watchdog_timeout_(declare_parameter<double>("watchdog_timeout", 0.5))
        , f_receive_ack_(false)
        , f_receive_info_(false)
        , detect_counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "constructor called");

        /* Subscriber */
        sub_can_frame_ = this->create_subscription<can_msgs::msg::Frame>(
            "in_gmps_can_frame", rclcpp::QoS(10), std::bind(&GMPSDriver::callbackCanFrame, this, std::placeholders::_1));
        sub_velocity_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "in_velocity", rclcpp::QoS(1), std::bind(&GMPSDriver::callbackVelocity, this, std::placeholders::_1));
        sub_soft_reset_ = this->create_subscription<std_msgs::msg::Bool>(
            "in_soft_reset", rclcpp::QoS(1), std::bind(&GMPSDriver::callbackSoftReset, this, std::placeholders::_1));

        /* Publisher */
        pub_gmps_detect_ = this->create_publisher<gmps_msgs::msg::GmpsDetect>("out_gmps_detect", rclcpp::QoS(1));
        pub_gmps_error_ = this->create_publisher<gmps_msgs::msg::GmpsError>("out_gmps_error", rclcpp::QoS(1));
        pub_can_frame_ = this->create_publisher<can_msgs::msg::Frame>("out_gmps_can_frame", rclcpp::QoS(1));

        /* Timer */
        state_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(20).period(), std::bind(&GMPSDriver::callbackTimer, this));

        /* Diagnostic Updater */
        diagnostic_updater_.setHardwareID("gmps_driver");
        diagnostic_updater_.add("GMPS Driver Status", this, &GMPSDriver::updateDiagnostic);

        /**/
        if (param_offset_search_mode_ == true)
        {
            RCLCPP_INFO(this->get_logger(), "begin offset_search sequence");
            state_ = STEP101;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "begin normal sequence");
            state_ = STEP1;
        }
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions option;
    std::shared_ptr<GMPSDriver> node;
    try{//メンバ初期化リスト(コンストラクタ宣言と中括弧の間の記述)で例外がthrowされた場合のtry-catch
        node = std::make_shared<GMPSDriver>(option);
    } catch(std::runtime_error &e) {//ros2の例外型の継承元クラスはstd::runtime_error
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("gmps_localizer"), e.what());
		return -1;
	}

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

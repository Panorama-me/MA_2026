#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "sentry/msg/send.hpp" 
#include<vector>
namespace sentry{
    const MAX=1000000
    struct Nav2Aim
    {
        float vx;
        float vy;
        float wz;
    };
    
    
    class Sentry: public rclcpp::Node{
        public:
            Sentry(const rclcpp::NodeOptions &options);
            ~Sentry() override;
            std::vector<Nav2Aim> sum_nav2aim(MAX);
        private:
            Nav2Aim DataNav2Aim;
            rclcpp::Subscription<sentry::msg::Receive>::SharedPtr receive_sub_;
            rclcpp::Publisher<sentry::msg::Send>::SharedPtr send_pub_;
            void send_data(const sentry::msg::Send::SharedPtr msg);
            void receive_data(const sentry::msg::Receive::SharedPtr msg);
    };
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <SDL2/SDL.h>
#include <iostream>

class KeyboardController : public rclcpp::Node {
public:
    KeyboardController() : Node("keyboard_controller"), manual_mode_(true) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Keyboard controller node started (manual mode).");

        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SDL could not initialize! %s", SDL_GetError());
            rclcpp::shutdown();
        }

        window_ = SDL_CreateWindow("Keyboard Controller",
                                   SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                   400, 200, SDL_WINDOW_SHOWN);

        if (!window_) {
            RCLCPP_ERROR(this->get_logger(), "Window creation failed! %s", SDL_GetError());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&KeyboardController::loop, this));
    }

    ~KeyboardController() override {
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

private:
    void loop() {
        SDL_Event e;
        geometry_msgs::msg::Twist msg;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                rclcpp::shutdown();
                return;
            }

            if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                SDL_Keycode key = e.key.keysym.sym;

                // Mode toggle
                if (key == SDLK_x) {
                    manual_mode_ = !manual_mode_;
                    std::string mode = manual_mode_ ? "Manual" : "Autonomous";
                    RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", mode.c_str());
                }
            }
        }

        if (manual_mode_) {
            const Uint8* state = SDL_GetKeyboardState(NULL);

            if (state[SDL_SCANCODE_W]) msg.linear.x = 2.0;
            else if (state[SDL_SCANCODE_S]) msg.linear.x = -2.0;
            else msg.linear.x = 0.0;

            if (state[SDL_SCANCODE_A]) msg.angular.z = 2.0;
            else if (state[SDL_SCANCODE_D]) msg.angular.z = -2.0;
            else msg.angular.z = 0.0;

            
        } else {
            msg.linear.x = 1.5;
            msg.angular.z = 1.5;
        }
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
    bool manual_mode_;
};
  
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

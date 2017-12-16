#include <math.h>
#include <getopt.h>
#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <fstream>
#include <string>

#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

using std::cout;
using std::endl;
using std::vector;

// using namespace std;
// using namespace std::chrono;

// for convenience
using json = nlohmann::json;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void show_usage() {
  std::cout <<
      "--tune <t>:           Auto Tune(steering - 1, speed - 2, others - all\n"
      "--help:              Show help\n";
  exit(1);
}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  PID steering_ctrl;
  PID speed_ctrl;
  Twiddle steering_twid;
  Twiddle speed_twid;

  bool auto_tune_steering_params = false;
  bool auto_tune_speed_params = false;
  bool auto_tune_params = false;

  const char* const short_opts = "t:h";
  const option long_opts[] = {
    {"tune", 1, nullptr, 't'},
    {"help", 0, nullptr, 'h'},
    {nullptr, 0, nullptr, 0}
  };

  while (true) {
    int opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);
    if (-1 == opt)
      break;

    switch (opt) {
      case 't': {
        int val = std::stoi(optarg);
        if (val == 1) {
          auto_tune_steering_params = true;
        } else if (val == 2) {
          auto_tune_speed_params = true;
        } else {
          auto_tune_steering_params = true;
          auto_tune_speed_params = true;
        }
        auto_tune_params = true;
      }
        break;
      case 'h':  // -h or --help
      case '?':  // Unrecognized option
      default:
        show_usage();
        break;
    }
  }

  // const double Kp_s = 0.088169;
  // const double Ki_s = 0.0001901;
  // const double Kd_s = 1.8867;

  // const double Kp_t = 1.18314;
  // const double Ki_t = 0.00041975;
  // const double Kd_t = 2.13892;

  // const double Kp_s = 0.2;
  // const double Ki_s = 0.0008;
  // const double Kd_s = 3;

  // const double Kp_t = 1.2;
  // const double Ki_t = 0.0002;
  // const double Kd_t = 3;

  const double Kp_s = 0.112403;
  const double Ki_s = 0.00125197;
  const double Kd_s = 2.88999;

  const double Kp_t = 0.968039;
  const double Ki_t = 0.00026506;
  const double Kd_t = 2.59908;

  if (auto_tune_params) {
    std::cout << "Auto Tune PID Parameters" << std::endl;

    const double Kp = 0;
    const double Ki = 0;
    const double Kd = 0;
    const double tolerance = 0.001;
    const std::vector<double> p = {Kp, Ki, Kd};
    // const std::vector<double> dp = {0.04, 0.0002, 1};
    const std::vector<double> dp = {0.04, 0.0004, 1};

    steering_twid.Init(tolerance, p, dp, "steering");
    if (auto_tune_steering_params) {
      steering_ctrl.Init(Kp, Ki, Kd);
    } else {
      steering_ctrl.Init(Kp_s, Ki_s, Kd_s);
    }

    const std::vector<double> dp2 = {0.2, 0.0001, 0.4};
    speed_twid.Init(tolerance, p, dp2, "speed");
    if (auto_tune_speed_params) {
      speed_ctrl.Init(Kp, Ki, Kd);
    } else {
      speed_ctrl.Init(Kp_t, Ki_t, Kd_t);
    }
  } else {
    steering_ctrl.Init(Kp_s, Ki_s, Kd_s);
    speed_ctrl.Init(Kp_t, Ki_t, Kd_t);
  }

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          /*
           * TODO: Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe use
           * another PID controller to control the speed!
           */
          steering_ctrl.UpdateError(cte);
          if (auto_tune_params && auto_tune_steering_params) {
            steering_twid.UpdateError(cte);
            vector<double> params = steering_twid.getParams();
            steering_ctrl.setKp(params[0]);
            steering_ctrl.setKi(params[1]);
            steering_ctrl.setKd(params[2]);

            if (steering_twid.isDone()) {
              auto_tune_steering_params = false;
            }
          }
          steer_value = steering_ctrl.TotalError();

          const double kTargetSpeed = 40;
          double speed_error = speed - kTargetSpeed;
          speed_ctrl.UpdateError(speed_error);
          if (auto_tune_params && auto_tune_speed_params) {
            speed_twid.UpdateError(speed_error);
            vector<double> params = speed_twid.getParams();
            speed_ctrl.setKp(params[0]);
            speed_ctrl.setKi(params[1]);
            speed_ctrl.setKd(params[2]);

            if (speed_twid.isDone()) {
              auto_tune_speed_params = false;
            }
          }
          throttle_value = speed_ctrl.TotalError();

          if (!auto_tune_steering_params && !auto_tune_speed_params && auto_tune_params) {
            auto_tune_params = false;
            restart(ws);
            return;
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

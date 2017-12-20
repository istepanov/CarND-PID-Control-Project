#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void resetSimulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid_steering;

  // these values were found using Twiddle (if twiddle_in_progress is true)
  pid_steering.Init(0.0837465, 2.0899e-06, 1.44656);
  Twiddle pid_steering_twiddle(pid_steering, 0.0109888, 1.0943e-07, 0.110536);

  PID pid_throttling;
  pid_throttling.Init(0.2, 0.0, 3.0);

  bool twiddle_in_progress = false;  // set to true to enable twiddle

  const double TARGET_SPEED = 60;

  h.onMessage([&pid_steering, &pid_throttling, &pid_steering_twiddle, &TARGET_SPEED, &twiddle_in_progress]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // steering
          pid_steering.UpdateError(cte);
          double steer_value = pid_steering.TotalError() / -deg2rad(25.0);

          double speed_error = TARGET_SPEED - speed;
          pid_throttling.UpdateError(speed_error);
          double throttle_value = pid_throttling.TotalError() * (1.0 / (1.0 + fabs(angle)));

          if (twiddle_in_progress) {
            if (pid_steering.Iteration() > 2000) {
              // during twiddle run, we run the simulation for 2000 iterations,
              // trying to find best PID gains.

              // if Iterate() return false, twiddle is complete.
              twiddle_in_progress = pid_steering_twiddle.Iterate();
              auto gains = pid_steering.Gains();
              auto gains_deltas = pid_steering_twiddle.GainDeltas();

              // DEBUG
              std::cout << "PID parameters: " << gains[0] << " " << gains[1] << " " << gains[2] << std::endl;
              std::cout << "Deltas: " << gains_deltas[0] << " " << gains_deltas[1] << " " << gains_deltas[2] << std::endl;
              std::cout << "Accumulated squared error: " << pid_steering.AccumulatedSquaredError() << std::endl;

              if (twiddle_in_progress) {
                std::cout << "Keep optimizing..." << std::endl;
              } else {
                std::cout << "Optimization finished!" << std::endl;
                pid_steering_twiddle.Reset();
              }
              resetSimulator(ws);
              pid_steering.Reset();
            } else {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//For PID control
int num = 0;
double int_cte = 0.0;
double prev_cte;
int reset_times = 0;
double total_err = 0.0;
double cte_2_sum=0;
double avg_err = 0;
// For twiddle
double best_err;
double dp[] = {1.0,0.001,1.0};
int twiddle_index = 0;
double twiddle_condition = 0.04;
int twiddle_flag1 = 0;
int twddle_flag2 = 0;
int first_flag = 0;
int twiddle_enable = 1;
int total_num = 0;
int pre_num = 0;

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


void reset_sim(uWS::WebSocket<uWS::SERVER> ws){
    // reset simulator
    std::string msg = "42[\"reset\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    num=0;
    int_cte = 0;
    total_err = 0;
    avg_err = 0;
    cte_2_sum = 0;
    // reset end
}

int main()
{
  uWS::Hub h;

  PID pid_angle;
  // TODO: Initialize the pid_angle variable.
  pid_angle.Init(1.14239,0.00001,7.06388);
  twiddle_enable = 0 ;


  h.onMessage([&pid_angle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          double dp_sum = dp[0]+dp[1]+dp[2];
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          cte_2_sum += cte*cte;
          pid_angle.UpdateError(cte);
          //std::cout << "int CTE: " << int_cte << " total error: " << total_err << std::endl;
          //std::cout << "Ki: " << pid_angle.Ki << std::endl;
          steer_value = -pid_angle.TotalError();

          if(steer_value>1.0){
              steer_value = 1.0;
          }else if(steer_value < -1.0){
              steer_value = -1.0;
          }
          num += 1;
          total_num += 1;



          if((num%5000==0) || ((std::fabs(cte) >= 2.4)&&((total_num-pre_num) > 100 ))){
              reset_times += 1;
              pre_num = total_num;
              avg_err = cte_2_sum/num;
              std::cout << "twiddle index:" << twiddle_index << std::endl;


              std::cout << "------------------"  << std::endl;
              std::cout << " Avg error:" << avg_err << std::endl;
              std::cout << "------------------"  << std::endl;
              std::cout << " reset times:" << reset_times << std::endl;
              std::cout << " dp_angle_sum:" << dp_sum << std::endl;

              if(first_flag == 0){
                  best_err = avg_err;
                  first_flag = 1;
              }
              

          std::cout << " best error:" << best_err << std::endl;
          std::cout << " twiddle condition:" << twiddle_condition << std::endl;
          if(avg_err > twiddle_condition && twiddle_enable ==1 && dp_sum > 0.08){  // quit twiddle if average error is small enough or dp sum is too small
            next_dp:
            ;
              std::cout << " start twiddle ......" << std::endl;

              if(twiddle_flag1 == 0){
                  switch (twiddle_index) {
                  case 0:
                      pid_angle.Kp += dp[0];
                      break;
                  case 1:
                      pid_angle.Ki += dp[1];
                      break;
                  case 2:
                      pid_angle.Kd += dp[2];
                  default:
                      break;
                  }

                   std::cout << "twiddle index:" << twiddle_index << " Kp:" << pid_angle.Kp << " Kd:" << pid_angle.Kd << " Ki:" << pid_angle.Ki << std::endl;
                  twiddle_flag1 = 1;
                  reset_sim(ws);
                  goto end_cycle;
              }

              if(avg_err < best_err){
                  best_err = avg_err;
                  dp[twiddle_index] *= 1.1;
                  std::cout << "dp[" << twiddle_index << "] go up 1.1 times!! now is " << dp[twiddle_index] << std::endl;
                  // reset control flags
                  twiddle_flag1 = 0;
                  twddle_flag2 = 0;
                  twiddle_index = (twiddle_index+1)%3;
                  reset_sim(ws);
                  goto next_dp;
              }else{
                  if(twddle_flag2 == 0){
                      switch (twiddle_index) {
                      case 0:
                          pid_angle.Kp -= 2.0*dp[0];
                          break;
                      case 1:
                          pid_angle.Ki -= 2.0*dp[1];
                          break;
                      case 2:
                          pid_angle.Kd -= 2.0*dp[2];
                      default:
                          break;
                      }
                      std::cout << "second round // twiddle index:" << twiddle_index << "Kp:" << pid_angle.Kp << " Kd:" << pid_angle.Kd << " Ki:" << pid_angle.Ki << std::endl;
                      twddle_flag2 = 1;
                      reset_sim(ws);
                      goto end_cycle;
                  }

                  if(avg_err < best_err){
                      best_err = avg_err;
                      dp[twiddle_index] *= 1.1;
                      std::cout << "Second *** dp[" << twiddle_index << "] go up 1.1 times!! now is " << dp[twiddle_index] << std::endl;

                      // reset control flags
                      twiddle_flag1 = 0;
                      twddle_flag2 = 0;
                      twiddle_index = (twiddle_index+1)%3;
                      reset_sim(ws);
                      goto next_dp;
                  }else{
                      switch (twiddle_index) {
                      case 0:
                          pid_angle.Kp += dp[0];
                          break;
                      case 1:
                          pid_angle.Ki += dp[1];
                          break;
                      case 2:
                          pid_angle.Kd += dp[2];
                      default:
                          break;
                      }


                       std::cout << "third twiddle index:" << twiddle_index << "Kp:" << pid_angle.Kp << " Kd:" << pid_angle.Kd << " Ki:" << pid_angle.Ki << std::endl;
                      dp[twiddle_index] *= 0.9;
                      std::cout << "bottom *** dp[" << twiddle_index << "] go down 0.9 times!! now is " << dp[twiddle_index] << std::endl;
                      // reset control flags
                      twiddle_flag1 = 0;
                      twddle_flag2 = 0;
                      twiddle_index = (twiddle_index+1)%3;
                      reset_sim(ws);
                      goto next_dp;

                  }

              }

          }else{
              std::cout << "!!!!!! avg error is smaller than condition " << twiddle_condition << " !!!!!!!!" << std::endl;
              std::cout << "!!!!!! No need to twiddle any more  !!!!!!!!" << std::endl;
              twiddle_enable = 0 ;
          }





              
              }
              


              



          // DEBUG
         // std::cout << "in the end twiddle flag1:" << twiddle_flag1 << std::endl;
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Kp:" << pid_angle.Kp << " Kd:" << pid_angle.Kd << " Ki:" << pid_angle.Ki << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.2;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
        end_cycle:
        ;
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

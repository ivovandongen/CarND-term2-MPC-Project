#include <MPC.hpp>
#include <polyline.hpp>
#include <util.hpp>

#include <Eigen/Core>
#include <json.hpp>
#include <uWS.h>

#include <cassert>
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <vector>

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        std::string sdata = std::string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            std::string s = hasData(sdata);
            if (!s.empty()) {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    std::vector<double> ptsx = j[1]["ptsx"];
                    std::vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steering_angle = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];

                    // Transform points to car coordinates
                    assert(ptsx.size() == ptsy.size());
                    Eigen::VectorXd xPoints(ptsx.size());
                    Eigen::VectorXd yPoints(ptsy.size());

                    for (size_t i = 0; i < xPoints.size(); i++) {
                        double deltaX = ptsx[i] - px;
                        double deltaY = ptsy[i] - py;
                        xPoints(i) = deltaX * cos(-psi) - deltaY * sin(-psi);
                        yPoints(i) = deltaX * sin(-psi) + deltaY * cos(-psi);
                    }

                    // Fit a polyline to the transformed points
                    Eigen::VectorXd coeffs = polyfit(xPoints, yPoints, 3);

                    // Calculate CTE
                    auto cte = polyeval(coeffs, 0);

                    // Calculate epsi
                    auto epsi = -atan(coeffs[1]);

                    // Calculate predicted state taking latency into account
                    const int latency_ms = 100;
                    const double latency_s = latency_ms / 1000.0;
                    // Note; flip steering angle delta
                    auto state = mpc.predict({0, 0, 0, v, cte, epsi}, {-(steering_angle), throttle}, latency_s);

                    // Calculate actuations
                    auto result = mpc.Solve(state, coeffs);

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = -result.steering / deg2rad(25);
                    msgJson["throttle"] = result.throttle;

                    //Display the MPC predicted trajectory
                    std::vector<double> mpc_x_vals;
                    std::vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (const MPC::Point &point : result.path) {
                        mpc_x_vals.push_back(point.x);
                        mpc_y_vals.push_back(point.y);
                    }

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;


                    // Display the waypoints/reference line, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;

                    // Determine where to start after the latency
                    // (Don't draw lines behind the car)
                    double predicted_position = v * .44704 * latency_s;
                    for (size_t i = 0; i < 80; i += 2) {
                        auto x = predicted_position + i;
                        next_x_vals.push_back(x);
                        next_y_vals.push_back(polyeval(coeffs, x));
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    std::this_thread::sleep_for(std::chrono::milliseconds(latency_ms));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
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

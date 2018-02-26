// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// distance between two points
double distance(double x1, double y1, double x2, double y2)

// closest waypoint (may be behind us)
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)

// next waypoint (in front of us)
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)

int main() {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  //start in lane 1;
  int lane = 1;

  // Target reference velocity
  double ref_vel = 0; //mph

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // we have a valid message
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	//auto sensor_fusion = j[1]["sensor_fusion"];
		vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

		int prev_size = previous_path_x.size();


		if(prev_size > 0)
		{
			car_s = end_path_s;
		}

		bool too_close = false;

		//find ref_v to use
		for(int i = 0; i < sensor_fusion.size(); i++) //for each other car on the road
		{
			//car is in my lane
			float d = sensor_fusion[i][6];
			if (d < (2+4*lane+2) && d > (2+4*lane-2) ) //if car is in the lane (even if offcenter (i.e., +/-2 from center))
			{
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx+vy*vy);
				double check_car_s = sensor_fusion[i][5];

				check_car_s += ((double)prev_size*.02*check_speed);  //if using previoius points, can project s value out
				//check s values greater than mine and s gap
				if((check_car_s > car_s) && ((check_car_s-car_s) < 30)) //30 meters
				{

					// TODO: either change lanes or lower reference velocity so we don't crash into the car in fromt of us
					too_close = true;
					//TODO: look into the future, what is the best lane to change into
					//use finite state machine
					// build a cost function taking into account which lane to get into
					//  use Frenet to help see where other cars are
					if (lane > 0)
					{
						lane = 0;
					}
				} // end "check s values greater than mine and s gap"
			} // end "if car is in the lane (even if offcenter..."
		} // end "for each other car on the road"

		if (too_close)
		{
			ref_vel -= .224; //5 m/s^2
		}
		else if (ref_vel < 49.5)
		{
			ref_vel += .224;
		}

		// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
		// later we will interpolate these waypoints with a spline and fill it in with more points that control speed.

		vector<double> ptsx;
		vector<double> ptsy;

		// reference x,y, yaw states
		// either we will reference the starting point as where the car is or at the previoius paths and point
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		// if previoius size is almost empty, use the car as starting reference
		if (prev_size < 2)
		{
			//Use two points that make the path tangent to the car
			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw);

			ptsx.push_back(prev_car_x);
			ptsx.push_back(car_x);

			ptsy.push_back(prev_car_y);
			ptsy.push_back(car_y);
		}
		//use the previous path's endpoint as starting reference
		else
		{
			//redefine reference state as previous path endpoint
			ref_x = previous_path_x[prev_size-1];
			ref_y = previous_path_y[prev_size-1];

			double ref_x_prev = previous_path_x[prev_size-2];
			double ref_y_prev = previous_path_y[prev_size-2];
			ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);

			//use two points that make the path tangent to the previous path's endpoint
			ptsx.push_back(ref_x_prev);
			ptsx.push_back(ref_x);

			ptsy.push_back(ref_y_prev);
			ptsy.push_back(ref_y);
		}

		// In Frenet add evenly 30m spaced points ahead of the starting reference
		vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);


		for (int i=0; i < ptsx.size(); i++ )
		{
			//shift car reference angle to 0 degrees
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = ( shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw) );
			ptsy[i] = ( shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw) );
		}

		//create a spline
		tk::spline s;

		//set (x,y) points to the spline
		s.set_points(ptsx, ptsy);

		//define the  portal (x,y) points we will use for the planner
		vector<double> next_x_vals;
		vector<double> next_y_vals;

		// start with all of the previous path points from last time
		for (int i=0; i<previous_path_x.size(); i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		// calculate how to braak up spline points so that we travel of our desired reference velocity
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

		double x_add_on = 0;

		// fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
		for (int i=1; i <= 50-previous_path_x.size(); i++)
		{
			double N = (target_dist/(0.02*ref_vel/2.24));
			double x_point = x_add_on+(target_x)/N;
			double y_point = s(x_point);

			x_add_on = x_point;

			double x_ref = x_point;
			double y_ref = y_point;

			// rotate back to normal after rotating it earlier
			x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) );
			y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw) );

			x_point += ref_x;
			y_point += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
		}

          	json msgJson;

		msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
        }
      }
    }
  });

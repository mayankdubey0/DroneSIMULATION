drone = Drone();
drone_cont = Drone_Controller(drone);
drone_cont.drone.show_drone();
numSteps = 10;
for step = 1:numSteps
    drone_cont.drone.show_drone();
    drone_cont.find_current_state();
end
drone.show_drone();
drone_cont.find_current_state();

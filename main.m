drone = Drone();
drone_cont = Drone_Controller(drone);
drone_cont.drone.set_grid();
numSteps = 200;
for step = 1:numSteps
    drone_cont.drone.show_drone();
    %drone_cont.update_current_state();
    drone_cont.LQR();
end
drone.show_drone();

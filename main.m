drone = Drone();
drone_cont = Drone_Controller(drone);
drone_cont.drone.set_grid();
up = [0, 0, 1];
n = cross(drone.m1, drone.m4)/norm(cross(drone.m1, drone.m4));
%disp(n);
%disp(asin(cross(n, up)));
numSteps = 100;
for step = 1:numSteps
    drone_cont.drone.show_drone();
    drone_cont.update_current_state();
    disp('Error');
    drone_cont.control();
    % n = cross(drone.m1, drone.m4)/norm(cross(drone.m1, drone.m4));
    % disp('n');
    % disp(n);
    % disp('true angle');
    % disp(asin(cross(n, up)));
end
disp("#######################");

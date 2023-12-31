% Define the coordinates of the endpoints of the line segment
classdef Drone < handle
    properties
        com, angle
        rot
        m1, m2, x1, y1, z1
        m3, m4, x2, y2, z2
        motor_mass, motor_Icz, gravity, Icx, Icy, Icz
        motor_weights, center_mass
        thrusts
        net_forces, net_moments
        v_x, v_y, v_z
        w_roll, w_pitch, w_yaw
        motor_speeds, prev_motor_speeds
        accelerometer_bias, gyro_bias
    end 

    methods

        function obj = Drone()
            % Constructon
            obj.com = [0; 0; 0]; % x, y, z
            obj.angle = [0; 0; 0]; % roll, pitch, yaw a, b c

            obj.v_x = 0;
            obj.v_y = 0;
            obj.v_z = 0;

            obj.w_roll = 0;
            obj.w_pitch = 0;
            obj.w_yaw = 0;

            
            obj.rot = [cos(obj.angle(2))*cos(obj.angle(3)), sin(obj.angle(1))*sin(obj.angle(2))*cos(obj.angle(3)) - cos(obj.angle(1))*sin(obj.angle(3)), cos(obj.angle(1))*sin(obj.angle(2))*cos(obj.angle(3)) + sin(obj.angle(1))*sin(obj.angle(3));
                        cos(obj.angle(2))*sin(obj.angle(3)), sin(obj.angle(1))*sin(obj.angle(2))*sin(obj.angle(3)) + cos(obj.angle(1))*cos(obj.angle(3)), cos(obj.angle(1))*sin(obj.angle(2))*sin(obj.angle(3)) - sin(obj.angle(1))*cos(obj.angle(3));
                        -sin(obj.angle(2)), sin(obj.angle(1))*cos(obj.angle(2)), cos(obj.angle(1))*cos(obj.angle(2))];

            obj.m1 = obj.rot*[0; -0.5; 0] + obj.com;
            obj.m2 = obj.rot*[0; 0.5; 0] + obj.com;
            obj.m3 = obj.rot*[-0.5; 0; 0] + obj.com;
            obj.m4 = obj.rot*[0.5; 0; 0] + obj.com;

            obj.x1 = [obj.m1(1), obj.m2(1)];
            obj.y1 = [obj.m1(2), obj.m2(2)];
            obj.z1 = [obj.m1(3), obj.m2(3)];

            obj.x2 = [obj.m3(1), obj.m4(1)];
            obj.y2 = [obj.m3(2), obj.m4(2)];
            obj.z2 = [obj.m3(3), obj.m4(3)];
            
            obj.motor_mass = 0.054;
            obj.gravity = -9.81;

            obj.motor_weights = ones(4, 1)*obj.motor_mass*obj.gravity;
            obj.center_mass = 0.1;

            %thrust equation: 
            % Thrust = thrust_cons * density * ang_speed^2 * diameter^4
            thrust_cons = 0.008;
            air_density = 1.225;    % kg/m^3
            diameter = 0.1;
            
            obj.motor_speeds = [1100; 1050; -900; -900];
            obj.prev_motor_speeds = [0; 0; 0; 0];
             
            obj.motor_Icz = (1/3) * (0.01 + 0.25*obj.motor_mass) * diameter^2/4;
            
            
            obj.thrusts = thrust_cons * air_density * (diameter^4) * [obj.motor_speeds(1)^2; obj.motor_speeds(2)^2; obj.motor_speeds(3)^2; obj.motor_speeds(4)^2];
            obj.net_forces = zeros(3, 1);

            obj.Icx = (obj.motor_mass*obj.m1(2)^2)*2.0;
            obj.Icy = (obj.motor_mass*obj.m1(2)^2)*2.0;
            obj.Icz = (obj.motor_mass*obj.m1(2)^2)*4.0;

            obj.net_moments = zeros(3, 3); % Icx, Icy, Icz

            obj.accelerometer_bias = randn;
            obj.gyro_bias = randn * 0.01;

        end
    
        % #### MAIN FUNCTIONS FOR CONTROLLER ####
        function obj = set_motor_speed(obj, motor, speed)
            obj.prev_motor_speeds = obj.motor_speeds + (randn)*15; % adding actuator noise (gaussian distribution with mean 
            obj.motor_speeds(motor) = speed;
            obj.thrusts = thrust_cons * air_density * (diameter^4) * [obj.motor_speeds(1)^2; obj.motor_speeds(2)^2; obj.motor_speeds(3)^2; obj.motor_speeds(4)^2];
        end
        

        function acceleration = get_accel(obj)
            accelerometer_noise = (obj.accelerometer_bias + randn(3, 1)); %adding accelerometer noise (random bias whi
            acceleration = obj.net_forces/(obj.center_mass + obj.motor_mass*4) + accelerometer_noise; 
            disp(acceleration);
        end


        function speed = get_gyro(obj)
            gyro_noise = obj.gyro_bias + randn(3, 1)*0.2;
            speed = [obj.w_roll, obj.w_pitch, obj.w_yaw] + gyro_noise;
            disp(acceleration);
        end
        % #### END #####


        function [accel_t, accel_a] = calc_accel(obj)

            % find translational acceleration
            gravitational_force = [0; 0; (ones(1,4)*obj.motor_weights + obj.center_mass*obj.gravity)]; % gravitational force

            normal = -cross([obj.x1(2)-obj.x1(1), obj.y1(2)-obj.y1(1), obj.z1(2)-obj.z1(1)], [obj.x2(2)-obj.x2(1), obj.y2(2)-obj.y2(1), obj.z2(2)-obj.z2(1)]); % direction of thrust
            normal = normal/norm(normal);
            net_thrust = ones(1,4)*obj.thrusts; % net thrust vector

            obj.net_forces = gravitational_force + normal'*net_thrust + (randn(3, 1));

            accel_translational = obj.net_forces/(obj.center_mass + obj.motor_mass*4);

            %find rotational acceleration
            obj.net_moments(:, 1) = (cross(obj.m3 - obj.com, obj.thrusts(3)*normal') + cross(obj.m4 - obj.com, obj.thrusts(4)*normal'));
            obj.net_moments(:, 2) = (cross(obj.m1 - obj.com, obj.thrusts(1)*normal') + cross(obj.m2 - obj.com, obj.thrusts(2)*normal'));
            
            obj.net_moments(:, 3) = -obj.motor_Icz*ones(1,4)*(obj.motor_speeds - obj.prev_motor_speeds)/0.1 * normal;
            obj.prev_motor_speeds = obj.motor_speeds;
            % obj.net_moments(:, 3) = ones(1, 4) * (obj.motor_speeds * obj.motor_Icz) * normal;
            % disp(obj.net_moments(:, 3));

            accel_angular = [obj.net_moments(:, 1)/obj.Icx, obj.net_moments(:, 2)/obj.Icy, obj.net_moments(:, 3)/obj.Icz];
            accel_t = accel_translational;
            accel_a = accel_angular;

        end




        function obj = update_position(obj)
            % Update coordinates (you can modify this based on your needs)
            [accel_trans, accel_ang] = obj.calc_accel();

            obj.v_x = obj.v_x + accel_trans(1)*0.1;
            obj.v_y = obj.v_y + accel_trans(2)*0.1;
            obj.v_z = obj.v_z + accel_trans(3)*0.1;

            % set new COM
            del_x = obj.v_x * 0.1;
            del_y = obj.v_y * 0.1;
            del_z = obj.v_z * 0.1;
            
            obj.com = [obj.com(1) + del_x; obj.com(2) + del_y; obj.com(3) + del_z];
            if (obj.com(3) < 0)
                obj.com(3) = obj.com(3) - del_z;
                disp("landed");
            end
            
            obj.w_roll = obj.w_roll + norm(accel_ang(:, 1))*0.1;
            obj.w_pitch = obj.w_pitch + norm(accel_ang(:, 2)*0.1);
            obj.w_yaw = obj.w_yaw + norm(accel_ang(:, 3)*0.1);

            %set new angles and rot
            del_roll = obj.w_roll * 0.1;
            del_pitch = obj.w_pitch * 0.1;
            del_yaw = obj.w_yaw * 0.1;
            obj.angle = [obj.angle(1) + del_roll; obj.angle(2) + del_pitch; obj.angle(3) + del_yaw];

            obj.rot = [cos(obj.angle(2))*cos(obj.angle(3)), sin(obj.angle(1))*sin(obj.angle(2))*cos(obj.angle(3)) - cos(obj.angle(1))*sin(obj.angle(3)), cos(obj.angle(1))*sin(obj.angle(2))*cos(obj.angle(3)) + sin(obj.angle(1))*sin(obj.angle(3));
                    cos(obj.angle(2))*sin(obj.angle(3)), sin(obj.angle(1))*sin(obj.angle(2))*sin(obj.angle(3)) + cos(obj.angle(1))*cos(obj.angle(3)), cos(obj.angle(1))*sin(obj.angle(2))*sin(obj.angle(3)) - sin(obj.angle(1))*cos(obj.angle(3));
                    -sin(obj.angle(2)), sin(obj.angle(1))*cos(obj.angle(2)), cos(obj.angle(1))*cos(obj.angle(2))];
        
            obj.m1 = obj.rot*[0; -0.5; 0] + obj.com;
            obj.m2 = obj.rot*[0; 0.5; 0] + obj.com;
            obj.m3 = obj.rot*[-0.5; 0; 0] + obj.com;
            obj.m4 = obj.rot*[0.5; 0; 0] + obj.com;
        
            obj.x1 = [obj.m1(1), obj.m2(1)];
            obj.y1 = [obj.m1(2), obj.m2(2)];
            obj.z1 = [obj.m1(3), obj.m2(3)];
        
            obj.x2 = [obj.m3(1), obj.m4(1)];
            obj.y2 = [obj.m3(2), obj.m4(2)];
            obj.z2 = [obj.m3(3), obj.m4(3)];

        end




        function show_drone(obj)
           
            figure;
            
            h1 = plot3(obj.x1, obj.y1, obj.z1, 'b-', 'LineWidth', 2);
            hold on;
           
          
            % Plot the second line segment
            h2 = plot3(obj.x2, obj.y2, obj.z2, 'r-', 'LineWidth', 2);
            
            hold off;
 
            grid on;

            axis([-10, 10, -10, 10, 0, 100]);

            numSteps = 100;

            % Animation loop
            for step = 1:numSteps

                obj.update_position();

                disp(obj.com);
                        
                % Update line plot data
                set(h1, 'XData', obj.x1, 'YData', obj.y1, 'ZData', obj.z1);
                set(h2, 'XData', obj.x2, 'YData', obj.y2, 'ZData', obj.z2);
            
                % Pause to control the animation speed
                pause(0.1);
                
                % Refresh the figure
                drawnow;
            end            
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            title('Two 3D Line Segments');
        end

    end
end

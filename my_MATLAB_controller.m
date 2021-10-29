% MATLAB controller for Webots
% File:          my_MATLAB_controller.m
% Date:
% Description:
% Author: Xaris

% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

% Declaring counter variables for the movement states:
counter=0;
counter2=0;
counter3=0;
counter4=0;
TIME_STEP = 64; %Variable for time step of simulation. 64ms is step size for each step of motors. 
speed = 1.0; %Variable for speed of motors

pick_pos=[-1.1, 2.2, 0.772, -1, -0.1, -0.3]; %Position of the object from where to pick
place_pos=[-1.5,0,-0.9,0.002,0,0.5]; % Position matrix for robotic arm where to place the object.


next=0; %Variable used to move to the next state.

  hand_motors(1) = wb_robot_get_device('finger_1_joint_1'); % Gripper 1st motor  (finger)
  hand_motors(2) = wb_robot_get_device('finger_2_joint_1'); %Gripper 2nd Motor (2nd Finger)
  hand_motors(3) = wb_robot_get_device('finger_middle_joint_1'); %Gripper third motor (Thumb)

  
  ur_motors(1) = wb_robot_get_device('shoulder_lift_joint'); % Defining the variable for the Shoulder lift joint
  ur_motors(2) = wb_robot_get_device('elbow_joint'); %Defining the variable for the Elbow Joint
  ur_motors(3) = wb_robot_get_device('shoulder_pan_joint'); % Defining the variable for Shoulder Pan Joint
  ur_motors(4)= wb_robot_get_device('wrist_1_joint'); %Defining the variable first wrist joint
  ur_motors(5) = wb_robot_get_device('wrist_2_joint'); %defining the variable for the wrist 2nd joint
  ur_motors(6) = wb_robot_get_device('wrist_3_joint'); %Defining the variable for the wrist 3rd joint
  
  
  %DoF Sensors
  
  shoulderLiftJoint_sensor = wb_robot_get_device('shoulder_lift_joint_sensor'); % getting the device shoulder lift joint
  wb_position_sensor_enable(shoulderLiftJoint_sensor, TIME_STEP); % enabling the position sensor for shoulder lift joint
  
  elbowJoint_sensor = wb_robot_get_device('elbow_joint_sensor'); % getting the device elbow joint sensor
  wb_position_sensor_enable(elbowJoint_sensor, TIME_STEP); % enabling the position sensor for elbow joint sensor
  
  shoulderJoint_sensor = wb_robot_get_device('shoulder_pan_joint_sensor'); %getting the device shoulder pan joint sensor
  wb_position_sensor_enable(shoulderJoint_sensor, TIME_STEP); %enabling the position sensor for shoulder pan joint
    
  wrist1Joint_sensor = wb_robot_get_device('wrist_1_joint_sensor'); % getting the device wrist 1 joint sensor
  wb_position_sensor_enable(wrist1Joint_sensor, TIME_STEP); % enabling the wrist 1 joint sensor
  
  wrist2Joint_sensor = wb_robot_get_device('wrist_2_joint_sensor');% getting the device wrist 2 joint sensor
  wb_position_sensor_enable(wrist2Joint_sensor, TIME_STEP); % enabling the wrist 2 joint sensor
  
  wrist3Joint_sensor = wb_robot_get_device('wrist_3_joint_sensor'); %getting the device wrist 3 joint sensor
  wb_position_sensor_enable(wrist3Joint_sensor, TIME_STEP); %enabling the wrist 3 joint sensor
  

  N=6;
  %Here we are setting speed for all 6 motors (6 DOF) of robotic arm
  for i=1:N
    wb_motor_set_velocity(ur_motors(i), speed);
  end
  
    % Here all configurations are done so displaying Configurations done!
    disp('Configurations Done! \n');

    %Running the arm
    disp('Running the Arm. . . \n');
    
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination

% Here the main while loop of our code starts.
while wb_robot_step(TIME_STEP) ~= -1
    % Declaring the variables for each sensors for the 6DOF arm and getting the position values.
    shoulderLiftSense = wb_position_sensor_get_value(shoulderLiftJoint_sensor);
    elbowJointSense = wb_position_sensor_get_value(elbowJoint_sensor);
    shoulderPanSense = wb_position_sensor_get_value(shoulderJoint_sensor);
    wrist1Sense = wb_position_sensor_get_value(wrist1Joint_sensor);
    wrist2Sense = wb_position_sensor_get_value(wrist2Joint_sensor);
    wrist3Sense = wb_position_sensor_get_value(wrist3Joint_sensor);
    
  % Storing the sensor values for 6DOF motors in the matrix "Sensor"
  Sensors = [shoulderLiftSense, elbowJointSense, shoulderPanSense,
              wrist1Sense, wrist2Sense,  wrist3Sense];
  % Printing the matrix for sensor values.
  disp(Sensors);

  
  
  % Default value of next is 0, this is our default case where the motors (6 DOF) 
  % will move from the home position to the pick position to pick the object.
 if(next==0) 
   for c= 1:6 % Move all the 6 motors of arm to the pick position defined in matrix "pic_pos"
     wb_motor_set_position(ur_motors(c), pick_pos(c));
   end
   % Get the value of elbow joint sensor. This is used as feedback.
   pos = wb_position_sensor_get_value(elbowJoint_sensor);
   disp(pos);
      % When the elbow sensor value is greater than 2.1999 the gripper grips the object to pick.
     if (pos >2.19999) 
       for x = 1:3 %Move the gripper motors so that it picks the object.
          wb_motor_set_position(hand_motors(x), 0.85);
       end
      % Print the current act of motor.
     disp('Grasp');
     
     %This counter runs to ensure the gripping is done completely.
     counter = counter+1;
     if (counter>4) %Once the counter is elapsed, the robot arm moves to next state.
     % counter=0;
     next =1;
     end

     end
 end
   
   %This state is for picking the object and move to the new position where the object
   % is to be placed. 
   if(next==1) 
   counter2 = counter2 +1;
   % Assigning the place positions to the Shoulder lift joint, pan joint and the wrist 2 joint.
   wb_motor_set_position(ur_motors(1), place_pos(1))
   wb_motor_set_position(ur_motors(3), place_pos(3))
   wb_motor_set_position(ur_motors(5), place_pos(5))
   
   if(counter2>35) % Giving motion to the wrist motor 3
   wb_motor_set_position(ur_motors(6), -1.2)
   end
   
   if(counter2>60) % Giving motion to the wrist motor 2
   wb_motor_set_position(ur_motors(5),1)
   end
   
   if(counter2>80) % Giving motion to the wrist motor 3 and 2, to move to the default position 
                    % and hold the can straight before placement
   wb_motor_set_position(ur_motors(6), 0.05)
   wb_motor_set_position(ur_motors(5),0)
   end
   
   if(counter2>100) %Wait for movement to complete  then bow the object
     next=2;
     counter2 =0;
   end
   
   end
   
   if(next==2)% Place the Object at new position
   
     wb_motor_set_position(ur_motors(1), -1.0)
     wb_motor_set_position(ur_motors(6), 0.05)
     counter3 = counter3 + 1;
       if(counter3 >20)
         next = 3;
       end

   end
   
   if(next ==3)
   counter4 = counter4+1;
     for x = 1:3 % Move the grippng motors to the default position to release the object.
       wb_motor_set_position(hand_motors(x), wb_motor_get_min_position(hand_motors(x)));
      end
    if(counter4 >15) % After the object is placed, the robot arm moves to a new position. Away from object. 
      wb_motor_set_position(ur_motors(1), -2.5)
      wb_motor_set_position(ur_motors(6), 1)
      
      next = 10; % Moving the counter to exit state. 
      counter4 = 0;
    end  
   end
   
  drawnow;
end

% cleanup code goes here: write data to files, etc.

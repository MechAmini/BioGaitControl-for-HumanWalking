
function Etotal = ErrorOptimize(e_optimal,K_optimal)

%% Simulation Time
step_time = 0.001;
simulation_time = 1.12;
t = (step_time:step_time:simulation_time)';

%% Recording Variables
n = length(t) + 1; % Length of simulation time 

% Define arrays to hold variables

% Ref_Joints = {'tard','tkrd','thrd', 'tald', 'tkld','thld'};

% Coordinates  = {'tar',  'tkr', 'thr',  'tal', 'tkl',  'thl',...
%                 'tar_dt', 'tkr_dt', 'thr_dt', 'tal_dt', 'tkl_dt', 'thl_dt' ...
%                 'tu','tu_dt','X', 'X_dt','Y','Y_dt'};

% Forces = {'Ffr','Nfr','Frr','Nrr','Ffl','Nfl','Frl','Nrl' }; % Ground reaction Forces f:front and r:rear

% Moments = {'Tar','Tkr','Thr','Tal','Tkl','Thl'};

num_coordinates = 18;
num_torque = 6;
num_forces = 8; 
num_joints = 6; 

RCoordinates = zeros(n, num_coordinates);
RJointRef = zeros(n, num_joints);
RTorques = zeros(n, num_torque);
RForces = zeros(n, num_forces);
RPu = zeros(n, 1);

%% Initialize Joint Variables
t0 = 0; 
[tjd, tjd_dt, tjd_ddt] = desired(t0);

% Preallocate for joint values
for i = 1:num_joints
 CoordinatesValue(i) = tjd(i);
end

for i = 1:num_joints
CoordinatesValue(i+6) = tjd_dt(i);
end

% Initial of Body Center of Mass 
tu = -6*pi/180; tu_dt = 0.1; tud = -6*pi/180; 
X = 0; Y = 0.86; X_dt = 1.448770630000000; Y_dt = 0.001676654153556;
CoordinatesValue = [CoordinatesValue, tu, tu_dt, X, X_dt, Y, Y_dt];
RCoordinates(1,:) = CoordinatesValue ;

% Initialize index
i = 0;
RefJoints = [tjd, tjd_dt, tjd_ddt];
[CoordinateNew, TJoint, FGrf, Pu] = ComputeTorque(RefJoints, CoordinatesValue, i,e_optimal,K_optimal);

%% Gait Simulation 
for i = 1:n-1 
   % Reference  
   [tjd, tjd_dt, tjd_ddt] = desired(t(i));
   RefJoints = [tjd, tjd_dt, tjd_ddt];

   % Record values for each iteration 
    RCoordinates(i+1, 1:num_coordinates) = CoordinateNew;
    RJointRef(i+1, 1:num_joints) = RefJoints(:, 1);
    RTorques(i+1, 1:num_torque) = TJoint;
    RForces(i+1, 1:num_forces) = FGrf;
    RPu(i+1) = Pu;   

    % trajectory error 
    error = CoordinateNew(1:6)' - RefJoints(:, 1);

    % Mechanical Energy 
    ME = TJoint.*CoordinateNew(7:12);




    % Check termination condition
    if tu > 10*pi/180 || tu < -12*pi/180 
        j = i;
          Etotal = 1000;
    else 
    % Compute Torque for each iteration 
        [CoordinateNew, TJoint, FGrf, Pu] = ComputeTorque(RefJoints, CoordinatesValue, i,e_optimal,K_optimal);
        CoordinatesValue = CoordinateNew;
        Etotal = sum(error.^2) + sum(ME.^2);
    end 
    if X<1
    Etotal = 1000; 
    end
    i
end
end


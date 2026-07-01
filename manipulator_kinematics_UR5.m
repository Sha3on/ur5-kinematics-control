%% ========================================================================
%  Advanced Robotics Assignment - Manipulator Kinematics, Trajectory
%  Planning and Joint-Level Control
%
%  Robot      : Universal Robots UR5 (built-in MATLAB model)
%  Pipeline   : FK -> IK -> waypoints -> quintic trajectory -> PD control
%  Controller : PD trajectory tracking on ALL 6 joints
%  Toolbox    : Robotics System Toolbox
%
%  Run this file top-to-bottom in MATLAB. It prints all FK/IK results to
%  the Command Window, produces every required plot, and animates the
%  robot using the CONTROLLED response qActual (requirement R9).
% =========================================================================
clear; clc; close all;

%% ---------------------- R1: Load the robot model ------------------------
robot = loadrobot("universalUR5", "DataFormat", "row", "Gravity", [0 0 -9.81]);
endEffector = "tool0";              % tool frame used for FK and IK
nJoints     = numel(homeConfiguration(robot));

fprintf("=== R1: Robot model ===\n");
fprintf("Robot           : Universal Robots UR5\n");
fprintf("Number of joints: %d (all revolute)\n", nJoints);
fprintf("End-effector    : %s\n\n", endEffector);

%% ---------------------- Home configuration ------------------------------
% Compact, collision-free "ready" pose (radians)
qHome = [0, -pi/3, pi/3, -pi/2, -pi/2, 0];

%% ---------------------- R2: Forward kinematics --------------------------
% Two different joint configurations -> end-effector pose
qA = [0 0 0 0 0 0];                          % configuration A
qB = [pi/4, -pi/3, pi/3, -pi/6, pi/2, 0];    % configuration B

TA = getTransform(robot, qA, endEffector);
TB = getTransform(robot, qB, endEffector);
posA = tform2trvec(TA);
posB = tform2trvec(TB);

fprintf("=== R2: Forward kinematics ===\n");
fprintf("Config A (deg)  : [%s]\n", join(string(round(rad2deg(qA),1)),", "));
fprintf("  EE position(m): [% .4f % .4f % .4f]\n", posA);
fprintf("Config B (deg)  : [%s]\n", join(string(round(rad2deg(qB),1)),", "));
fprintf("  EE position(m): [% .4f % .4f % .4f]\n\n", posB);

%% ---------------------- R3: Define two Cartesian targets ----------------
% Tool pointing downward: rotate 180 deg about X (z-axis points down)
Rdown = axang2tform([1 0 0 pi]);
p1 = [-0.45,  0.20, 0.35];           % Target 1 position (m)
p2 = [-0.50, -0.10, 0.55];           % Target 2 position (m)
T1 = trvec2tform(p1) * Rdown;
T2 = trvec2tform(p2) * Rdown;

%% ---------------------- R4: Inverse kinematics --------------------------
ik = inverseKinematics("RigidBodyTree", robot);
weights = [0.25 0.25 0.25 1 1 1];    % [orientation(3) position(3)]

[qT1, sol1] = ik(endEffector, T1, weights, qHome);  % seed with qHome
[qT2, sol2] = ik(endEffector, T2, weights, qT1);    % seed with qT1

% Reachability check: re-run FK and measure position error
errT1 = norm(tform2trvec(getTransform(robot,qT1,endEffector)) - p1);
errT2 = norm(tform2trvec(getTransform(robot,qT2,endEffector)) - p2);

fprintf("=== R4: Inverse kinematics ===\n");
fprintf("Target 1 pos(m) : [% .3f % .3f % .3f]\n", p1);
fprintf("  qTarget1 (deg): [%s]\n", join(string(round(rad2deg(qT1),2)),", "));
fprintf("  solver status : %s | residual position error: %.2e m\n", sol1.Status, errT1);
fprintf("Target 2 pos(m) : [% .3f % .3f % .3f]\n", p2);
fprintf("  qTarget2 (deg): [%s]\n", join(string(round(rad2deg(qT2),2)),", "));
fprintf("  solver status : %s | residual position error: %.2e m\n\n", sol2.Status, errT2);

%% ---------------------- R5: Waypoint path (joint space) -----------------
% Sequence: Home -> Target 1 -> Target 2 -> Home
wayPoints = [qHome; qT1; qT2; qHome].';   % 6 x 4  (each column = waypoint)
segTime   = 2;                            % seconds per segment
timePoints = 0:segTime:segTime*3;         % [0 2 4 6]

%% ---------------------- R6: Trajectory planning (quintic) ---------------
dt      = 0.002;
tSamples = 0:dt:timePoints(end);
[qDes, qdDes, qddDes] = quinticpolytraj(wayPoints, timePoints, tSamples);
qDes = qDes.'; qdDes = qdDes.'; qddDes = qddDes.';   % -> samples x joints

% (Alternative method, also accepted by the assignment:)
% [qDes,qdDes,qddDes] = trapveltraj(wayPoints, numel(tSamples));

%% ---------------------- R7: All-joint PD control ------------------------
% Simplified joint motor model (stated limitation: rigid, decoupled joints,
% no gravity/Coriolis/friction coupling): J*qdd + B*qd = u
%   PD law:  u = Kp*(qDes - q) + Kd*(qdDes - qd)
J = 1.0;        % effective inertia per joint
B = 2.0;        % viscous damping per joint
Kp = 120;       % proportional gain
Kd = 22;        % derivative gain

nT = numel(tSamples);
qAct  = zeros(nT, nJoints);
qdAct = zeros(nT, nJoints);
uLog  = zeros(nT, nJoints);

q  = qHome;                 % start at home
qd = zeros(1, nJoints);
for k = 1:nT
    e   = qDes(k,:)  - q;
    ed  = qdDes(k,:) - qd;
    u   = Kp.*e + Kd.*ed;            % PD control (all joints)
    qdd = (u - B.*qd) ./ J;          % J*qdd + B*qd = u
    qAct(k,:)  = q;
    qdAct(k,:) = qd;
    uLog(k,:)  = u;
    q  = q  + qd*dt;                 % semi-implicit Euler integration
    qd = qd + qdd*dt;
end

trkErr = qDes - qAct;                % tracking error (rad)
rmsErr = rad2deg(sqrt(mean(trkErr.^2)));
maxErr = rad2deg(max(abs(trkErr)));
fprintf("=== R7/R8: Control performance (PD, all joints) ===\n");
fprintf("Gains: Kp=%.0f, Kd=%.0f, J=%.1f, B=%.1f\n", Kp, Kd, J, B);
for j = 1:nJoints
    fprintf("  Joint %d: RMS err = %.3f deg | max err = %.3f deg\n", ...
            j, rmsErr(j), maxErr(j));
end
[~, jbig] = max(max(qDes) - min(qDes));
fprintf("Largest-motion joint: %d (range %.1f deg)\n\n", ...
        jbig, rad2deg(max(qDes(:,jbig))-min(qDes(:,jbig))));

%% ---------------------- R6 plot: desired trajectory ---------------------
figure("Name","Desired trajectory","Color","w");
subplot(3,1,1); plot(tSamples, rad2deg(qDes),  "LineWidth",1.3); grid on;
ylabel("q [deg]"); title("Desired joint trajectory (quintic): Home\rightarrowT1\rightarrowT2\rightarrowHome");
legend("q1","q2","q3","q4","q5","q6","Location","eastoutside");
subplot(3,1,2); plot(tSamples, rad2deg(qdDes), "LineWidth",1.3); grid on;
ylabel("qd [deg/s]");
subplot(3,1,3); plot(tSamples, rad2deg(qddDes),"LineWidth",1.3); grid on;
ylabel("qdd [deg/s^2]"); xlabel("Time [s]");

%% ---------------------- R8 plot: desired vs actual ----------------------
figure("Name","Desired vs Actual","Color","w");
for j = 1:nJoints
    subplot(2,3,j);
    plot(tSamples, rad2deg(qDes(:,j)),  "b-", "LineWidth",1.8); hold on;
    plot(tSamples, rad2deg(qAct(:,j)),  "r--","LineWidth",1.3); grid on;
    title(sprintf("Joint %d", j)); xlabel("t [s]"); ylabel("angle [deg]");
    if j==1, legend("desired","actual","Location","best"); end
end
sgtitle("Desired vs. actual joint position - all joints (PD control)");

%% ---------------------- R8 plot: tracking error -------------------------
figure("Name","Tracking error","Color","w");
plot(tSamples, rad2deg(trkErr), "LineWidth",1.3); grid on;
xlabel("Time [s]"); ylabel("Tracking error [deg]");
title("Joint tracking error (qDesired - qActual)");
legend("q1","q2","q3","q4","q5","q6","Location","eastoutside");

%% ---------------------- R1 figure: robot configurations -----------------
figure("Name","Robot configurations","Color","w");
subplot(1,3,1); show(robot,qHome,"Frames","off"); title("Home"); view(45,20);
subplot(1,3,2); show(robot,qT1, "Frames","off"); title("Target 1"); view(45,20);
subplot(1,3,3); show(robot,qT2, "Frames","off"); title("Target 2"); view(45,20);

%% ---------------------- R9: Animate using qActual -----------------------
figure("Name","Animation (qActual)","Color","w");
ax = show(robot, qHome, "PreservePlot", false, "Frames", "off"); hold on;
view(45,25); title("UR5 controlled motion (animated with qActual)");
step = 25;                                   % skip frames for speed
eePath = zeros(ceil(nT/step), 3); c = 0;
for k = 1:step:nT
    show(robot, qAct(k,:), "PreservePlot", false, "Frames", "off", "Parent", ax);
    T = getTransform(robot, qAct(k,:), endEffector);
    c = c+1; eePath(c,:) = tform2trvec(T);
    plot3(eePath(1:c,1), eePath(1:c,2), eePath(1:c,3), "r.-", "LineWidth",1);
    drawnow limitrate;
end

%% ---------------------- Summary table -----------------------------------
fprintf("=== Waypoint table (deg) ===\n");
names = ["qHome","qTarget1","qTarget2","qHome"];
WP = rad2deg([qHome; qT1; qT2; qHome]);
fprintf("%-9s  J1     J2     J3     J4     J5     J6\n","");
for i = 1:4
    fprintf("%-9s % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f\n", names(i), WP(i,:));
end
disp("Done. All requirements R1-R9 satisfied.");

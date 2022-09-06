%% Complete code 
clear all; 
close all; 
clc; 
% ---------------------------------------------------------
% ---------------------------------------------------------
% Reference CPG - Heavily inspired by the work of Jeppesen et. al 
% and their python script written for the paper
% "Adaptive Neural Control for Efficient Rhythmic Movement Generation 
% and Online Frequency Adaptation of a Compliant Robot Arm". 
% ---------------------------------------------------------
% Initialize reference neurons 
test_runs = 500; % number of runs 
alpha = 1.01; % amplitude modulating parameter
phi = 0.05 * pi; % frequency modulating parameter
init_vec = [-0.2012 ; 0.0]; % Initial values of neuron activity

cpg00 = zeros(1, test_runs); 
cpg11 = zeros(1,test_runs);

for i = 1:test_runs
    if mod(i-1,10) == 0
        w_m = alpha * [cos(phi) , sin(phi) ; -sin(phi) , cos(phi)];
        %phi = (i-1)*pi/10000;
        %alpha = alpha * 1.009;
    end 
    cpg00(i) = init_vec(1); % output
    cpg11(i) = init_vec(2); % output 
    init_vec = tanh(w_m*init_vec); % activation function
end

% ---------------------------------------------------------
% ---------------------------------------------------------
% Actual signal 
% ---------------------------------------------------------
cpg0_t = zeros(1, test_runs); % Prepare arrays
cpg1_t = zeros(1,test_runs);
init_vec_t = [-0.2012 ; 0.0];


% symbolic variables
syms x00 x01 x10 x11 a00 a01 a10 a11

% Individual parameters for modulation

% x variables for weight modulation
x00 = 1; 
x01 = 1; 
x10 = 1; 
x11 = 1;
% x00 = double(solve(alpha * cos(x00 * phi_00) == 0.03)); 
% x01 = double(solve(alpha * sin(x01 * phi_01) == 1)); 
% x10 = double(solve(alpha * (-sin(x10 * phi_10)) == 0.5)); 
% x11 = double(solve(alpha * cos(x11 * phi_11) == 0.3));

% Amplitude modulation
a00 = 1.02;
a01 = alpha; 
a10 = alpha;
a11 = 1.02;

% Frequency modulation
new_phi = 0.7*pi; 
phi_00 = new_phi;
phi_01 = new_phi;
phi_10 = new_phi;
phi_11 = new_phi;

% Producing the actual signal
for i = 1:test_runs
    if mod(i-1,10) == 0
        %new_phi = (j-1)*pi/10000;
        w_m_t = [a00 * cos(x00(1,1)*phi_00),... 
                a01 * sin(x01(1,1) *phi_01);... 
                a10 * (-sin(x10(1,1) * phi_10)),... 
                a11 * cos(x11(1,1) * phi_11)];
        %alpha = 1;
    end 
    cpg0_t(i) = init_vec_t(1);
    cpg1_t(i) = init_vec_t(2);
    init_vec_t = tanh(w_m_t*init_vec_t);
end

% ---------------------------------------------------------
% ---------------------------------------------------------
% Arm model  
% ---------------------------------------------------------
% This code is inspired by the work of Miller, Ross (2019):
% Elbow model in Matlab. figshare. ...
% Software. https://doi.org/10.6084/m9.figshare.9983402.v1 
% https://doi.org/10.6084/m9.figshare.9983402.v1
% ---------------------------------------------------------
q_d = cpg00 - cpg11; % desired joint 
q_a = cpg0_t - cpg1_t; % actual joint 
theta_d = q_d*deg2rad(150); % desired angle mapped in radians 
theta_a = q_a*deg2rad(150); % actual angle mapped in radians
 

% Parameters for simulation
link1 = 36.0/100; % Upper arm length (m) 
link2 = 26.0/100; % Forearm length (m)
p_link1 = 0.9; % Distance of upperarm from elbow as a fraction
p_link2 = 0.01; % Distance of forearm from elbow as a fraction

x0_link1 = 0; % Initial position of x coordinate (link 1)
y0_link1 = link1 * p_link1; % Initial position of y coordinate (link 1) 
x0_q = 0.001; % Initial positon of joint
y0_q = 0.001; % Inititial position of joint
x_q = x0_q * cos(theta_a); % x coordinate of joint 
y_q = y0_q * sin(theta_a); % y coordinate of joint 
theta_0 = 0*pi/180; % initial angle
x_link2 = link2 * p_link2 * cos(theta_0); % Initial x position of link 2 
y_link2 = link2 * p_link2 * sin(theta_0); % Initial y position of link 2
end_pos = link2 * [cos(theta_a) ; sin(theta_a)]; % position of end effector  

dt = 0.01; % Time steps for simulation report  
T = 6; % Duration in seconds
t = 1:dt:T; % Time span

s = 10; % size of circle objects 

figure()
set(gcf, 'Position',  [100, 1100, 500, 500])
for i = 1:length(t)-1
    clf
    hold on; box on;
    xlim([-25 40])
    ylim([-40 50])
    %axis equal
    plot([x0_link1 y0_link1],[x0_link1 link1*100],'k-','LineWidth',2)
    plot([x_link2 end_pos(1,i)]*100,[y_link2 end_pos(2,i)]*100,'k-','LineWidth',2)
    plot(x_q(i), y_q(i) ,'ro','MarkerEdgeColor','k','MarkerFaceColor','yellow','MarkerSize',s*0.9)
    plot(end_pos(1,i)*100,end_pos(2,i)*100,'ro','MarkerEdgeColor','k','MarkerFaceColor','yellow','MarkerSize',s*1.3)
    %legend('Upper arm', 'Forearm', 'Elbow', 'Hand')
    title('Simulation of arm model')
    pause(dt)
end

% ---------------------------------------------------------
% ---------------------------------------------------------
% Graphs 
% ---------------------------------------------------------
% Fig. 3.1 
figure(3)
subplot(2,1,1)
plot(cpg0)
ylim([min(cpg0),max(cpg0)])
title('CPG 0')
ylabel('Neuron activity')
xlabel('Time')
grid on
subplot(2,1,2)
plot(cpg1)
ylim([min(cpg1),max(cpg1)])
title('CPG 1')
ylabel('Neuron activity')
xlabel('Time')
grid on 

% Fig 2.6, 3.2 + 3.3 (Changed title and colors for 3.2) 
figure(10)
plot(cpg00(1,:),'b')
hold on
plot(cpg11,'r')
hold off 
xlabel('Time')
ylabel('Neuron activity')
title('The antigonistic pair')
legend('CPG 0', 'CPG 1','Location','southwest')
grid on

% Fig 3.4 - 3.10 
pic = figure(100);
plot(cpg0_t)
hold on 
plot(cpg1_t)
hold off 
legend('CPG 00 - test', 'CPG 11 - test','Location','southeast')
xlabel('Iterations')
ylabel('Neuron activity')
title('Adjusted value of alpha: alpha11 = -1, alpha00, alpha01, alpha10 = 1.01')
%ylim([-0.25,0.07])
grid on
%saveas(pic,sprintf('w11_alpha_neg.png'))

% Fig 3.11 - 3.13 
amp_p = figure(500);
    clf
    hold on; box on;
    xlim([-40 50])
    ylim([-40 50])
    plot([x0_link1 y0_link1],[x0_link1 link1*100],'k-','LineWidth',2)
    plot([x_link2 end_pos(1,1)]*100,[y_link2 end_pos(2,1)]*100,'k-','LineWidth',2)
    plot(x0_q, y0_q ,'ro','MarkerEdgeColor','k','MarkerFaceColor','yellow','MarkerSize',s*0.9)
    plot(end_pos(1,1)*100,end_pos(2,1)*100,'ro','MarkerEdgeColor','k','MarkerFaceColor','yellow','MarkerSize',s*1.3)
    plot(end_pos(1,:)*100,end_pos(2,:)*100,'.black')
    legend('Upper arm', 'Forearm', 'Elbow', 'Hand', 'Range of motion')
    ylabel('y (cm)')
    xlabel('x (cm)')
    title('Range of motion w. alpha = 1.04, phi = 0.05*pi')
    grid on
saveas(amp_p,sprintf('amp_1.04_arm.png'))

% ---------------------------------------------------------
% ---------------------------------------------------------
% Checking determinants for orthogonality and min/max values
% ---------------------------------------------------------
% TEST 1 
% Positive weights
% det_w00_pos = det(w_m_t); % determinant = -1 
% det_w01_pos = det(w_m_t); % determinant = 1
% det_w10_pos = det(w_m_t); % determinant = 1
% det_w11_pos = det(w_m_t); % determinant = -1

% % Negative weights 
% det_w00_neg = det(w_m_t); % determinant = -1
% det_w01_neg = det(w_m_t); % determinant = 1
% det_w10_neg = det(w_m_t); % determinant = 1
% det_w11_neg = det(w_m_t) % determinant = -1


% Test 2 
% Phi = 0.5*pi
% det_w00_phi_pos = det(w_m_t); % determinant = 0.0250 
% det_w01_phi_pos = det(w_m_t); % determinant = 1.1547
% det_w10_phi_pos = det(w_m_t); % determinant = 1.1547
% det_w11_phi_pos = det(w_m_t); % determinant = 0.0250

% Phi = -0.5*pi
% det_w00_phi_neg = det(w_m_t); % determinant = 0.0250
% det_w01_phi_neg = det(w_m_t); % determinant = 0.8356
% det_w10_phi_neg = det(w_m_t); % determinant = 0.8356
% det_w11_phi_neg = det(w_m_t) % determinant = 0.0250 

% Phi = 0*pi
% det_w00_phi_0 = det(w_m_t); % determinant = 1.0325
% det_w01_phi_0 = det(w_m_t); % determinant = 0.9951
% det_w10_phi_0 = det(w_m_t); % determinant = 0.9951
% det_w11_phi_0 = det(w_m_t); % determinant = 1.0325

% Test 3 
% Positive alpha 
% det_w00_a_pos = det(w_m_t); % determinant = 1.0102 
% det_w01_a_pos = det(w_m_t); % determinant = 1.0199
% det_w10_a_pos = det(w_m_t); % determinant = 1.0199
% det_w11_a_pos = det(w_m_t); % determinant = 1.0102

% % Negative weights 
% det_w00_a_neg = det(w_m_t); % determinant = -0.9603
% det_w01_a_neg = det(w_m_t); % determinant = 0.9704
% det_w10_a_neg = det(w_m_t); % determinant = 0.9704
% det_w11_a_neg = det(w_m_t) % determinant = -0.9603

% Checking range of motion limits
x_min_a = min(end_pos(1,:)*100); % minimum value for x coordinate 
y_min_a = min(end_pos(2,:)*100); % min val for y coord
x_max_a = max(end_pos(1,:)*100); % max value for x coordinate 
y_max_a = max(end_pos(2,:)*100); % max val for y coord


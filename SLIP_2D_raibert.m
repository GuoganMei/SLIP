clc;
clear;
close all;

fix_params.body_mass=0.1;
fix_params.leg_k=500;
fix_params.g=9.81;
fix_params.leg_origin_length=0.1;
fix_params.sim_dt=0.01;
k_ctr=0.02;
i_ctr=0.007;

des_vel=0.5;
leg_angle=0;


x_init=0;
z_init=0.2;
dx_init=0;
dz_init=0;
r_init=fix_params.leg_origin_length;
theta_init=0;
dr_init=0;
dtheta_init=0;
vel_err_sum=0;

full_state=[x_init;z_init;dx_init;dz_init;r_init;theta_init;dr_init;dtheta_init];
all_full_state=[];
for i=1:20
    %flight
    full_states=sim_flight(full_state,leg_angle,fix_params);
    all_full_state=[all_full_state;full_states];
    %flight to stance
    full_state=full_states(end,:)';
    full_state=flight2stance(full_state);
    all_full_state=[all_full_state;full_state'];
    %stance
    [Ts,full_states]=sim_stance(full_state,fix_params);
    all_full_state=[all_full_state;full_states];
    %stance to flight
    full_state=full_states(end,:)';
    full_state=stance2flight(full_state);
    all_full_state=[all_full_state;full_state'];
    
    %Calcuate foot's neutral point
    stance_pos = 0.5*Ts*full_state(3);
    vel_err = full_state(3) - des_vel;
    vel_err_sum = vel_err_sum + vel_err;
    %Raibert style controller
    x_foot_pos_tar = stance_pos + k_ctr * vel_err+ i_ctr * vel_err_sum;
    leg_angle = asin(x_foot_pos_tar/fix_params.leg_origin_length);
end

%plot(1:size(all_full_state),all_full_state(:,3));
vis(all_full_state,des_vel);

%% flight phase dynmaics
function dstate_out=flight_dyn(flight_state,fix_params)
    dstate_out=zeros(4,1);
    dstate_out(1)=flight_state(3);
    dstate_out(2)=flight_state(4);
    dstate_out(3)=0;
    dstate_out(4)=-fix_params.g;
end
%% stance phase dynamics
function dstate_out=stance_dyn(stance_state,fix_params)
    dstate_out=zeros(4,1);
    r=stance_state(1);
    theta=stance_state(2);
    dr=stance_state(3);
    dtheta=stance_state(4);
    k=fix_params.leg_k;
    l0=fix_params.leg_origin_length;
    m=fix_params.body_mass;
    g=fix_params.g;
    dstate_out(1)=dr;
    dstate_out(2)=dtheta;
    dstate_out(3)=k*(l0-r)/m-g*cos(theta)+r*dtheta^2;
    dstate_out(4)=(g*r*sin(theta)-2*r*dr*dtheta)/(r^2);
end
%% ODE event detection,when changing flight to stance phase
function [val, is_end, dir] = check_stance(s_f,leg_l_ori,leg_ang_touchdown)
    val = s_f(2) - leg_l_ori * cos(leg_ang_touchdown)+ max(s_f(4),0);
    is_end = 1;
    dir = [];
end
%% ODE event detection, when changing to flight phase
function [val, is_end, dir] = check_flight(s_s,leg_l_ori)
val = s_s(1) - leg_l_ori + min(s_s(3),0);
is_end = 1;
dir = [];
end
%% flight phase simulator
function full_states=sim_flight(full_state_init,leg_angle,fix_params)
    flight_state_init=full_state_init(1:4,1);
    options_flight = odeset('Events',@(t,state) check_stance(state,fix_params.leg_origin_length,leg_angle));
    t_arr=0:fix_params.sim_dt:4;
    [t_out,f_s_out]=ode45(@(t,state)flight_dyn(state,fix_params),t_arr,flight_state_init,options_flight);
    steps_counter=ones(size(t_out(:,1)));
    full_states=[f_s_out,...
        steps_counter*fix_params.leg_origin_length,...
        steps_counter*leg_angle,...
        steps_counter*0,...
        steps_counter*0];%x,z,dx,dz,r,theta,dr,dtheta
    %full_states(end,:)=[];
end
%% stance phase simulator
function [stance_time,full_states]=sim_stance(full_state_init,fix_params)
    stance_state_init=full_state_init(5:8,1);
    x_init=full_state_init(1)+sin(full_state_init(6))*full_state_init(5);
    options_stance = odeset('Events',@(t,state) check_flight(state,fix_params.leg_origin_length));
    t_arr=0:fix_params.sim_dt:4;
    [t_out,s_s_out]=ode45(@(t,state)stance_dyn(state,fix_params),t_arr,stance_state_init,options_stance);
    steps_counter=ones(size(t_out(:,1)));
    full_states=[steps_counter*x_init-s_s_out(:,1).*sin(s_s_out(:,2)),...%x
        s_s_out(:,1).*cos(s_s_out(:,2)),...%z
        -s_s_out(:,3).*sin(s_s_out(:,2))-s_s_out(:,1).*s_s_out(:,4).*cos(s_s_out(:,2)),...%dx
        s_s_out(:,3).*cos(s_s_out(:,2))-s_s_out(:,1).*s_s_out(:,4).*sin(s_s_out(:,2)),...%dz
        s_s_out];%r,theta,dr,dtheta
    %full_states(end,:)=[];
    stance_time=t_out(end,1);
end
%% flight->stance
function full_state=flight2stance(full_state)
    theta=full_state(6);
    dx=full_state(3);
    dz=full_state(4);
    full_state(7)=-sin(theta)*dx+cos(theta)*dz;
    full_state(8)=-1/full_state(5)*(cos(theta)*dx+sin(theta)*dz);
end
%% stance->flight
function full_state=stance2flight(full_state)
end

%% Visualization
function [] = vis(state,des_vel)

x = state(:,1);
z = state(:,2);
l = state(:,5);
ang = state(:,6);
x_vel = state(:,3);
des_vel=ones(size(state,1),1)*des_vel;


%Draw the leg
subplot(5,1,1);
body = patch([0 0],[1 1],[1,0,0]);
leg = patch([0 0],[1 1],[1,1,1]);
ylim(subplot(5,1,1),[0,0.08])
xlim(subplot(5,1,1),[-0.05,0.1])
axis equal;
title('sim');

%Body height
subplot(5,1,2);
title('z');
grid on;
ylim(subplot(5,1,2),[0.02,0.12]);
xlim(subplot(5,1,2),[0,length(x)]);
z_draw = animatedline('Color','b');
ylabel('m');

%X velocity
subplot(5,1,3);
title('x velocity');
grid on;
ylim(subplot(5,1,3),[-0.8,0.8]);
xlim(subplot(5,1,3),[0,length(x)]);
x_vel_draw = animatedline('Color','b');
x_des_vel_draw = animatedline('Color','r');
ylabel('m/s');

%Hip rad
subplot(5,1,4);
title('hip rad');
grid on;
ylim(subplot(5,1,4),[-0.5,0.5]);
xlim(subplot(5,1,4),[0,length(x)]);
phi_draw = animatedline('Color','b');
ylabel('rad');

%Leg length
subplot(5,1,5);
title('leg length');
grid on;
ylim(subplot(5,1,5),[0.03,0.08]);
xlim(subplot(5,1,5),[0,length(x)]);
ll_draw = animatedline('Color','b');
ylabel('m');

%Add all elements
for i = 1:length(x)
    body.XData = [x(i) + 0.01*sin(0:0.1:2*pi)];
    body.YData = [z(i) + 0.01*cos(0:0.1:2*pi)];
    leg.XData = [x(i), x(i)+l(i)*sin(ang(i))];
    leg.YData = [z(i), z(i)-l(i)*cos(ang(i))];
    
    ylim(subplot(5,1,1),[0,0.2])
    xlim(subplot(5,1,1),[-0.05,2])
    
    addpoints(z_draw,i,z(i));
    addpoints(x_vel_draw,i,x_vel(i));
    addpoints(x_des_vel_draw,i,des_vel(i));
    addpoints(phi_draw,i,ang(i));
    addpoints(ll_draw,i,l(i));
    drawnow;
end
end





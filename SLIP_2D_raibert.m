clc;
clear;
close all;

fix_params.body_mass=0.1;
fix_params.leg_k=500;
fix_params.g=9.81;
fix_params.leg_origin_length=0.06;
fix_params.sim_dt=0.01;
k_ctr=0.02;

des_vel=0.3;
leg_angle=0;


x_init=0;
z_init=0.1;
dx_init=0;
dz_init=0;
r_init=fix_params.leg_origin_length;
theta_init=0;
dr_init=0;
dtheta_init=0;

full_state=[x_init;z_init;dx_init;dz_init;r_init;theta_init;dr_init;dtheta_init];
all_full_state=[];
for i=1:50
    %flight
    full_states=sim_flight(full_state,leg_angle,fix_params);
    all_full_state=[all_full_state;full_states];
    %flight to stance
    full_state=full_states(end,:)';
    full_state=flight2stance(full_state);
    all_full_state=[all_full_state,full_state'];
    %stance
    [Ts,full_states]=sim_stance(full_state_init,fix_params);
    all_full_state=[all_full_state;full_states];
    %stance to flight
    full_state=full_states(end,:)';
    full_state=stance2flight(full_state);
    all_full_state=[all_full_state,full_state'];
    
    %Calcuate foot's neutral point
    stance_pos = 0.5*Ts*full_state(3);
    vel_err = full_state(3) - des_vel;
    %vel_err_sum = vel_err_sum + vel_err;
    %Raibert style controller
    x_foot_pos_tar = stance_pos + k_ctr * vel_err;%+ i_ctr * vel_err_sum;
    leg_ang = asin(x_foot_pos_tar/leg_l_ori);
end


%% flight phase dynmaics
function dstate_out=flight_dyn(flight_state_init,fix_params)
    dstate_out=zeros(4,1);
    dstate_out(1)=flight_state_init(3);
    dstate_out(2)=flight_state_init(4);
    dstate_out(3)=0;
    dstate_out(4)=-fix_params.g;
end
%% stance phase dynamics
function dstate_out=stance_dyn(stance_state_init,fix_params)
    dstate_out=zeros(4,1);
    r=stance_state_init(1);
    theta=stance_state_init(2);
    dr=stance_state_init(3);
    dtheta=stance_state_init(4);
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
    [t_out,f_s_out]=ode45(flight_dyn,t_arr,flight_state_init,options_flight);
    steps_counter=ones(size(t_out(:,1)));
    full_states=[f_s_out,...
        steps_counter*fix_params.leg_origin_length,...
        steps_counter*fix_params.leg_angle,...
        steps_counter*0,...
        steps_counter*0];%x,z,dx,dz,r,theta,dr,dtheta
    full_states(end,:)=[];
end
%% stance phase simulator
function [stance_time,full_states]=sim_stance(full_state_init,fix_params)
    stance_state_init=full_state_init(5:8,1);
    x_init=full_state_init(1);
    options_stance = odeset('Events',@(t,state) check_flight(state,fix_params.leg_origin_length));
    t_arr=0:fix_params.sim_dt:4;
    [t_out,s_s_out]=ode45(stance_dyn,t_arr,stance_state_init,options_stance);
    steps_counter=ones(size(t_out(:,1)));
    full_states=[steps_counter*x_init-s_s_out(:,1).*sin(s_s_out(:,2)),...%x
        steps_counter*x_init+s_s_out(:,1).*cos(s_s_out(:,2)),...%z
        -s_s_out(:,3).*sin(s_s_out(:,2))-s_s_out(:,1).*s_s_out(:,4).*cos(s_s_out(:,2)),...%dx
        s_s_out(:,3).*cos(s_s_out(:,2))-s_s_out(:,1).*s_s_out(:,4).*sin(s_s_out(:,2)),...%dz
        s_s_out];%r,theta,dr,dtheta
    full_states(end,:)=[];
    stance_time=t_out(end,1);
end
%% flight->stance
function full_state=flight2stance(full_state)
    theta=full_state(6);
    dx=full_state(3);
    dz=full_state(4);
    full_state(7)=-sin(theta)*dx+cos(theta)*dz;
    full_state(8)=-1/full_state(1)*(cos(theta)*dx+sin(theta)*dz);
end
%% stance->flight
function full_state=stance2flight(full_state)
end




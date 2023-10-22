%{
*
 NAME           : ThirdOrder_Scurve.m
 AUTHOR         : Paweekorn Buasakorn
 DATE           : March 11th 2023
 MODIFIED BY    : Paweekorn Buasakorn
 DESCRIPTION    : This function generates a trajectory of a third-order S-curve based on the given initial and target positions, maximum velocity, maximum acceleration and maximum jerk.
*
%}
[q,w,e,r,t,y] = ThirdOrderScurve(0,700,600,1000,2500);
function [p1,p2,p3,p4,p5,p6] = ThirdOrderScurve(initial_p, target_p, vmax, amax, jmax)
    % Check for the feasibility of the trajectory
    if (vmax*jmax < amax^2)
        M = 1;
        N = 0;
    else
        M = 0;
        N = 1;
    end

    % Determine the direction of the motion
    if (target_p - initial_p < 0)
        dir = -1;
    else
        dir = 1;
    end

    % Calculate the required distance
    s = abs(target_p - initial_p); %required distance

    % Calculate the values of va, sa and sv
    va = (amax^2)/jmax;
    sa = 2*(amax^3)/(jmax^2);
    sv = vmax*((M * 2 * (vmax/jmax)^1/2) + N*((vmax/amax)+(amax/jmax)));
    
    % Determine the shape of the trajectory based on the values of va, sa and sv
    if (vmax <= va) && (s >= sa)
        pattern = 1;
    elseif (vmax > va) && (s < sa)
        pattern = 2;
    elseif (vmax < va) && (s < sa) && (s > sv)
        pattern = 3;
    elseif (vmax < va) && (s < sa) && (s < sv)
        pattern = 4;
    elseif (vmax >= va) && (s >= sa) && (s >= sv)
        pattern = 5;
    elseif (vmax >= va) && (s >= sa) && (s < sv)
        pattern = 6;
    end
    
    % Calculate the values of tj, ta and tv for each trajectory pattern
    switch pattern
        case 1 %longest
            tj = (vmax/jmax)^(1/2);
            ta = tj;
            tv = s/vmax;
        case 2
            tj = (s/(2*jmax))^(1/3);
            ta = tj;
            tv = 2*tj;
        case 3 %mid
            tj = (s/(2*jmax))^(1/3);
            ta = tj;
            tv = 2*tj;
        case 4 %shortest
            tj = (s/(2*jmax))^(1/3);
            ta = tj;
            tv = 2*tj;
        case 5
            tj = amax/jmax;
            ta = vmax/amax;
            tv = s/vmax;
        case 6
            tj = amax/jmax;
            ta = 0.5*(sqrt(((4*s*jmax^2)+amax^3) / (amax * jmax^2)) - (amax/jmax));
            tv = ta + tj;  
    end
    
    % Calculate the values of t1 to t7 and the total time
    t1 = tj;
    t2 = ta;
    t3 = ta + tj;
    t4 = tv;
    t5 = tv + tj;
    t6 = tv + ta;
    t7 = tv + tj + ta;
    time_total = t7
    time = 0:1/2000:time_total;

    % Initialize the position, velocity, acceleration and jerk arrays
    position = zeros(1, length(time));
    velocity = zeros(1, length(time));
    acceleration = zeros(1, length(time));
    jerk = zeros(1,length(time));
    
    %calculate
    for i = 1:length(time)
        t = time(i)
        if  t <= t1
            jerk(i) = jmax*dir;
            acceleration(i) = jmax * t* dir;
            velocity(i) =  1/2 * jmax * t^2 * dir;
            position(i) = initial_p + 1/6 * jmax * t^3 * dir;
            [a1, v1, p1] = deal(acceleration(i),velocity(i),position(i));
            [a2, v2, p2] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t1 < t, t <= t2) 
            jerk(i) = 0;
            acceleration(i) = a1;
            velocity(i) = v1 + a1*(t-t1);
            position(i) =  p1 + v1 * (t - t1) + 1/2 * a1 * (t - t1)^2;
            [a2, v2, p2] = deal(acceleration(i),velocity(i),position(i));
            [~, v3, p3] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t2 < t, t <= t3) 
            jerk(i) = -jmax*dir;
            acceleration(i) = a2 - (jmax * (t - t2))*dir;
            velocity(i) = v2 + a2 * (t - t2) + 1/2 * -jmax * dir * (t - t2)^2;
            position(i) = p2 + v2 * (t - t2) + 1/2 * a2 * (t - t2)^2 + 1/6 * - jmax * dir * (t - t2)^3;
            [~, v3, p3] = deal(acceleration(i),velocity(i),position(i));
            [~, v4, p4] = deal(acceleration(i),velocity(i),position(i));
            [a6, v6, p6] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t3 < t, t <= t4)
            jerk(i) = 0;
            acceleration(i) = 0;
            velocity(i) = v3;
            position(i) = p3 + v3 * (t - t3);
            [~, v4, p4] = deal(acceleration(i),velocity(i),position(i));
            [a5, v5, p5] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t4 < t, t <= t5)
            jerk(i) = -jmax * dir;
            acceleration(i) = (-jmax * (t - t4))*dir;
            velocity(i) = v4 + 1/2 * - jmax * dir * (t - t4)^2;
            position(i) = p4 + v4 * (t - t4) + 1/6 * dir *- jmax * (t - t4)^3;
            [a5, v5, p5] = deal(acceleration(i),velocity(i),position(i));
            [a6, v6, p6] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t5 < t, t <= t6)
            jerk(i) = 0;
            acceleration(i) = a5;
            velocity(i) = v5 + a5 * (t - t5);
            position(i) = p5 + v5 * (t - t5) + 1/2 * a5 * (t - t5)^2;
            [a6, v6, p6] = deal(acceleration(i),velocity(i),position(i));
        elseif and(t6 < t, t <= t7)
            a = 7;
            jerk(i) = jmax*dir;
            acceleration(i) = a6 + dir *jmax * (t - t6);
            velocity(i) = (v6 + a6 * (t - t6) + 1/2 * jmax*dir * (t - t6)^2);
            position(i) = p6 + v6 * (t - t6) + 1/2 * a6 * (t - t6)^2 + 1/6 * jmax* dir * (t - t6)^3;
        %[j7, a7, v7, ~] = deal(jerk(i),acceleration(i),velocity(i),position(i));
        end
    end
%     time(i+1) = time_total + (1/2000);
%     position(i+1) = target_p;
%     velocity(i+1) = v7;
%     acceleration(i+1) = a7;
%     jerk(i+1) = j7;

    subplot(4,1,1);
    plot(time,position)
    title('Third order S-curve trajectoty')
    xlabel('time (sec)')
    ylabel('position (mm)')
    
    subplot(4,1,2)
    plot(time,velocity)
    xlabel('time (sec)')
    ylabel('velo (mm/s)')
    
    subplot(4,1,3)
    plot(time,acceleration)
    xlabel('time (sec)')
    ylabel('accel (mm/s^2)')
    
    subplot(4,1,4)
    plot(time,jerk)
    xlabel('time (sec)')
    ylabel('jerk (mm/s^3)')
end


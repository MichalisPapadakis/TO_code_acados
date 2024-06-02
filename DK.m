function q_est = DK(qrobot)
%Direct Kinematics and fulls state estimation function:
%Given q = [qMH,qHI,0,qHO,0] estimates all joint angles

% q in robot frame  thus:
offsets = [0, 0.738704,-1.17042, -0.735976,1.17777];
projection_front_left= [-1,-1,-1,-1,-1]          ;
projection_front_right= [-1,-1,-1, 1,-1]          ; 
projection_rear_left=   [-1,-1, 1,-1, 1]           ;
projection_rear_right=  [-1, 1, 1, 1, 1]          ;

q = zeros(5,2);
for i =1:5
    q(i,1) = projection_front_right(i)*qrobot(i) +  offsets(i)  ;
end


%% defines
qMH_offset = -pi/2;
q11_offset = 2.3095;
q21_offset = 1.3265;
q12_offset = 0.83482;
q22_offset = -1.3233;

j11_Dx = 0.0517; %Horizontal distance of joint11 from MH frame
l11    = 0.175;
l21    = 0.29977;

j12_Dx = 0.1417; %Horizontal distance of joint12 from MH frame
l12    = 0.175;
l22    = 0.29929;

j_Dy = -1.1338e-05; %Vertical Distance of joint11 and joint12 from MH frame
pj11 = [j11_Dx, -1.1338e-05]; % `pj11` is the center of joint11
pj12 = [j12_Dx, -1.1338e-05]; % `pj12` is the center of joint12 
%% Intersection point
q11 = -q(2) + q11_offset;
q12 = -q(4) + q12_offset;

%centers
p1c(1) = j11_Dx + l11*cos(q11);
p1c(2) = j_Dy   + l11*sin(q11);

p2c(1) = j12_Dx + l12*cos(q12);
p2c(2) = j_Dy   + l12*sin(q12);

% normal and center vector
vc = p2c-p1c;
d  = norm(vc); %distance of circle centers `p1c` and `p2c`
vc = vc/d;
vn = [-vc(2),vc(1)];

%%triangle solution
a = ( (d*d) + (l21*l21) - (l22*l22) )/( 2*d );
h = sqrt( (l21*l21) - (a*a));

p_EE_22d = p1c + (a*vc) + (h*vn);


%% joint angles
link11_v = p1c-pj11;
link12_v = p2c-pj12;

link21_v = p_EE_22d-p1c;
link22_v = p_EE_22d-p2c;

theta1 = acos(dot(link11_v, link21_v) / (norm(link11_v) * norm(link21_v)));
theta2 = -acos(dot(link12_v, link22_v) / (norm(link12_v) * norm(link22_v)));

% % Check direction
% link11_vn = link11_v / norm(link11_v);
% side1_extended_x = link11_vn(1) * (l11 + l21) + pj11(1);
% 
% link12_vn = link12_v / norm(link12_v);
% side2_extended_x = link12_vn(1) * (l12 + l22) + pj12(1);


%check chain 1:
% if p1c(2) > j_Dy 
%     link11_vn = link11_v / norm(link11_v);
%     side1_extended_x = link11_vn(1) * (l11 + l21) + pj11(1);
%     if p_EE_22d(1) < side1_extended_x
%         % joint12 is outwards
%         theta1 = -theta1;
%     end
% else
    theta_j1 = q11; 
    theta_EE = atan2(p_EE_22d(2)-pj11(2),p_EE_22d(1)-pj11(1));
    if theta_EE > theta_j1
        theta1 = -theta1;
    end
% end

% check chain 2:
% if p2c(2) > j_Dy 
%     link12_vn = link12_v / norm(link12_v);
%     side2_extended_x = link12_vn(1) * (l12 + l22) + pj12(1);
%     if p_EE_22d(1) > side2_extended_x
%     % joint22 is outwards
%         theta2 = -theta2;
%     end
% else 
    theta_j2 = q12; 
    theta_EE = atan2(p_EE_22d(2)-pj12(2),p_EE_22d(1)-pj12(1));

    if theta_EE < 0;
        theta_EE = 2*pi+theta_EE;
    end
    if theta_EE < theta_j2
        theta2 = -theta2;
    end
% end

% %%THIS IS BAD -> IF q11 < -1.57
% if p_EE_22d(1) < side1_extended_x
%     % joint12 is outwards
%     theta1 = -theta1;
% elseif p_EE_22d(1) > side2_extended_x
%     % joint22 is outwards
%     theta2 = -theta2;
% end

q_est    = zeros(5,1);
q_est(1) = q(1);
q_est(2) = q(2);
q_est(3) = theta1 - q21_offset;
q_est(4) = q(4);
q_est(5) = theta2 - q22_offset;

%% to robot coordinates again:
for i =1:5
    q_est(i) = projection_front_right(i)*( q_est(i) -  offsets(i))  ;
end

end
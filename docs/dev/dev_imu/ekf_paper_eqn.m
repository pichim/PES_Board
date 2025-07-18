% Improved State Estimation in Quadrotor MAVs A Novel Drift-Free Velocity Estimator
% pmic - 06.2019
% pmic - 07.2025 (added Omega for pitch, roll, yaw according to Tait-Bryan angles ZXY)

clc, clear all

syms phi(t) theta(t) psi(t)

Rz_psi = [[cos(psi), -sin(psi), 0]; ...
          [sin(psi),  cos(psi), 0]; ...
          [      0,          0, 1]];
      
Ry_the = [[ cos(theta), 0, sin(theta)]; ...
          [         0,  1,          0]
          [-sin(theta), 0, cos(theta)]];
      
Rx_phi = [[1,        0,         0]; ...
          [0, cos(phi), -sin(phi)]; ...
          [0, sin(phi),  cos(phi)]];
      
% roll, pitch, yaw according to Tait-Bryan angles ZYX
% -------------------------------------------------------------------------
% 1.) yaw
% 2.) pitch
% 3.) roll, see notes
Omega = [1;0;0]*diff(phi, t) + ...
        Rx_phi.'*[0;1;0]*diff(theta,t) + ...
        Rx_phi.'*Ry_the.'*[0;0;1]*diff(psi,t);
simplify(Omega)
%                           diff(phi(t), t) - sin(theta(t))*diff(psi(t), t)
% cos(phi(t))*diff(theta(t), t) + cos(theta(t))*sin(phi(t))*diff(psi(t), t)
% cos(phi(t))*cos(theta(t))*diff(psi(t), t) - sin(phi(t))*diff(theta(t), t)

% [wx; wy; wz] = Omega * [dphi; dtheta; dpsi]
Omega_ = [[1,         0,         -sin(theta)]; ...
          [0,  cos(phi), cos(theta)*sin(phi)]; ...
          [0, -sin(phi), cos(theta)*cos(phi)]];
% [ 1,            0,            -sin(theta(t))]
% [ 0,  cos(phi(t)), cos(theta(t))*sin(phi(t))]
% [ 0, -sin(phi(t)), cos(phi(t))*cos(theta(t))]

% check Omega
simplify(Omega - Omega_*[diff(phi, t);diff(theta,t);diff(psi,t)])

% [dphi; dtheta; dpsi] = Omega^-1 * [wx; wy; wz]
OmegaInv_ = simplify(Omega_^-1)
% [ 1, (sin(phi(t))*sin(theta(t)))/cos(theta(t)), (cos(phi(t))*sin(theta(t)))/cos(theta(t))]
% [ 0,                               cos(phi(t)),                              -sin(phi(t))]
% [ 0,                 sin(phi(t))/cos(theta(t)),                 cos(phi(t))/cos(theta(t))]

OmegaInv = [[ 1, sin(phi(t))*tan(theta(t)), cos(phi(t))*tan(theta(t))]; ...
            [ 0,               cos(phi(t)),              -sin(phi(t))]; ...
            [ 0, sin(phi(t))/cos(theta(t)), cos(phi(t))/cos(theta(t))]];

% check OmegaInv
simplify(OmegaInv - OmegaInv_)

% pitch, roll, yaw according to Tait-Bryan angles ZXY
% -------------------------------------------------------------------------
% 1.) yaw
% 2.) roll
% 3.) pitch
Omega = [0;1;0]*diff(theta, t) + ...
        Ry_the.'*[1;0;0]*diff(phi,t) + ...bisbisbissdfsdfsf
        Ry_the.'*Rx_phi.'*[0;0;1]*diff(psi,t);
simplify(Omega)
% cos(theta(t))*diff(phi(t), t) - cos(phi(t))*sin(theta(t))*diff(psi(t), t)
%                           sin(phi(t))*diff(psi(t), t) + diff(theta(t), t)
% sin(theta(t))*diff(phi(t), t) + cos(phi(t))*cos(theta(t))*diff(psi(t), t)

% [wx; wy; wz] = Omega * [dphi, dtheta; dpsi]
Omega_ = [[cos(theta), 0, -cos(phi)*sin(theta)]; ...
          [         0, 1,             sin(phi)]; ...
          [sin(theta), 0,  cos(phi)*cos(theta)]];

% check Omega
simplify(Omega - Omega_*[diff(phi, t);diff(theta,t);diff(psi,t)])

% reordering so that we map [wy; wx; wz] = Omega * [dtheta, dphi; dpsi]
Omega = [                          sin(phi(t))*diff(psi(t), t) + diff(theta(t), t); ...
         cos(theta(t))*diff(phi(t), t) - cos(phi(t))*sin(theta(t))*diff(psi(t), t); ...
         sin(theta(t))*diff(phi(t), t) + cos(phi(t))*cos(theta(t))*diff(psi(t), t)]

% [wy; wx; wz] = Omega * [dtheta, dphi; dpsi]
Omega_ = [[1,          0,             sin(phi)]; ...
          [0, cos(theta), -cos(phi)*sin(theta)]; ...
          [0, sin(theta),  cos(phi)*cos(theta)]];

% check Omega
simplify(Omega - Omega_*[diff(theta, t);diff(phi,t);diff(psi,t)])

% [dtheta; dphi; dpsi] = Omega^-1 * [wy; wx; wz]
OmegaInv_ = simplify(Omega_^-1)
% [1, (sin(phi(t))*sin(theta(t)))/cos(phi(t)), -(cos(theta(t))*sin(phi(t)))/cos(phi(t))]
% [0,                           cos(theta(t)),                            sin(theta(t))]
% [0,              -sin(theta(t))/cos(phi(t)),                cos(theta(t))/cos(phi(t))]

OmegaInv = [[1,  tan(phi(t))*sin(theta(t)), -cos(theta(t))*tan(phi(t))]; ...
            [0,              cos(theta(t)),              sin(theta(t))]; ...
            [0, -sin(theta(t))/cos(phi(t)),  cos(theta(t))/cos(phi(t))]];

% check OmegaInv
simplify(OmegaInv - OmegaInv_)

return % stopped here, pmic - 07.2025


% transforms from body to earth or rotates a vector in earth w.r.t. earth frame
CEB = Ry_the*Rx_phi
% [ cos(psi(t))*cos(theta(t)), cos(psi(t))*sin(phi(t))*sin(theta(t)) - cos(phi(t))*sin(psi(t)), sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t))]
% [ cos(theta(t))*sin(psi(t)), cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t)), cos(phi(t))*sin(psi(t))*sin(theta(t)) - cos(psi(t))*sin(phi(t))]
% [            -sin(theta(t)),                                       cos(theta(t))*sin(phi(t)),                                       cos(phi(t))*cos(theta(t))]

% transforms earth to body or rotates a vector in body w.r.t. body frame
CBE = CEB.'
% [                                       cos(psi(t))*cos(theta(t)),                                       cos(theta(t))*sin(psi(t)),            -sin(theta(t))]
% [ cos(psi(t))*sin(phi(t))*sin(theta(t)) - cos(phi(t))*sin(psi(t)), cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t)), cos(theta(t))*sin(phi(t))]
% [ sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t)), cos(phi(t))*sin(psi(t))*sin(theta(t)) - cos(psi(t))*sin(phi(t)), cos(phi(t))*cos(theta(t))]

%%

temp = [[cos(psi)*cos(theta), -sin(psi), 0]; [cos(theta)*sin(psi), cos(psi), 0]; [-sin(theta), 0, 1]]
% [ cos(psi(t))*cos(theta(t)), -sin(psi(t)), 0]
% [ cos(theta(t))*sin(psi(t)),  cos(psi(t)), 0]
% [            -sin(theta(t)),            0, 1]
simplify(CBE*temp)
% [ 1,            0,            -sin(theta(t))]
% [ 0,  cos(phi(t)), cos(theta(t))*sin(phi(t))]
% [ 0, -sin(phi(t)), cos(phi(t))*cos(theta(t))]

%%

dCEB = simplify(diff(CEB, t))
SkewEEB = simplify(dCEB*CEB.')
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0, (cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t)))*(sin(phi(t))*sin(psi(t))*diff(phi(t), t) - cos(phi(t))*cos(psi(t))*diff(psi(t), t) + cos(phi(t))*cos(psi(t))*sin(theta(t))*diff(phi(t), t) + cos(psi(t))*cos(theta(t))*sin(phi(t))*diff(theta(t), t) - sin(phi(t))*sin(psi(t))*sin(theta(t))*diff(psi(t), t)) - (cos(psi(t))*sin(phi(t)) - cos(phi(t))*sin(psi(t))*sin(theta(t)))*(cos(phi(t))*sin(psi(t))*diff(phi(t), t) + cos(psi(t))*sin(phi(t))*diff(psi(t), t) - cos(psi(t))*sin(phi(t))*sin(theta(t))*diff(phi(t), t) - cos(phi(t))*sin(psi(t))*sin(theta(t))*diff(psi(t), t) + cos(phi(t))*cos(psi(t))*cos(theta(t))*diff(theta(t), t)) - cos(theta(t))*sin(psi(t))*(cos(theta(t))*sin(psi(t))*diff(psi(t), t) + cos(psi(t))*sin(theta(t))*diff(theta(t), t)), cos(psi(t))*diff(theta(t), t) + cos(theta(t))*sin(psi(t))*diff(phi(t), t)]
% [ (sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t)))*(sin(phi(t))*sin(psi(t))*diff(psi(t), t) - cos(phi(t))*cos(psi(t))*diff(phi(t), t) + cos(phi(t))*cos(psi(t))*sin(theta(t))*diff(psi(t), t) + cos(phi(t))*cos(theta(t))*sin(psi(t))*diff(theta(t), t) - sin(phi(t))*sin(psi(t))*sin(theta(t))*diff(phi(t), t)) - (cos(phi(t))*sin(psi(t)) - cos(psi(t))*sin(phi(t))*sin(theta(t)))*(cos(phi(t))*sin(psi(t))*sin(theta(t))*diff(phi(t), t) - cos(phi(t))*sin(psi(t))*diff(psi(t), t) - cos(psi(t))*sin(phi(t))*diff(phi(t), t) + cos(psi(t))*sin(phi(t))*sin(theta(t))*diff(psi(t), t) + cos(theta(t))*sin(phi(t))*sin(psi(t))*diff(theta(t), t)) + cos(psi(t))*cos(theta(t))*(cos(psi(t))*cos(theta(t))*diff(psi(t), t) - sin(psi(t))*sin(theta(t))*diff(theta(t), t)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0, sin(psi(t))*diff(theta(t), t) - cos(psi(t))*cos(theta(t))*diff(phi(t), t)]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             - cos(psi(t))*diff(theta(t), t) - cos(theta(t))*sin(psi(t))*diff(phi(t), t),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(psi(t))*cos(theta(t))*diff(phi(t), t) - sin(psi(t))*diff(theta(t), t),                                                                         0]

% clear all
% syms psi phi theta dpsi dphi dtheta
% wx = cos(psi)*cos(theta)*dphi - sin(psi)*dtheta;
% wy = cos(psi)*dtheta + cos(theta)*sin(psi)*dphi;
% wz = (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(sin(phi)*sin(psi)*dpsi - cos(phi)*cos(psi)*dphi + cos(phi)*cos(psi)*sin(theta)*dpsi + cos(phi)*cos(theta)*sin(psi)*dtheta - sin(phi)*sin(psi)*sin(theta)*dphi) - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(cos(phi)*sin(psi)*sin(theta)*dphi - cos(phi)*sin(psi)*dpsi - cos(psi)*sin(phi)*dphi + cos(psi)*sin(phi)*sin(theta)*dpsi + cos(theta)*sin(phi)*sin(psi)*dtheta) + cos(psi)*cos(theta)*(cos(psi)*cos(theta)*dpsi - sin(psi)*sin(theta)*dtheta);
% wx = simplify(wx)
% wy = simplify(wy)
% wz = simplify(wz)
% wx = dphi*cos(psi)*cos(theta) - dtheta*sin(psi)
% wy = dtheta*cos(psi) + dphi*cos(theta)*sin(psi)
% wz = dpsi - dphi*sin(theta)

%%

clc, clear all

syms psi theta phi

Rz_psi = [[cos(psi), -sin(psi), 0]; ...
          [sin(psi),  cos(psi), 0]; ...
          [      0,          0, 1]];
      
Ry_the = [[ cos(theta), 0, sin(theta)]; ...
          [         0,  1,          0]
          [-sin(theta), 0, cos(theta)]];
      
Rx_phi = [[1,        0,         0]; ...
          [0, cos(phi), -sin(phi)]; ...
          [0, sin(phi),  cos(phi)]];

% transforms from body to earth or rotates a vector in earth w.r.t. earth frame
CEB = Rz_psi*Ry_the*Rx_phi
% [ cos(psi(t))*cos(theta(t)), cos(psi(t))*sin(phi(t))*sin(theta(t)) - cos(phi(t))*sin(psi(t)), sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t))]
% [ cos(theta(t))*sin(psi(t)), cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t)), cos(phi(t))*sin(psi(t))*sin(theta(t)) - cos(psi(t))*sin(phi(t))]
% [            -sin(theta(t)),                                       cos(theta(t))*sin(phi(t)),                                       cos(phi(t))*cos(theta(t))]

% transforms earth to body or rotates a vector in body w.r.t. body frame
CBE = CEB.'

Omega = [[1,         0,         -sin(theta)]; ...
         [0,  cos(phi), cos(theta)*sin(phi)]; ...
         [0, -sin(phi), cos(theta)*cos(phi)]];
simplify(det(Omega)) % cos(theta)

% eq (2): b3
b3B = [0;0;1]
b3E = CEB * b3B
%  sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t))
%  cos(phi(t))*sin(psi(t))*sin(theta(t)) - cos(psi(t))*sin(phi(t))
%                                        cos(phi(t))*cos(theta(t))

% eq (2): V, Vtilde
syms vx(t) vy(t) vz(t)
VE = [vx(t); vy(t); vz(t)]
VB = [[1, 0, 0];[0, 1, 0];[0, 0, 0]] * CBE * VE
%  cos(psi(t))*cos(theta(t))*vx(t) - sin(theta(t))*vz(t) + cos(theta(t))*sin(psi(t))*vy(t)
%  vy(t)*(cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t))) - vx(t)*(cos(phi(t))*sin(psi(t)) - cos(psi(t))*sin(phi(t))*sin(theta(t))) + cos(theta(t))*sin(phi(t))*vz(t)
%  0 

syms g
gE = [0;0;g];
gB = [[1, 0, 0];[0, 1, 0];[0, 0, 0]] * CBE * gE
%  -g*sin(theta(t))
%  g*cos(theta(t))*sin(phi(t))
%  0

syms mx my mz
mE = [mx; my; mz];
mB = [[1, 0, 0];[0, 1, 0];[0, 0, 0]] * CBE * mE
%  mx*cos(psi(t))*cos(theta(t)) + my*cos(theta(t))*sin(psi(t)) - mz*sin(theta(t))
%  my*(cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t))) - mx*(cos(phi(t))*sin(psi(t)) - cos(psi(t))*sin(phi(t))*sin(theta(t))) + mz*cos(theta(t))*sin(phi(t))
%  0

syms s_phi c_phi s_theta c_theta s_psi c_psi
Hm = jacobian(mB, [phi, theta, psi]);
Hm = simplify(subs(Hm, [sin(phi) cos(phi) sin(theta) cos(theta) sin(psi) cos(psi)], [s_phi c_phi s_theta c_theta s_psi c_psi]))

% [                                                                                                  0,       - c_theta*mz - c_psi*mx*s_theta - my*s_psi*s_theta,                                                     c_theta*(c_psi*my - mx*s_psi)]
% [ mx*(s_phi*s_psi + c_phi*c_psi*s_theta) - my*(c_psi*s_phi - c_phi*s_psi*s_theta) + c_phi*c_theta*mz, s_phi*(c_psi*c_theta*mx - mz*s_theta + c_theta*my*s_psi), - mx*(c_phi*c_psi + s_phi*s_psi*s_theta) - my*(c_phi*s_psi - c_psi*s_phi*s_theta)]
% [                                                                                                  0,                                                        0,                                                                                 0]

%%

syms a11 a12 a13 a14
syms a21 a22 a23 a24
syms a31 a32 a33 a34
syms a41 a42 a43 a44
A = [a11 a12 a13 a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44]

inv(A)


function torque  = conputed_torque(theta_error,dtheta_error,ddtheta_desired,M,C)
Kp = 80;
Kv = 30;

torque = M*(ddtheta_desired+Kv*dtheta_error+Kp*theta_error)+C*dtheta_error;
end
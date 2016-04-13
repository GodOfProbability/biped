void apply_action(float action[]) 
{
  float state_temp[4];
  float motor[4];
  float error[4] = {0.,0.,0.,0.};
  float p_error[4] = {0.,0.,0.,0.}; 
  float error_sum[4] = {0.,0.,0.,0.};
  float kp = 1.0;
  float kd = 1.0;
  float ki = 0.0;
  
//  read_state(state_temp);
  for(int i=0;i<4;i++)
  { error[i] = state_temp[i] - action[i];
    motor[i] = kp*error[i] + kd*(error[i]-p_error[i]) + ki*error_sum[i];
    p_error[i] = error[i];
    error_sum[i] = error_sum[i] + error[i];
  }
  
  motor_hip_left.write((motor[0]*(180/3.1416))+90);
  motor_hip_right.write((motor[1]*(180/3.1416))+98);
  motor_knee_left.write((motor[2]*(180/3.1416))+90);
  motor_knee_right.write((motor[3]*(180/3.1416))+90);
    
}
// calculate initial coordinates
//
//float init_xfootB = (2) * ((Lthigh * sin(init_thetaBhip)) + (Lshin * sin(init_thetaBhip + init_thetaBknee)));
//float init_yfootB = 0;
//
//float init_xkneeB = (Lthigh * sin(init_thetaBhip)) - (Lthigh * sin(init_thetaFhip)) - (Lshin * sin(init_thetaFhip + init_thetaFknee));
//float init_ykneeB = (Lshin * cos(init_thetaBhip + init_thetaBknee));
//
//float init_xhip = -(Lthigh * sin(init_thetaFhip)) - (Lshin * sin(init_thetaFhip + init_thetaFknee));
//float init_yhip = ((Lthigh * cos(init_thetaFhip)) + (Lshin * cos(init_thetaFhip + init_thetaFknee)));
//
//float init_xfootF = 0;
//float init_yfootF = 0;
//
//float init_xkneeA = -(Lshin * sin(init_thetaFhip + init_thetaFknee));
//float init_ykneeA = (Lshin * cos(init_thetaFhip + init_thetaFknee));
//  
//float N_xfootB = init_xfootB - ((6*init_xfootB)/pow(t_step,2))*pow(T,2) + ((4*init_xfootB)/pow(t_step,3))*pow(T,3);
//float N_yfootB = H1 - ((H1)/pow(init_xfootB,2))*(pow(N_xfootB,2));
//
//float N_xhip = init_xhip - ((6*init_xhip)/pow(t_step,2))*pow(T,2) + ((4*init_xhip)/pow(t_step,3))*pow(T,3);
//float N_yhip = sqrt(pow((Lthigh + Lshin),2) - pow(N_xhip,2));
//
//float x = N_xfootB - N_xhip;
//float y = N_yfootB - N_yhip;
//
//float N_thetaBknee = -acos(((pow(x,2) + pow(y,2) - pow(Lthigh,2) - pow(Lshin,2))/(2*Lshin*Lthigh)));
//  
//float m = Lthigh + (Lshin*cos(N_thetaBknee));
//float n = (Lshin*sin(N_thetaBknee));
//
//float N_thetaBhip = atan(((m*x) + (n*y))/((n*x) - (m*y)));
//
//float N_xkneeB = N_xhip + (Lthigh * sin(N_thetaBhip));
//float N_ykneeB = N_yhip - (Lthigh * cos(N_thetaBhip));
//
//float N_thetaFknee = 0;
//
//float x1 = init_xfootF - N_xhip;
//float y1 = init_yfootF - N_yhip;
//float m1 = Lthigh + (Lshin*cos(N_thetaFknee));
//float n1 = (Lshin*sin(N_thetaFknee));
//
//float N_thetaFhip = atan(((m1*x1) + (n1*y1))/((n1*x1) - (m1*y1)));
//
//val[0] = N_thetaBhip;
//val[1] = N_thetaFhip;
//val[2] = N_thetaBknee;
//val[3] = N_thetaFknee;
//}

void read_state(float state[])
{
  float hip_left, hip_right, knee_left, knee_right;
  //static float state[4];
  
  hip_left   = read_IMU(1);
  hip_right  = read_IMU(2);
  knee_left  = read_IMU(3);
  knee_right = read_IMU(4);
  
  state[0] = hip_left   ;
  state[1] = hip_right  ;
  state[2] = knee_left  ; 
  state[3] = knee_right ;
  
//  Serial.print("hip: ");
//  Serial.println(hip_left);
//  Serial.print("state: ");
//  Serial.println(state[1]);


//  return state;
}

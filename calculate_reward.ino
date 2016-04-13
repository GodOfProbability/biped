float calculate_reward(float out[2])
{
  int fall, movement;

  fall = out[0];
  movement = out[1];
  float reward;
  
  if(fall == 1)
  {
    reward = -10;
  }
  else
  {
    reward = movement;
  }
  return reward;
}

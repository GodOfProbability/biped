void read_output(float output[])
{

  int encoder_count, prev_count, movement, fall;
  
//  encoder_count = read_encoder();
  movement = encoder_count - prev_count;
//  fall = read_bump();
  
  output[0] = fall;
  output[1] = movement;

  
}  

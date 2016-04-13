float read_IMU(int a){
  
  int x,y,z; //triple axis data
  float angle;

  digitalWrite(IMU_1,LOW);
  digitalWrite(IMU_2,LOW);
  digitalWrite(IMU_3,LOW);
  digitalWrite(IMU_4,LOW);
  
  switch(a) {
    case 1:
      digitalWrite(IMU_1,HIGH);
      break;
    case 2:
      digitalWrite(IMU_2,HIGH);
      break;
    case 3:
      digitalWrite(IMU_3,HIGH);
      break;
    case 4:
      digitalWrite(IMU_4,HIGH);
      break;
    default:
      SerialUSB.println("Wrong IMU select");
      break;
  }
  return 0;
}
//  
//   // if programming failed, don't try to do anything
//    if (!dmpReady) return;
//
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    // get current FIFO count
//    fifoCount = mpu.getFIFOCount();
//
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
//
//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else if (mpuIntStatus & 0x02) {
//        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//        // read a packet from FIFO
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//        
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;
//
//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif
//
//        #ifdef OUTPUT_READABLE_YAWPITCHROLL
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
//        #endif
//
//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
//    
//        #ifdef OUTPUT_TEAPOT
//            // display quaternion values in InvenSense Teapot demo format:
//            teapotPacket[2] = fifoBuffer[0];
//            teapotPacket[3] = fifoBuffer[1];
//            teapotPacket[4] = fifoBuffer[4];
//            teapotPacket[5] = fifoBuffer[5];
//            teapotPacket[6] = fifoBuffer[8];
//            teapotPacket[7] = fifoBuffer[9];
//            teapotPacket[8] = fifoBuffer[12];
//            teapotPacket[9] = fifoBuffer[13];
//            Serial.write(teapotPacket, 14);
//            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//        #endif
//
//        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
//    }
//  }
//
////  //Tell the HMC5883 where to begin reading data
////  Wire.beginTransmission(address);
////  Wire.write(0x03); //select register 3, X MSB register
////  Wire.endTransmission();
////  
//// 
//// //Read data from each axis, 2 registers per axis
////  Wire.requestFrom(address, 6);
////  if(6<=Wire.available()){
////    x = Wire.read()<<8; //X msb
////    x |= Wire.read(); //X lsb
////    z = Wire.read()<<8; //Z msb
////    z |= Wire.read(); //Z lsb
////    y = Wire.read()<<8; //Y msb
////    y |= Wire.read(); //Y lsb
////  }
////  angle = atan2((double)y,(double)z)*(180/3.1415926);
////  
//////  //Print out values of each axis
//////  Serial.print("x: ");
//////  Serial.print(x);
//////  Serial.print("  y: ");
//////  Serial.print(y);
//////  Serial.print("  z: ");
//////  Serial.println(z);
////  Serial.print("  angle: ");
////  Serial.println(angle);
////
////  return angle;
////  delay(250);


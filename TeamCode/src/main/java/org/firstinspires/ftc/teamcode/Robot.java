package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot
{
  public DcMotor leftFront;
  public DcMotor rightFront;
  public DcMotor leftRear;
  public DcMotor rightRear;
  
  public Robot(HardwareMap hardwareMap)
  {
    leftFront = hardwareMap.get(DcMotor.class, "lFront");
    rightFront = hardwareMap.get(DcMotor.class, "rFront");
    leftRear = hardwareMap.get(DcMotor.class, "lRear");
    rightRear = hardwareMap.get(DcMotor.class, "rRear");
    
    leftFront.setDirection(DcMotor.Direction.REVERSE);
    leftRear.setDirection(DcMotor.Direction.REVERSE);
    rightFront.setDirection(DcMotor.Direction.FORWARD);
    rightRear.setDirection(DcMotor.Direction.FORWARD);
  
  }
}

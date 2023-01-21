package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl
{
  private final DcMotor armMotor;
  private final Telemetry telemetry;
  
  
  ArmControl(HardwareMap hardwareMap, Telemetry theTelemetry)
  {
    telemetry = theTelemetry;
    // Initialize the hardware variables
    armMotor = hardwareMap.get(DcMotor.class, "arm");
    
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
  
    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's position on the robot.
    armMotor.setDirection(DcMotor.Direction.REVERSE);
  
    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  
  public void lower()
  {
    armMotor.setPower(-0.6);
    telemetry.addData("Raising", "True");
  }
  
  public void raise()
  {
    armMotor.setPower(0.6);
    telemetry.addData("Lowering", "True");
  }
  
  public void stop()
  {
    armMotor.setPower(0);
  }
  
}









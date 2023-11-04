// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Scoring {
    public static class Cones{
      public static class Extension {
        public static double high = -256;
        public static double low = -109;
      }
      public static class Lift {
        public static double high = -188;
        public static double low = -161;
      }
    }

    public static class Cubes {
      public static class Extension{
        public static double high = 143;
        public static double low = 6.0;
      }
      public static class Lift {
        public static double high = 536/2;
        public static double low = 365/2;
      }
    }
  }

  public static class HumanStation {
    public static double Lift = 232;
    public static double extension = 10;
  }

  public static class Intake {
    public static double Lift = -30;
    public static double Extension = -62;
  }

  public static class ll_scan {
    public static double Lift = -70;
    public static double Extension = 0;

  }
  public static class Servo {
    public static double open = 0;
    public static double closed = 1.0;
  }

  public static int driverstick = 0;

  public static int driverforward = 1;
  public static int driversteer = 4;

  public static int drA = 1;
  public static int drB = 2;
  public static int drY = 4;
  public static int drX = 3;
  public static int drLB = 5;
  public static int drRB = 6;
  

  public static int operatorstick = 1;
  public static int op1 = 1;
  public static int op2 = 2;
  public static int op3 = 3;
  public static int op4 = 4;
  public static int op5 = 5;
  public static int op6 = 6;
  public static int op8 = 8;
  public static int op7 = 7;
  public static int op9 = 9;
  public static int op10 = 10;
  public static int op11 = 11;
  public static int op12 = 12;
  public static int op13 = 13;
  public static int op14 = 14;
  public static int op15 = 15;
  public static int op16 = 16;
  public static int op17 = 17;
  public static int op18 = 18;
  public static int op19 = 19;
  public static int op20 = 20;
  public static int op21 = 21;
  public static int op22 = 22;
  public static int op23 = 23;
  public static int op24 = 24;
  
  //rocket ship controller looking thing
  public static int CatStick = 2;
  public static int cat2 = 2;
  public static int cat3 = 3;
  public static int cat4 = 4;
  
  public static double community_distance = 196.25;
  public static class GearRatio{
    public static double lowDrive = 20.88;
  }
}

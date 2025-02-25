//https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
package frc.robot;

public enum AprilTags {
  None(0.0),
  RedCoral1(1.0),
  RedCoral2(2.0),
  RedSideRedBarge(5.0),
  RedSideBlueBarge(4.0),
  RedProcessor(16.0),
  RedReef1(6.0),
  RedReef2(7.0),
  RedReef3(8.0),
  RedReef4(9.0),
  RedReef5(10.0),
  RedReef6(11.0),
  BlueCoral1(13.0),
  BlueCoral2(12.0),
  BlueSideRedBarge(15.0),
  BlueSideBlueBarge(14.0),
  BlueProcessor(3.0),
  BlueReef1(19.0),
  BlueReef2(18.0),
  BlueReef3(17.0),
  BlueReef4(20.0),
  BlueReef5(21.0),
  BlueReef6(22.0),
  ;

  private double value;

  private AprilTags(double id) {
    this.value = id;
  }

  public double getId() {
    return this.value;
  }

}
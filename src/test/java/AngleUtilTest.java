import static org.junit.Assert.*;
import org.junit.*;

import frc.robot.subsystems.AngleUtil;

public class AngleUtilTest {
  public static final double DELTA = 0; // acceptable deviation range

  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {

  }

  @Test // marks this method as a test
  public void TestClampAngle() {
    assertEquals(0, AngleUtil.clampAngle(0), DELTA);
    assertEquals(90, AngleUtil.clampAngle(90), DELTA);
    assertEquals(-90, AngleUtil.clampAngle(-90), DELTA);
    assertEquals(179, AngleUtil.clampAngle(179), DELTA);
    assertEquals(-179, AngleUtil.clampAngle(-179), DELTA);
    assertEquals(180, AngleUtil.clampAngle(180), DELTA);
    assertEquals(-180, AngleUtil.clampAngle(-180), DELTA);
    assertEquals(-179, AngleUtil.clampAngle(181), DELTA);
    assertEquals(179, AngleUtil.clampAngle(-181), DELTA);
    assertEquals(1, AngleUtil.clampAngle(361), DELTA);
    assertEquals(-1, AngleUtil.clampAngle(-361), DELTA);
  }

  // "Inflation happens man" - JoshuaAllard2022
}
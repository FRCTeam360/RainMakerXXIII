import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.ArmPoseCalculator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class RunCodeTest {
    @Test
    void getSetPoints() {
        Translation3d trans = new Translation3d(0.3, 0, 0.05);
        ArmPoseCalculator calculator = new ArmPoseCalculator();

        calculator.setTargetTrans(trans);

        System.out.println("Tilt: " + calculator.getActualElevationAngleDegrees());
        System.out.println("Extend: " + calculator.getExtendDistance());
        System.out.println("Rotation: " + calculator.getTurretRotation());

        assertEquals(1, 1);
    }
}

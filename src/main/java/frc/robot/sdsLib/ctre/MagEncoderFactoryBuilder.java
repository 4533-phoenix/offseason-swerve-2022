package frc.robot.sdsLib.ctre;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.sdsLib.AbsoluteEncoder;
import frc.robot.sdsLib.AbsoluteEncoderFactory;

public class MagEncoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;

    public MagEncoderFactoryBuilder withReadingUpdatePeriod() {
        return this;
    }

    public AbsoluteEncoderFactory<MagEncoderAbsoluteConfiguration> build() {
        return configuration -> {
            OffsetMagEncoder encoder = new OffsetMagEncoder(new DutyCycle(new DigitalInput(configuration.getId())), configuration.getOffset());
            encoder.setDistancePerRotation(360 * (direction == Direction.COUNTER_CLOCKWISE ? 1 : -1)); // TODO check direction

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final OffsetMagEncoder encoder;

        private EncoderImplementation(OffsetMagEncoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = encoder.getAbsolutePosition();
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}

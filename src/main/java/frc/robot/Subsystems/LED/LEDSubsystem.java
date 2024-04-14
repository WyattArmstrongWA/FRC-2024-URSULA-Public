// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class LEDSubsystem extends SubsystemBase {

    // Subsystems to query
    IntakeSubsystem m_intakeSub;
    ShooterSubsystem m_shooterSub;
    AmpSubsystem m_ampSub;
    FeederSubsystem m_feederSub;
    CommandSwerveDrivetrain m_driveSub;
    
    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(20, "rio");

    /*
     * Robot LED States
     */
    private static enum LEDState {
        START, DISABLED, DISABLED_LOW_BATTERY, DISABLED_TARGET, AUTONOMOUS, ENABLED, INTAKING, FEEDING_SHOOTER, FEEDING_AMP, CLIMBING, HAVENOTE_INTAKE, HAVENOTE_SCORING, AMPING, SHOOTING_SEQUENCE, READY_TO_SHOOT
    }
    LEDState m_currentState = LEDState.START;

    /*
     * Colors
     */
    class Color {
        int r, g, b;

        private Color(int red, int green, int blue) {
            this.r = red; this.g = green; this.b = blue;
        }
    }

    Color black = new Color(0, 0, 0); // This will Turn off the CANdle
    Color white = new Color(255, 255, 255);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 255, 0);
    Color blue = new Color(0, 0, 255);
    Color yellow = new Color(255, 255, 0);
    Color cyan = new Color(0, 255, 240);
    Color magenta = new Color(255, 0, 255);
    Color brown = new Color(166, 41, 41);
    Color pink = new Color(255, 60, 150);
    Color purple = new Color(170, 0, 255);


    /*
     * LED Segments
     */
    class LEDSegment {

        int startIndex;
        int segmentSize;
        int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation) {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.95);
        }

        public void setOff() {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);
        }

    }
    
    /*
     * Constructor
     * Creates a new LEDSubsystem
     */
    public LEDSubsystem(
                        IntakeSubsystem intakeSub,
                        ShooterSubsystem shootSub,
                        CommandSwerveDrivetrain driveSub, AmpSubsystem ampSub, FeederSubsystem feederSub) {
        m_intakeSub = intakeSub;
        m_shooterSub = shootSub;
        m_driveSub = driveSub;
        m_ampSub = ampSub;
        m_feederSub = feederSub;

        m_candle.configFactoryDefault();
        
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        m_candle.getAllConfigs(candleConfiguration);
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.GRB;
        candleConfiguration.brightnessScalar = 0.9;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

        m_candle.getAllConfigs(candleConfiguration);

        m_candle.configLEDType(LEDStripType.GRB, 300);

        m_candle.getAllConfigs(candleConfiguration);
    }

    @Override
    public void periodic() {
        double a = 2;
        double x = 3;

        LEDState newState = LEDState.DISABLED;

        if (DriverStation.isDisabled()) {
            if (RobotController.getBatteryVoltage() < 11.8) {
                newState = LEDState.DISABLED_LOW_BATTERY;
            } else if (a==x) {
                newState = LEDState.DISABLED_TARGET;
            } else {
                newState = LEDState.DISABLED;
            }            

        } else if (DriverStation.isAutonomousEnabled()) {
            newState = LEDState.AUTONOMOUS;

        } else {

            // If Note in Intake
            if (m_intakeSub.isNotePresentTOF()) {
                if (m_ampSub.isFeedToAmp()) {
                    newState = LEDState.FEEDING_AMP;
                } else if (m_ampSub.isFeedToShooter()) {
                    newState = LEDState.FEEDING_SHOOTER;
                } else if(m_feederSub.isNotePresentTOF()) {
                    newState = LEDState.HAVENOTE_SCORING;
                }
                else {
                    newState = LEDState.HAVENOTE_INTAKE;
                }
            } else if (m_ampSub.isFeedToAmp() && m_intakeSub.isIntakeRunning()  && !m_ampSub.isNotePresentTOF()) {
                    newState = LEDState.FEEDING_AMP;
            } else if (m_ampSub.isFeedToShooter() && m_intakeSub.isIntakeRunning()  && !m_feederSub.isNotePresentTOF()) {
                    newState = LEDState.FEEDING_SHOOTER;
            } else if(m_ampSub.isNotePresentTOF() && !m_ampSub.isFeedToAmp()) {
                    newState = LEDState.HAVENOTE_SCORING;
            } else if(m_feederSub.isNotePresentTOF()) {
                    newState = LEDState.HAVENOTE_SCORING;
            } else if(m_ampSub.isFeedToAmp() && !m_intakeSub.isIntakeRunning()) {
                    newState = LEDState.AMPING;
            } else {
                // Just Enabled
                newState = LEDState.ENABLED;
            }
        }

        // If State has changed, run the state machine to change LED patterns
        if (newState != m_currentState) {
            LEDStateMachine(newState);
            m_currentState = newState;
        }

    }
    
    private void LEDStateMachine(LEDState newState) {
        
        switch (newState) {
            case DISABLED:
                m_Matrix.setOff();
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        m_VerticalLeft.setAnimation(a_LeftBlueFlow);
                        m_VerticalRight.setAnimation(a_RightBlueFlow);
                    } else {
                        m_VerticalLeft.setAnimation(a_LeftRedFlow);
                        m_VerticalRight.setAnimation(a_RightRedFlow);
                    }
                }
            break;

        case DISABLED_TARGET:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftRainbow);
            m_VerticalRight.setAnimation(a_RightRainbow);
            break;

        case DISABLED_LOW_BATTERY:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_BrownLeftStrobe);
            m_VerticalRight.setAnimation(a_BrownRightStrobe);
            break;

        case AUTONOMOUS:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftFlame);
            m_VerticalRight.setAnimation(a_RightFlame);
            break;

        case ENABLED:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(white);
            m_VerticalRight.setColor(white);
            break;

        case INTAKING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_WhiteLeftStrobe);
            m_VerticalRight.setAnimation(a_WhiteRightStrobe);
            break;

        case FEEDING_AMP:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_PinkLeftPingPong);
            m_VerticalRight.setAnimation(a_PinkRightPingPong);
            break;

        case FEEDING_SHOOTER:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_CyanLeftPingPong);
            m_VerticalRight.setAnimation(a_CyanRightPingPong);
            break;

        case CLIMBING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftFlame);
            m_VerticalRight.setAnimation(a_RightFlame);
            break;

        case HAVENOTE_INTAKE:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(yellow);
            m_VerticalRight.setColor(yellow);
            break;

        case HAVENOTE_SCORING:
            m_Matrix.setOff();
            m_VerticalLeft.setColor(green);
            m_VerticalRight.setColor(green);
        break;

        case SHOOTING_SEQUENCE:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftCyanFlow);
            m_VerticalRight.setAnimation(a_RightCyanFlow);
        break;

        case READY_TO_SHOOT:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_GreenLeftStrobe);
            m_VerticalRight.setAnimation(a_GreenRightStrobe);
        break;

        case AMPING:
            m_Matrix.setOff();
            m_VerticalLeft.setAnimation(a_LeftPurpleFlow);
            m_VerticalRight.setAnimation(a_RightPurpleFlow);
        break;
            

            
                



        default:
            break;
        }
    }

    public void setBrightness(double percent) {
        /* Here we will set the brightness of the LEDs */
        m_candle.configBrightnessScalar(percent, 100);
    }

    public void fullClear() {
        clearAnimations();
        disableLEDs();
        m_candle.modulateVBatOutput(0.0);
    }

    public void clearAnimations() {
        m_candle.clearAnimation(m_Matrix.animationSlot);
        m_candle.clearAnimation(m_VerticalRight.animationSlot);
        m_candle.clearAnimation(m_VerticalLeft.animationSlot);
    }

    public void disableLEDs() {
        m_Matrix.setOff();
        m_VerticalRight.setOff();
        m_VerticalLeft.setOff();
    }
    
    LEDSegment m_Matrix = new LEDSegment(0, 1, 0);
    LEDSegment m_VerticalLeft = new LEDSegment(2, 47, 1);
    LEDSegment m_VerticalRight = new LEDSegment(50, 80, 2);
   
    Animation a_WhiteLeftStrobe = new StrobeAnimation(white.r, white.g, white.b, 0, 0.5, m_VerticalLeft.segmentSize, m_VerticalLeft.startIndex); // Flash
    Animation a_WhiteRightStrobe = new StrobeAnimation(white.r, white.g, white.b, 0, 0.5, m_VerticalRight.segmentSize, m_VerticalRight.startIndex);

    Animation a_GreenLeftStrobe = new StrobeAnimation(green.r, green.g, green.b, 0, 0.7, m_VerticalLeft.segmentSize, m_VerticalLeft.startIndex); // Flash
    Animation a_GreenRightStrobe = new StrobeAnimation(green.r, green.g, green.b, 0, 0.7, m_VerticalRight.segmentSize, m_VerticalRight.startIndex);

    Animation a_BrownLeftStrobe = new StrobeAnimation(brown.r, brown.g, brown.b, 0, 0.5, m_VerticalLeft.segmentSize, m_VerticalLeft.startIndex); // Flash
    Animation a_BrownRightStrobe = new StrobeAnimation(brown.r, brown.g, brown.b, 0, 0.5, m_VerticalRight.segmentSize, m_VerticalRight.startIndex);

    Animation a_CyanLeftPingPong = new LarsonAnimation(cyan.r, cyan.g, cyan.b, 0, 0.8, m_VerticalLeft.segmentSize, BounceMode.Back, 6, m_VerticalLeft.startIndex);
    Animation a_CyanRightPingPong = new LarsonAnimation(cyan.r, cyan.g, cyan.b, 0, 0.8, m_VerticalRight.segmentSize, BounceMode.Back, 6, m_VerticalRight.startIndex);

    Animation a_PinkLeftPingPong = new LarsonAnimation(pink.r, pink.g, pink.b, 0, 0.8, m_VerticalLeft.segmentSize, BounceMode.Back, 6, m_VerticalLeft.startIndex);
    Animation a_PinkRightPingPong = new LarsonAnimation(pink.r, pink.g, pink.b, 0, 0.8, m_VerticalRight.segmentSize, BounceMode.Back, 6, m_VerticalRight.startIndex);

    Animation a_LeftRainbow = new RainbowAnimation(0.7, 0.5, m_VerticalLeft.segmentSize, true, m_VerticalLeft.startIndex);
    Animation a_RightRainbow = new RainbowAnimation(0.7, 0.5, m_VerticalRight.segmentSize, false, m_VerticalRight.startIndex);

    Animation a_LeftFlame = new FireAnimation(0.9, 0.75, m_VerticalLeft.segmentSize, 1.0, 0.3, true, m_VerticalLeft.startIndex);
    Animation a_RightFlame = new FireAnimation(0.9, 0.75, m_VerticalRight.segmentSize, 1.0, 0.3, false, m_VerticalRight.startIndex);
   
    Animation a_LeftRedFlow = new ColorFlowAnimation(red.r, red.g, red.b, 0, 0.7, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightRedFlow = new ColorFlowAnimation(red.r, red.g, red.b, 0, 0.7, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_LeftBlueFlow = new ColorFlowAnimation(blue.r, blue.g, blue.b, 0, 0.7, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightBlueFlow = new ColorFlowAnimation(blue.r, blue.g, blue.b, 0, 0.7, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_LeftCyanFlow = new ColorFlowAnimation(cyan.r, cyan.g, cyan.b, 0, 0.95, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightCyanFlow = new ColorFlowAnimation(cyan.r, cyan.g, cyan.b, 0, 0.95, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_LeftPurpleFlow = new ColorFlowAnimation(purple.r, purple.g, purple.b, 0, 0.95, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightPurpleFlow = new ColorFlowAnimation(purple.r, purple.g, purple.b, 0, 0.95, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

    Animation a_LeftGreenTwinkle = new ColorFlowAnimation(green.r, green.g, green.b, 0, 0.2, m_VerticalLeft.segmentSize, Direction.Backward, m_VerticalLeft.startIndex);
    Animation a_RightGreenTwinkle = new ColorFlowAnimation(green.r, green.g, green.b, 0, 0.2, m_VerticalRight.segmentSize, Direction.Forward, m_VerticalRight.startIndex);

}


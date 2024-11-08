package org.firstinspires.ftc.teamcode.constants

object DeviceIDs {
    const val LF_DRIVE_MOTOR = "motorLF"
    const val LF_TURN_MOTOR = "servoLF"
    const val LF_ENCODER = "encoderLF"

    const val RF_DRIVE_MOTOR = "motorRF"
    const val RF_TURN_MOTOR = "servoRF"
    const val RF_ENCODER = "encoderRF"

    const val LR_DRIVE_MOTOR = "motorLR"
    const val LR_TURN_MOTOR = "servoLR"
    const val LR_ENCODER = "encoderLR"

    const val RR_DRIVE_MOTOR = "motorRR"
    const val RR_TURN_MOTOR = "servoRR"
    const val RR_ENCODER = "encoderRR"

    const val OTOS = "otos"

    const val ODOMETRY_POD_PARA = LR_DRIVE_MOTOR // 0
    const val ODOMETRY_POD_PERP = RR_DRIVE_MOTOR // 3

    const val ELEVATOR_LEFT = "elevatorLeft"
    const val ELEVATOR_RIGHT = "elevatorRight"

    const val ARM = "arm"
    const val INTAKE = "intake"

    const val WRIST = "wrist"
    const val CLAW_LEFT = "clawLeft"
    const val CLAW_RIGHT = "clawRight"

    /*
    Control Hub:
        Motor:
            0: LF
            1: RF
            2: LR
            3: RR
        Servo:
            0: LF
            1: RF
            2: LR
            3: RR
        Analog Input:
            0: LF
            1: RF
            2: LR
            3: RR
        I2C bus 0:
            0: imu
            1: otos

     Expansion Hub:
        Motor:
            0: elevatorLeft
            1: elevatorRight
            2: arm
            3: intake
        Servo:
            0: clawLeft
            1: clawRight
            2: wrist

    Webcam 1

    Axon Mini+ drivetrain:
    PWM Power:  100.0% //86.3%
    ProPTL:     3.0s
    Inversion:  false

    */

}
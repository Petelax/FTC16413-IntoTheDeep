package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign

class SwerveModule {
    private var motor: DcMotorEx
    private var servo: CRServoImplEx
    private var encoder: AbsoluteAnalogEncoder
    private var desiredState: SwerveModuleState
    private var driveFeedForward: SimpleMotorFeedforward
    private var turnPID: PIDController
    private var turnPower = 0.0
    private var lastTurnPower = 0.0
    private var drivePower = 0.0
    private var lastDrivePower = 0.0
    private var delta = Rotation2d()

    /**
     * @param hardwareMap hardwareMap
     * @param m drive motor id
     * @param s steer servo id
     * @param e analog encoder id
     * @param encoderOffset offset for analog encoder
     */
    constructor(hardwareMap: HardwareMap, m: String, s: String, e: String, encoderOffset: Double) {
        this.motor = hardwareMap.get(DcMotorEx::class.java, m)
        this.servo = hardwareMap.get(CRServoImplEx::class.java, s)
        this.encoder = AbsoluteAnalogEncoder(hardwareMap, e, encoderOffset, true)
        this.desiredState = SwerveModuleState()
        val c = DrivebaseConstants.ModuleCoefficients
        this.driveFeedForward = SimpleMotorFeedforward(c.KS, c.KV, c.KA)
        this.turnPID = PIDController(c.KP, c.KI, c.KD)
        initialize()
    }

    constructor(m: DcMotorEx, s: CRServoImplEx, e: AbsoluteAnalogEncoder) {
        this.motor = m
        this.servo = s
        this.encoder = e
        this.desiredState = SwerveModuleState()
        val c = DrivebaseConstants.ModuleCoefficients
        this.driveFeedForward = SimpleMotorFeedforward(c.KS, c.KV, c.KA)
        this.turnPID = PIDController(c.KP, c.KI, c.KD)
        initialize()
    }

    private fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        servo.direction = DcMotorSimple.Direction.REVERSE

        turnPID.enableContinuousInput(0.0, 2.0*Math.PI)
    }

    /**
     * @return radians
     */
    fun getHeading(): Double {
        return encoder.getHeading()
    }

    fun getEncoderVoltage(): Double {
        return encoder.getVoltage()
    }

    fun getDesiredState(): SwerveModuleState {
        return desiredState
    }

    fun setDesiredState(state: SwerveModuleState) {
        desiredState = SwerveModuleState.optimize(state, Rotation2d(getHeading()))
        // desiredState = state
        delta = state.angle.minus(Rotation2d(getHeading()))

        drivePower = driveFeedForward.calculate(desiredState.speedMetersPerSecond) / 12.0 * abs(delta.cos)
        turnPower = turnPID.calculate(getHeading(), desiredState.angle.radians)

        //(Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power)
        turnPower += if (abs(turnPID.positionError) > 0.02) 0.03 else 0.0 * sign(turnPower)

        //servo.power = turnPower
        //motor.power = drivePower
        write()

    }

    fun setDesiredStateAccel(state: SwerveModuleStateAccel) {
        val delta = state.w - getHeading()
        var desiredState = state
        var flip = 1.0
        if (abs(delta) > Math.PI/2) {
            desiredState.w = Rotation2d(state.w).rotateBy(Rotation2d(2*Math.PI)).radians
            flip = -1.0
        }

        drivePower = flip * (driveFeedForward.calculate(desiredState.v, desiredState.a) / 12.0)
        turnPower = turnPID.calculate(getHeading(), desiredState.w)
        turnPower += if (abs(turnPID.positionError) > 0.02) 0.03 else 0.0 * sign(turnPower)

        write()

    }

    fun write() {
        if (abs(drivePower) < 0.0001) {
            turnPower = 0.0
        }

        servo.power = turnPower
        motor.power = drivePower

//        if (abs(turnPower - lastTurnPower) > 0.005) {
//            servo.power = turnPower
//            lastTurnPower = turnPower
//        }
//
//        if (abs(lastDrivePower - drivePower) > 0.02) {
//            motor.power = drivePower
//            lastDrivePower = drivePower
//        }

    }

    /**
     * Spins modules
     * @param drive drive motor power. -1 to 1
     * @param steer steer motor power. -1 to 1
     */
    fun spin(drive: Double, steer: Double) {
        motor.power = drive
        servo.power = steer
    }

    fun getDelta(): Double {
        return delta.degrees
    }

    class SwerveModuleStateAccel {
        constructor() {}

        constructor(v: Double, w:Double, a: Double, vw: Double) {
            this.v = v
            this.w = w
            this.a = a
            this.vw = vw
        }
        /**
         * linear velocity m/s
         */
        var v: Double = 0.0

        /**
         * angle in radians
         */
        var w: Double = 0.0

        /**
         * linear accel in m/s^2
         */
        var a: Double = 0.0

        /**
         * angular velocity in radians/s
         */
        var vw: Double = 0.0

    }

}
package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder
import org.firstinspires.ftc.teamcode.utils.Cache
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule
/**
 * @param hardwareMap hardwareMap
 * @param m drive motor id
 * @param s steer servo id
 * @param e analog encoder id
 * @param encoderOffset offset for analog encoder
 * @param voltage robot idle voltage
 */(
    hardwareMap: HardwareMap,
    m: String,
    s: String,
    e: String,
    encoderOffset: Double,
    private var voltage: Double = 12.0
) {
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
    private var maxTurnPower = 0.863
    private var currentHeading = 0.0

    init {
        this.motor = hardwareMap.get(DcMotorEx::class.java, m)
        this.servo = hardwareMap.get(CRServoImplEx::class.java, s)
        this.encoder = AbsoluteAnalogEncoder(hardwareMap, e, encoderOffset, true)
        this.desiredState = SwerveModuleState()
        val c = DrivebaseConstants.ModuleCoefficients
        this.driveFeedForward = SimpleMotorFeedforward(c.KS, c.KV, c.KA)
        this.turnPID = PIDController(c.KP, c.KI, c.KD)
        initialize()
    }

    private fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE

        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        servo.direction = DcMotorSimple.Direction.REVERSE

        turnPID.enableContinuousInput(0.0, 2.0*Math.PI)
    }

    fun periodic() {
        currentHeading = encoder.getHeading()
    }

    /**
     * set power of the turn servo. Limits the max power
     */
    private fun setServoPower(n: Double) {
        /*
        var power = n
        if (abs(power) > 1.0)
            power = sign(n)
        power *= maxTurnPower
        */
        val corrected = n.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastTurnPower, corrected)) {
            servo.power = corrected
            lastTurnPower = corrected
        }
    }

    /**
     * set power of motor (write caching)
     */
    private fun setMotorPower(n: Double) {
        val corrected = n.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastDrivePower, corrected)) {
            motor.power = corrected
            lastDrivePower = corrected
        }
    }

    /**
     * @return radians
     */
    fun getHeading(): Double {
        return currentHeading
    }

    fun getEncoderVoltage(): Double {
        return encoder.getVoltage()
    }

    fun getDesiredState(): SwerveModuleState {
        return desiredState
    }

    fun getCurrentState(): SwerveModuleState {
        return SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d(getHeading()))
    }

    /**
     * set state of module
     */
    fun setDesiredState(state: SwerveModuleState) {
        setDesiredState(state, true)

    }

    /**
     * set state of module
     * @param state desired state
     * @param drive power the drive motor
     */
    fun setDesiredState(state: SwerveModuleState, drive: Boolean) {
        desiredState = SwerveModuleState.optimize(state, Rotation2d(getHeading()))
        delta = desiredState.angle.minus(Rotation2d(getHeading()))

        turnPower = turnPID.calculate(getHeading(), desiredState.angle.radians)

        turnPower += if (abs(turnPID.positionError) > 0.02) 0.035 else 0.0 * sign(turnPower)

        if (drive) {
            drivePower = driveFeedForward.calculate(desiredState.speedMetersPerSecond) / 12.0 // * abs(delta.cos)
            if (abs(drivePower) < 0.0001) {
                turnPower = 0.0
            }
        } else {
            drivePower = 0.0
        }

        write()

    }

    fun stopTurnServo() {
        setServoPower(0.0)
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

        setServoPower(turnPower)
        setMotorPower(drivePower)

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

    fun optimizeHeading(desiredRotation: Rotation2d): Rotation2d {
        val delta = desiredRotation.minus(Rotation2d(getHeading()))
        return if (abs(delta.degrees) > 90.0) {
            desiredRotation.rotateBy(Rotation2d.fromDegrees(180.0))
        } else {
            desiredRotation
        }
    }

    fun stop() {
        motor.power = 0.0
        servo.power = 0.0
        desiredState = SwerveModuleState()
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

    /**
     * return difference in degrees between desired and current heading
     * @return degrees
     */
    fun getDelta(): Double {
        return delta.degrees
    }

    fun kill() {
        motor.setMotorDisable()
        servo.setPwmDisable()
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
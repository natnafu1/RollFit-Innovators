#include "ClearCore.h"
// Defines the motor's connector as ConnectorM0
// Defines the second motor's connector as ConnectorM1
#define motor0 ConnectorM0
#define motor1 ConnectorM1
// Select the baud rate to match the target device.
#define baudRate 9600
// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.----------------------------------------------------------------------------------------------------------------------------------------------------------
#define SerialPort ConnectorUsb
// This example has built-in functionality to automatically clear motor faults. 
//  Any uncleared fault will cancel and disallow motion.
// WARNING: enabling automatic fault handling will clear faults immediately when 
//  encountered and return a motor to a state in which motion is allowed. Before 
//  enabling this functionality, be sure to understand this behavior and ensure 
//  your system will not enter an unsafe state. 
// To enable automatic fault handling, #define HANDLE_MOTOR_FAULTS (1)
// To disable automatic fault handling, #define HANDLE_MOTOR_FAULTS (0)
#define HANDLE_MOTOR_FAULTS (0)
// This is the variable used to keep track of the current commanded velocity
double commandedVelocity0 = 0;
double commandedVelocity1 = 0;
// A reference to the maximum clockwise and counter-clockwise velocities set in
// the MSP software. These must match the values in MSP
int32_t maxVelocityCW = 1000;
int32_t maxVelocityCCW = 1000;
// Each velocity commanded will be a multiple of this value, which must match
// the Velocity Resolution value in MSP. Use a lower value here (and in MSP) to
// command velocity with a finer resolution
double velocityResolution = 2.0;
// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
bool MoveAtVelocity0(double velocity);
bool MoveAtVelocity1(double velocity);
void HandleMotorFaults();
int main() {
    // Set all motor connectors to the correct mode for Manual Velocity
    // mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_DIRECT);
    // Set the motor's HLFB mode to bipolar PWM
    motor0.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Enforces the state of the motor's A and B inputs before enabling
    // the motor.
    motor0.MotorInAState(false);
    motor0.MotorInBState(false);
    motor1.MotorInAState(false);
    motor1.MotorInBState(false);
    // Sets up serial communication and waits up to 5 seconds for a port to open.
    // Serial communication is not required for this example to run.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }
    // Enables the motors
    motor0.EnableRequest(true);
    motor1.EnableRequest(true);
    SerialPort.SendLine("Motors Enabled");
    // Waits for HLFB to assert
    SerialPort.SendLine("Waiting for HLFB...");
    SerialPort.SendLine("Waiting for HLFB...");
    while (motor0.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor0.StatusReg().bit.MotorInFault) {
        continue;
    }
    while (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor1.StatusReg().bit.MotorInFault) {
        continue;
    }
    // Check if a motor faulted during enabling
    // Clear fault if configured to do so 
    if (motor0.StatusReg().bit.MotorInFault) {
        SerialPort.SendLine("Motor fault detected.");       
        if(HANDLE_MOTOR_FAULTS){
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        SerialPort.SendLine("Enabling may not have completed as expected. Proceed with caution.");      
        SerialPort.SendLine();
    } else {
        SerialPort.SendLine("Motor Ready"); 
    }
    if (motor1.StatusReg().bit.MotorInFault) {
        SerialPort.SendLine("Motor fault detected.");       
        if(HANDLE_MOTOR_FAULTS){
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        SerialPort.SendLine("Enabling may not have completed as expected. Proceed with caution.");      
        SerialPort.SendLine();
    } else {
        SerialPort.SendLine("Motor Ready"); 
    }
// MAIN CODE HERE @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    while (true) {
        MoveAtVelocity(30,30);
        Delay_ms(2000);
	      MoveAtVelocity(0,0);
        Delay_ms(2000);
        MoveAtVelocity(70,-40);
        Delay_ms(2000);
	      MoveAtVelocity(-10,10);
        Delay_ms(2000);
        MoveAtVelocity(0,0);
        Delay_ms(2000);
	      MoveAtVelocity(50,50);
        Delay_ms(2000);
        MoveAtVelocity(0,0);
        Delay_ms(2000);
        break;
    }
}
/*------------------------------------------------------------------------------
 * MoveAtVelocity
 *
 *    Triggers a quadrature output commanding the desired velocity.
 *    Prints the velocity and move status to the USB serial port.
 *    Returns when HLFB asserts (indicating move has successfully completed).
 *
 * Parameters:
 *    double velocity  - The velocity in RPM to command
 *
 * Returns: True/False depending on whether a new velocity was reached
 */
bool MoveAtVelocity(double velocity0,double velocity1) {
    // If the same velocity is commanded there's nothing to do.
    if (velocity0 == commandedVelocity0) {
        return false;
    }
    if (velocity1 == commandedVelocity1) {
        return false;
    }
    // Check to see if the requested velocity exceeds the valid range.
    if (velocity0 > maxVelocityCCW || velocity0 < -maxVelocityCW) {
        SerialPort.Send("M0 An invalid velocity of ");
        SerialPort.Send(velocity0);
        SerialPort.SendLine(" RPM has been requested.");
        return false;
    }
    if (velocity1 > maxVelocityCCW || velocity1 < -maxVelocityCW) {
        SerialPort.Send("M1 An invalid velocity of ");
        SerialPort.Send(velocity1);
        SerialPort.SendLine(" RPM has been requested.");
        return false;
    }
    // Check if a motor fault is currently preventing motion
    // Clear fault if configured to do so 
    if (motor0.StatusReg().bit.MotorInFault) {
        if(HANDLE_MOTOR_FAULTS){
            SerialPort.SendLine("M0 Motor fault detected. Move canceled.");
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("M0 Motor fault detected. Move canceled. Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        return false;
    }
    if (motor1.StatusReg().bit.MotorInFault) {
        if(HANDLE_MOTOR_FAULTS){
            SerialPort.SendLine("M1 Motor fault detected. Move canceled.");
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("M1 Motor fault detected. Move canceled. Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        return false;
    }
    SerialPort.Send("M0 Commanding ");
    SerialPort.Send(velocity0);
    SerialPort.SendLine(" RPM");
    SerialPort.Send("M1 Commanding ");
    SerialPort.Send(velocity1);
    SerialPort.SendLine(" RPM");
    // Determine which order the quadrature must be sent by determining if the
    // new velocity is greater or less than the previously commanded velocity
    // If greater, Input A begins the quadrature. If less, Input B begins the
    // quadrature.
    int32_t currentVelocityRounded0 = round(commandedVelocity0 / velocityResolution);
    int32_t targetVelocityRounded0 = round(velocity0 / velocityResolution);
    int32_t velocityDifference0 = labs(targetVelocityRounded0 - currentVelocityRounded0);
    int32_t currentVelocityRounded1 = round(commandedVelocity1 / velocityResolution);
    int32_t targetVelocityRounded1 = round(velocity1 / velocityResolution);
    int32_t velocityDifference1 = labs(targetVelocityRounded1 - currentVelocityRounded1);
    for (int32_t i = 0; i < velocityDifference0; i++) {
        if (velocity0 > commandedVelocity0) {
            // Toggle Input A to begin the quadrature signal.
            motor0.MotorInAState(true);
            // Command a 5 microsecond delay to ensure proper signal timing.
            Delay_us(5);
            motor0.MotorInBState(true);
            Delay_us(5);
            motor0.MotorInAState(false);
            Delay_us(5);
            motor0.MotorInBState(false);
            Delay_us(5);
        }
        else {
            motor0.MotorInBState(true);
            Delay_us(5);
            motor0.MotorInAState(true);
            Delay_us(5);
            motor0.MotorInBState(false);
            Delay_us(5);
            motor0.MotorInAState(false);
            Delay_us(5);
        }
    }
    for (int32_t i = 0; i < velocityDifference1; i++) {
        if (velocity1 > commandedVelocity1) {
            // Toggle Input A to begin the quadrature signal.
            motor1.MotorInAState(true);
            // Command a 5 microsecond delay to ensure proper signal timing.
            Delay_us(5);
            motor1.MotorInBState(true);
            Delay_us(5);
            motor1.MotorInAState(false);
            Delay_us(5);
            motor1.MotorInBState(false);
            Delay_us(5);
        }
        else {
            motor1.MotorInBState(true);
            Delay_us(5);
            motor1.MotorInAState(true);
            Delay_us(5);
            motor1.MotorInBState(false);
            Delay_us(5);
            motor1.MotorInAState(false);
            Delay_us(5);
        }
    }
    // Keeps track of the new commanded velocity
    commandedVelocity0 = velocity0;
    commandedVelocity1 = velocity1;
    // Waits for HLFB to assert (signaling the motor has successfully reached
    // its target velocity).
    SerialPort.SendLine("M0 Ramping Speed... Waiting for HLFB");
    while (motor0.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor0.StatusReg().bit.MotorInFault) {
        continue;
    }
    SerialPort.SendLine("M1 Ramping Speed... Waiting for HLFB");
    while (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor1.StatusReg().bit.MotorInFault) {
        continue;
    }
    // Check if a motor faulted during move
    // Clear fault if configured to do so 
    if (motor0.StatusReg().bit.MotorInFault) {
        SerialPort.SendLine("M0 Motor fault detected.");       
        if(HANDLE_MOTOR_FAULTS){
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("M0 Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        SerialPort.SendLine("M0 Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("M0 Move Done");
        return true;
    }
    SerialPort.SendLine("M1 Target Velocity Reached");
    return true;
    if (motor1.StatusReg().bit.MotorInFault) {
        SerialPort.SendLine("M1 Motor fault detected.");       
        if(HANDLE_MOTOR_FAULTS){
            HandleMotorFaults();
        } else {
            SerialPort.SendLine("M1 Enable automatic fault handling by setting HANDLE_MOTOR_FAULTS to 1.");
        }
        SerialPort.SendLine("M1 Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("M1 Move Done");
        return true;
    }
    SerialPort.SendLine("M1 Target Velocity Reached");
    return true;
}

//------------------------------------------------------------------------------
 
/*------------------------------------------------------------------------------
 * HandleMotorFaults
 *
 *    Clears motor faults by cycling enable to the motor.
 *    Assumes motor is in fault 
 *      (this function is called when motor.StatusReg.MotorInFault == true)
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
 void HandleMotorFaults(){
    SerialPort.SendLine("Handling fault: clearing faults by cycling enable signal to motors.");
    motor0.EnableRequest(false);
    Delay_ms(10);
    motor0.EnableRequest(true);
    Delay_ms(100);
    motor1.EnableRequest(false);
    Delay_ms(10);
    motor1.EnableRequest(true);
    Delay_ms(100);
 }
//
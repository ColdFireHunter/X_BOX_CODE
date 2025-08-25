#ifndef GPIO_H
#define GPIO_H

#include <Arduino.h>
#include <global.h>
#include <neotimer.h>

class gpio
{
public:
    gpio();
    void handler();
    void init();

    bool getRF_In1() const { return !digitalRead(RF_IN1); }       // Getter for RF_IN1
    bool getRF_In2() const { return !digitalRead(RF_IN2); }       // Getter for RF_IN2
    bool getKeySwitch1() const { return debounced_key_switch_1; } // Getter for KEY_SWITCH_1
    bool getKeySwitch2() const { return debounced_key_switch_2; } // Getter for KEY_SWITCH_2

    void setSensEnable(bool state) { digitalWrite(SENS_ENABLE, state ? HIGH : LOW); } // Set SENS_ENABLE pin state
    int getPotiTime() const { return poti_time; }                                     // Getter for poti_time
    int getSolarVoltage() const { return solar_voltage; }                             // Getter for solar_voltage
    int getBattery1Voltage() const { return battery1_voltage; }                       // Getter for battery1_voltage
    int getBattery2Voltage() const { return battery2_voltage; }                       // Getter for battery
    int getOutputCurrent() const { return output_current; }                           // Getter for output_current

    void setBoostEnable(bool state) { digitalWrite(EN_BOOST, state ? HIGH : LOW); }   // Set EN_BOOST pin state
    void setOutputEnable(bool state) { digitalWrite(EN_OUTPUT, state ? HIGH : LOW); } // Set EN_OUTPUT pin state
    bool getBoostStatus() const { return digitalRead(PG_BOOST); }                     // Get status of PG_BOOST pin

    void setLightBar1Enable(bool state) { digitalWrite(LIGHT_BAR1_ENABLE, state ? HIGH : LOW); } // Set LIGHT_BAR1_ENABLE pin state
    void setLightBar2Enable(bool state) { digitalWrite(LIGHT_BAR2_ENABLE, state ? HIGH : LOW); } // Set LIGHT_BAR2_ENABLE pin state
    bool getLightBar1() const { return debounced_light_bar1; }                                   // Getter for LIGHT_BAR1
    bool getLightBar2() const { return debounced_light_bar2; }                                   // Getter for LIGHT_BAR2

    void enableBatteryCharger1(bool state) { digitalWrite(BAT1_CHGR_ENABLE, state ? HIGH : LOW); }                   // Enable or disable battery charger 1
    void enableBatteryCharger2(bool state) { digitalWrite(BAT2_CHGR_ENABLE, state ? HIGH : LOW); }                   // Enable or disable battery charger 2
    void setBatteryCharger1VoltageSwitch(bool state) { digitalWrite(BAT1_CHGR_VOLTAGE_SWITCH, state ? HIGH : LOW); } // Set BAT1_CHGR_VOLTAGE_SWITCH pin state
    void setBatteryCharger2VoltageSwitch(bool state) { digitalWrite(BAT2_CHGR_VOLTAGE_SWITCH, state ? HIGH : LOW); } // Set BAT2_CHGR_VOLTAGE_SWITCH pin state
    bool getBatteryCharger1Status() const { return !digitalRead(BAT1_CHGR_STATUS); }                                 // Get status of BAT1_CHGR_STATUS pin
    bool getBatteryCharger2Status() const { return !digitalRead(BAT2_CHGR_STATUS); }                                 // Get status of BAT2_CHGR_STATUS pin

    bool getDip1() const { return debounced_dip_switch_1; } // Getter for DIP_SWITCH_1
    bool getDip2() const { return debounced_dip_switch_2; } // Getter for DIP_SWITCH_2
    bool getDip3() const { return debounced_dip_switch_3; } // Getter for DIP_SWITCH_3

    int getMotorDirection(); // Getter for Motor Direction

    bool getFrontPanelButton() const { return debounced_front_panel_button; } // Getter for FRONT_PANEL_BUTTON

    void triggerWisch1();
    void triggerWisch2();

private:
    void convertADCValues();
    void debounceInputs();
    bool debounce(bool signal);

    int poti_time = 0;        // Variable to store potentiometer time
    int solar_voltage = 0;    // Variable to store solar voltage
    int battery1_voltage = 0; // Variable to store battery 1 voltage
    int battery2_voltage = 0; // Variable to store battery 2 voltage
    int output_current = 0;   // Variable to store output current

    bool debounced_key_switch_1 = false; // Debounced state for key switch 1
    bool debounced_key_switch_2 = false; // Debounced state for key switch 2

    bool debounced_light_bar1 = false; // Debounced state for light bar 1
    bool debounced_light_bar2 = false; // Debounced state for light bar 2

    bool debounced_front_panel_button = false; // Debounced state for front panel button

    bool debounced_dip_switch_1 = false; // Debounced state for DIP_SWITCH_1
    bool debounced_dip_switch_2 = false; // Debounced state for DIP_SWITCH
    bool debounced_dip_switch_3 = false; // Debounced state for DIP_SWITCH_3^

    bool motor_opto = false; // State for MOTOR_OPTO

    Neotimer adcTimer = Neotimer(ADC_CONVERT_TIME); // Timer for ADC conversion
    Neotimer wisch1Timer = Neotimer(500);           // Timer for Wisch 1
    Neotimer wisch2Timer = Neotimer(500);           // Timer for Wisch 2
};

#endif // GPIO_H
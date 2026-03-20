//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS_model_balancing.h
//
// Code generated for Simulink model 'BMS_model_balancing'.
//
// Model version                  : 3.13
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Tue Feb  3 11:30:56 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef BMS_model_balancing_h_
#define BMS_model_balancing_h_
#include <stdbool.h>
#include <cstdint>

// Class declaration for model BMS_model_balancing
class BMS_logic final
{
    // public data and function members
  public:
    // Block states (default storage) for system '<S7>/Enabled Subsystem'
    struct DW_EnabledSubsystem_BMS_model_T {
        bool Relay_Mode[13];           // '<S14>/Relay'
    };

    // Block signals (default storage)
    struct B_BMS_model_balancing_T {
        bool Compare;                  // '<S32>/Compare'
        bool Compare_d;                // '<S18>/Compare'
    };

    // Block states (default storage) for system '<Root>'
    struct DW_BMS_model_balancing_T {
        double UnitDelay1_DSTATE;      // '<S23>/Unit Delay1'
        double UnitDelay1_DSTATE_d;    // '<S37>/Unit Delay1'
        double UnitDelay_DSTATE;       // '<S33>/Unit Delay'
        double UnitDelay_DSTATE_g;     // '<S19>/Unit Delay'
        int32_t UnitDelay7_DSTATE[24]; // '<S6>/Unit Delay7'
        int32_t UnitDelay3_DSTATE[14]; // '<S6>/Unit Delay3'
        int32_t UnitDelay6_DSTATE;     // '<S6>/Unit Delay6'
        bool UnitDelay5_DSTATE;        // '<Root>/Unit Delay5'
        bool UnitDelay_DSTATE_m;       // '<S17>/Unit Delay'
        bool DelayInput1_DSTATE;       // '<S26>/Delay Input1'
        bool UnitDelay_DSTATE_k;       // '<S31>/Unit Delay'
        bool DelayInput1_DSTATE_c;     // '<S40>/Delay Input1'
        bool DelayInput1_DSTATE_h;     // '<S36>/Delay Input1'
        bool DelayInput1_DSTATE_a;     // '<S22>/Delay Input1'
        DW_EnabledSubsystem_BMS_model_T EnabledSubsystem_n;// '<S8>/Enabled Subsystem' 
        DW_EnabledSubsystem_BMS_model_T EnabledSubsystem;// '<S7>/Enabled Subsystem' 
    };

    // Invariant block signals (default storage)
    struct ConstB_BMS_model_balancing_T {
        int32_t MinMax1;               // '<Root>/MinMax1'
        bool Compare;                  // '<S1>/Compare'
    };

    // Real-time Model Data Structure
    struct RT_MODEL_BMS_model_balancing_T {
        const char * volatile errorStatus;

        //
        //  Timing:
        //  The following substructure contains information regarding
        //  the timing information for the model.

        struct {
            bool stopRequestedFlag;
        } Timing;

        bool getStopRequested() const;
        void setStopRequested(bool aStopRequested);
        const char* getErrorStatus() const;
        void setErrorStatus(const char* const volatile aErrorStatus);
        bool* getStopRequestedPtr();
    };

    // Copy Constructor
    BMS_logic(BMS_logic const&) = delete;

    // Assignment Operator
    BMS_logic& operator= (BMS_logic const&) & = delete;

    // Move Constructor
    BMS_logic(BMS_logic &&) = delete;

    // Move Assignment Operator
    BMS_logic& operator= (BMS_logic &&) = delete;

    // Real-Time Model get method
    BMS_logic::RT_MODEL_BMS_model_balancing_T * getRTM();

    // model initialize function
    static void initialize();

    // model step function
    void step();

    // model terminate function
    static void terminate();

    // Constructor
    BMS_logic();

    // Destructor
    ~BMS_logic();

    // private data and function members
  private:
    // Block signals
    B_BMS_model_balancing_T BMS_model_balancing_B;

    // Block states
    DW_BMS_model_balancing_T BMS_model_balancing_DW;

    // private member function(s) for subsystem '<S7>/Enabled Subsystem'
    static void BMS_model_bala_EnabledSubsystem(bool rtu_Enable, int32_t rtu_u,
        int32_t rtu_u_j, int32_t rtu_u_jf, int32_t rtu_u_b, int32_t rtu_u_e,
        int32_t rtu_u_d, int32_t rtu_u_p, int32_t rtu_u_f, int32_t rtu_u_pn,
        int32_t rtu_u_p5, int32_t rtu_u_l, int32_t rtu_u_a, int32_t rtu_u_br,
        int32_t rty_Command[13], bool *rty_Flag, int32_t rtp_Threshold,
        DW_EnabledSubsystem_BMS_model_T *localDW);

    // Real-Time Model
    RT_MODEL_BMS_model_balancing_T BMS_model_balancing_M;
};

extern const BMS_logic::ConstB_BMS_model_balancing_T BMS_model_balancing_ConstB;// constant block i/o 

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Add' : Unused code path elimination
//  Block '<S2>/Module3V' : Unused code path elimination
//  Block '<S3>/Module4V' : Unused code path elimination
//  Block '<S4>/Module1V' : Unused code path elimination
//  Block '<S5>/Module2V' : Unused code path elimination
//  Block '<Root>/Contactors' : Unused code path elimination
//  Block '<Root>/MinMax' : Unused code path elimination
//  Block '<S7>/Data Type Conversion1' : Unused code path elimination
//  Block '<S21>/Data Type Duplicate' : Unused code path elimination
//  Block '<S25>/Data Type Duplicate' : Unused code path elimination
//  Block '<S8>/Data Type Conversion1' : Unused code path elimination
//  Block '<S35>/Data Type Duplicate' : Unused code path elimination
//  Block '<S39>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Scope2' : Unused code path elimination
//  Block '<Root>/Scope3' : Unused code path elimination
//  Block '<Root>/Scope4' : Unused code path elimination
//  Block '<Root>/To Workspace' : Unused code path elimination
//  Block '<Root>/To Workspace1' : Unused code path elimination
//  Block '<Root>/To Workspace3' : Unused code path elimination
//  Block '<Root>/To Workspace4' : Unused code path elimination
//  Block '<Root>/Unit Delay1' : Unused code path elimination
//  Block '<Root>/Unit Delay2' : Unused code path elimination
//  Block '<Root>/Unit Delay4' : Unused code path elimination
//  Block '<S7>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S21>/Conversion' : Eliminate redundant data type conversion
//  Block '<S19>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S25>/Conversion' : Eliminate redundant data type conversion
//  Block '<S23>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/Rate Transition' : Eliminated since input and output rates are identical
//  Block '<S7>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S8>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S35>/Conversion' : Eliminate redundant data type conversion
//  Block '<S33>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S39>/Conversion' : Eliminate redundant data type conversion
//  Block '<S37>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S8>/Rate Transition' : Eliminated since input and output rates are identical
//  Block '<S8>/Rate Transition1' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'BMS_model_balancing'
//  '<S1>'   : 'BMS_model_balancing/Balancing true if threshold exceeded for any cell1'
//  '<S2>'   : 'BMS_model_balancing/Cells 1-2'
//  '<S3>'   : 'BMS_model_balancing/Cells 1-3'
//  '<S4>'   : 'BMS_model_balancing/Cells 1-6'
//  '<S5>'   : 'BMS_model_balancing/Cells 7-12'
//  '<S6>'   : 'BMS_model_balancing/Fault Logic'
//  '<S7>'   : 'BMS_model_balancing/Passive Cell Balancing - Slave 1'
//  '<S8>'   : 'BMS_model_balancing/Passive Cell Balancing - Slave 2'
//  '<S9>'   : 'BMS_model_balancing/Fault Logic/Balancing true if threshold exceeded for any cell'
//  '<S10>'  : 'BMS_model_balancing/Fault Logic/Balancing true if threshold exceeded for any cell2'
//  '<S11>'  : 'BMS_model_balancing/Fault Logic/Balancing true if threshold exceeded for any cell3'
//  '<S12>'  : 'BMS_model_balancing/Fault Logic/Balancing true if threshold exceeded for any cell4'
//  '<S13>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Delay'
//  '<S14>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Enabled Subsystem'
//  '<S15>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Off delay'
//  '<S16>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/On delay'
//  '<S17>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Delay/Discrete'
//  '<S18>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Enabled Subsystem/Balancing true if threshold exceeded for any cell'
//  '<S19>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Off delay/Discrete'
//  '<S20>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Off delay/Discrete/Compare To Zero'
//  '<S21>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Off delay/Discrete/Data Type Conversion Inherited'
//  '<S22>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/Off delay/Discrete/Detect Decrease'
//  '<S23>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/On delay/Discrete'
//  '<S24>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/On delay/Discrete/Compare To Zero1'
//  '<S25>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/On delay/Discrete/Data Type Conversion Inherited'
//  '<S26>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 1/On delay/Discrete/Detect Increase'
//  '<S27>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Delay'
//  '<S28>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Enabled Subsystem'
//  '<S29>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Off delay'
//  '<S30>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/On delay'
//  '<S31>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Delay/Discrete'
//  '<S32>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Enabled Subsystem/Balancing true if threshold exceeded for any cell'
//  '<S33>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Off delay/Discrete'
//  '<S34>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Off delay/Discrete/Compare To Zero'
//  '<S35>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Off delay/Discrete/Data Type Conversion Inherited'
//  '<S36>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/Off delay/Discrete/Detect Decrease'
//  '<S37>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/On delay/Discrete'
//  '<S38>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/On delay/Discrete/Compare To Zero1'
//  '<S39>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/On delay/Discrete/Data Type Conversion Inherited'
//  '<S40>'  : 'BMS_model_balancing/Passive Cell Balancing - Slave 2/On delay/Discrete/Detect Increase'

#endif                                 // BMS_model_balancing_h_

//
// File trailer for generated code.
//
// [EOF]
//

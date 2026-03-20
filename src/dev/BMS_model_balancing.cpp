//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS_model_balancing.cpp
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
#include "dev/BMS_model_balancing.h"

// Invariant block signals (default storage)
const BMS_logic::ConstB_BMS_model_balancing_T BMS_model_balancing_ConstB{
    1
    ,                                  // '<Root>/MinMax1'
    1
    // '<S1>/Compare'
};

// Variable: BalThreshold
//  Referenced by:
//    '<S7>/Enabled Subsystem'
//    '<S8>/Enabled Subsystem'

#define rtCP_pooled1                   ((static_cast<int32_t>(0L)))

// Pooled Parameter (Mixed Expressions)
//  Referenced by:
//    '<S19>/Constant'
//    '<S19>/Constant1'
//    '<S23>/Constant'
//    '<S23>/Constant2'
//    '<S33>/Constant'
//    '<S33>/Constant1'
//    '<S37>/Constant'
//    '<S37>/Constant2'

#define rtCP_pooled2                   (1.0)

// Pooled Parameter (Expression: 0)
//  Referenced by:
//    '<S19>/Unit Delay'
//    '<S23>/Unit Delay1'
//    '<S33>/Unit Delay'
//    '<S37>/Unit Delay1'
//    '<S20>/Constant'
//    '<S24>/Constant'
//    '<S34>/Constant'
//    '<S38>/Constant'

#define rtCP_pooled3                   (0.0)

// Pooled Parameter (Mixed Expressions)
//  Referenced by:
//    '<S1>/Constant'
//    '<S6>/Unit Delay3'
//    '<S6>/Unit Delay6'
//    '<S6>/Unit Delay7'
//    '<S11>/Constant'
//    '<S14>/Relay'
//    '<S28>/Relay'

#define rtCP_pooled5                   ((static_cast<int32_t>(0L)))

// Pooled Parameter (Mixed Expressions)
//  Referenced by:
//    '<Root>/Charge Current'
//    '<S2>/Cell13SOC'
//    '<S2>/Cell13Temp'
//    '<S2>/Cell13V'
//    '<S2>/Cell14SOC'
//    '<S2>/Cell14Temp'
//    '<S2>/Cell14V'
//    '<S2>/Cell15SOC'
//    '<S2>/Cell15Temp'
//    '<S2>/Cell15V'
//    '<S2>/Cell16SOC'
//    '<S2>/Cell16Temp'
//    '<S2>/Cell16V'
//    '<S2>/Cell17SOC'
//    '<S2>/Cell17Temp'
//    '<S2>/Cell17V'
//    '<S2>/Cell18SOC'
//    '<S2>/Cell18Temp'
//    '<S2>/Cell18V'
//    '<S3>/Cell19SOC'
//    '<S3>/Cell19Temp'
//    '<S3>/Cell19V'
//    '<S3>/Cell20SOC'
//    '<S3>/Cell20V'
//    '<S3>/Cell21SOC'
//    '<S3>/Cell21V'
//    '<S3>/Cell22SOC'
//    '<S3>/Cell22V'
//    '<S3>/Cell23SOC'
//    '<S3>/Cell23V'
//    '<S3>/Cell24SOC'
//    '<S3>/Cell24V'
//    '<S4>/Cell1SOC'
//    '<S4>/Cell1Temp'
//    '<S4>/Cell1V'
//    '<S4>/Cell2SOC'
//    '<S4>/Cell2Temp'
//    '<S4>/Cell2V'
//    '<S4>/Cell3SOC'
//    '<S4>/Cell3Temp'
//    '<S4>/Cell3V'
//    '<S4>/Cell4SOC'
//    '<S4>/Cell4Temp'
//    '<S4>/Cell4V'
//    '<S4>/Cell5SOC'
//    '<S4>/Cell5Temp'
//    '<S4>/Cell5V'
//    '<S4>/Cell6SOC'
//    '<S4>/Cell6Temp'
//    '<S4>/Cell6V'
//    '<S5>/Cell10SOC'
//    '<S5>/Cell10V'
//    '<S5>/Cell11SOC'
//    '<S5>/Cell11V'
//    '<S5>/Cell12SOC'
//    '<S5>/Cell12V'
//    '<S5>/Cell7SOC'
//    '<S5>/Cell7Temp'
//    '<S5>/Cell7V'
//    '<S5>/Cell8SOC'
//    '<S5>/Cell8V'
//    '<S5>/Cell9SOC'
//    '<S5>/Cell9V'
//    '<S12>/Constant'
//    '<S14>/Relay'
//    '<S28>/Relay'

#define rtCP_pooled6                   ((static_cast<int32_t>(1L)))

// Computed Parameter: Constant_Value
//  Referenced by: '<S9>/Constant'

#define rtCP_Constant_Value            ((static_cast<int32_t>(318L)))

// Computed Parameter: Constant_Value_e
//  Referenced by: '<S10>/Constant'

#define rtCP_Constant_Value_e          ((static_cast<int32_t>(225L)))

// Pooled Parameter (Mixed Expressions)
//  Referenced by:
//    '<Root>/Unit Delay5'
//    '<S17>/Unit Delay'
//    '<S18>/Constant'
//    '<S31>/Unit Delay'
//    '<S32>/Constant'
//    '<S22>/Delay Input1'
//    '<S26>/Delay Input1'
//    '<S36>/Delay Input1'
//    '<S40>/Delay Input1'

#define rtCP_pooled8                   (false)

//
// Output and update for enable system:
//    '<S7>/Enabled Subsystem'
//    '<S8>/Enabled Subsystem'
//
void BMS_logic::BMS_model_bala_EnabledSubsystem(bool rtu_Enable, int32_t rtu_u,
    int32_t rtu_u_j, int32_t rtu_u_jf, int32_t rtu_u_b, int32_t rtu_u_e, int32_t
    rtu_u_d, int32_t rtu_u_p, int32_t rtu_u_f, int32_t rtu_u_pn, int32_t
    rtu_u_p5, int32_t rtu_u_l, int32_t rtu_u_a, int32_t rtu_u_br, int32_t
    rty_Command[13], bool *rty_Flag, int32_t rtp_Threshold,
    DW_EnabledSubsystem_BMS_model_T *localDW)
{
    int32_t rtb_Sum1[13];
    int32_t minV;
    int32_t u1;
    int16_t k;
    bool rtb_DataTypeConversion[13];
    bool maxV;

    // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (rtu_Enable) {
        // MinMax: '<S14>/MinMax'
        rtb_Sum1[0] = rtu_u;
        rtb_Sum1[1] = rtu_u_j;
        rtb_Sum1[2] = rtu_u_jf;
        rtb_Sum1[3] = rtu_u_b;
        rtb_Sum1[4] = rtu_u_e;
        rtb_Sum1[5] = rtu_u_d;
        rtb_Sum1[6] = rtu_u_p;
        rtb_Sum1[7] = rtu_u_f;
        rtb_Sum1[8] = rtu_u_pn;
        rtb_Sum1[9] = rtu_u_p5;
        rtb_Sum1[10] = rtu_u_l;
        rtb_Sum1[11] = rtu_u_a;
        rtb_Sum1[12] = rtu_u_br;
        minV = rtu_u;
        for (k = 0; k < 12; k++) {
            u1 = rtb_Sum1[static_cast<int16_t>(k + 1)];
            if (minV > u1) {
                minV = u1;
            }
        }

        // Sum: '<S14>/Sum1' incorporates:
        //   MinMax: '<S14>/MinMax'

        rtb_Sum1[0] = static_cast<int32_t>(rtu_u - minV);
        rtb_Sum1[1] = static_cast<int32_t>(rtu_u_j - minV);
        rtb_Sum1[2] = static_cast<int32_t>(rtu_u_jf - minV);
        rtb_Sum1[3] = static_cast<int32_t>(rtu_u_b - minV);
        rtb_Sum1[4] = static_cast<int32_t>(rtu_u_e - minV);
        rtb_Sum1[5] = static_cast<int32_t>(rtu_u_d - minV);
        rtb_Sum1[6] = static_cast<int32_t>(rtu_u_p - minV);
        rtb_Sum1[7] = static_cast<int32_t>(rtu_u_f - minV);
        rtb_Sum1[8] = static_cast<int32_t>(rtu_u_pn - minV);
        rtb_Sum1[9] = static_cast<int32_t>(rtu_u_p5 - minV);
        rtb_Sum1[10] = static_cast<int32_t>(rtu_u_l - minV);
        rtb_Sum1[11] = static_cast<int32_t>(rtu_u_a - minV);
        rtb_Sum1[12] = static_cast<int32_t>(rtu_u_br - minV);
        for (k = 0; k < 13; k++) {
            // Relay: '<S14>/Relay' incorporates:
            //   Sum: '<S14>/Sum1'

            minV = rtb_Sum1[static_cast<int32_t>(k)];
            if (minV >= rtp_Threshold) {
                localDW->Relay_Mode[static_cast<int32_t>(k)] = true;
            } else if (minV <= rtCP_pooled5) {
                localDW->Relay_Mode[static_cast<int32_t>(k)] = false;
            } else {
                // no actions
            }

            if (localDW->Relay_Mode[static_cast<int32_t>(k)]) {
                minV = rtCP_pooled6;
                rty_Command[static_cast<int32_t>(k)] = rtCP_pooled6;
            } else {
                minV = rtCP_pooled5;
                rty_Command[static_cast<int32_t>(k)] = rtCP_pooled5;
            }

            // DataTypeConversion: '<S14>/Data Type Conversion' incorporates:
            //   Relay: '<S14>/Relay'

            rtb_DataTypeConversion[k] = (minV != 0L);
        }

        // MinMax: '<S14>/MinMax1' incorporates:
        //   DataTypeConversion: '<S14>/Data Type Conversion'

        maxV = rtb_DataTypeConversion[0];
        for (k = 0; k < 12; k++) {
            if (static_cast<int16_t>(maxV) < static_cast<int16_t>
                    (rtb_DataTypeConversion[static_cast<int16_t>(k + 1)])) {
                maxV = true;
            }
        }

        // RelationalOperator: '<S18>/Compare' incorporates:
        //   Constant: '<S18>/Constant'
        //   MinMax: '<S14>/MinMax1'

        *rty_Flag = (static_cast<int16_t>(maxV) > static_cast<int16_t>
                     (rtCP_pooled8));
    }

    // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
}

// Model step function
void BMS_logic::step()
{
    int32_t rtb_Relay[13];
    int32_t UnitDelay7_DSTATE;
    int32_t maxV;
    int32_t maxV_0;
    int32_t u1;
    int16_t k;
    bool rtb_LogicalOperator;
    bool rtb_LogicalOperator_d;

    // Stop: '<Root>/Stop Simulation' incorporates:
    //   UnitDelay: '<Root>/Unit Delay5'

    if (BMS_model_balancing_DW.UnitDelay5_DSTATE) {
        (&BMS_model_balancing_M)->setStopRequested(1);
    }

    // End of Stop: '<Root>/Stop Simulation'

    // MinMax: '<S6>/MinMax2' incorporates:
    //   UnitDelay: '<S6>/Unit Delay3'

    maxV = BMS_model_balancing_DW.UnitDelay3_DSTATE[0];
    for (k = 0; k < 13; k++) {
        u1 = BMS_model_balancing_DW.UnitDelay3_DSTATE[static_cast<int16_t>(k + 1)];
        if (maxV < u1) {
            maxV = u1;
        }
    }

    // MinMax: '<S6>/MinMax4' incorporates:
    //   UnitDelay: '<S6>/Unit Delay7'

    u1 = BMS_model_balancing_DW.UnitDelay7_DSTATE[0];

    // MinMax: '<S6>/MinMax5' incorporates:
    //   UnitDelay: '<S6>/Unit Delay7'

    maxV_0 = BMS_model_balancing_DW.UnitDelay7_DSTATE[0];
    for (k = 0; k < 23; k++) {
        // MinMax: '<S6>/MinMax4' incorporates:
        //   UnitDelay: '<S6>/Unit Delay7'

        UnitDelay7_DSTATE = BMS_model_balancing_DW.UnitDelay7_DSTATE[
            static_cast<int16_t>(k + 1)];
        if (u1 > UnitDelay7_DSTATE) {
            u1 = UnitDelay7_DSTATE;
        }

        // MinMax: '<S6>/MinMax5' incorporates:
        //   UnitDelay: '<S6>/Unit Delay7'

        if (maxV_0 < UnitDelay7_DSTATE) {
            maxV_0 = UnitDelay7_DSTATE;
        }
    }

    // MinMax: '<S6>/MinMax3' incorporates:
    //   Constant: '<S10>/Constant'
    //   Constant: '<S11>/Constant'
    //   Constant: '<S12>/Constant'
    //   Constant: '<S9>/Constant'
    //   MinMax: '<S6>/MinMax2'
    //   MinMax: '<S6>/MinMax4'
    //   MinMax: '<S6>/MinMax5'
    //   RelationalOperator: '<S10>/Compare'
    //   RelationalOperator: '<S11>/Compare'
    //   RelationalOperator: '<S12>/Compare'
    //   RelationalOperator: '<S9>/Compare'
    //   UnitDelay: '<S6>/Unit Delay6'

    BMS_model_balancing_DW.UnitDelay5_DSTATE = (maxV_0 > rtCP_pooled6);
    if (static_cast<int16_t>(BMS_model_balancing_DW.UnitDelay5_DSTATE) <
            static_cast<int16_t>(u1 < rtCP_pooled5)) {
        BMS_model_balancing_DW.UnitDelay5_DSTATE = true;
    }

    if (static_cast<int16_t>(BMS_model_balancing_DW.UnitDelay5_DSTATE) <
            static_cast<int16_t>(maxV > rtCP_Constant_Value)) {
        BMS_model_balancing_DW.UnitDelay5_DSTATE = true;
    }

    if (static_cast<int16_t>(BMS_model_balancing_DW.UnitDelay5_DSTATE) <
            static_cast<int16_t>(BMS_model_balancing_DW.UnitDelay6_DSTATE >
            rtCP_Constant_Value_e)) {
        BMS_model_balancing_DW.UnitDelay5_DSTATE = true;
    }

    // End of MinMax: '<S6>/MinMax3'

    // Logic: '<S7>/Logical Operator' incorporates:
    //   UnitDelay: '<S17>/Unit Delay'

    rtb_LogicalOperator = (BMS_model_balancing_ConstB.Compare ||
                           BMS_model_balancing_DW.UnitDelay_DSTATE_m);

    // Switch: '<S23>/Switch1' incorporates:
    //   Constant: '<S23>/Constant'
    //   Constant: '<S23>/Constant2'
    //   RelationalOperator: '<S26>/FixPt Relational Operator'
    //   Sum: '<S23>/Sum'
    //   UnitDelay: '<S26>/Delay Input1'
    //
    //  Block description for '<S26>/Delay Input1':
    //
    //   Store in Global RAM

    if (static_cast<int16_t>(rtb_LogicalOperator) > static_cast<int16_t>
            (BMS_model_balancing_DW.DelayInput1_DSTATE)) {
        BMS_model_balancing_DW.UnitDelay1_DSTATE = rtCP_pooled2;
    }

    BMS_model_balancing_DW.UnitDelay1_DSTATE -= rtCP_pooled2;

    // End of Switch: '<S23>/Switch1'

    // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // Logic: '<S23>/Logical Operator3' incorporates:
    //   Constant: '<S24>/Constant'
    //   Constant: '<S4>/Cell1V'
    //   Constant: '<S4>/Cell2V'
    //   Constant: '<S4>/Cell3V'
    //   Constant: '<S4>/Cell4V'
    //   Constant: '<S4>/Cell5V'
    //   Constant: '<S4>/Cell6V'
    //   Constant: '<S5>/Cell10V'
    //   Constant: '<S5>/Cell11V'
    //   Constant: '<S5>/Cell12V'
    //   Constant: '<S5>/Cell7V'
    //   Constant: '<S5>/Cell8V'
    //   Constant: '<S5>/Cell9V'
    //   RelationalOperator: '<S24>/Compare'
    //   UnitDelay: '<S23>/Unit Delay1'

    BMS_model_bala_EnabledSubsystem((rtb_LogicalOperator &&
        (BMS_model_balancing_DW.UnitDelay1_DSTATE < rtCP_pooled3)),
        BMS_model_balancing_ConstB.MinMax1, rtCP_pooled6, rtCP_pooled6,
        rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6,
        rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6,
        rtb_Relay, &BMS_model_balancing_B.Compare_d, rtCP_pooled1,
        &BMS_model_balancing_DW.EnabledSubsystem);

    // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'

    // Logic: '<S8>/Logical Operator' incorporates:
    //   UnitDelay: '<S31>/Unit Delay'

    rtb_LogicalOperator_d = (BMS_model_balancing_ConstB.Compare ||
        BMS_model_balancing_DW.UnitDelay_DSTATE_k);

    // Switch: '<S37>/Switch1' incorporates:
    //   Constant: '<S37>/Constant'
    //   Constant: '<S37>/Constant2'
    //   RelationalOperator: '<S40>/FixPt Relational Operator'
    //   Sum: '<S37>/Sum'
    //   UnitDelay: '<S40>/Delay Input1'
    //
    //  Block description for '<S40>/Delay Input1':
    //
    //   Store in Global RAM

    if (static_cast<int16_t>(rtb_LogicalOperator_d) > static_cast<int16_t>
            (BMS_model_balancing_DW.DelayInput1_DSTATE_c)) {
        BMS_model_balancing_DW.UnitDelay1_DSTATE_d = rtCP_pooled2;
    }

    BMS_model_balancing_DW.UnitDelay1_DSTATE_d -= rtCP_pooled2;

    // End of Switch: '<S37>/Switch1'

    // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // Logic: '<S37>/Logical Operator3' incorporates:
    //   Constant: '<S2>/Cell13V'
    //   Constant: '<S2>/Cell14V'
    //   Constant: '<S2>/Cell15V'
    //   Constant: '<S2>/Cell16V'
    //   Constant: '<S2>/Cell17V'
    //   Constant: '<S2>/Cell18V'
    //   Constant: '<S38>/Constant'
    //   Constant: '<S3>/Cell19V'
    //   Constant: '<S3>/Cell20V'
    //   Constant: '<S3>/Cell21V'
    //   Constant: '<S3>/Cell22V'
    //   Constant: '<S3>/Cell23V'
    //   Constant: '<S3>/Cell24V'
    //   RelationalOperator: '<S38>/Compare'
    //   UnitDelay: '<S37>/Unit Delay1'

    BMS_model_bala_EnabledSubsystem((rtb_LogicalOperator_d &&
        (BMS_model_balancing_DW.UnitDelay1_DSTATE_d < rtCP_pooled3)),
        BMS_model_balancing_ConstB.MinMax1, rtCP_pooled6, rtCP_pooled6,
        rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6,
        rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6, rtCP_pooled6,
        rtb_Relay, &BMS_model_balancing_B.Compare, rtCP_pooled1,
        &BMS_model_balancing_DW.EnabledSubsystem_n);

    // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'

    // Switch: '<S33>/Switch' incorporates:
    //   Constant: '<S33>/Constant'
    //   Constant: '<S33>/Constant1'
    //   RelationalOperator: '<S36>/FixPt Relational Operator'
    //   Sum: '<S33>/Sum'
    //   UnitDelay: '<S36>/Delay Input1'
    //
    //  Block description for '<S36>/Delay Input1':
    //
    //   Store in Global RAM

    if (static_cast<int16_t>(BMS_model_balancing_B.Compare) <
            static_cast<int16_t>(BMS_model_balancing_DW.DelayInput1_DSTATE_h)) {
        BMS_model_balancing_DW.UnitDelay_DSTATE = rtCP_pooled2;
    }

    BMS_model_balancing_DW.UnitDelay_DSTATE -= rtCP_pooled2;

    // End of Switch: '<S33>/Switch'

    // Logic: '<S33>/Logical Operator5' incorporates:
    //   Constant: '<S34>/Constant'
    //   Logic: '<S33>/Logical Operator'
    //   Logic: '<S33>/Logical Operator1'
    //   RelationalOperator: '<S34>/Compare'
    //   UnitDelay: '<S31>/Unit Delay'
    //   UnitDelay: '<S33>/Unit Delay'

    BMS_model_balancing_DW.UnitDelay_DSTATE_k =
        ((BMS_model_balancing_DW.UnitDelay_DSTATE >= rtCP_pooled3) ||
         BMS_model_balancing_B.Compare);

    // Switch: '<S19>/Switch' incorporates:
    //   Constant: '<S19>/Constant'
    //   Constant: '<S19>/Constant1'
    //   RelationalOperator: '<S22>/FixPt Relational Operator'
    //   Sum: '<S19>/Sum'
    //   UnitDelay: '<S22>/Delay Input1'
    //
    //  Block description for '<S22>/Delay Input1':
    //
    //   Store in Global RAM

    if (static_cast<int16_t>(BMS_model_balancing_B.Compare_d) <
            static_cast<int16_t>(BMS_model_balancing_DW.DelayInput1_DSTATE_a)) {
        BMS_model_balancing_DW.UnitDelay_DSTATE_g = rtCP_pooled2;
    }

    BMS_model_balancing_DW.UnitDelay_DSTATE_g -= rtCP_pooled2;

    // End of Switch: '<S19>/Switch'

    // Logic: '<S19>/Logical Operator5' incorporates:
    //   Constant: '<S20>/Constant'
    //   Logic: '<S19>/Logical Operator'
    //   Logic: '<S19>/Logical Operator1'
    //   RelationalOperator: '<S20>/Compare'
    //   UnitDelay: '<S17>/Unit Delay'
    //   UnitDelay: '<S19>/Unit Delay'

    BMS_model_balancing_DW.UnitDelay_DSTATE_m =
        ((BMS_model_balancing_DW.UnitDelay_DSTATE_g >= rtCP_pooled3) ||
         BMS_model_balancing_B.Compare_d);

    // Update for UnitDelay: '<S6>/Unit Delay7' incorporates:
    //   Constant: '<S2>/Cell13SOC'
    //   Constant: '<S2>/Cell14SOC'
    //   Constant: '<S2>/Cell15SOC'
    //   Constant: '<S2>/Cell16SOC'
    //   Constant: '<S2>/Cell17SOC'
    //   Constant: '<S2>/Cell18SOC'
    //   Constant: '<S3>/Cell19SOC'
    //   Constant: '<S3>/Cell20SOC'
    //   Constant: '<S3>/Cell21SOC'
    //   Constant: '<S3>/Cell22SOC'
    //   Constant: '<S3>/Cell23SOC'
    //   Constant: '<S3>/Cell24SOC'
    //   Constant: '<S4>/Cell1SOC'
    //   Constant: '<S4>/Cell2SOC'
    //   Constant: '<S4>/Cell3SOC'
    //   Constant: '<S4>/Cell4SOC'
    //   Constant: '<S4>/Cell5SOC'
    //   Constant: '<S4>/Cell6SOC'
    //   Constant: '<S5>/Cell10SOC'
    //   Constant: '<S5>/Cell11SOC'
    //   Constant: '<S5>/Cell12SOC'
    //   Constant: '<S5>/Cell7SOC'
    //   Constant: '<S5>/Cell8SOC'
    //   Constant: '<S5>/Cell9SOC'

    BMS_model_balancing_DW.UnitDelay7_DSTATE[0] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[1] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[2] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[3] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[4] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[5] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[6] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[7] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[8] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[9] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[10] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[11] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[12] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[13] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[14] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[15] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[16] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[17] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[18] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[19] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[20] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[21] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[22] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay7_DSTATE[23] = rtCP_pooled6;

    // Update for UnitDelay: '<S6>/Unit Delay3' incorporates:
    //   Constant: '<S2>/Cell13Temp'
    //   Constant: '<S2>/Cell14Temp'
    //   Constant: '<S2>/Cell15Temp'
    //   Constant: '<S2>/Cell16Temp'
    //   Constant: '<S2>/Cell17Temp'
    //   Constant: '<S2>/Cell18Temp'
    //   Constant: '<S3>/Cell19Temp'
    //   Constant: '<S4>/Cell1Temp'
    //   Constant: '<S4>/Cell2Temp'
    //   Constant: '<S4>/Cell3Temp'
    //   Constant: '<S4>/Cell4Temp'
    //   Constant: '<S4>/Cell5Temp'
    //   Constant: '<S4>/Cell6Temp'
    //   Constant: '<S5>/Cell7Temp'

    BMS_model_balancing_DW.UnitDelay3_DSTATE[0] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[1] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[2] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[3] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[4] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[5] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[6] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[7] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[8] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[9] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[10] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[11] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[12] = rtCP_pooled6;
    BMS_model_balancing_DW.UnitDelay3_DSTATE[13] = rtCP_pooled6;

    // Update for UnitDelay: '<S6>/Unit Delay6' incorporates:
    //   Constant: '<Root>/Charge Current'

    BMS_model_balancing_DW.UnitDelay6_DSTATE = rtCP_pooled6;

    // Update for UnitDelay: '<S26>/Delay Input1'
    //
    //  Block description for '<S26>/Delay Input1':
    //
    //   Store in Global RAM

    BMS_model_balancing_DW.DelayInput1_DSTATE = rtb_LogicalOperator;

    // Update for UnitDelay: '<S40>/Delay Input1'
    //
    //  Block description for '<S40>/Delay Input1':
    //
    //   Store in Global RAM

    BMS_model_balancing_DW.DelayInput1_DSTATE_c = rtb_LogicalOperator_d;

    // Update for UnitDelay: '<S36>/Delay Input1'
    //
    //  Block description for '<S36>/Delay Input1':
    //
    //   Store in Global RAM

    BMS_model_balancing_DW.DelayInput1_DSTATE_h = BMS_model_balancing_B.Compare;

    // Update for UnitDelay: '<S22>/Delay Input1'
    //
    //  Block description for '<S22>/Delay Input1':
    //
    //   Store in Global RAM

    BMS_model_balancing_DW.DelayInput1_DSTATE_a =
        BMS_model_balancing_B.Compare_d;
}

// Model initialize function
void BMS_logic::initialize()
{
    // (no initialization code required)
}

// Model terminate function
void BMS_logic::terminate()
{
    // (no terminate code required)
}

bool BMS_logic::RT_MODEL_BMS_model_balancing_T::getStopRequested() const
{
    return (Timing.stopRequestedFlag);
}

void BMS_logic::RT_MODEL_BMS_model_balancing_T::setStopRequested(bool
    aStopRequested)
{
    (Timing.stopRequestedFlag = aStopRequested);
}

const char* BMS_logic::RT_MODEL_BMS_model_balancing_T::getErrorStatus() const
{
    return (errorStatus);
}

void BMS_logic::RT_MODEL_BMS_model_balancing_T::setErrorStatus(const char* const
    volatile aErrorStatus)
{
    (errorStatus = aErrorStatus);
}

bool* BMS_logic::RT_MODEL_BMS_model_balancing_T::getStopRequestedPtr()
{
    return (&(Timing.stopRequestedFlag));
}

// Constructor
BMS_logic::BMS_logic() :
    BMS_model_balancing_B(),
    BMS_model_balancing_DW(),
    BMS_model_balancing_M()
{
    // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
BMS_logic::~BMS_logic() = default;

// Real-Time Model get method
BMS_logic::RT_MODEL_BMS_model_balancing_T * BMS_logic::getRTM()
{
    return (&BMS_model_balancing_M);
}

//
// File trailer for generated code.
//
// [EOF]
//

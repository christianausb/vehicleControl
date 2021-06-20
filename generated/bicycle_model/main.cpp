        
#include <math.h>
#include <stdio.h>
#include <emscripten/bind.h>

using namespace emscripten;
        
// namespace for simulation {
  // global variables

  class simulation {
    public:


    // state update
    double block_39_mem;
    double block_43_mem;
    int32_t block_15_mem;
    double block_18_mem;
    double block_37_mem;


    // state update


    //
    // cached output values
    //

    double y__block_39;
    double s47__block_43;
    int32_t s19_shared_counter__block_15;
    int32_t s45__block_41;
    bool s46__block_42;
    double s9__block_5;
    double s10__block_6;
    double psi__block_44;
    double s22__block_18;
    int32_t s20__block_16;
    bool s21__block_17;
    double s6__block_3;
    double s7__block_4;
    double s23__block_19;
    double s24__block_20;
    bool s25__block_21;
    double s26__block_22;
    double s27__block_23;
    bool s28__block_24;
    double steering__block_25;
    double x__block_37;

    // API-function resetStates
    void resetStates() { // created by cpp_define_function

      block_39_mem = 0.0;
      block_43_mem = 0.0;
      block_15_mem = 0;
      block_18_mem = 0.0;
      block_37_mem = 0.0;
    }
    // output signals of  resetStates
    struct Outputs_resetStates{
      ;

    };
    // input signals of resetStates
    struct Inputs_resetStates{
      ;

    };
    // wrapper function for resetStates
    Outputs_resetStates resetStates__(Inputs_resetStates inputs) {
      Outputs_resetStates outputs;

      resetStates();

      return outputs;
    }
    // API-function updateStates
    void updateStates(double velocity, double steering_rate) { // created by cpp_define_function
      double s33;
      double s34;
      double s35;
      double s42;
      double s36;
      double s37;
      double s38;
      double s39;
      double s44;
      int32_t s17_cnt_increase;
      int32_t s18;
      double s3;
      double s4;
      double s16;
      double s30;
      double s31;
      double s32;
      double s40;


      // restoring the signals y, s47, s19_shared_counter, s45, s46, s9, s10, psi, s22, s20, s21, s6, s7, s23, s24, s25, s26, s27, s28, steering, x from the states 
      double &y = y__block_39;
      double &s47 = s47__block_43;
      int32_t &s19_shared_counter = s19_shared_counter__block_15;
      int32_t &s45 = s45__block_41;
      bool &s46 = s46__block_42;
      double &s9 = s9__block_5;
      double &s10 = s10__block_6;
      double &psi = psi__block_44;
      double &s22 = s22__block_18;
      int32_t &s20 = s20__block_16;
      bool &s21 = s21__block_17;
      double &s6 = s6__block_3;
      double &s7 = s7__block_4;
      double &s23 = s23__block_19;
      double &s24 = s24__block_20;
      bool &s25 = s25__block_21;
      double &s26 = s26__block_22;
      double &s27 = s27__block_23;
      bool &s28 = s28__block_24;
      double &steering = steering__block_25;
      double &x = x__block_37;


      // calculating the block outputs in the following order s33, s34, s35, s42, s36, s37, s38, s39, s44, s17_cnt_increase, s18, s3, s4, s16, s30, s31, s32, s40
      // that depend on velocity, steering_rate
      // dependencies that require a state update are  

      s33 = steering + psi;
      s34 = sin(s33);
      s35 = velocity * s34;
      s42 = 1 * y + 0.01 * s35;
      s36 = 3.0;
      s37 = velocity / s36;
      s38 = sin(steering);
      s39 = s37 * s38;
      s44 = 1 * psi + 0.01 * s39;
      s17_cnt_increase = 1;
      s18 = s19_shared_counter + s17_cnt_increase;
      s3 = 0.017453292519943295;
      s4 = steering_rate * s3;
      s16 = 1 * s23 + 0.01 * s4;
      s30 = steering + psi;
      s31 = cos(s30);
      s32 = velocity * s31;
      s40 = 1 * x + 0.01 * s32;

      block_39_mem = s42;
      block_43_mem = s44;
      block_15_mem = s18;
      block_18_mem = s16;
      block_37_mem = s40;

      // calculating the block outputs in the following order 
      // that depend on 
      // dependencies that require a state update are  


    }
    // output signals of  updateStates
    struct Outputs_updateStates{
      ;

    };
    // input signals of updateStates
    struct Inputs_updateStates{
      double velocity;
      double steering_rate;

    };
    // wrapper function for updateStates
    Outputs_updateStates updateStates__(Inputs_updateStates inputs) {
      Outputs_updateStates outputs;

      updateStates(inputs.velocity, inputs.steering_rate);

      return outputs;
    }
    // API-function calcResults_1 to compute: x, y, psi, steering
    void calcResults_1(double &x, double &y, double &psi, double &steering, double initial_orientation, double initial_steering) { // created by cpp_define_function
      double s47;
      int32_t s19_shared_counter;
      int32_t s45;
      bool s46;
      double s9;
      double s10;
      double s22;
      int32_t s20;
      bool s21;
      double s6;
      double s7;
      double s23;
      double s24;
      bool s25;
      double s26;
      double s27;
      bool s28;


      // calculating the block outputs in the following order y, s47, s19_shared_counter, s45, s46, s9, s10, psi, s22, s20, s21, s6, s7, s23, s24, s25, s26, s27, s28, steering, x
      // that depend on initial_orientation, initial_steering
      // dependencies that require a state update are s42, s44, s18, s16, s40 

      y = block_39_mem;
      s47 = block_43_mem;
      s19_shared_counter = block_15_mem;
      s45 = 0;
      s46 = s19_shared_counter == s45;
      s9 = 0.017453292519943295;
      s10 = initial_orientation * s9;
      if (s46) {
        psi = s10;
      } else {
        psi = s47;
      }
      s22 = block_18_mem;
      s20 = 0;
      s21 = s19_shared_counter == s20;
      s6 = 0.017453292519943295;
      s7 = initial_steering * s6;
      if (s21) {
        s23 = s7;
      } else {
        s23 = s22;
      }
      s24 = -1.5707963267948966;
      s25 = s23 < s24;
      if (s25) {
        s26 = -1.5707963267948966;
      } else {
        s26 = s23;
      }
      s27 = 1.5707963267948966;
      s28 = s26 > s27;
      if (s28) {
        steering = 1.5707963267948966;
      } else {
        steering = s26;
      }
      x = block_37_mem;

      // saving the signals y, s47, s19_shared_counter, s45, s46, s9, s10, psi, s22, s20, s21, s6, s7, s23, s24, s25, s26, s27, s28, steering, x into the states 
      y__block_39 = y;
      s47__block_43 = s47;
      s19_shared_counter__block_15 = s19_shared_counter;
      s45__block_41 = s45;
      s46__block_42 = s46;
      s9__block_5 = s9;
      s10__block_6 = s10;
      psi__block_44 = psi;
      s22__block_18 = s22;
      s20__block_16 = s20;
      s21__block_17 = s21;
      s6__block_3 = s6;
      s7__block_4 = s7;
      s23__block_19 = s23;
      s24__block_20 = s24;
      s25__block_21 = s25;
      s26__block_22 = s26;
      s27__block_23 = s27;
      s28__block_24 = s28;
      steering__block_25 = steering;
      x__block_37 = x;
    }
    // output signals of  calcResults_1
    struct Outputs_calcResults_1{
      double x;
      double y;
      double psi;
      double steering;

    };
    // input signals of calcResults_1
    struct Inputs_calcResults_1{
      double initial_orientation;
      double initial_steering;

    };
    // wrapper function for calcResults_1
    Outputs_calcResults_1 calcResults_1__(Inputs_calcResults_1 inputs) {
      Outputs_calcResults_1 outputs;

      calcResults_1(outputs.x, outputs.y, outputs.psi, outputs.steering,   inputs.initial_orientation, inputs.initial_steering);

      return outputs;
    }
    // all system inputs and outputs combined
    struct Inputs{
      double velocity;
      double initial_orientation;
      double initial_steering;
      double steering_rate;

    };
    struct Outputs{
      double x;
      double y;
      double psi;
      double steering;

    };
    // main step function 
    void step(Outputs & outputs, Inputs const & inputs, int calculate_outputs, bool update_states, bool reset_states) {
      if (reset_states) {
        resetStates();

      }
      if (calculate_outputs==1) {
        calcResults_1(outputs.x, outputs.y, outputs.psi, outputs.steering,   inputs.initial_orientation, inputs.initial_steering);

      }
      if (update_states) {
        updateStates(inputs.velocity, inputs.steering_rate);

      }

    }
  };

// end of namespace for simulation





// Binding code
EMSCRIPTEN_BINDINGS(my_class_example) {
  class_<simulation>("simulation")
    .constructor<>()
    .function("resetStates", &simulation::resetStates__)
    .function("calcResults_1", &simulation::calcResults_1__)
    .function("updateStates", &simulation::updateStates__)
    .function("step", &simulation::step)
    ;

        
value_object<simulation::Inputs>("simulation__Inputs")
.field("initial_orientation", &simulation::Inputs::initial_orientation)
.field("initial_steering", &simulation::Inputs::initial_steering)
.field("velocity", &simulation::Inputs::velocity)
.field("steering_rate", &simulation::Inputs::steering_rate)
;

value_object<simulation::Outputs>("simulation__Outputs")
.field("x", &simulation::Outputs::x)
.field("y", &simulation::Outputs::y)
.field("psi", &simulation::Outputs::psi)
.field("steering", &simulation::Outputs::steering)
;

value_object<simulation::Inputs_calcResults_1>("simulation__Inputs_calcResults_1")
.field("initial_orientation", &simulation::Inputs_calcResults_1::initial_orientation)
.field("initial_steering", &simulation::Inputs_calcResults_1::initial_steering)
;

value_object<simulation::Outputs_calcResults_1>("simulation__Outputs_calcResults_1")
.field("x", &simulation::Outputs_calcResults_1::x)
.field("y", &simulation::Outputs_calcResults_1::y)
.field("psi", &simulation::Outputs_calcResults_1::psi)
.field("steering", &simulation::Outputs_calcResults_1::steering)
;

value_object<simulation::Inputs_updateStates>("simulation__Inputs_updateStates")
.field("velocity", &simulation::Inputs_updateStates::velocity)
.field("steering_rate", &simulation::Inputs_updateStates::steering_rate)
;

value_object<simulation::Outputs_updateStates>("simulation__Outputs_updateStates")
;

value_object<simulation::Inputs_resetStates>("simulation__Inputs_resetStates")
;

value_object<simulation::Outputs_resetStates>("simulation__Outputs_resetStates")
;


}

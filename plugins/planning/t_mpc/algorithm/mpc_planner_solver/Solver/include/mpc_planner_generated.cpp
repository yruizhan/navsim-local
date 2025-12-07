#include <mpc_planner_generated.h>

#include <ros_tools/logging.h>

#include <stdexcept>

namespace MPCPlanner{

double getForcesOutput(const Solver_output& output, const int k, const int var_index){
		if(k == 0)
			{
				if(var_index >= 2)					LOG_WARN("getForcesOutput for k = 0 returns the initial state.");
			return output.x01[var_index];
		}
		if(k == 1)
			return output.x02[var_index];
		if(k == 2)
			return output.x03[var_index];
		if(k == 3)
			return output.x04[var_index];
		if(k == 4)
			return output.x05[var_index];
		if(k == 5)
			return output.x06[var_index];
		if(k == 6)
			return output.x07[var_index];
		if(k == 7)
			return output.x08[var_index];
		if(k == 8)
			return output.x09[var_index];
		if(k == 9)
			return output.x10[var_index];
		if(k == 10)
			return output.x11[var_index];
		if(k == 11)
			return output.x12[var_index];
		if(k == 12)
			return output.x13[var_index];
		if(k == 13)
			return output.x14[var_index];
		if(k == 14)
			return output.x15[var_index];
		if(k == 15)
			return output.x16[var_index];
		if(k == 16)
			return output.x17[var_index];
		if(k == 17)
			return output.x18[var_index];
		if(k == 18)
			return output.x19[var_index];
		if(k == 19)
			return output.x20[var_index];
		if(k == 20)
			return output.x21[var_index];
		if(k == 21)
			return output.x22[var_index];
		if(k == 22)
			return output.x23[var_index];
		if(k == 23)
			return output.x24[var_index];
		if(k == 24)
			return output.x25[var_index];
		if(k == 25)
			return output.x26[var_index];
		if(k == 26)
			return output.x27[var_index];
		if(k == 27)
			return output.x28[var_index];
		if(k == 28)
			return output.x29[var_index];
		if(k == 29)
			return output.x30[var_index];
		if(k == 30)
			return output.x31[var_index];
		if(k == 31)
			return output.x32[var_index];
		if(k == 32)
			return output.x33[var_index];
		if(k == 33)
			return output.x34[var_index];
		if(k == 34)
			return output.x35[var_index];
		if(k == 35)
			return output.x36[var_index];
		if(k == 36)
			return output.x37[var_index];
		if(k == 37)
			return output.x38[var_index];
		if(k == 38)
			return output.x39[var_index];
		if(k == 39)
			return output.x40[var_index];
		if(k == 40)
			return output.x41[var_index];
		if(k == 41)
			return output.x42[var_index];
		if(k == 42)
			return output.x43[var_index];
		if(k == 43)
			return output.x44[var_index];
		if(k == 44)
			return output.x45[var_index];
		if(k == 45)
			return output.x46[var_index];
		if(k == 46)
			return output.x47[var_index];
		if(k == 47)
			return output.x48[var_index];
		if(k == 48)
			return output.x49[var_index];
		if(k == 49)
			return output.x50[var_index];
		if(k == 50)
			return output.x51[var_index];
		if(k == 51)
			return output.x52[var_index];
		if(k == 52)
			return output.x53[var_index];
		if(k == 53)
			return output.x54[var_index];
		if(k == 54)
			return output.x55[var_index];
		if(k == 55)
			return output.x56[var_index];
		if(k == 56)
			return output.x57[var_index];
		if(k == 57)
			return output.x58[var_index];
		if(k == 58)
			return output.x59[var_index];
		if(k == 59)
			return output.x60[var_index];
throw std::runtime_error("Invalid k value for getForcesOutput");
}

void loadForcesWarmstart(Solver_params& params, const Solver_output& output){
		for (int i = 0; i < 2; i++){
			params.z_init_00[i] = params.x0[i];
		}
		for (int i = 0; i < 7; i++){
			params.z_init_01[i] = params.x0[7*1 + i];
			params.z_init_02[i] = params.x0[7*2 + i];
			params.z_init_03[i] = params.x0[7*3 + i];
			params.z_init_04[i] = params.x0[7*4 + i];
			params.z_init_05[i] = params.x0[7*5 + i];
			params.z_init_06[i] = params.x0[7*6 + i];
			params.z_init_07[i] = params.x0[7*7 + i];
			params.z_init_08[i] = params.x0[7*8 + i];
			params.z_init_09[i] = params.x0[7*9 + i];
			params.z_init_10[i] = params.x0[7*10 + i];
			params.z_init_11[i] = params.x0[7*11 + i];
			params.z_init_12[i] = params.x0[7*12 + i];
			params.z_init_13[i] = params.x0[7*13 + i];
			params.z_init_14[i] = params.x0[7*14 + i];
			params.z_init_15[i] = params.x0[7*15 + i];
			params.z_init_16[i] = params.x0[7*16 + i];
			params.z_init_17[i] = params.x0[7*17 + i];
			params.z_init_18[i] = params.x0[7*18 + i];
			params.z_init_19[i] = params.x0[7*19 + i];
			params.z_init_20[i] = params.x0[7*20 + i];
			params.z_init_21[i] = params.x0[7*21 + i];
			params.z_init_22[i] = params.x0[7*22 + i];
			params.z_init_23[i] = params.x0[7*23 + i];
			params.z_init_24[i] = params.x0[7*24 + i];
			params.z_init_25[i] = params.x0[7*25 + i];
			params.z_init_26[i] = params.x0[7*26 + i];
			params.z_init_27[i] = params.x0[7*27 + i];
			params.z_init_28[i] = params.x0[7*28 + i];
			params.z_init_29[i] = params.x0[7*29 + i];
			params.z_init_30[i] = params.x0[7*30 + i];
			params.z_init_31[i] = params.x0[7*31 + i];
			params.z_init_32[i] = params.x0[7*32 + i];
			params.z_init_33[i] = params.x0[7*33 + i];
			params.z_init_34[i] = params.x0[7*34 + i];
			params.z_init_35[i] = params.x0[7*35 + i];
			params.z_init_36[i] = params.x0[7*36 + i];
			params.z_init_37[i] = params.x0[7*37 + i];
			params.z_init_38[i] = params.x0[7*38 + i];
			params.z_init_39[i] = params.x0[7*39 + i];
			params.z_init_40[i] = params.x0[7*40 + i];
			params.z_init_41[i] = params.x0[7*41 + i];
			params.z_init_42[i] = params.x0[7*42 + i];
			params.z_init_43[i] = params.x0[7*43 + i];
			params.z_init_44[i] = params.x0[7*44 + i];
			params.z_init_45[i] = params.x0[7*45 + i];
			params.z_init_46[i] = params.x0[7*46 + i];
			params.z_init_47[i] = params.x0[7*47 + i];
			params.z_init_48[i] = params.x0[7*48 + i];
			params.z_init_49[i] = params.x0[7*49 + i];
			params.z_init_50[i] = params.x0[7*50 + i];
			params.z_init_51[i] = params.x0[7*51 + i];
			params.z_init_52[i] = params.x0[7*52 + i];
			params.z_init_53[i] = params.x0[7*53 + i];
			params.z_init_54[i] = params.x0[7*54 + i];
			params.z_init_55[i] = params.x0[7*55 + i];
			params.z_init_56[i] = params.x0[7*56 + i];
			params.z_init_57[i] = params.x0[7*57 + i];
			params.z_init_58[i] = params.x0[7*58 + i];
			params.z_init_59[i] = params.x0[7*59 + i];
		}
	}
	void setForcesReinitialize(Solver_params& params, const bool value){
	}
}

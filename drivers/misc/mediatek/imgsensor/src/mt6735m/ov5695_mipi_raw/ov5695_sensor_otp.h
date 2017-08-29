struct otp_struct {
	int flag;
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
};
struct otp_struct current_otp;
int RG_Ratio_Typical = 312; // rg_ratio of WB typical module
int BG_Ratio_Typical = 332; // bg_ratio of WB typical module
int flag_otp_read = 0;
int R_gain = 0;
int G_gain = 0;
int B_gain = 0;
static kal_uint16 read_cmos_sensor(kal_uint32 addr);
static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);



// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int ov5695_read_otp(struct otp_struct *otp_ptr)
{
	int otp_flag, otp_base, temp, i;
	// Pre-processing
//	write_cmos_sensor(0x5001, 0x20); // disable OTP_DPC
	// read OTP into buffer
	write_cmos_sensor(0x3d84, 0xC0);
	write_cmos_sensor(0x3d88, 0x70); // OTP start address
	write_cmos_sensor(0x3d89, 0x0C);
	write_cmos_sensor(0x3d8A, 0x70); // OTP end address
	write_cmos_sensor(0x3d8B, 0x1B);
	write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
	mDELAY(10);
	// OTP into
	otp_flag = read_cmos_sensor(0x700C);
	LOG_INF("otp flag addr[0x700C] value = 0x%x\n",  otp_flag);
	otp_base = 0;
	if((otp_flag & 0xc0) == 0x40) {
		otp_base = 0x700D; // base address of info group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		otp_base = 0x7012; // base address of info group 2
	}
	else if((otp_flag & 0x0c) == 0x04) {
		otp_base = 0x7017; // base address of info group 3
	}
	if(otp_base != 0) {
		(*otp_ptr).flag = 0x40; // valid info in OTP
		(*otp_ptr).module_integrator_id = read_cmos_sensor(otp_base);
		(*otp_ptr).lens_id = read_cmos_sensor(otp_base + 1);
		
		temp = read_cmos_sensor(otp_base + 4);
		(*otp_ptr).rg_ratio = (read_cmos_sensor(otp_base+2)<<2) + ((temp>>6) & 0x03);
		(*otp_ptr).bg_ratio = (read_cmos_sensor(otp_base+3)<<2) + ((temp>>4) & 0x03);
		LOG_INF(" ModuleId = 0x%x, lens_id = 0x%x, rg =  0x%x ,bg =  0x%x\n", (*otp_ptr).module_integrator_id, (*otp_ptr).lens_id, (*otp_ptr).rg_ratio, (*otp_ptr).bg_ratio);
	}
	else {
		LOG_INF("err:otp date wrong!!!\n");
		(*otp_ptr).flag = 0x00; // not info in OTP
		(*otp_ptr).module_integrator_id = 0;
		(*otp_ptr).lens_id = 0;
		(*otp_ptr).rg_ratio = 0;
		(*otp_ptr).bg_ratio = 0;		
	}

	for(i=0;i<5;i++) {
		temp = read_cmos_sensor(otp_base + i);
		LOG_INF("otp debug 0x%x 0x%x\n", otp_base+i, temp);
	}


	// Post-processing
	if((*otp_ptr).flag != 0) {
		for(i=0x700C;i<=0x701B;i++) {
			write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
		}
	}
	//write_cmos_sensor(0x5001, 0x28); // enable OTP_DPC
	return (*otp_ptr).flag;
}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int ov5695_apply_otp(void)
{
	int rg, bg, Base_gain;
	LOG_INF("E\n");
	if(flag_otp_read<1)
	{
		ov5695_read_otp(&current_otp);
		// apply OTP WB Calibration
		if (current_otp.flag & 0x40) {
			rg = current_otp.rg_ratio;
			bg = current_otp.bg_ratio;
			//calculate sensor WB gain, 0x400 = 1x gain

			R_gain = (1000 * RG_Ratio_Typical )/ rg;
			G_gain = 1000;
			B_gain = (1000 * BG_Ratio_Typical )/ bg;
			
			if (R_gain < 1000 || B_gain < 1000)
			{
				if(R_gain<B_gain){
					Base_gain = R_gain;
				}
				else{
					Base_gain = B_gain;
				}
			}
			else{
				Base_gain = G_gain;
			}
			// set min gain to 0x400
			R_gain = 0x400 * R_gain / Base_gain;
			G_gain = 0x400 * G_gain / Base_gain;
			B_gain = 0x400 * B_gain / Base_gain;
			flag_otp_read = 1;
		}
	}
		printk(" R_gain =0x%x , G_gain =0x%x , B_gain =0x%x \n", R_gain, G_gain, B_gain);

		// update sensor WB gain
		if (R_gain>0x400) {
			write_cmos_sensor(0x5019, R_gain>>8);
			write_cmos_sensor(0x501A, R_gain & 0x00ff);
		}
		if (G_gain>0x400) {
			write_cmos_sensor(0x501B, G_gain>>8);
			write_cmos_sensor(0x501C, G_gain & 0x00ff);
		}
		if (B_gain>0x400) {
			write_cmos_sensor(0x501D, B_gain>>8);
			write_cmos_sensor(0x501E, B_gain & 0x00ff);
		}

	
	return current_otp.flag;
}

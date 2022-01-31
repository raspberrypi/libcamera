#pragma once

// Definition of the PiSP Front End class.

#include "pisp_fe_config.h"
#include "pisp_statistics.h"

namespace PiSP {

class FrontEnd
{
public:
	FrontEnd(FrontEndHal *hal, Device *device, int align = 64);
	~FrontEnd();

	void SetGlobal(PISP_FE_GLOBAL_CONFIG_T const &global);
	void SetInput(PISP_FE_INPUT_CONFIG_T const &input);
	void SetInputBuffer(PISP_FE_INPUT_BUFFER_CONFIG_T const &input_buffer);
	void SetDecompress(PISP_FE_DECOMPRESS_CONFIG_T const &decompress);
	void SetDecompand(PISP_FE_DECOMPAND_CONFIG_T const &decompand);
	void SetDpc(PISP_FE_DPC_CONFIG_T const &dpc);
	void SetBla(PISP_FE_BLA_CONFIG_T const &bla);
	void SetStatsCrop(PISP_FE_CROP_CONFIG_T const &stats_crop);
	void SetBlc(PISP_FE_BLA_CONFIG_T const &blc);
	void SetRGBY(PISP_FE_RGBY_CONFIG_T const &rgby);
	void SetLsc(PISP_FE_LSC_CONFIG_T const &lsc);
	void SetAgcStats(PISP_FE_AGC_STATS_CONFIG_T const &agc_stats);
	void SetAwbStats(PISP_FE_AWB_STATS_CONFIG_T const &awb_stats);
	void SetFloatingStats(PISP_FE_FLOATING_STATS_CONFIG_T const &floating_stats);
	void SetCdafStats(PISP_FE_CDAF_STATS_CONFIG_T const &cdaf_stats);
	void SetCrop(int output_num, PISP_FE_CROP_CONFIG_T const &crop);
	void SetDownscale(int output_num, PISP_FE_DOWNSCALE_CONFIG_T const &downscale);
	void SetCompress(int output_num, PISP_FE_COMPRESS_CONFIG_T const &compress);
	void SetOutputFormat(int output_num, PISP_IMAGE_FORMAT_CONFIG_T const &output_format);
	void SetOutputBuffer(int output_num, PISP_FE_OUTPUT_BUFFER_CONFIG_T const &output_buffer);
	void SetOutputIntrLines(int output_num, int lines);
	void SetOutputAXI(PISP_FE_OUTPUT_AXI_CONFIG_T const &output_axi);

	void WriteRegisters(HAL_REG_VALUE_T const *reg_values, int num_reg_values);

private:
	void finalise();
	void getOutputSize(int output_num, uint16_t *width, uint16_t *height) const;

	pisp_fe_config fe_config_;
	int align_;
};

} // namespace PiSP

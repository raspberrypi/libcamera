// Implementation of the PiSP Front End driver.

#include "frontend.h"

using namespace PiSP;
namespace arg = std::placeholders;

FrontEnd::FrontEnd(bool streaming = true)
{
	PISP_FE_INPUT_CONFIG_T input;

	memset(&fe_config_, 0, sizeof(fe_config_));
	memset(&input, 0, sizeof(input));

	input.streaming = !!streaming;

	if (!input.streaming) {
		// Configure some plausible default AXI reader settings
		input.axi.maxlen_flags = PISP_AXI_FLAG_ALIGN | 7;
		input.axi.cache_prot = 0x33;
		input.axi.qos = 0;
		input.holdoff = 0;
	}

	SetInput(input);
	SetGlobal(PISP_FE_ENABLE_INPUT);
}

FrontEnd::~FrontEnd()
{
}

void FrontEnd::switchMode(PISP_DEVICE_MODE_T const &pending_mode)
{
	// Set the input format configuration.
	PISP_FE_INPUT_CONFIG_T input;
	memset(&input, 0, sizeof(input));
	input.format = pending_mode.base_mode->format;
	input.streaming = (device_->GetDeviceType() == Device::Type::Streaming);
	if (!input.streaming) {
		// Configure some plausible default AXI reader settings
		input.axi.maxlen_flags = PISP_AXI_FLAG_ALIGN | 7;
		input.axi.cache_prot = 0x33;
		input.axi.qos = 0;
		input.holdoff = 0;
	}
	SetInput(input);
	// If we have a triggered device, provide it the necessary callback.
	if (!input.streaming)
		device_->SetTriggerCallback(std::bind(&device_queue_buffer, this, arg::_1, arg::_2));
	// Make sure the input block is enabled!
	PISP_FE_GLOBAL_CONFIG_T global;
	GetGlobal(global);
	global.enables |= PISP_FE_ENABLE_INPUT;
	SetGlobal(global);
	mode_ = pending_mode;
	// Trigger the controller callbacks.
	if (switch_mode_callback_)
		switch_mode_callback_(mode_);
}

void FrontEnd::submitInputToHal(FrontEndInput &input)
{
	FrontEndConfigPtr config = hal_->GetNextConfigBuffer();
	if (!config)
		throw std::runtime_error("FrontEnd: insufficient configuration buffers");

	FrontEndHalInput hal_input;
	hal_input.input = std::move(input.input);
	hal_input.frame_id = input.frame_id;
	hal_input.device_mode = mode_;
	hal_input.metadata = std::make_shared<Metadata>();
	hal_input.front_end = this;
	hal_input.device = device_;
	if (pre_callback_)
		pre_callback_(hal_input);
	finalise();
	*config = fe_config_;
	fe_config_.dirty_flags = fe_config_.dirty_flags_extra = 0;
	hal_input.config = std::move(config);
	hal_input.front_end = this;
	hal_->Queue(hal_input);
}

void FrontEnd::feThreadFunc()
{
	try {
		thread_id_ = std::this_thread::get_id();
		state_ = State::Started;
		EventMsg device_stop_msg;
		hal_->Start();
		if (initialise_callback_)
			initialise_callback_();
		uint32_t jobs_done = 0, jobs_submitted = 0;

		do {
			EventMsg msg;
			event_queue.wait(msg);

			switch (msg.type) {
			case Event::DeviceTrigger: {
				PISP_LOG(SEVERITY, "Device Trigger...");
				FrontEndInput input = boost::get<FrontEndInput>(msg.payload);
				if (state_ == State::Started && device_state_ == State::Started) {
					submitInputToHal(input);
					jobs_submitted++;
				}
				break;
			}

			case Event::PostCallback: {
				PISP_LOG(SEVERITY, "Post Callback...");
				jobs_done++;
				FrontEndOutput output = boost::get<FrontEndOutput>(msg.payload);
				if (device_->Streaming() && state_ == State::Started && device_state_ == State::Started) {
					FrontEndInput input;
					submitInputToHal(input);
					jobs_submitted++;
				}
				post_callback_(output);
				break;
			}

			case Event::SetMode: {
				PISP_LOG(SEVERITY, "Set Mode...");
				assert(device_state_ == State::Stopped);
				PISP_DEVICE_MODE_T pending_mode = boost::get<PISP_DEVICE_MODE_T>(msg.payload);
				if (device_->SetMode(pending_mode) != PISP_SUCCESS)
					throw std::runtime_error("Device SetMode failed.");
				switchMode(pending_mode);
				break;
			}

			case Event::FEStop: {
				PISP_LOG(SEVERITY, "FE stop pending...");
				// Might as well stop the device.
				device_->Stop();
				state_ = State::Stopping;
				if (jobs_done == jobs_submitted)
					state_ = State::Stopped;
				else
					hal_->Abort();
				break;
			}

			case Event::DeviceStop: {
				PISP_LOG(SEVERITY, "Device stop pending...");
				device_state_ = State::Stopping;
				device_->Stop();
				if (jobs_done == jobs_submitted) {
					device_state_ = State::Stopped;
					event_queue.respond(msg);
				} else {
					hal_->Abort();
					// Hold on to the message so we can reply to it when we do finally stop.
					device_stop_msg = msg;
				}
				break;
			}

			case Event::DeviceStart: {
				PISP_LOG(SEVERITY, "Device start...");
				device_->Start();
				device_state_ = State::Started;
				if (device_->Streaming()) {
					// For streaming devices we must submitt *two* FrontEndHalInputs to the HAL to keep it fed.
					FrontEndInput input;
					submitInputToHal(input);
					submitInputToHal(input);
					jobs_submitted += 2;
				}
				break;
			}

			default:
				PISP_LOG(SEVERITY, "Unknown event...");
				assert(0);
			}
		} while (state_ != State::Stopped);
		PISP_LOG(SEVERITY, "Front End thread stopping");
		hal_->Stop();
	} catch (std::exception const &e) {
		PISP_LOG(fatal, "thread aborting (" << e.what() << ")");
		assert(0);
		abort();
	}
}

void FrontEnd::Start()
{
	checkFrontEndThreadContext(false);
	assert(state_ == State::Stopped);
	thread_ = std::thread(&FrontEnd::feThreadFunc, this);
}

void FrontEnd::Stop()
{
	checkFrontEndThreadContext(false);
	event_queue.post(EventMsg(Event::FEStop));
	thread_.join();
}

void FrontEnd::StartDevice()
{
	checkFrontEndThreadContext(false);
	event_queue.post(EventMsg(Event::DeviceStart));
}

void FrontEnd::StopDevice()
{
	checkFrontEndThreadContext(false);
	event_queue.post_wait(EventMsg(Event::DeviceStop));
}

static void finalise_lsc(PISP_FE_LSC_CONFIG_T &lsc, uint16_t width, uint16_t height)
{
	if (lsc.centre_x == 0)
		lsc.centre_x = width / 2;
	if (lsc.centre_y == 0)
		lsc.centre_y = height / 2;
	if (lsc.scale == 0) {
		// XXX magic constants taken from the specification
		static const int SCALE_PRECISION = 10;
		static const int INTERP_PRECISION = 6;
		uint16_t max_dx = std::max(width - lsc.centre_x, (int)lsc.centre_x);
		uint16_t max_dy = std::max(height - lsc.centre_y, (int)lsc.centre_y);
		uint32_t max_r2 = max_dx * (uint32_t)max_dx + max_dy * (uint32_t)max_dy;
		assert(max_r2 < (1u << 31)); // spec requires r^2 to fit 31 bits
		lsc.shift = 0;
		while (max_r2 >= 2 * ((PISP_FE_LSC_LUT_SIZE - 1) << INTERP_PRECISION))
			max_r2 >>= 1, lsc.shift++;
		lsc.scale = ((1 << SCALE_PRECISION) * ((PISP_FE_LSC_LUT_SIZE - 1) << INTERP_PRECISION) - 1) / max_r2;
		if (lsc.scale >= (1 << SCALE_PRECISION))
			lsc.scale = (1 << SCALE_PRECISION) - 1;
	}
}

static void finalise_agc(PISP_FE_AGC_STATS_CONFIG_T &agc, uint16_t width, uint16_t height)
{
	if (agc.size_x == 0)
		agc.size_x = std::max(2, ((width - 2 * agc.offset_x) / PISP_AGC_STATS_SIZE) & ~1);
	if (agc.size_y == 0)
		agc.size_y = std::max(2, ((height - 2 * agc.offset_y) / PISP_AGC_STATS_SIZE) & ~1);
	if (agc.row_size_x == 0)
		agc.row_size_x = std::max(2, (width - 2 * agc.row_offset_x) & ~1);
	if (agc.row_size_y == 0)
		agc.row_size_y = std::max(2, ((height - 2 * agc.row_offset_y) / PISP_AGC_STATS_NUM_ROW_SUMS) & ~1);
}

static void finalise_awb(PISP_FE_AWB_STATS_CONFIG_T &awb, uint16_t width, uint16_t height)
{
	// Just a warning that ACLS algorithms might want the size calculations here to match the Back End LSC.
	if (awb.size_x == 0)
		awb.size_x = std::max(2, ((width - 2 * awb.offset_x + PISP_AWB_STATS_SIZE - 1) / PISP_AWB_STATS_SIZE));
	awb.size_x += (awb.size_x & 1);
	if (awb.size_y == 0)
		awb.size_y = std::max(2, ((height - 2 * awb.offset_y + PISP_AWB_STATS_SIZE - 1) / PISP_AWB_STATS_SIZE));
	awb.size_y += (awb.size_y & 1);
}

static void finalise_cdaf(PISP_FE_CDAF_STATS_CONFIG_T &cdaf, uint16_t width, uint16_t height)
{
	if (cdaf.size_x == 0)
		cdaf.size_x = std::max(2, ((width - 2 * cdaf.offset_x) / PISP_CDAF_STATS_SIZE) & ~1);
	if (cdaf.size_y == 0)
		cdaf.size_y = std::max(2, ((height - 2 * cdaf.offset_y) / PISP_CDAF_STATS_SIZE) & ~1);
}

static void finalise_downscale(PISP_FE_DOWNSCALE_CONFIG_T &downscale, uint16_t width, uint16_t height)
{
	downscale.output_width = (((width >> 1) * downscale.xout) / downscale.xin) * 2;
	downscale.output_height = (((height >> 1) * downscale.yout) / downscale.yin) * 2;
}

static void finalise_compression(PISP_FRONT_END_CONFIG_T const &fe_config, int i)
{
	uint32_t fmt = fe_config.ch[i].output.format.format, enables = fe_config.global.enables;
	if (PISP_IMAGE_FORMAT_compressed(fmt) && !(enables & PISP_FE_ENABLE_COMPRESS(i)))
		throw std::runtime_error("FrontEnd::finalise: output compressed but compression not enabled");
	if (!PISP_IMAGE_FORMAT_compressed(fmt) && (enables & PISP_FE_ENABLE_COMPRESS(i)))
		throw std::runtime_error("FrontEnd::finalise: output uncompressed but compression enabled");
	if ((enables & PISP_FE_ENABLE_COMPRESS(i)) && !PISP_IMAGE_FORMAT_bps_8(fmt))
		throw std::runtime_error("FrontEnd::finalise: compressed output is not 8 bit");
}

void FrontEnd::finalise()
{
	uint32_t dirty_flags = fe_config_.dirty_flags & fe_config_.global.enables; // only finalise blocks that are dirty *and* enabled

	uint16_t width = fe_config_.input.format.width, height = fe_config_.input.format.height;
	if (fe_config_.global.enables & PISP_FE_ENABLE_STATS_CROP) {
		width = fe_config_.stats_crop.width;
		height = fe_config_.stats_crop.height;
	}
	if (fe_config_.global.enables & PISP_FE_ENABLE_DECIMATE) {
		width = ((width + 2) & ~3) >> 1;
		height = 2 * (height >> 2) + ((height & 3) ? 1 : 0);
	}
	if (dirty_flags & PISP_FE_ENABLE_LSC)
		finalise_lsc(fe_config_.lsc, width, height);
	if (dirty_flags & PISP_FE_ENABLE_AGC_STATS)
		finalise_agc(fe_config_.agc_stats, width, height);
	if (dirty_flags & PISP_FE_ENABLE_AWB_STATS)
		finalise_awb(fe_config_.awb_stats, width, height);
	if (dirty_flags & PISP_FE_ENABLE_CDAF_STATS)
		finalise_cdaf(fe_config_.cdaf_stats, width, height);

	width = fe_config_.input.format.width, height = fe_config_.input.format.height;
	for (int i = 0; i < PISP_FRONT_END_NUM_OUTPUTS; i++) {
		if (dirty_flags & PISP_FE_ENABLE_DOWNSCALE(i)) {
			int cwidth = width, cheight = height;
			if (fe_config_.global.enables & PISP_FE_ENABLE_CROP(i))
				cwidth = fe_config_.ch[i].crop.width, cheight = fe_config_.ch[i].crop.height;
			finalise_downscale(fe_config_.ch[i].downscale, cwidth, cheight);
		}
		if (dirty_flags & (PISP_FE_ENABLE_OUTPUT(i) | PISP_FE_ENABLE_COMPRESS(i)))
			finalise_compression(fe_config_, i);
		if (dirty_flags & PISP_FE_ENABLE_OUTPUT(i)) {
			PISP_IMAGE_FORMAT_CONFIG_T &config = fe_config_.ch[i].output.format;
			getOutputSize(i, &config.width, &config.height);
			PISP_IMAGE_FORMAT_compute_stride_align(&config, align_);
		}
	}
}

void FrontEnd::SetGlobal(PISP_FE_GLOBAL_CONFIG_T const &global)
{
	checkFrontEndThreadContext(true);
	fe_config_.dirty_flags |= (global.enables & ~fe_config_.global.enables); // label anything that has become enabled as dirty
	fe_config_.global = global;
	fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_GLOBAL;
}

void FrontEnd::GetGlobal(PISP_FE_GLOBAL_CONFIG_T &global) const
{
	global = fe_config_.global;
}

void FrontEnd::SetInput(PISP_FE_INPUT_CONFIG_T const &input)
{
	checkFrontEndThreadContext(true);
	fe_config_.input = input;
	memset(fe_config_.input.pad, 0, sizeof(fe_config_.input.pad)); // prevents some verification errors with unset registers
	fe_config_.dirty_flags |= PISP_FE_ENABLE_INPUT;
}

void FrontEnd::GetInput(PISP_FE_INPUT_CONFIG_T &input) const
{
	input = fe_config_.input;
}

void FrontEnd::SetInputBuffer(PISP_FE_INPUT_BUFFER_CONFIG_T const &input_buffer)
{
	checkFrontEndThreadContext(true);
	fe_config_.input_buffer = input_buffer;
	fe_config_.input_buffer.pad = 0;
	// Assume this address will always get written.
}

void FrontEnd::SetDecompress(PISP_FE_DECOMPRESS_CONFIG_T const &decompress)
{
	checkFrontEndThreadContext(true);
	fe_config_.decompress = decompress;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DECOMPRESS;
}

void FrontEnd::SetDecompand(PISP_FE_DECOMPAND_CONFIG_T const &decompand)
{
	checkFrontEndThreadContext(true);
	fe_config_.decompand = decompand;
	fe_config_.decompand.pad = 0;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DECOMPAND;
}

void FrontEnd::SetDpc(PISP_FE_DPC_CONFIG_T const &dpc)
{
	checkFrontEndThreadContext(true);
	fe_config_.dpc = dpc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DPC;
}

void FrontEnd::SetBla(PISP_FE_BLA_CONFIG_T const &bla)
{
	checkFrontEndThreadContext(true);
	fe_config_.bla = bla;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_BLA;
}

void FrontEnd::SetStatsCrop(PISP_FE_CROP_CONFIG_T const &stats_crop)
{
	checkFrontEndThreadContext(true);
	fe_config_.stats_crop = stats_crop;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_STATS_CROP;
}

void FrontEnd::SetBlc(PISP_FE_BLA_CONFIG_T const &blc)
{
	checkFrontEndThreadContext(true);
	fe_config_.blc = blc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_BLC;
}

void FrontEnd::SetLsc(PISP_FE_LSC_CONFIG_T const &lsc)
{
	checkFrontEndThreadContext(true);
	fe_config_.lsc = lsc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_LSC;
}

void FrontEnd::SetRGBY(PISP_FE_RGBY_CONFIG_T const &rgby)
{
	checkFrontEndThreadContext(true);
	fe_config_.rgby = rgby;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_RGBY;
}

void FrontEnd::SetAgcStats(PISP_FE_AGC_STATS_CONFIG_T const &agc_stats)
{
	checkFrontEndThreadContext(true);
	fe_config_.agc_stats = agc_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_AGC_STATS;
}

void FrontEnd::GetAgcStats(PISP_FE_AGC_STATS_CONFIG_T &agc_stats)
{
	agc_stats = fe_config_.agc_stats;
}

void FrontEnd::SetAwbStats(PISP_FE_AWB_STATS_CONFIG_T const &awb_stats)
{
	checkFrontEndThreadContext(true);
	fe_config_.awb_stats = awb_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_AWB_STATS;
}

void FrontEnd::GetAwbStats(PISP_FE_AWB_STATS_CONFIG_T &awb_stats) const
{
	awb_stats = fe_config_.awb_stats;
}

void FrontEnd::SetFloatingStats(PISP_FE_FLOATING_STATS_CONFIG_T const &floating_stats)
{
	checkFrontEndThreadContext(true);
	fe_config_.floating_stats = floating_stats;
	fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_FLOATING;
}

void FrontEnd::GetFloatingStats(PISP_FE_FLOATING_STATS_CONFIG_T &floating_stats) const
{
	floating_stats = fe_config_.floating_stats;
}

void FrontEnd::SetCdafStats(PISP_FE_CDAF_STATS_CONFIG_T const &cdaf_stats)
{
	checkFrontEndThreadContext(true);
	fe_config_.cdaf_stats = cdaf_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_CDAF_STATS;
}

void FrontEnd::GetCdafStats(PISP_FE_CDAF_STATS_CONFIG_T &cdaf_stats) const
{
	cdaf_stats = fe_config_.cdaf_stats;
}

void FrontEnd::SetCrop(int output_num, PISP_FE_CROP_CONFIG_T const &crop)
{
	checkFrontEndThreadContext(true);
	fe_config_.ch[output_num].crop = crop;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_CROP(output_num);
}

void FrontEnd::SetDownscale(int output_num, PISP_FE_DOWNSCALE_CONFIG_T const &downscale)
{
	checkFrontEndThreadContext(true);
	fe_config_.ch[output_num].downscale = downscale;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DOWNSCALE(output_num);
}

void FrontEnd::SetCompress(int output_num, PISP_FE_COMPRESS_CONFIG_T const &compress)
{
	checkFrontEndThreadContext(true);
	fe_config_.ch[output_num].compress = compress;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_COMPRESS(output_num);
}

void FrontEnd::SetOutputFormat(int output_num, PISP_IMAGE_FORMAT_CONFIG_T const &output_format)
{
	checkFrontEndThreadContext(true);
	fe_config_.ch[output_num].output.format = output_format;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_OUTPUT(output_num);
}

void FrontEnd::SetOutputIntrLines(int output_num, int ilines)
{
	checkFrontEndThreadContext(true);
	fe_config_.ch[output_num].output.ilines = ilines;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_OUTPUT(output_num);
}

void FrontEnd::SetOutputBuffer(int output_num, PISP_FE_OUTPUT_BUFFER_CONFIG_T const &output_buffer)
{
	checkFrontEndThreadContext(true);
	fe_config_.output_buffer[output_num] = output_buffer;
	// Assume these always get written.
}

void FrontEnd::SetOutputAXI(PISP_FE_OUTPUT_AXI_CONFIG_T const &output_axi)
{
	checkFrontEndThreadContext(true);
	fe_config_.output_axi = output_axi;
	fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_OUTPUT_AXI;
}

void FrontEnd::WriteRegisters(HAL_REG_VALUE_T const *reg_values, int num_reg_values)
{
	hal_registers_to_config(&fe_config_, 0, hal_front_end_config_reg_table, reg_values, num_reg_values);
}

void FrontEnd::checkFrontEndThreadContext(bool from_frontend_thread) const
{
	bool is_same = (thread_id_ == std::this_thread::get_id());
	if (from_frontend_thread ^ is_same)
		throw std::logic_error("FrontEnd API called from an invalid thread context.");
}

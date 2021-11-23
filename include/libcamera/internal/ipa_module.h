/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module.h - Image Processing Algorithm module
 */

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/pipeline_handler.h"

namespace libcamera {

class IPAModule : public Loggable
{
public:
	explicit IPAModule(const std::string &libPath);
	~IPAModule();

	bool isValid() const;

	const struct IPAModuleInfo &info() const;
	const std::vector<uint8_t> signature() const;
	const std::string &path() const;

	bool load();

	IPAInterface *createInterface();

	bool match(PipelineHandler *pipe,
		   uint32_t minVersion, uint32_t maxVersion) const;

protected:
	std::string logPrefix() const override;

private:
	int loadIPAModuleInfo();

	struct IPAModuleInfo info_;
	std::vector<uint8_t> signature_;

	std::string libPath_;
	bool valid_;
	bool loaded_;

	void *dlHandle_;
	typedef IPAInterface *(*IPAIntfFactory)(void);
	IPAIntfFactory ipaCreate_;
};

} /* namespace libcamera */

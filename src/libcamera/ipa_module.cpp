/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Image Processing Algorithm module
 */

#include "libcamera/internal/ipa_module.h"

#include <algorithm>
#include <ctype.h>
#include <dlfcn.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <link.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/pipeline_handler.h"

/**
 * \file ipa_module.h
 * \brief Image Processing Algorithm module
 */

/**
 * \file ipa_module_info.h
 * \brief Image Processing Algorithm module information
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAModule)

namespace {

template<typename T>
typename std::remove_extent_t<T> *elfPointer(Span<const uint8_t> elf,
					     off_t offset, size_t objSize)
{
	size_t size = offset + objSize;
	if (size > elf.size() || size < objSize)
		return nullptr;

	return reinterpret_cast<typename std::remove_extent_t<T> *>(
		reinterpret_cast<const char *>(elf.data()) + offset);
}

template<typename T>
typename std::remove_extent_t<T> *elfPointer(Span<const uint8_t> elf,
					     off_t offset)
{
	return elfPointer<T>(elf, offset, sizeof(T));
}

int elfVerifyIdent(Span<const uint8_t> elf)
{
	const char *e_ident = elfPointer<const char[EI_NIDENT]>(elf, 0);
	if (!e_ident)
		return -ENOEXEC;

	if (e_ident[EI_MAG0] != ELFMAG0 ||
	    e_ident[EI_MAG1] != ELFMAG1 ||
	    e_ident[EI_MAG2] != ELFMAG2 ||
	    e_ident[EI_MAG3] != ELFMAG3 ||
	    e_ident[EI_VERSION] != EV_CURRENT)
		return -ENOEXEC;

	int bitClass = sizeof(unsigned long) == 4 ? ELFCLASS32 : ELFCLASS64;
	if (e_ident[EI_CLASS] != bitClass)
		return -ENOEXEC;

	int a = 1;
	unsigned char endianness = *reinterpret_cast<char *>(&a) == 1
				 ? ELFDATA2LSB : ELFDATA2MSB;
	if (e_ident[EI_DATA] != endianness)
		return -ENOEXEC;

	return 0;
}

const ElfW(Shdr) *elfSection(Span<const uint8_t> elf, const ElfW(Ehdr) *eHdr,
			     ElfW(Half) idx)
{
	if (idx >= eHdr->e_shnum)
		return nullptr;

	off_t offset = eHdr->e_shoff + idx *
				       static_cast<uint32_t>(eHdr->e_shentsize);
	return elfPointer<const ElfW(Shdr)>(elf, offset);
}

/**
 * \brief Retrieve address and size of a symbol from an mmap'ed ELF file
 * \param[in] elf Address and size of mmap'ed ELF file
 * \param[in] symbol Symbol name
 *
 * \return The memory region storing the symbol on success, or an empty span
 * otherwise
 */
Span<const uint8_t> elfLoadSymbol(Span<const uint8_t> elf, const char *symbol)
{
	const ElfW(Ehdr) *eHdr = elfPointer<const ElfW(Ehdr)>(elf, 0);
	if (!eHdr)
		return {};

	const ElfW(Shdr) *sHdr = elfSection(elf, eHdr, eHdr->e_shstrndx);
	if (!sHdr)
		return {};
	off_t shnameoff = sHdr->sh_offset;

	/* Locate .dynsym section header. */
	const ElfW(Shdr) *dynsym = nullptr;
	for (unsigned int i = 0; i < eHdr->e_shnum; i++) {
		sHdr = elfSection(elf, eHdr, i);
		if (!sHdr)
			return {};

		off_t offset = shnameoff + sHdr->sh_name;
		const char *name = elfPointer<const char[8]>(elf, offset);
		if (!name)
			return {};

		if (sHdr->sh_type == SHT_DYNSYM && !strcmp(name, ".dynsym")) {
			dynsym = sHdr;
			break;
		}
	}

	if (dynsym == nullptr) {
		LOG(IPAModule, Error) << "ELF has no .dynsym section";
		return {};
	}

	sHdr = elfSection(elf, eHdr, dynsym->sh_link);
	if (!sHdr)
		return {};
	off_t dynsym_nameoff = sHdr->sh_offset;

	/* Locate symbol in the .dynsym section. */
	const ElfW(Sym) *targetSymbol = nullptr;
	unsigned int dynsym_num = dynsym->sh_size / dynsym->sh_entsize;
	for (unsigned int i = 0; i < dynsym_num; i++) {
		off_t offset = dynsym->sh_offset + dynsym->sh_entsize * i;
		const ElfW(Sym) *sym = elfPointer<const ElfW(Sym)>(elf, offset);
		if (!sym)
			return {};

		offset = dynsym_nameoff + sym->st_name;
		const char *name = elfPointer<const char>(elf, offset,
							  strlen(symbol) + 1);
		if (!name)
			return {};

		if (!strcmp(name, symbol) &&
		    sym->st_info & STB_GLOBAL) {
			targetSymbol = sym;
			break;
		}
	}

	if (targetSymbol == nullptr) {
		LOG(IPAModule, Error) << "Symbol " << symbol << " not found";
		return {};
	}

	/* Locate and return data of symbol. */
	sHdr = elfSection(elf, eHdr, targetSymbol->st_shndx);
	if (!sHdr)
		return {};
	off_t offset = sHdr->sh_offset + (targetSymbol->st_value - sHdr->sh_addr);
	const uint8_t *data = elfPointer<const uint8_t>(elf, offset,
							targetSymbol->st_size);
	if (!data)
		return {};

	return { data, targetSymbol->st_size };
}

} /* namespace */

/**
 * \def IPA_MODULE_API_VERSION
 * \brief The IPA module API version
 *
 * This version number specifies the version for the layout of
 * struct IPAModuleInfo. The IPA module shall use this macro to
 * set its moduleAPIVersion field.
 *
 * \sa IPAModuleInfo::moduleAPIVersion
 */

/**
 * \struct IPAModuleInfo
 * \brief Information of an IPA module
 *
 * This structure contains the information of an IPA module. It is loaded,
 * read, and validated before anything else is loaded from the shared object.
 *
 * \var IPAModuleInfo::moduleAPIVersion
 * \brief The IPA module API version that the IPA module implements
 *
 * This version number specifies the version for the layout of
 * struct IPAModuleInfo. The IPA module shall report here the version that
 * it was built for, using the macro IPA_MODULE_API_VERSION.
 *
 * \var IPAModuleInfo::pipelineVersion
 * \brief The pipeline handler version that the IPA module is for
 *
 * \var IPAModuleInfo::pipelineName
 * \brief The name of the pipeline handler that the IPA module is for
 *
 * This name is used to match a pipeline handler with the module.
 *
 * \var IPAModuleInfo::name
 * \brief The name of the IPA module
 *
 * The name may be used to build file system paths to IPA-specific resources.
 * It shall only contain printable characters, and may not contain '*', '?' or
 * '\'. For IPA modules included in libcamera, it shall match the directory of
 * the IPA module in the source tree.
 *
 * \todo Allow user to choose to isolate open source IPAs
 */

/**
 * \var ipaModuleInfo
 * \brief Information of an IPA module
 *
 * An IPA module must export a struct IPAModuleInfo of this name.
 */

/**
 * \class IPAModule
 * \brief Wrapper around IPA module shared object
 */

/**
 * \brief Construct an IPAModule instance
 * \param[in] libPath path to IPA module shared object
 *
 * Loads the IPAModuleInfo from the IPA module shared object at libPath.
 * The IPA module shared object file must be of the same endianness and
 * bitness as libcamera.
 *
 * The caller shall call the isValid() function after constructing an
 * IPAModule instance to verify the validity of the IPAModule.
 */
IPAModule::IPAModule(const std::string &libPath)
	: libPath_(libPath), valid_(false), loaded_(false),
	  dlHandle_(nullptr), ipaCreate_(nullptr)
{
	if (loadIPAModuleInfo() < 0)
		return;

	valid_ = true;
}

IPAModule::~IPAModule()
{
	if (dlHandle_)
		dlclose(dlHandle_);
}

int IPAModule::loadIPAModuleInfo()
{
	File file{ libPath_ };
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPAModule, Error) << "Failed to open IPA library: "
				      << strerror(-file.error());
		return file.error();
	}

	Span<const uint8_t> data = file.map();
	int ret = elfVerifyIdent(data);
	if (ret) {
		LOG(IPAModule, Error) << "IPA module is not an ELF file";
		return ret;
	}

	Span<const uint8_t> info = elfLoadSymbol(data, "ipaModuleInfo");
	if (info.size() < sizeof(info_)) {
		LOG(IPAModule, Error) << "IPA module has no valid info";
		return -EINVAL;
	}

	memcpy(&info_, info.data(), sizeof(info_));

	if (info_.moduleAPIVersion != IPA_MODULE_API_VERSION) {
		LOG(IPAModule, Error) << "IPA module API version mismatch";
		return -EINVAL;
	}

	/*
	 * Validate the IPA module name.
	 *
	 * \todo Consider module naming restrictions to avoid escaping from a
	 * base directory. Forbidding ".." may be enough, but this may be best
	 * implemented in a different layer.
	 */
	std::string ipaName = info_.name;
	auto iter = std::find_if_not(ipaName.begin(), ipaName.end(),
				     [](unsigned char c) -> bool {
					     return isprint(c) && c != '?' &&
						    c != '*' && c != '\\';
				     });
	if (iter != ipaName.end()) {
		LOG(IPAModule, Error)
			<< "Invalid IPA module name '" << ipaName << "'";
		return -EINVAL;
	}

	/* Load the signature. Failures are not fatal. */
	File sign{ libPath_ + ".sign" };
	if (!sign.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPAModule, Debug)
			<< "IPA module " << libPath_ << " is not signed";
		return 0;
	}

	data = sign.map(0, -1, File::MapFlag::Private);
	signature_.resize(data.size());
	memcpy(signature_.data(), data.data(), data.size());

	LOG(IPAModule, Debug) << "IPA module " << libPath_ << " is signed";

	return 0;
}

/**
 * \brief Check if the IPAModule instance is valid
 *
 * An IPAModule instance is valid if the IPA module shared object exists and
 * the IPA module information it contains was successfully retrieved and
 * validated.
 *
 * \return True if the IPAModule is valid, false otherwise
 */
bool IPAModule::isValid() const
{
	return valid_;
}

/**
 * \brief Retrieve the IPA module information
 *
 * The content of the IPA module information is loaded from the module,
 * and is valid only if the module is valid (as returned by isValid()).
 * Calling this function on an invalid module is an error.
 *
 * \return the IPA module information
 */
const struct IPAModuleInfo &IPAModule::info() const
{
	return info_;
}

/**
 * \brief Retrieve the IPA module signature
 *
 * The IPA module signature is stored alongside the IPA module in a file with a
 * '.sign' suffix, and is loaded when the IPAModule instance is created. This
 * function returns the signature without verifying it. If the signature is
 * missing, the returned vector will be empty.
 *
 * \return The IPA module signature
 */
const std::vector<uint8_t> &IPAModule::signature() const
{
	return signature_;
}

/**
 * \brief Retrieve the IPA module path
 *
 * The IPA module path is the file name and path of the IPA module shared
 * object from which the IPA module was created.
 *
 * \return The IPA module path
 */
const std::string &IPAModule::path() const
{
	return libPath_;
}

/**
 * \brief Load the IPA implementation factory from the shared object
 *
 * The IPA module shared object implements an IPAInterface object to be used
 * by pipeline handlers. This function loads the factory function from the
 * shared object. Later, createInterface() can be called to instantiate the
 * IPAInterface.
 *
 * This function only needs to be called successfully once, after which
 * createInterface() can be called as many times as IPAInterface instances are
 * needed.
 *
 * Calling this function on an invalid module (as returned by isValid()) is
 * an error.
 *
 * \return True if load was successful, or already loaded, and false otherwise
 */
bool IPAModule::load()
{
	if (!valid_)
		return false;

	if (loaded_)
		return true;

	dlHandle_ = dlopen(libPath_.c_str(), RTLD_LAZY);
	if (!dlHandle_) {
		LOG(IPAModule, Error)
			<< "Failed to open IPA module shared object: "
			<< dlerror();
		return false;
	}

	void *symbol = dlsym(dlHandle_, "ipaCreate");
	if (!symbol) {
		LOG(IPAModule, Error)
			<< "Failed to load ipaCreate() from IPA module shared object: "
			<< dlerror();
		dlclose(dlHandle_);
		dlHandle_ = nullptr;
		return false;
	}

	ipaCreate_ = reinterpret_cast<IPAIntfFactory>(symbol);

	loaded_ = true;

	return true;
}

/**
 * \brief Instantiate an IPA interface
 *
 * After loading the IPA module with load(), this function creates an instance
 * of the IPA module interface.
 *
 * Calling this function on a module that has not yet been loaded, or an
 * invalid module (as returned by load() and isValid(), respectively) is
 * an error.
 *
 * \return The IPA interface on success, or nullptr on error
 */
IPAInterface *IPAModule::createInterface()
{
	if (!valid_ || !loaded_)
		return nullptr;

	return ipaCreate_();
}

/**
 * \brief Verify if the IPA module matches a given pipeline handler
 * \param[in] pipe Pipeline handler to match with
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 *
 * This function checks if this IPA module matches the \a pipe pipeline handler,
 * and the input version range.
 *
 * \return True if the pipeline handler matches the IPA module, or false otherwise
 */
bool IPAModule::match(PipelineHandler *pipe,
		      uint32_t minVersion, uint32_t maxVersion) const
{
	return info_.pipelineVersion >= minVersion &&
	       info_.pipelineVersion <= maxVersion &&
	       !strcmp(info_.pipelineName, pipe->name());
}

std::string IPAModule::logPrefix() const
{
	return utils::basename(libPath_.c_str());
}

} /* namespace libcamera */

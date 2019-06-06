/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module.cpp - Image Processing Algorithm module
 */

#include "ipa_module.h"

#include <dlfcn.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include "log.h"
#include "pipeline_handler.h"

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
typename std::remove_extent<T>::type *elfPointer(void *map, off_t offset,
						 size_t fileSize, size_t objSize)
{
	size_t size = offset + objSize;
	if (size > fileSize || size < objSize)
		return nullptr;

	return reinterpret_cast<typename std::remove_extent<T>::type *>
		(static_cast<char *>(map) + offset);
}

template<typename T>
typename std::remove_extent<T>::type *elfPointer(void *map, off_t offset,
						 size_t fileSize)
{
	return elfPointer<T>(map, offset, fileSize, sizeof(T));
}

int elfVerifyIdent(void *map, size_t soSize)
{
	char *e_ident = elfPointer<char[EI_NIDENT]>(map, 0, soSize);
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

/**
 * \brief Retrieve address and size of a symbol from an mmap'ed ELF file
 * \param[in] map Address of mmap'ed ELF file
 * \param[in] soSize Size of mmap'ed ELF file (in bytes)
 * \param[in] symbol Symbol name
 *
 * \return zero or error code, address or nullptr, size of symbol or zero,
 * respectively
 */
template<class ElfHeader, class SecHeader, class SymHeader>
std::tuple<void *, size_t>
elfLoadSymbol(void *map, size_t soSize, const char *symbol)
{
	ElfHeader *eHdr = elfPointer<ElfHeader>(map, 0, soSize);
	if (!eHdr)
		return std::make_tuple(nullptr, 0);

	off_t offset = eHdr->e_shoff + eHdr->e_shentsize * eHdr->e_shstrndx;
	SecHeader *sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return std::make_tuple(nullptr, 0);
	off_t shnameoff = sHdr->sh_offset;

	/* Locate .dynsym section header. */
	SecHeader *dynsym = nullptr;
	for (unsigned int i = 0; i < eHdr->e_shnum; i++) {
		offset = eHdr->e_shoff + eHdr->e_shentsize * i;
		sHdr = elfPointer<SecHeader>(map, offset, soSize);
		if (!sHdr)
			return std::make_tuple(nullptr, 0);

		offset = shnameoff + sHdr->sh_name;
		char *name = elfPointer<char[8]>(map, offset, soSize);
		if (!name)
			return std::make_tuple(nullptr, 0);

		if (sHdr->sh_type == SHT_DYNSYM && !strcmp(name, ".dynsym")) {
			dynsym = sHdr;
			break;
		}
	}

	if (dynsym == nullptr) {
		LOG(IPAModule, Error) << "ELF has no .dynsym section";
		return std::make_tuple(nullptr, 0);
	}

	offset = eHdr->e_shoff + eHdr->e_shentsize * dynsym->sh_link;
	sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return std::make_tuple(nullptr, 0);
	off_t dynsym_nameoff = sHdr->sh_offset;

	/* Locate symbol in the .dynsym section. */
	SymHeader *targetSymbol = nullptr;
	unsigned int dynsym_num = dynsym->sh_size / dynsym->sh_entsize;
	for (unsigned int i = 0; i < dynsym_num; i++) {
		offset = dynsym->sh_offset + dynsym->sh_entsize * i;
		SymHeader *sym = elfPointer<SymHeader>(map, offset, soSize);
		if (!sym)
			return std::make_tuple(nullptr, 0);

		offset = dynsym_nameoff + sym->st_name;
		char *name = elfPointer<char>(map, offset, soSize,
					      strlen(symbol) + 1);
		if (!name)
			return std::make_tuple(nullptr, 0);

		if (!strcmp(name, symbol) &&
		    sym->st_info & STB_GLOBAL) {
			targetSymbol = sym;
			break;
		}
	}

	if (targetSymbol == nullptr) {
		LOG(IPAModule, Error) << "Symbol " << symbol << " not found";
		return std::make_tuple(nullptr, 0);
	}

	/* Locate and return data of symbol. */
	if (targetSymbol->st_shndx >= eHdr->e_shnum)
		return std::make_tuple(nullptr, 0);
	offset = eHdr->e_shoff + targetSymbol->st_shndx * eHdr->e_shentsize;
	sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return std::make_tuple(nullptr, 0);
	offset = sHdr->sh_offset + (targetSymbol->st_value - sHdr->sh_addr);
	char *data = elfPointer<char>(map, offset, soSize, targetSymbol->st_size);
	if (!data)
		return std::make_tuple(nullptr, 0);

	return std::make_tuple(data, targetSymbol->st_size);
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
 * The caller shall call the isValid() method after constructing an
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
	int fd = open(libPath_.c_str(), O_RDONLY);
	if (fd < 0) {
		int ret = -errno;
		LOG(IPAModule, Error) << "Failed to open IPA library: "
				      << strerror(-ret);
		return ret;
	}

	void *data;
	size_t dataSize;
	void *map;
	size_t soSize;
	struct stat st;
	int ret = fstat(fd, &st);
	if (ret < 0)
		goto close;
	soSize = st.st_size;
	map = mmap(NULL, soSize, PROT_READ, MAP_PRIVATE, fd, 0);
	if (map == MAP_FAILED) {
		ret = -errno;
		goto close;
	}

	ret = elfVerifyIdent(map, soSize);
	if (ret)
		goto unmap;

	if (sizeof(unsigned long) == 4)
		std::tie(data, dataSize) =
			elfLoadSymbol<Elf32_Ehdr, Elf32_Shdr, Elf32_Sym>
				     (map, soSize, "ipaModuleInfo");
	else
		std::tie(data, dataSize) =
			elfLoadSymbol<Elf64_Ehdr, Elf64_Shdr, Elf64_Sym>
				     (map, soSize, "ipaModuleInfo");

	if (data && dataSize == sizeof(info_))
		memcpy(&info_, data, dataSize);

	if (!data)
		goto unmap;

	if (info_.moduleAPIVersion != IPA_MODULE_API_VERSION) {
		LOG(IPAModule, Error) << "IPA module API version mismatch";
		ret = -EINVAL;
	}

unmap:
	munmap(map, soSize);
close:
	if (ret || !data)
		LOG(IPAModule, Error)
			<< "Error loading IPA module info for " << libPath_;

	close(fd);
	return ret;
}

/**
 * \brief Check if the IPAModule instance is valid
 *
 * An IPAModule instance is valid if the IPA module shared object exists and
 * the IPA module information it contains was successfully retrieved and
 * validated.
 *
 * \return true if the the IPAModule is valid, false otherwise
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
 * The IPA module shared object implements an IPAInterface class to be used
 * by pipeline handlers. This method loads the factory function from the
 * shared object. Later, createInstance() can be called to instantiate the
 * IPAInterface.
 *
 * This method only needs to be called successfully once, after which
 * createInstance() can be called as many times as IPAInterface instances are
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
 * \brief Instantiate an IPAInterface
 *
 * After loading the IPA module with load(), this method creates an
 * instance of the IPA module interface.
 *
 * Calling this function on a module that has not yet been loaded, or an
 * invalid module (as returned by load() and isValid(), respectively) is
 * an error.
 *
 * \return The IPA implementation as a new IPAInterface instance on success,
 * or nullptr on error
 */
std::unique_ptr<IPAInterface> IPAModule::createInstance()
{
	if (!valid_ || !loaded_)
		return nullptr;

	return std::unique_ptr<IPAInterface>(ipaCreate_());
}

/**
 * \brief Verify if the IPA module maches a given pipeline handler
 * \param[in] pipe Pipeline handler to match with
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 *
 * This method checks if this IPA module matches the \a pipe pipeline handler,
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

} /* namespace libcamera */

/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module.cpp - Image Processing Algorithm module
 */

#include "ipa_module.h"

#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"

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

template<class ElfHeader, class SecHeader, class SymHeader>
int elfLoadSymbol(void *dst, size_t size, void *map, size_t soSize,
			  const char *symbol)
{
	ElfHeader *eHdr = elfPointer<ElfHeader>(map, 0, soSize);
	if (!eHdr)
		return -ENOEXEC;

	off_t offset = eHdr->e_shoff + eHdr->e_shentsize * eHdr->e_shstrndx;
	SecHeader *sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return -ENOEXEC;
	off_t shnameoff = sHdr->sh_offset;

	/* Locate .dynsym section header. */
	SecHeader *dynsym = nullptr;
	for (unsigned int i = 0; i < eHdr->e_shnum; i++) {
		offset = eHdr->e_shoff + eHdr->e_shentsize * i;
		sHdr = elfPointer<SecHeader>(map, offset, soSize);
		if (!sHdr)
			return -ENOEXEC;

		offset = shnameoff + sHdr->sh_name;
		char *name = elfPointer<char[8]>(map, offset, soSize);
		if (!name)
			return -ENOEXEC;

		if (sHdr->sh_type == SHT_DYNSYM && !strcmp(name, ".dynsym")) {
			dynsym = sHdr;
			break;
		}
	}

	if (dynsym == nullptr) {
		LOG(IPAModule, Error) << "ELF has no .dynsym section";
		return -ENOEXEC;
	}

	offset = eHdr->e_shoff + eHdr->e_shentsize * dynsym->sh_link;
	sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return -ENOEXEC;
	off_t dynsym_nameoff = sHdr->sh_offset;

	/* Locate symbol in the .dynsym section. */
	SymHeader *targetSymbol = nullptr;
	unsigned int dynsym_num = dynsym->sh_size / dynsym->sh_entsize;
	for (unsigned int i = 0; i < dynsym_num; i++) {
		offset = dynsym->sh_offset + dynsym->sh_entsize * i;
		SymHeader *sym = elfPointer<SymHeader>(map, offset, soSize);
		if (!sym)
			return -ENOEXEC;

		offset = dynsym_nameoff + sym->st_name;
		char *name = elfPointer<char>(map, offset, soSize,
					      strlen(symbol) + 1);
		if (!name)
			return -ENOEXEC;

		if (!strcmp(name, symbol) &&
		    sym->st_info & STB_GLOBAL && sym->st_size == size) {
			targetSymbol = sym;
			break;
		}
	}

	if (targetSymbol == nullptr) {
		LOG(IPAModule, Error) << "Symbol " << symbol << " not found";
		return -ENOEXEC;
	}

	/* Locate and return data of symbol. */
	if (targetSymbol->st_shndx >= eHdr->e_shnum)
		return -ENOEXEC;
	offset = eHdr->e_shoff + targetSymbol->st_shndx * eHdr->e_shentsize;
	sHdr = elfPointer<SecHeader>(map, offset, soSize);
	if (!sHdr)
		return -ENOEXEC;
	offset = sHdr->sh_offset + (targetSymbol->st_value - sHdr->sh_addr);
	char *data = elfPointer<char>(map, offset, soSize, size);
	if (!data)
		return -ENOEXEC;

	memcpy(dst, data, size);

	return 0;
}

} /* namespace */

/**
 * \struct IPAModuleInfo
 * \brief Information of an IPA module
 *
 * This structure contains the information of an IPA module. It is loaded,
 * read, and validated before anything else is loaded from the shared object.
 *
 * \var IPAModuleInfo::name
 * \brief The name of the IPA module
 *
 * \var IPAModuleInfo::version
 * \brief The version of the IPA module
 *
 * \todo abi compatability version
 * \todo pipeline compatability matcher
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
 * \todo load funtions from the IPA to be used by pipelines
 *
 * The caller shall call the isValid() method after constructing an
 * IPAModule instance to verify the validity of the IPAModule.
 */
IPAModule::IPAModule(const std::string &libPath)
	: libPath_(libPath), valid_(false)
{
	if (loadIPAModuleInfo() < 0)
		return;

	valid_ = true;
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

	size_t soSize;
	void *map;
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
		ret = elfLoadSymbol<Elf32_Ehdr, Elf32_Shdr, Elf32_Sym>
				   (&info_, sizeof(info_), map, soSize, "ipaModuleInfo");
	else
		ret = elfLoadSymbol<Elf64_Ehdr, Elf64_Shdr, Elf64_Sym>
				   (&info_, sizeof(info_), map, soSize, "ipaModuleInfo");

unmap:
	munmap(map, soSize);
close:
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

} /* namespace libcamera */

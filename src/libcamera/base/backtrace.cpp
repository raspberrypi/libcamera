/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * backtrace.h - Call stack backtraces
 */

#include <libcamera/base/backtrace.h>

#if HAVE_BACKTRACE
#include <execinfo.h>
#include <stdlib.h>
#endif

#ifdef HAVE_DW
#include <elfutils/libdwfl.h>
#include <unistd.h>
#endif

#if HAVE_UNWIND
/*
 * Disable support for remote unwinding to enable a more optimized
 * implementation.
 */
#define UNW_LOCAL_ONLY
#include <libunwind.h>
#endif

#include <cxxabi.h>
#include <sstream>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

/**
 * \file backtrace.h
 * \brief Generate call stack backtraces
 */

namespace libcamera {

namespace {

#if HAVE_DW
class DwflParser
{
public:
	DwflParser();
	~DwflParser();

	bool isValid() const { return valid_; }
	std::string stackEntry(const void *ip);

private:
	Dwfl_Callbacks callbacks_;
	Dwfl *dwfl_;
	bool valid_;
};

DwflParser::DwflParser()
	: callbacks_({}), dwfl_(nullptr), valid_(false)
{
	callbacks_.find_elf = dwfl_linux_proc_find_elf;
	callbacks_.find_debuginfo = dwfl_standard_find_debuginfo;

	dwfl_ = dwfl_begin(&callbacks_);
	if (!dwfl_)
		return;

	int ret = dwfl_linux_proc_report(dwfl_, getpid());
	if (ret)
		return;

	ret = dwfl_report_end(dwfl_, nullptr, nullptr);
	if (ret)
		return;

	valid_ = true;
}

DwflParser::~DwflParser()
{
	if (dwfl_)
		dwfl_end(dwfl_);
}

std::string DwflParser::stackEntry(const void *ip)
{
	Dwarf_Addr addr = reinterpret_cast<Dwarf_Addr>(ip);

	Dwfl_Module *module = dwfl_addrmodule(dwfl_, addr);
	if (!module)
		return std::string();

	std::ostringstream entry;

	GElf_Off offset;
	GElf_Sym sym;
	const char *symbol = dwfl_module_addrinfo(module, addr, &offset, &sym,
						  nullptr, nullptr, nullptr);
	if (symbol) {
		char *name = abi::__cxa_demangle(symbol, nullptr, nullptr, nullptr);
		entry << (name ? name : symbol) << "+0x" << std::hex << offset
		      << std::dec;
		free(name);
	} else {
		entry << "??? [" << utils::hex(addr) << "]";
	}

	entry << " (";

	Dwfl_Line *line = dwfl_module_getsrc(module, addr);
	if (line) {
		const char *filename;
		int lineNumber = 0;

		filename = dwfl_lineinfo(line, &addr, &lineNumber, nullptr,
					 nullptr, nullptr);

		entry << (filename ? filename : "???") << ":" << lineNumber;
	} else {
		const char *filename = nullptr;

		dwfl_module_info(module, nullptr, nullptr, nullptr, nullptr,
				 nullptr, &filename, nullptr);

		entry << (filename ? filename : "???") << " [" << utils::hex(addr) << "]";
	}

	entry << ")";
	return entry.str();
}
#endif /* HAVE_DW */

} /* namespace */

/**
 * \class Backtrace
 * \brief Representation of a call stack backtrace
 *
 * The Backtrace class represents a function call stack. Constructing an
 * instance captures the call stack at the point the instance is constructed.
 * The instance can later be used to access the call stack and to generate a
 * human-readable representation with the toString() function.
 *
 * Depending on the platform, different backends can be used to generate the
 * backtrace. The Backtrace class provides a best effort to capture accurate
 * backtraces, but doesn't offer any guarantee of a particular backtrace format.
 */

/**
 * \brief Construct a backtrace
 *
 * The backtrace captures the call stack at the point where it is constructed.
 * It can later be converted to a string with toString().
 */
Backtrace::Backtrace()
{
	/* Try libunwind first and fall back to backtrace() if it fails. */
	if (unwindTrace())
		return;

	backtraceTrace();
}

/*
 * Avoid inlining to make sure that the Backtrace constructor adds exactly two
 * calls to the stack, which are later skipped in toString().
 */
__attribute__((__noinline__))
bool Backtrace::backtraceTrace()
{
#if HAVE_BACKTRACE
	backtrace_.resize(32);

	int num_entries = backtrace(backtrace_.data(), backtrace_.size());
	if (num_entries < 0) {
		backtrace_.clear();
		return false;
	}

	backtrace_.resize(num_entries);

	return true;
#else
	return false;
#endif
}

__attribute__((__noinline__))
bool Backtrace::unwindTrace()
{
#if HAVE_UNWIND
/*
 * unw_getcontext() for ARM32 is an inline assembly function using the stmia
 * instruction to store SP and PC. This is considered by clang-11 as deprecated,
 * and generates a warning.
 */
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winline-asm"
#endif
	unw_context_t uc;
	int ret = unw_getcontext(&uc);
	if (ret)
		return false;
#ifdef __clang__
#pragma clang diagnostic pop
#endif

	unw_cursor_t cursor;
	ret = unw_init_local(&cursor, &uc);
	if (ret)
		return false;

	do {
#if HAVE_BACKTRACE || HAVE_DW
		/*
		 * If backtrace() or libdw is available, they will be used in
		 * toString() to provide symbol information for the stack
		 * frames using the IP register value.
		 */
		unw_word_t ip;
		ret = unw_get_reg(&cursor, UNW_REG_IP, &ip);
		if (ret) {
			backtrace_.push_back(nullptr);
			continue;
		}

		backtrace_.push_back(reinterpret_cast<void *>(ip));
#else
		/*
		 * Otherwise, use libunwind to get the symbol information. As
		 * the libunwind API uses cursors, we can't store the IP values
		 * and delay symbol lookup to toString().
		 */
		char symbol[256];
		unw_word_t offset = 0;
		ret = unw_get_proc_name(&cursor, symbol, sizeof(symbol), &offset);
		if (ret) {
			backtraceText_.emplace_back("???\n");
			continue;
		}

		std::ostringstream entry;

		char *name = abi::__cxa_demangle(symbol, nullptr, nullptr, nullptr);
		entry << (name ? name : symbol);
		free(name);

		entry << "+0x" << std::hex << offset << "\n";
		backtraceText_.emplace_back(entry.str());
#endif
	} while (unw_step(&cursor) > 0);

	return true;
#else
	return false;
#endif
}

/**
 * \brief Convert a backtrace to a string representation
 * \param[in] skipLevels Number of initial levels to skip in the backtrace
 *
 * The string representation of the backtrace is a multi-line string, with one
 * line per call stack entry. The format of the entries isn't specified and is
 * platform-dependent.
 *
 * The \a skipLevels parameter indicates how many initial entries to skip from
 * the backtrace. This can be used to hide functions that wrap the construction
 * of the Backtrace instance from the call stack. The Backtrace constructor
 * itself is automatically skipped and never shown in the backtrace.
 *
 * If backtrace generation fails for any reason (usually because the platform
 * doesn't support this feature), an empty string is returned.
 *
 * \return A string representation of the backtrace, or an empty string if
 * backtrace generation isn't possible
 */
std::string Backtrace::toString(unsigned int skipLevels) const
{
	/*
	 * Skip the first two entries, corresponding to the Backtrace
	 * construction.
	 */
	skipLevels += 2;

	if (backtrace_.size() <= skipLevels &&
	    backtraceText_.size() <= skipLevels)
		return std::string();

	if (!backtraceText_.empty()) {
		Span<const std::string> trace{ backtraceText_ };
		return utils::join(trace.subspan(skipLevels), "");
	}

#if HAVE_DW
	DwflParser dwfl;

	if (dwfl.isValid()) {
		std::ostringstream msg;

		Span<void *const> trace{ backtrace_ };
		for (const void *ip : trace.subspan(skipLevels)) {
			if (ip)
				msg << dwfl.stackEntry(ip) << std::endl;
			else
				msg << "???" << std::endl;
		}

		return msg.str();
	}
#endif

#if HAVE_BACKTRACE
	Span<void *const> trace{ backtrace_ };
	trace = trace.subspan(skipLevels);

	char **strings = backtrace_symbols(trace.data(), trace.size());
	if (strings) {
		std::ostringstream msg;

		for (unsigned int i = 0; i < trace.size(); ++i)
			msg << strings[i] << std::endl;

		free(strings);
		return msg.str();
	}
#endif

	return std::string();
}

} /* namespace libcamera */

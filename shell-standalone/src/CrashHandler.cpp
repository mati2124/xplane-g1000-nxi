#include "CrashHandler.h"

#if defined(_WIN32)

// clang-format off
#include <windows.h>
#include <dbghelp.h>
// clang-format on

#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

namespace avionics {

// Latest "what was happening" breadcrumb (written by the render loop, read only
// by the crash handler). A fixed buffer so SetCrashBreadcrumb never allocates.
namespace {
char g_breadcrumb[256] = "(none)";
}  // namespace

namespace {

// Resolves "<exe dir>/crash_log.txt".
std::string CrashLogPath() {
  char exe[MAX_PATH] = {0};
  GetModuleFileNameA(nullptr, exe, MAX_PATH);
  std::string path(exe);
  const std::string::size_type slash = path.find_last_of("\\/");
  if (slash != std::string::npos) path.resize(slash + 1);
  else path.clear();
  return path + "crash_log.txt";
}

void WriteFrame(std::FILE* f, HANDLE proc, DWORD64 addr, int index) {
  char modName[MAX_PATH] = "?";
  const DWORD64 modBase = SymGetModuleBase64(proc, addr);
  if (modBase != 0) {
    IMAGEHLP_MODULE64 mi;
    std::memset(&mi, 0, sizeof(mi));
    mi.SizeOfStruct = sizeof(mi);
    if (SymGetModuleInfo64(proc, modBase, &mi)) {
      std::strncpy(modName, mi.ModuleName, MAX_PATH - 1);
    }
  }

  char symBuf[sizeof(SYMBOL_INFO) + 512] = {0};
  auto* sym = reinterpret_cast<SYMBOL_INFO*>(symBuf);
  sym->SizeOfStruct = sizeof(SYMBOL_INFO);
  sym->MaxNameLen = 511;
  DWORD64 disp = 0;
  if (SymFromAddr(proc, addr, &disp, sym)) {
    IMAGEHLP_LINE64 line;
    std::memset(&line, 0, sizeof(line));
    line.SizeOfStruct = sizeof(line);
    DWORD lineDisp = 0;
    if (SymGetLineFromAddr64(proc, addr, &lineDisp, &line)) {
      std::fprintf(f, "  %2d  %s!%s +0x%llx  (%s:%lu)\n", index, modName,
                   sym->Name, static_cast<unsigned long long>(disp),
                   line.FileName, line.LineNumber);
    } else {
      std::fprintf(f, "  %2d  %s!%s +0x%llx\n", index, modName, sym->Name,
                   static_cast<unsigned long long>(disp));
    }
  } else {
    std::fprintf(f, "  %2d  %s!0x%llx\n", index, modName,
                 static_cast<unsigned long long>(addr));
  }
}

LONG WINAPI HandleException(EXCEPTION_POINTERS* info) {
  // Guard against re-entry if symbol resolution itself faults.
  static volatile LONG entered = 0;
  if (InterlockedExchange(&entered, 1) != 0) {
    return EXCEPTION_EXECUTE_HANDLER;
  }

  const HANDLE proc = GetCurrentProcess();
  SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
  SymInitialize(proc, nullptr, TRUE);

  const std::string logPath = CrashLogPath();
  std::FILE* f = std::fopen(logPath.c_str(), "a");
  if (f == nullptr) {
    SymCleanup(proc);
    return EXCEPTION_EXECUTE_HANDLER;
  }

  const std::time_t now = std::time(nullptr);
  std::fprintf(f, "\n==== CRASH %s", std::ctime(&now));
  std::fprintf(f, "breadcrumb: %s\n", g_breadcrumb);
  const EXCEPTION_RECORD* er = info->ExceptionRecord;
  std::fprintf(f, "exception 0x%08lX at %p",
               static_cast<unsigned long>(er->ExceptionCode),
               er->ExceptionAddress);
  if (er->ExceptionCode == EXCEPTION_ACCESS_VIOLATION &&
      er->NumberParameters >= 2) {
    std::fprintf(f, "  (%s 0x%llx)",
                 er->ExceptionInformation[0] != 0 ? "write" : "read",
                 static_cast<unsigned long long>(er->ExceptionInformation[1]));
  }
  std::fprintf(f, "\n");

  CONTEXT ctx = *info->ContextRecord;
  STACKFRAME64 frame;
  std::memset(&frame, 0, sizeof(frame));
#if defined(_M_X64)
  const DWORD machine = IMAGE_FILE_MACHINE_AMD64;
  frame.AddrPC.Offset = ctx.Rip;
  frame.AddrFrame.Offset = ctx.Rbp;
  frame.AddrStack.Offset = ctx.Rsp;
#else
  const DWORD machine = IMAGE_FILE_MACHINE_I386;
  frame.AddrPC.Offset = ctx.Eip;
  frame.AddrFrame.Offset = ctx.Ebp;
  frame.AddrStack.Offset = ctx.Esp;
#endif
  frame.AddrPC.Mode = AddrModeFlat;
  frame.AddrFrame.Mode = AddrModeFlat;
  frame.AddrStack.Mode = AddrModeFlat;

  for (int i = 0; i < 64; ++i) {
    if (!StackWalk64(machine, proc, GetCurrentThread(), &frame, &ctx, nullptr,
                     SymFunctionTableAccess64, SymGetModuleBase64, nullptr)) {
      break;
    }
    if (frame.AddrPC.Offset == 0) break;
    WriteFrame(f, proc, frame.AddrPC.Offset, i);
  }

  std::fprintf(f, "==== end crash ====\n");
  std::fflush(f);
  std::fclose(f);
  SymCleanup(proc);
  // Fall through to the default handler (WER) so the process still terminates.
  return EXCEPTION_EXECUTE_HANDLER;
}

}  // namespace

void InstallCrashHandler() { SetUnhandledExceptionFilter(HandleException); }

void SetCrashBreadcrumb(const char* text) {
  if (text == nullptr) return;
  std::strncpy(g_breadcrumb, text, sizeof(g_breadcrumb) - 1);
  g_breadcrumb[sizeof(g_breadcrumb) - 1] = '\0';
}

}  // namespace avionics

#else  // !_WIN32

namespace avionics {
void InstallCrashHandler() {}
void SetCrashBreadcrumb(const char*) {}
}  // namespace avionics

#endif

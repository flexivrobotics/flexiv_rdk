#!/usr/bin/env bash
# ==================================================================================================
#  Release guard: a shipped RDK library must depend on nothing but the OS C/C++ runtime.
#
#  RDK is distributed as a self-contained shared library that statically embeds every thirdparty
#  dependency except Eigen, so the only runtime dependencies left should be the compiler runtime and
#  the loader. Anything else is a leak from the build machine, and it does not fail at build time on
#  ours: it links, installs, passes every static check, then aborts at process startup on the user's
#  machine. That is how v1.9.4's macOS artifact shipped with
#  /opt/homebrew/opt/openssl@3/lib/libssl.3.dylib recorded in its load commands.
#
#  Checks, per artifact:
#    ELF    - DT_NEEDED limited to the C/C++ runtime and the loader; no DT_RPATH/DT_RUNPATH other
#             than $ORIGIN-relative ones.
#    Mach-O - every dependent library and LC_RPATH entry under @rpath/@loader_path/@executable_path,
#             /usr/lib/ or /System/Library/.
#    PE     - imported DLLs limited to the Windows system DLLs and the MSVC C/C++ runtime, and
#             API set contracts, which the loader resolves internally and which are never files.
#             Reading an import table needs one of dumpbin, llvm-readobj, llvm-objdump or objdump;
#             dumpbin is located through vswhere when Visual Studio has not put it on PATH.
#             Skipped, not failed, when none of them is available.
#
#  Usage: check_artifacts.sh <library> [<library> ...]
#
#  Platforms whose runtime is not glibc (QNX) can extend the allowed dependency set, for ELF and
#  for PE alike:
#    RDK_EXTRA_ALLOWED_LIBS="libc.so.4 libcpp.so.5" check_artifacts.sh <library>
# ==================================================================================================
set -eu

# Shared objects that belong to the OS toolchain runtime and are therefore present on every target
# machine. Nothing else may appear in DT_NEEDED.
ALLOWED_ELF_LIBS="
libc.so.6
libm.so.6
libdl.so.2
librt.so.1
libpthread.so.0
libstdc++.so.6
libgcc_s.so.1
libatomic.so.1
ld-linux-x86-64.so.2
ld-linux-aarch64.so.1
${RDK_EXTRA_ALLOWED_LIBS:-}
"

# Path prefixes a Mach-O load command may use: the SDK's own @-relative paths, plus the two system
# locations guaranteed to exist on every macOS install.
ALLOWED_MACHO_PREFIXES="@rpath/ @loader_path/ @executable_path/ /usr/lib/ /System/Library/"

# DLLs that ship with Windows itself, plus the MSVC C/C++ runtime, which is the Windows counterpart
# of libstdc++/libgcc: redistributable, and already present wherever a consumer builds against RDK
# with MSVC. Matched case-insensitively, since an import table preserves whatever case the linker
# recorded. Nothing else may be imported.
ALLOWED_PE_LIBS="
kernel32.dll
kernelbase.dll
ntdll.dll
user32.dll
advapi32.dll
shell32.dll
shlwapi.dll
ole32.dll
oleaut32.dll
rpcrt4.dll
version.dll
psapi.dll
dbghelp.dll
winmm.dll
powrprof.dll
userenv.dll
iphlpapi.dll
ws2_32.dll
mswsock.dll
crypt32.dll
secur32.dll
bcrypt.dll
ncrypt.dll
msvcrt.dll
ucrtbase.dll
vcruntime140.dll
vcruntime140_1.dll
msvcp140.dll
msvcp140_1.dll
msvcp140_2.dll
concrt140.dll
${RDK_EXTRA_ALLOWED_LIBS:-}
"

# Import table reader, resolved once on the first PE artifact.
PE_READER=""
PE_READER_KIND=""

failures=0
inspected=0

fail() {
    echo "    FAIL: $*"
    failures=$((failures + 1))
}

# Identify the object file format from its magic number, so one script covers every artifact we
# publish without being told which platform produced it.
file_format() {
    local magic
    magic=$(od -An -N4 -tx1 "$1" | tr -d ' \n')
    case "$magic" in
        7f454c46)                            echo elf ;;
        cffaedfe|cefaedfe|feedfacf|feedface) echo macho ;;   # thin Mach-O, either endianness
        cafebabe|bebafeca)                   echo macho ;;   # universal ("fat") binary
        4d5a*)                               echo pe ;;
        *)                                   echo unknown ;;
    esac
}

check_elf() {
    local lib=$1 reader needed entry search_path

    if command -v readelf > /dev/null 2>&1; then
        reader="readelf -d"
    elif command -v objdump > /dev/null 2>&1; then
        reader="objdump -p"
    else
        echo "    ERROR: neither readelf nor objdump found, cannot inspect an ELF artifact"
        exit 2
    fi

    needed=$($reader "$lib" | awk '/\(?NEEDED\)?/ {gsub(/[][]/, "", $NF); print $NF}')
    for entry in $needed; do
        echo "    NEEDED  $entry"
        case " $(echo $ALLOWED_ELF_LIBS) " in
            *" $entry "*) ;;
            *) fail "$entry is not part of the OS C/C++ runtime" ;;
        esac
    done

    # A run path baked into the artifact points at a directory on the build machine unless it is
    # relative to the artifact itself.
    for search_path in $($reader "$lib" \
        | awk '/\(?R(UN)?PATH\)?/ {gsub(/[][]/, "", $NF); gsub(/:/, " ", $NF); print $NF}'); do
        echo "    RPATH   $search_path"
        case "$search_path" in
            '$ORIGIN'*) ;;
            *) fail "run path $search_path is absolute, it will not exist on the user's machine" ;;
        esac
    done
}

check_macho() {
    local lib=$1 entry prefix allowed

    if ! command -v otool > /dev/null 2>&1; then
        echo "    ERROR: otool not found, cannot inspect a Mach-O artifact"
        exit 2
    fi

    # Dependent libraries are the tab-indented lines of otool -L; skipping the un-indented ones also
    # skips the per-architecture headers of a universal binary.
    # LC_RPATH entries are printed as a "path <value> (offset N)" line after each LC_RPATH command.
    for entry in \
        $(otool -L "$lib" | grep -E '^[[:space:]]' | awk '{print $1}') \
        $(otool -l "$lib" | awk '/LC_RPATH/ {f = 1} f && /^ *path /{print $2; f = 0}'); do
        echo "    LOADS   $entry"
        allowed=0
        for prefix in $ALLOWED_MACHO_PREFIXES; do
            case "$entry" in "$prefix"*) allowed=1; break ;; esac
        done
        if [ $allowed -eq 0 ]; then
            fail "$entry is outside the SDK and outside the OS, it will not exist on the user's machine"
        fi
    done
}

# Visual Studio ships dumpbin but does not put it on PATH; vswhere does sit at a fixed location on
# every install, so the toolchain can be found without the caller having entered a VS developer
# shell. The LLVM and binutils readers are accepted as fallbacks: any of them can list an import
# table, and a runner that has one but not the other should still check rather than skip.
find_pe_reader() {
    local vswhere vs_path candidate tool

    if [ -n "$PE_READER" ]; then
        return 0
    fi

    if command -v dumpbin > /dev/null 2>&1; then
        PE_READER=dumpbin
        PE_READER_KIND=dumpbin
        return 0
    fi

    vswhere="/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
    if [ -x "$vswhere" ]; then
        vs_path=$("$vswhere" -latest -products '*' -property installationPath 2> /dev/null | tr -d '\r')
        if [ -n "$vs_path" ]; then
            if command -v cygpath > /dev/null 2>&1; then
                vs_path=$(cygpath -u "$vs_path")
            fi
            for candidate in "$vs_path"/VC/Tools/MSVC/*/bin/Host*/*/dumpbin.exe; do
                if [ -x "$candidate" ]; then
                    PE_READER=$candidate
                    PE_READER_KIND=dumpbin
                    return 0
                fi
            done
        fi
    fi

    for tool in llvm-readobj; do
        if command -v "$tool" > /dev/null 2>&1; then
            PE_READER=$tool
            PE_READER_KIND=readobj
            return 0
        fi
    done

    for tool in llvm-objdump objdump; do
        if command -v "$tool" > /dev/null 2>&1; then
            PE_READER=$tool
            PE_READER_KIND=objdump
            return 0
        fi
    done

    return 1
}

# Print one imported DLL name per line. dumpbin lists both the ordinary and the delay-load
# dependencies, and both are loaded on the user's machine, so both are collected.
pe_imports() {
    local lib=$1 target=$1 raw

    case "$PE_READER_KIND" in
        dumpbin)
            if command -v cygpath > /dev/null 2>&1; then
                target=$(cygpath -w "$lib")
            fi
            raw=$("$PE_READER" /nologo /dependents "$target") || return 1
            echo "$raw" \
                | awk '/following.*dependencies/ {f = 1; next} /^ *Summary/ {f = 0} f && NF == 1 {print $1}'
            ;;
        readobj)
            raw=$("$PE_READER" --coff-imports "$lib") || return 1
            echo "$raw" | awk '/^ *Name: / {print $2}'
            ;;
        objdump)
            raw=$("$PE_READER" -p "$lib") || return 1
            echo "$raw" | awk '/DLL Name: / {print $NF}'
            ;;
    esac
}

# Returns non-zero when the artifact could not be inspected, so the caller does not count it.
check_pe() {
    local lib=$1 imports entry name

    if ! find_pe_reader; then
        echo "    SKIP: no import table reader found (dumpbin, llvm-readobj, llvm-objdump, objdump)"
        return 1
    fi

    if ! imports=$(pe_imports "$lib"); then
        fail "$PE_READER could not read the import table, so this artifact was not verified"
        return 0
    fi

    # Every real PE links against the OS runtime, so an empty import table means the reader
    # understood the file but found nothing -- a silent pass is exactly what must not happen here.
    if [ -z "$imports" ]; then
        fail "no imported DLLs were found, so this artifact was not verified"
        return 0
    fi

    for entry in $imports; do
        echo "    IMPORTS $entry"
        name=$(echo "$entry" | tr '[:upper:]' '[:lower:]')
        case "$name" in
            # API set contracts: resolved inside the loader, never present as files on disk.
            api-ms-win-*.dll | ext-ms-win-*.dll) continue ;;
        esac
        case " $(echo $ALLOWED_PE_LIBS | tr '[:upper:]' '[:lower:]') " in
            *" $name "*) ;;
            *) fail "$entry is not part of the OS C/C++ runtime" ;;
        esac
    done
    return 0
}

if [ $# -eq 0 ]; then
    echo "Usage: $0 <library> [<library> ...]"
    echo "No artifact was given, so nothing was checked. This is a failure: the build step that"
    echo "produces or downloads the library did not leave it where this guard expected it."
    exit 2
fi

for lib in "$@"; do
    if [ ! -f "$lib" ]; then
        echo "$lib"
        echo "    ERROR: no such file"
        exit 2
    fi

    echo "$lib"
    case "$(file_format "$lib")" in
        elf)   check_elf "$lib";   inspected=$((inspected + 1)) ;;
        macho) check_macho "$lib"; inspected=$((inspected + 1)) ;;
        pe)    if check_pe "$lib"; then inspected=$((inspected + 1)); fi ;;
        *)     echo "    ERROR: unrecognized object file format"; exit 2 ;;
    esac
done

if [ $failures -ne 0 ]; then
    echo ""
    echo "$failures dependency violation(s). A released RDK library must load nothing but the OS"
    echo "C/C++ runtime. A dependency listed above came from the build machine and would abort the"
    echo "user's process at startup, so it has to be removed from the build rather than shipped."
    exit 1
fi

echo ""
if [ $inspected -eq 0 ]; then
    echo "Nothing was inspected: every artifact given was skipped."
else
    echo "$inspected artifact(s) depend only on the OS C/C++ runtime."
fi

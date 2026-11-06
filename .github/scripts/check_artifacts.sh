#!/usr/bin/env bash
# ==================================================================================================
#  Release guard: a shipped RDK library must depend on nothing but the OS C/C++ runtime, and must not
#  give away more of its internals than the public API.
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
#             Every exported symbol belongs to the public flexiv::rdk API: none of the pimpl
#             (::Impl) or flexiv::rdk_impl implementation classes. Needs nm; skipped without it.
#             No .comment section (strip removes it, it names the exact toolchain).
#             Full RELRO (GNU_RELRO and BIND_NOW) and a non-executable stack.
#    ELF and Mach-O - no build machine path (a home directory, or the cargo registry and git
#             checkouts) anywhere in the file. These come from Rust panic locations and __FILE__,
#             which strip does not remove; the RDK client's dependency build remaps them, so a hit
#             means the library was built from dependencies built before that, which have to be
#             rebuilt from clean.
#    Mach-O - every dependent library and LC_RPATH entry under @rpath/@loader_path/@executable_path,
#             /usr/lib/ or /System/Library/.
#             Every exported symbol belongs to the public flexiv::rdk API, as for ELF. Needs nm and
#             c++filt; skipped without them.
#    PE     - imported DLLs limited to the Windows system DLLs and the MSVC C/C++ runtime, and
#             API set contracts, which the loader resolves internally and which are never files.
#             Reading an import table needs one of dumpbin, llvm-readobj, llvm-objdump or objdump;
#             dumpbin is located through vswhere when Visual Studio has not put it on PATH.
#             Skipped, not failed, when none of them is available.
#
#  Usage: check_artifacts.sh <library> [<library> ...]
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
"

# Path prefixes a Mach-O load command may use: the SDK's own @-relative paths, plus the two system
# locations guaranteed to exist on every macOS install.
ALLOWED_MACHO_PREFIXES="@rpath/ @loader_path/ @executable_path/ /usr/lib/ /System/Library/"

# DLLs that ship with Windows itself, plus the MSVC C/C++ runtime, which is the Windows counterpart
# of libstdc++/libgcc: redistributable, and already present wherever a consumer builds against RDK
# with MSVC. Matched case-insensitively, since an import table preserves whatever case the linker
# recorded. Nothing else may be imported. bcryptprimitives.dll is where the Rust standard library
# (1.78 and later) gets its random numbers from on Windows (ProcessPrng); it is part of Windows 8
# and later.
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
bcryptprimitives.dll
ncrypt.dll
msvcrt.dll
ucrtbase.dll
vcruntime140.dll
vcruntime140_1.dll
msvcp140.dll
msvcp140_1.dll
msvcp140_2.dll
concrt140.dll
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

    check_elf_exports "$lib"
    check_elf_hardening "$lib"
    check_build_paths "$lib"
}

# Only the public flexiv::rdk API may be exported. The implementation classes are hidden at compile
# time (RDK_HIDDEN on the pimpl declarations, hidden default visibility) and at link time (version
# script); an export of one means one of those was lost.
# Fail on every exported symbol outside the public flexiv::rdk API. Takes the demangled names of the
# exported symbols, one per line.
check_export_names() {
    local exports=$1 leaked

    echo "    EXPORTS $(echo "$exports" | grep -c .) symbols"
    leaked=$(echo "$exports" | grep -E '::Impl\b|flexiv::rdk_impl|flexiv::scheduler' || true)
    leaked="$leaked
$(echo "$exports" | grep -v 'flexiv::rdk::' || true)"
    leaked=$(echo "$leaked" | grep . | sort -u || true)
    if [ -n "$leaked" ]; then
        echo "$leaked" | head -5 | sed 's/^/      /'
        fail "$(echo "$leaked" | wc -l) exported symbol(s) outside the public flexiv::rdk API"
    fi
}

check_elf_exports() {
    local lib=$1

    if ! command -v nm > /dev/null 2>&1; then
        echo "    SKIP: nm not found, exported symbols not checked"
        return
    fi
    check_export_names "$(nm -DC --defined-only "$lib" | cut -d' ' -f3-)"
}

# Mach-O symbol names carry an extra leading underscore, which c++filt on macOS strips before
# demangling.
check_macho_exports() {
    local lib=$1

    if ! command -v nm > /dev/null 2>&1 || ! command -v c++filt > /dev/null 2>&1; then
        echo "    SKIP: nm or c++filt not found, exported symbols not checked"
        return
    fi
    check_export_names "$(nm -gU "$lib" | awk '{print $3}' | c++filt)"
}

check_elf_hardening() {
    local lib=$1

    if ! command -v readelf > /dev/null 2>&1; then
        echo "    SKIP: readelf not found, load-time hardening not checked"
        return
    fi
    if readelf -SW "$lib" | grep -q ' \.comment '; then
        fail ".comment section present, it names the toolchain (strip with --remove-section=.comment)"
    fi
    if ! readelf -lW "$lib" | grep -q 'GNU_RELRO'; then
        fail "no GNU_RELRO segment (link with -z relro)"
    fi
    if ! readelf -dW "$lib" | grep -qE 'BIND_NOW|Flags:.* NOW'; then
        fail "no BIND_NOW, so RELRO is partial (link with -z now)"
    fi
    if readelf -lW "$lib" | grep -E 'GNU_STACK' | grep -qE 'RWE'; then
        fail "executable stack (link with -z noexecstack)"
    fi
}

# Absolute paths of the build machine, found anywhere in the file. grep -a reads the binary as text,
# so this needs no extra tool.
check_build_paths() {
    local lib=$1 paths

    # Rust strings are not NUL-terminated, so match path characters only rather than whole strings.
    # Besides home directories, a cargo home is caught under its usual names: .cargo wherever it
    # lives, and /usr/local/cargo of the official Rust images. The bare /cargo/ that the RDK
    # client's dependency build remaps it to is fine: it names only the crates and their versions,
    # which the published SBOM lists anyway.
    paths=$(LC_ALL=C grep -a -o -E '(/home/[[:alnum:]._-]+|/Users/[[:alnum:]._-]+|/root)/[[:alnum:]._/+-]*|/[[:alnum:]._/+-]*\.cargo/(registry|git)/[[:alnum:]._/+-]*|/usr/local/cargo/(registry|git)/[[:alnum:]._/+-]*' "$lib" \
        | sort -u || true)
    if [ -n "$paths" ]; then
        echo "$paths" | head -3 | cut -c1-100 | sed 's/^/      /'
        fail "$(echo "$paths" | wc -l) build machine path(s) embedded; rebuild the RDK client's dependencies from clean (its thirdparty/clean.sh, then thirdparty/build_and_install_deps.sh), which remaps them"
    fi
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
            # Options with '-', not '/': Git Bash rewrites an argument like /dependents into a path
            # under its install folder before dumpbin sees it
            raw=$("$PE_READER" -nologo -dependents "$target") || return 1
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
        macho) check_macho "$lib"; check_macho_exports "$lib"; check_build_paths "$lib"
               inspected=$((inspected + 1)) ;;
        pe)    if check_pe "$lib"; then inspected=$((inspected + 1)); fi ;;
        *)     echo "    ERROR: unrecognized object file format"; exit 2 ;;
    esac
done

if [ $failures -ne 0 ]; then
    echo ""
    echo "$failures release guard violation(s), see FAIL above. A released RDK library must load"
    echo "nothing but the OS C/C++ runtime (anything else came from the build machine and would abort"
    echo "the user's process at startup), and must export nothing but the public API, carry no build"
    echo "machine path and keep its load-time hardening. Fix the build rather than ship it."
    exit 1
fi

echo ""
if [ $inspected -eq 0 ]; then
    echo "Nothing was inspected: every artifact given was skipped."
else
    echo "$inspected artifact(s) passed the release guard."
fi

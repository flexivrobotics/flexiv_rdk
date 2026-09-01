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
#    PE     - skipped: listing DLL imports needs the MSVC toolchain, which is not assumed here.
#
#  Usage: check_artifacts.sh <library> [<library> ...]
#
#  Platforms whose runtime is not glibc (QNX) can extend the allowed DT_NEEDED set:
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
        pe)    echo "    SKIP: PE artifact, listing DLL imports needs the MSVC toolchain" ;;
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

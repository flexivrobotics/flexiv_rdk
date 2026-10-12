# Verify downloaded RDK release assets against the release's signed checksum list.
#
# Every release publishes SHA256SUMS (sha256sum format) and SHA256SUMS.asc, a detached ASCII-armored
# GPG signature of it made with the Flexiv RDK release signing key (see RELEASING.md). This function
# checks that signature against the public key committed in this repository, accepting only a
# signature by the key with the given fingerprint, then checks that every artifact's SHA-256 is the
# one SHA256SUMS lists for its file name.
#
# flexiv_verify_release_signature(SUMS_FILE <file> SIGNATURE_FILE <file> KEY_FILE <file>
#                                 FINGERPRINT <fingerprint> WORK_DIR <dir> ARTIFACTS <file>...)

# Check that the release signing key and its fingerprint are actually set, and return the
# normalized fingerprint in [out_var].
function(flexiv_check_release_key key_file fingerprint_in out_var)
    # Refuse to run with the placeholders committed until the release signing key is published
    string(TOUPPER "${fingerprint_in}" fingerprint)
    string(REPLACE " " "" fingerprint "${fingerprint}")
    string(LENGTH "${fingerprint}" fingerprint_length)
    if(NOT fingerprint MATCHES "^[0-9A-F]+$" OR NOT fingerprint_length EQUAL 40)
        message(FATAL_ERROR "Release signature verification is enabled, but the release signing "
                            "key fingerprint is not set (got '${fingerprint_in}')")
    endif()
    file(READ "${key_file}" key_content)
    if(NOT key_content MATCHES "-----BEGIN PGP PUBLIC KEY BLOCK-----")
        message(FATAL_ERROR "Release signature verification is enabled, but ${key_file} "
                            "does not contain the release signing public key")
    endif()
    set(${out_var} ${fingerprint} PARENT_SCOPE)
endfunction()

function(flexiv_verify_release_signature)
    cmake_parse_arguments(ARG "" "SUMS_FILE;SIGNATURE_FILE;KEY_FILE;FINGERPRINT;WORK_DIR"
                          "ARTIFACTS" ${ARGN})

    flexiv_check_release_key(${ARG_KEY_FILE} "${ARG_FINGERPRINT}" fingerprint)

    find_program(GPG_EXECUTABLE NAMES gpg gpg2)
    if(NOT GPG_EXECUTABLE)
        message(FATAL_ERROR "Release signature verification is enabled, but gpg was not found")
    endif()

    # Use a throwaway keyring holding only the release key, so no key in the user's own keyring can
    # satisfy the check
    set(gnupg_home "${ARG_WORK_DIR}/gnupg")
    file(REMOVE_RECURSE "${gnupg_home}")
    file(MAKE_DIRECTORY "${gnupg_home}")
    execute_process(
        COMMAND ${GPG_EXECUTABLE} --homedir ${gnupg_home} --batch --no-tty --import ${ARG_KEY_FILE}
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Failed to import ${ARG_KEY_FILE}:\n${output}")
    endif()
    execute_process(
        COMMAND ${GPG_EXECUTABLE} --homedir ${gnupg_home} --batch --no-tty --trust-model always
                --status-fd 1 --verify ${ARG_SIGNATURE_FILE} ${ARG_SUMS_FILE}
        RESULT_VARIABLE result OUTPUT_VARIABLE status ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Bad or missing signature on ${ARG_SUMS_FILE}:\n${output}")
    endif()

    # The last field of VALIDSIG is the fingerprint of the primary key, which also covers a
    # signature made with one of its subkeys
    string(REGEX MATCH "\\[GNUPG:\\] VALIDSIG [^\n]*" validsig "${status}")
    string(REGEX MATCH "[0-9A-F]+$" signer "${validsig}")
    if(NOT signer STREQUAL fingerprint)
        message(FATAL_ERROR "${ARG_SUMS_FILE} is not signed by the release signing key "
                            "${fingerprint} (signer: '${signer}')")
    endif()
    message(STATUS "Verified release signature of ${ARG_SUMS_FILE} by key ${fingerprint}")

    # Every artifact must be listed exactly once, with its actual hash
    file(STRINGS "${ARG_SUMS_FILE}" sums_lines)
    foreach(artifact ${ARG_ARTIFACTS})
        get_filename_component(artifact_name "${artifact}" NAME)
        file(SHA256 "${artifact}" actual_hash)
        set(listed_hashes "")
        foreach(line IN LISTS sums_lines)
            if(line MATCHES "^([0-9a-fA-F]+) [ *]([^\r]+)\r?$" AND CMAKE_MATCH_2 STREQUAL artifact_name)
                string(TOLOWER "${CMAKE_MATCH_1}" listed_hash)
                list(APPEND listed_hashes ${listed_hash})
            endif()
        endforeach()
        list(LENGTH listed_hashes num_listed)
        if(NOT num_listed EQUAL 1)
            message(FATAL_ERROR "${artifact_name} is listed ${num_listed} times in ${ARG_SUMS_FILE}, "
                                "expected exactly once")
        endif()
        if(NOT listed_hashes STREQUAL actual_hash)
            message(FATAL_ERROR "SHA256 of ${artifact_name} (${actual_hash}) does not match "
                                "${ARG_SUMS_FILE} (${listed_hashes})")
        endif()
        message(STATUS "Verified ${artifact_name} against ${ARG_SUMS_FILE}")
    endforeach()
endfunction()

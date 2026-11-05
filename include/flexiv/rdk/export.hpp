/**
 * @file export.hpp
 * @brief Export macros for the public flexiv::rdk API.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */
#ifndef FLEXIV_RDK_EXPORT_HPP_
#define FLEXIV_RDK_EXPORT_HPP_

// RDK_API marks the public flexiv::rdk API for export from the library.
#if defined(_WIN32)
// Windows: export the API from the DLL when building it, import it otherwise.
#if defined(FLEXIV_RDK_BUILDING)
#define RDK_API __declspec(dllexport)
#else
#define RDK_API __declspec(dllimport)
#endif
#else
// Linux/macOS: export the API explicitly.
#define RDK_API __attribute__((visibility("default")))
#endif

// RDK_HIDDEN keeps internal classes out of the library's exports; it has no effect in user code.
#if !defined(_WIN32) && defined(FLEXIV_RDK_HIDE_INTERNALS)
#define RDK_HIDDEN __attribute__((visibility("hidden")))
#else
#define RDK_HIDDEN
#endif

#endif /* FLEXIV_RDK_EXPORT_HPP_ */

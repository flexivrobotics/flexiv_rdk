/**
 * @file export.hpp
 * @brief Export macro for the public flexiv::rdk API.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */
#ifndef FLEXIV_RDK_EXPORT_HPP_
#define FLEXIV_RDK_EXPORT_HPP_

// Marks the public flexiv::rdk API for export from the library.
#if defined(_WIN32)
// Windows: export the API from the DLL when building it, import it otherwise.
#if defined(FLEXIV_RDK_BUILDING)
#define RDK_API __declspec(dllexport)
#else
#define RDK_API __declspec(dllimport)
#endif
#else
// Linux/macOS: visibility is handled at link time, so the macro is empty.
#define RDK_API
#endif

#endif /* FLEXIV_RDK_EXPORT_HPP_ */

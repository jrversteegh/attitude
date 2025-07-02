/**
 * \file
 * \brief Provide program compilation configuration
 *
 * \author J.R. Versteegh <j.r.versteegh@gmail.com>
 *
 * Build configuration
 */

/*
 * Conditional includes and definitions of optional packages here
 */

#ifndef TRIX_CONFIG_H__
#define TRIX_CONFIG_H__

/* #undef NUMBER_FORMAT */
/* #undef FORMAT_STRING */

namespace trix {

#ifdef NUMBER_FORMAT
  using Number = NUMBER_FORMAT;
#else
  using Number = double;
#endif

#ifdef FORMAT_STRING
#define trix_fmtstr FORMAT_STRING
#else
#define trix_fmtstr "{}"
#endif

using size_t = std::size_t;

template <typename T>
concept ScalarConcept = std::is_integral_v<T> || std::is_floating_point_v<T>;

}

#endif

// ======================================================================== //
// Copyright SCI Institute, University of Utah, 2018
// ======================================================================== //

#pragma once

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif
#include <chrono>
#include <sstream>
#include <type_traits>

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");


// helper function to write the rendered image as PPM file
inline void writePPM(const char *fileName,
                     const size_t sizex,
                     const size_t sizey,
                     const uint32_t *pixel)
{
  FILE *file = fopen(fileName, "wb");
  if (!file) {
    fprintf(stderr, "fopen('%s', 'wb') failed: %d", fileName, errno);
    return;
  }
  fprintf(file, "P6\n%zu %zu\n255\n", sizex, sizey);
  unsigned char *out = (unsigned char *)alloca(3 * sizex);
  for (int y = 0; y < sizey; y++) {
    const unsigned char *in =
        (const unsigned char *)&pixel[(sizey - 1 - y) * sizex];
    for (int x = 0; x < sizex; x++) {
      out[3 * x + 0] = in[4 * x + 0];
      out[3 * x + 1] = in[4 * x + 1];
      out[3 * x + 2] = in[4 * x + 2];
    }
    fwrite(out, 3 * sizex, sizeof(char), file);
  }
  fprintf(file, "\n");
  fclose(file);
}
inline void writePPM(const std::string &fileName,
                     const size_t sizex,
                     const size_t sizey,
                     const uint32_t *pixel)
{
  writePPM(fileName.c_str(), sizex, sizey, pixel);
}

// timer
typedef std::chrono::high_resolution_clock::time_point time_point;
inline time_point Time()
{
  return std::chrono::high_resolution_clock::now();
}
inline double Time(const time_point &t1)
{
  time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> et =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  return et.count();
}


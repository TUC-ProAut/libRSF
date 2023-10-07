/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file Messages.h
 * @author Tim Pfeifer
 * @date 06.03.2019
 * @brief Functions for error messages and warnings.
 * @copyright GNU Public License.
 *
 */

#ifndef MESSAGES_H
#define MESSAGES_H

#include "Types.h"
#include "VectorMath.h"

#include <string>
#include <iostream>

#define FILENAME (std::string(__FILE__).substr(std::string(__FILE__).find_last_of('/') + 1))
#define PRINT_ERROR(...) libRSF::PrintError(FILENAME, __FUNCTION__, __LINE__, __VA_ARGS__)
#define PRINT_WARNING(...) libRSF::PrintWarning(FILENAME, __FUNCTION__, __LINE__, __VA_ARGS__)
#define PRINT_LOGGING(...) libRSF::PrintLogging(FILENAME, __FUNCTION__, __LINE__, __VA_ARGS__)

namespace libRSF
{
  /** recursive variadic function to print multiple strings */
  template<typename T>
  void Print(const T Arg)
  {
    std::cout << Arg;
  }

  void PrintVariadic();

  template<typename T, typename... MoreStrings>
  void PrintVariadic(const T Begin, const MoreStrings... Rest)
  {
      Print(Begin);
      PrintVariadic(Rest...);
  }

  template<typename... MoreStrings>
  void PrintError (const std::string &File,const  std::string &Function, const int Line, const MoreStrings... Messages)
  {
    PrintVariadic("Error in ", File, " | Line ", Line, " | ", Function, "(): ", Messages...);
  }

  template<typename... MoreStrings>
  void PrintWarning (const std::string &File, const std::string &Function, const int Line, const MoreStrings... Messages)
  {
    PrintVariadic("Warning in ", File, " | Line ", Line, " | ", Function, "(): ", Messages...);
  }

  template<typename... MoreStrings>
  void PrintLogging (const std::string &File, const std::string &Function, const int Line, const MoreStrings... Messages)
  {
    PrintVariadic("Logging in ", File, " | Line ", Line, " | ", Function, "(): ", Messages...);
  }
}


#endif // MESSAGES_H

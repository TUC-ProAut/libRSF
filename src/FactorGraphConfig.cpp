/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
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

#include "FactorGraphConfig.h"

namespace libRSF
{
  FactorGraphConfig::FactorGraphConfig()
  {
    InputFile = nullptr;
    OutputFile = nullptr;
  }

  bool FactorGraphConfig::ReadCommandLineOptions(int argc, char** argv)
  {
    int ArgCounter = 1;

    /** read in and output files */
    InputFile = argv[ArgCounter];
    ArgCounter++;
    OutputFile = argv[ArgCounter];
    ArgCounter++;

    while(ArgCounter < argc)
    {
      if(strcmp(argv[ArgCounter], "error:") == 0)
      {
        ArgCounter++;

        /** read error model */
        if(strcmp(argv[ArgCounter], "gauss") == 0)
        {
          RangeErrorModel.Type = ROBUST_NONE;
        }
        else if(strcmp(argv[ArgCounter], "dcs") == 0)
        {
          RangeErrorModel.Type = ROBUST_DCS;
        }
        else if(strcmp(argv[ArgCounter], "cdce") == 0)
        {
          RangeErrorModel.Type = ROBUST_CDCE;
        }
        else if(strcmp(argv[ArgCounter], "mm") == 0)
        {
          RangeErrorModel.Type = ROBUST_MM;
        }
        else if(strcmp(argv[ArgCounter], "sm") == 0)
        {
          RangeErrorModel.Type = ROBUST_SM;
        }
        else if(strcmp(argv[ArgCounter], "stsm") == 0)
        {
          RangeErrorModel.Type = ROBUST_STSM;
        }
        else if(strcmp(argv[ArgCounter], "stmm") == 0)
        {
          RangeErrorModel.Type = ROBUST_STMM;
        }
        else
        {
          std::cerr << "Wrong range error model: " << argv[ArgCounter];
          return false;
        }
      }

      ArgCounter++;
    }

    if(ArgCounter == argc)
    {
      return true;
    }
    else
    {
      std::cerr << "Wrong number of command line args";
      return false;
    }
  }
}

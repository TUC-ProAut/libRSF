/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C); 2019 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

/**
 * @file AppPool_Adaptive.h
 * @author Tim Pfeifer
 * @date 09 August 2019
 * @brief Contains a set of functions, related to adaptive error models.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_ADAPTIVE_H
#define APPPOOL_ADAPTIVE_H

#include "libRSF.h"

bool AdaptErrorModel(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     libRSF::Data &IterationSummary);

void AdaptGeneric1D(libRSF::FactorGraph &Graph,
                    libRSF::FactorType Factor,
                    const libRSF::FactorGraphConfig::ErrorModelConfig &ErrorConfig,
                    bool RemoveOffset,
                    bool UseAsymmetricInit);

void AdaptGeneric2D(libRSF::FactorGraph &Graph,
                    libRSF::FactorType Factor,
                    const libRSF::FactorGraphConfig::ErrorModelConfig &ErrorConfig,
                    bool EstimateMean);

#endif // APPPOOL_ADAPTIVE_H

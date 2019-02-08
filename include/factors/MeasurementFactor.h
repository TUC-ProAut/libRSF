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

/**
 * @file MeasurementFactor.h
 * @author Tim Pfeifer
 * @date 12.03.2018
 * @brief Factor base class that separates the probabilistic error model from the physical sensor model
 * @copyright GNU Public License.
 *
 */

#ifndef MEASUREMENTFACTOR_H
#define MEASUREMENTFACTOR_H
#include <ceres/ceres.h>
#include "../SensorData.h"
#include "../error_models/ErrorModel.h"

using ceres::AutoDiffCostFunction;

namespace libRSF
{
  class MeasurementList
  {
    public:
      MeasurementList(){};
      ~MeasurementList(){};

      void add(SensorData &Measurement);
      SensorData& get(SensorType Type) const;
      SensorData& getFirst() const;
      void clear();
      void remove(SensorData &Measurement);
      bool isEmpty() const;

    private:
      std::map<SensorType, SensorData*> _List;
  };

  class SensorModel
  {
    public:
      SensorModel(){};
      virtual ~SensorModel(){};

      int getOutputDim()
      {
        return _OutputDim;
      };

      int getInputDim()
      {
        return _InputDim;
      };

    protected:
      int _OutputDim;
      int _InputDim;

    private:

  };

  template <typename ModelType,
            typename ErrorType,
            int... StateDims>   // Number of parameters in block 9.
  class MeasurementFactor
  {
    public:
      /** Default constructor */
      MeasurementFactor() {};
      MeasurementFactor(ErrorType &Error, MeasurementList &Measurements) {};

      void CheckInput()
      {
        /** check if physical and probabilistic model have the same number of dimensions */
        if (_Model.getOutputDim() != _Error.getInputDim())
        {
          std::cerr << "Error: Model dimensionality " << _Model.getOutputDim()
                    << " does not match error model dim " << _Error.getInputDim()
                    << "!" <<std::endl;
        }

        /** check if physical model and provided measurement have the same number of dimensions */
        if (_MeasurementVector.size() != _Model.getInputDim())
        {
          std::cerr << "Error: Measurement dimensionality " << _MeasurementVector.size()
                    << " does not match sensor model dim " << _Model.getInputDim()
                    << "!" <<std::endl;
        }
      }

      ErrorType* getErrorModel() {return &_Error;}

      /** Default destructor */
      virtual ~MeasurementFactor(){};

      /** store the dimension of variables at compile time */
      using _StateDims = std::integer_sequence<int, StateDims...>;

    protected:
      ModelType   _Model; /**< represents the physical measurement function */
      ErrorType   _Error; /**< represent the probabilistic error function */
      ceres::Vector _MeasurementVector; /**< a efficient representation of the measurement */

  };
}

#endif // MEASUREMENTFACTOR_H

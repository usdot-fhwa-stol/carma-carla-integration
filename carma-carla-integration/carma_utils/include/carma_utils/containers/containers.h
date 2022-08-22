#pragma once
/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/**
 * Containers namespace contains utility functions that can be used with C++ containers.
 */

#include <vector>
namespace carma_utils
{
namespace containers
{
/**
 * \brief Downsamples an input vector by saving only each nth element.
 * For example, given an input vector of { 0, 1, 2, 3, 4, 5 } and n = 2
 * The output vector will be {0, 2, 4}
 * 
 * \param input The input vector to downsample
 * \param n The count of the elements to save
 * \return The downsampled vector
 */ 
template <class T, class A = std::allocator<T>>
std::vector<T, A> downsample_vector(const std::vector<T, A>& input, unsigned int n)
{
  std::vector<T, A> output;

  if (n == 0 || input.empty()) {
    return output;
  }

  output.reserve((input.size() / n) + 1);

  for (size_t i = 0; i < input.size(); i += n)
  {
    output.push_back(input[i]);
  }

  return output;
}
}  // namespace containers
}  // namespace carma_utils

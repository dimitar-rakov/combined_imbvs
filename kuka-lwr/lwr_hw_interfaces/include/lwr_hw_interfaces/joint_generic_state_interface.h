///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, TUM.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Dimitar Rakov

#ifndef HARDWARE_INTERFACE_GENERIC_JOINT_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_GENERIC_JOINT_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/** A handle used to read the state of a single joint. */
class JointGenericStateHandle
{
public:
  JointGenericStateHandle() : name_(), state_(0) {}

  /**
   * \param name The name of the joint
   * \param state  pointer to the storage for this joint's state
   */
  JointGenericStateHandle(const std::string& name, std::vector<double>* state)
    : name_(name), state_(state)
  {
    if (!state)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. State data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  std::vector<double> getState()    const {assert(state_); return *state_;}
private:
  std::string name_;
  std::vector<double>* state_;
};

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class JointGenericStateInterface : public HardwareResourceManager<JointGenericStateHandle> {};

}

#endif

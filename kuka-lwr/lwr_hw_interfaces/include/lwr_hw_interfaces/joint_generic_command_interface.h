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

#ifndef HARDWARE_INTERFACE_JOINT_GENERIC_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_GENERIC_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <lwr_hw_interfaces/joint_generic_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class JointGenericHandle : public JointGenericStateHandle
{
public:
  JointGenericHandle() : JointGenericStateHandle(), cmd_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  JointGenericHandle(const JointGenericStateHandle& js, std::vector<double>* cmd)
    : JointGenericStateHandle(js), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(std::vector<double> command) {assert(cmd_); *cmd_ = command;}
  std::vector<double> getCommand() const {assert(cmd_); return *cmd_;}

private:
  std::vector<double>* cmd_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class JointGenericCommandInterface : public HardwareResourceManager<JointGenericHandle, ClaimResources> {};

/// \ref JointCommandInterface for commanding effort-based joints.
//class ImpedanceJointInterface : public JointCommandInterface {};

}

#endif

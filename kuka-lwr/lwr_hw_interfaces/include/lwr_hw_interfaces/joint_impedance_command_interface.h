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

#ifndef HARDWARE_INTERFACE_JOINT_IMPEDANCE_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_IMPEDANCE_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class JointImpedanceHandle : public JointStateHandle
{
public:
  JointImpedanceHandle() : JointStateHandle(), pos_cmd_(0),stiffness_cmd_(0), damping_cmd_(0), effort_cmd_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  JointImpedanceHandle(const JointStateHandle& js,double* pos_cmd, double* stiffness_cmd, double* damping_cmd,double* effort_cmd)
    : JointStateHandle(js), pos_cmd_(pos_cmd),stiffness_cmd_(stiffness_cmd),damping_cmd_(damping_cmd), effort_cmd_(effort_cmd)
  {
    if (!pos_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Position command data pointer is null.");
    }
    if (!stiffness_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Stiffness command data pointer is null.");
    }
    if (!damping_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Damping command data pointer is null.");
    }
    if (!effort_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Effort command data pointer is null.");
    }
  }

  void setPositionCommand(double command) {assert(pos_cmd_); *pos_cmd_ = command;}
  double getPositionCommand() const {assert(pos_cmd_); return *pos_cmd_;}
  void setStiffnessCommand(double command) {assert(stiffness_cmd_); *stiffness_cmd_ = command;}
  double getStiffnessCommand() const {assert(stiffness_cmd_); return *stiffness_cmd_;}
  void setDampingCommand(double command) {assert(damping_cmd_); *damping_cmd_ = command;}
  double getDampingCommand() const {assert(damping_cmd_); return *damping_cmd_;}
  void setEffortCommand(double command) {assert(effort_cmd_); *effort_cmd_ = command;}
  double getEffortCommand() const {assert(effort_cmd_); return *effort_cmd_;}

private:
  double* pos_cmd_;
  double* stiffness_cmd_;
  double* damping_cmd_;
  double* effort_cmd_;
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
class ImpedanceJointInterface : public HardwareResourceManager<JointImpedanceHandle, ClaimResources> {};


}

#endif

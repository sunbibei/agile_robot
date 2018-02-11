/*
 * gazebo_propagate.h
 *
 *  Created on: Jan 30, 2018
 *      Author: bibei
 */

#ifndef PLUGINS_INCLUDE_GAZEBO_PROPAGATE_H_
#define PLUGINS_INCLUDE_GAZEBO_PROPAGATE_H_

#include "system/platform/protocol/qr_protocol.h"
#include "foundation/utf.h"
#include "foundation/thread/threadpool.h"

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

#include <vector>
#include <boost/lockfree/queue.hpp>
///! Forward declare
class MsgQueue;

namespace qr_gazebo {

class GzPropagateG : public gazebo::ModelPlugin {

public:
  GzPropagateG();
  virtual ~GzPropagateG();

///! Simulate the style of class middleware::Propagate
protected:
  /**
   * @brief This method completes the function that convert the Packet
   *        to the form of message which be needed by the specific communication
   *        type, and writing the converted message to robot.
   * @param pkt[in] The Packet object need to convert and write
   * @return Return true if successful, or return false
   */
  virtual bool write(const Packet& pkt);
  /**
   * @brief This method must be implemented by subclass, and convert the message
   *        which received from the specific communication type to the form of Packet,
   * @param pkt[out] The Packet object converted from the message
   * @return Return true if read successful, or return false
   */
  virtual bool read(Packet& pkt);

///! inherit from gazebo::ModelPlugin
public:
  /// @brief Load function
  ///
  /// Called when a Plugin is first created, and after the World has been
  /// loaded. This function should not be blocking.
  /// @param[in] _model Pointer to the Model
  /// @param[in] _sdf   Pointer to the SDF element of the plugin.
  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) /*= 0*/ override;
  virtual void Init()  override;
  virtual void Reset() override;

protected:
  virtual void event_update();
  void msg_w_update();
  void msg_r_update();

private:
  void __parse_command_pkg(const Packet&);
  void __write_command_to_sim(const Packet&);

  void __update_robot_stats();

protected:
  gazebo::physics::WorldPtr    world_;
  gazebo::physics::ModelPtr    parent_;
  gazebo::event::ConnectionPtr update_connection_;

  bool                            rw_thread_alive_;
  MsgQueue*                       msgq_;
  std::string                     cmd_msgq_name_;
  std::string                     leg_node_msgq_name_;
  boost::lockfree::queue<Packet>* swap_r_buffer_;
  boost::lockfree::queue<Packet>* swap_w_buffer_;
  ///! These interfaces for Joint, Order by leg * joint
  std::vector<std::vector<gazebo::physics::JointPtr>> joints_;
  ///! These interfaces for Actor
  // MiiVector<gazebo::physics::ActorPtr> actors_;
  ///! NODE_ID map to LEG
  std::vector<LegType>             id_2_leg_lut_;
  std::vector<unsigned char>       leg_2_id_lut_;
  class LinearParams*              linear_params_[LegType::N_LEGS];
};

} /* namespace qr_gazebo */

#endif /* PLUGINS_INCLUDE_GAZEBO_PROPAGATE_H_ */

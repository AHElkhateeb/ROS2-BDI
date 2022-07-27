#ifndef CNET_RESPONDER_HPP_INCLUDED
#define CNET_RESPONDER_HPP_INCLUDED

#include "ros2_bdi_behaviours/tinyfsm.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

// ----------------------------------------------------------------------------
// Event declarations
//

struct MsgReceived : tinyfsm::Event
{
  ACLMessage msg;
};

struct ActionOver : tinyfsm::Event{};


// ----------------------------------------------------------------------------
// ContractNetResponder (FSM base class) declaration
//

class ContractNetResponder
: public tinyfsm::Fsm<ContractNetResponder>, public ConversationsClient
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:

  ContractNetResponder(){}
  ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set);

  void receiveMsg(ACLMessage msg) override
  {
    MsgReceived Message;
    Message.msg = msg;
    dispatch<MsgReceived>(Message);
  }

  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };

  virtual void react(MsgReceived const &);
  virtual void react(ActionOver const &){};

  virtual ACLMessage handleCfp(ACLMessage cfp) { return ACLMessage();};
  virtual ACLMessage handleAcceptProposal(ACLMessage cfp) { return ACLMessage();};
  virtual void handleRejectProposal(ACLMessage cfp) { };

  virtual void entry(void) { };  /* entry actions in some states */
  void         exit(void)  { };  /* no exit actions at all */

  ACLMessage last_message;
};


#endif // CNET_RESPONDER_HPP_INCLUDED
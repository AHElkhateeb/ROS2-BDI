#include "ros2_bdi_behaviours/tinyfsm.hpp"

#include "ros2_bdi_behaviours/CNETResponderBehaviour.hpp"

#include <iostream>

class ReceiveCfp; // forward declaration

// ----------------------------------------------------------------------------
// Constructor with belief and desire sets arguments
//

ContractNetResponder::ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set) : ConversationsClient(desire_set, belief_set)
{
  ContractNetResponder::start();
}

// ----------------------------------------------------------------------------
// Transition functions
//


// ----------------------------------------------------------------------------
// State: EndProtocol
//

class EndProtocol
: public ContractNetResponder
{
  void entry() override {
    //deleteConvIDSharedPointer();
  }
};


// ----------------------------------------------------------------------------
// State: PerformAction
//

class PerformAction
: public ContractNetResponder
{
  void entry() override {
    //send the output of the handleAcceptProposal function
    handleAcceptProposal(last_message);
    dispatch(ActionOver());
  }

  void react(MsgReceived const & Message) 
  {
    if (Message.msg.getPerformative() == "CANCEL")
    {
      std::cout << "PerformAction state: CANCEL message received" << std::endl;
      last_message = Message.msg;
      transit<EndProtocol>();
    }
    else
    {
      std::cout << "PerformAction state: Out of sequence message received" << std::endl;
    } 
  }

  void react(ActionOver const & Message) 
  {
    transit<EndProtocol>();
  }

  virtual ACLMessage handleAcceptProposal(ACLMessage cfp)  override 
  {
    return ACLMessage();
  }
};


// ----------------------------------------------------------------------------
// State: CfpEvaluation
//

class CfpEvaluation
: public ContractNetResponder
{
  void entry() override 
  {
    //send the output of the handleCfp function
    handleCfp(last_message);
  }

  void react(MsgReceived const & Message) 
  {
    if (Message.msg.getPerformative() == "ACCEPT")
      {
        std::cout << "CfpEvaluation state: ACCEPT message received" << std::endl;
        last_message = Message.msg;
        transit<PerformAction>();
      }
    else if (Message.msg.getPerformative() == "REJECT")
      {
        std::cout << "CfpEvaluation state: REJECT message received" << std::endl;
        last_message = Message.msg;
        transit<EndProtocol>();
      }
      else
      std::cout << "CfpEvaluation state: Out of sequence message received" << std::endl;
  }

  virtual ACLMessage handleCfp(ACLMessage cfp)  override 
  {
    return ACLMessage();
  }
};


// ----------------------------------------------------------------------------
// State: ReceiveCfp
//

class ReceiveCfp
: public ContractNetResponder
{
  void entry() override {
    std::cout << "Waiting for CFP" << std::endl;
  }

  void react(MsgReceived const & Message) 
  {
    if (Message.msg.getPerformative() == "CFP")
      {
        std::cout << "ReceiveCfp state: CFP message received" << std::endl;
        last_message = Message.msg;
        transit<CfpEvaluation>();
      }
    else
      std::cout << "ReceiveCfp state: Out of sequence message received" << std::endl;
  }

};


// ----------------------------------------------------------------------------
// Base state: default implementations
//

void ContractNetResponder::react(MsgReceived const & Message) {
    std::cout << "Base state: Out of sequence message received" << std::endl;
}


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(ContractNetResponder, ReceiveCfp)